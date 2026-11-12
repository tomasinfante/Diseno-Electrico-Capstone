/*
 * Controlador pan-tilt: la Raspberry Pi le informa la etapa del programa y el error del objetivo
 * en la imagen, y este programa mueve los dos motores paso a paso según la etapa:
 *     Detenido     motores detenidos
 *     Calibrar     motores liberados: la plataforma se acomoda a mano y la posición en que
 *                  queda al salir de la etapa pasa a ser el ángulo cero de cada eje
 *     Búsqueda     barrido suave y periódico en pan y tilt, hasta que la cámara ve al objetivo
 *     Seguimiento  un filtro de Kalman por eje estima la dirección del objetivo y un PI de
 *                  velocidad lleva la plataforma hacia esa estimación
 *
 * Módulos (cada uno con su header y sin depender de los demás):
 *     receptor_pi  recibe por UART la etapa, el error ex, ey [px] y los parámetros del control
 *     camara       convierte el error en píxeles a la dirección del objetivo respecto del gimbal
 *     Kalman       estima la posición y la velocidad angular del objetivo en un eje
 *     control_pi   PI de velocidad de un eje
 *     barrido      movimiento de búsqueda de un eje
 *     motor_pasos  genera los pasos de un motor con DRV8825
 *     encoder_as5600  lee el ángulo del eje de un motor
 * Este archivo los configura, crea una instancia de cada uno y los conecta en la tarea de control.
 *
 * La UART0 (USB) queda para el monitor a 115200.
 */

#include <inttypes.h>
#include <math.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "barrido.h"
#include "camara.h"
#include "control_pi.h"
#include "encoder_as5600.h"
#include "Kalman.h"
#include "motor_pasos.h"
#include "receptor_pi.h"

// ---------------- Configuración ----------------
#define ACTIVAR_CONTROL true  // false: solo recepción, los motores quedan sin habilitar
#define TS (1.0f / 50.0f)      // período de control [s], a la par de la cámara (FPS_CAMARA = 50 en interfaz.py)
#define PERIODO_IMPRESION_MS 100
#define DOS_PI 6.28318531f
#define RAD_A_GRADOS (360.0f / DOS_PI)
#define GRADOS_A_RAD (DOS_PI / 360.0f)

#define ANCHO_IMG 640  // deben coincidir con interfaz.py
#define ALTO_IMG 480

static const ConfigReceptorPi CONFIG_RECEPTOR = {
    .puerto = UART_NUM_2,
    .pinRx = 16,
    .pinTx = 17,
    .baudios = 115200,
    .anchoImg = ANCHO_IMG,
    .altoImg = ALTO_IMG,
    .timeoutUs = 100 * 1000,  // ~5 frames a 50 FPS sin paquetes -> objetivo perdido
};

static const ConfigCamara CONFIG_CAMARA = {
    .ancho = ANCHO_IMG,
    .alto = ALTO_IMG,
    .fovH = 62.2f,  // Camera Module 2 con el sensor completo
    .fovV = 48.8f,
};

// El motor 1 mueve el pan y el motor 2 el tilt
static const ConfigMotorPasos CONFIG_MOTOR_PAN = {
    .pines = {.en = GPIO_NUM_23, .step = GPIO_NUM_2, .dir = GPIO_NUM_15,
              .m0 = GPIO_NUM_19, .m1 = GPIO_NUM_18, .m2 = GPIO_NUM_5},
    .micropasos = 4,
    .reduccion =4.3333f,
    .signo = -1.0f,
};

static const ConfigMotorPasos CONFIG_MOTOR_TILT = {
    .pines = {.en = GPIO_NUM_32, .step = GPIO_NUM_12, .dir = GPIO_NUM_13,
              .m0 = GPIO_NUM_26, .m1 = GPIO_NUM_27, .m2 = GPIO_NUM_14},
    .micropasos = 4,
    .reduccion = 3.5f,
    .signo = 1.0f,
};

// Un AS5600 por motor, cada uno en su bus I2C (pull-ups de 4.7 kΩ en la PCB)
static const ConfigEncoderAs5600 CONFIG_ENCODER_PAN = {
    .puerto = I2C_NUM_0,
    .sda = GPIO_NUM_21,
    .scl = GPIO_NUM_22,
    .reduccion = 4.3333f,  // la misma que la del motor del pan
    .signo = 1.0f,
};

static const ConfigEncoderAs5600 CONFIG_ENCODER_TILT = {
    .puerto = I2C_NUM_1,
    .sda = GPIO_NUM_25,
    .scl = GPIO_NUM_33,
    .reduccion = 3.5f,  // la misma que la del motor del tilt
    .signo = 1.0f,
};

// Sin mediciones el filtro de Kalman sigue prediciendo al objetivo durante este tiempo [s];
// después se olvida la estimación y los motores se detienen
#define T_PREDICCION 1.0f

// Valores iniciales: ajustarlos en la plataforma real
static const ConfigKalman CONFIG_KALMAN_PAN = {
    .ts = TS,
    .sigmaMedicion = 0.004f,    // ~2 px de ruido en la detección (1 px ≈ 0.0019 rad)
    .sigmaAceleracion = 0.5f,
    .sigmaOmegaInicial = 0.5f,
};

static const ConfigKalman CONFIG_KALMAN_TILT = {
    .ts = TS,
    .sigmaMedicion = 0.004f,
    .sigmaAceleracion = 0.5f,
    .sigmaOmegaInicial = 0.5f,
};

// Búsqueda: el pan recorre 180° (±90°) y el tilt 45° (±22.5°) alrededor del cero de cada eje.
// Los períodos son distintos para que el barrido vaya cubriendo toda la zona.
static const ConfigBarrido CONFIG_BARRIDO_PAN = {
    .amplitud = 90.0f * GRADOS_A_RAD,
    .periodo = 12.0f,
    .Kp = 2.0f,
    .tArranque = 1.0f,
    .ts = TS,
};

static const ConfigBarrido CONFIG_BARRIDO_TILT = {
    .amplitud = 22.5f * GRADOS_A_RAD,
    .periodo = 30.0f,
    .Kp = 2.0f,
    .tArranque = 1.0f,
    .ts = TS,
};

// En Búsqueda la Raspberry envía un paquete por frame: si pasa este tiempo [s] sin
// ninguno se asume que se cortó la comunicación y los motores se detienen
#define T_SIN_ENLACE 0.5f

// Al salir de Calibrar los drivers se vuelven a habilitar y cada motor se acomoda en su paso
// más cercano: se espera este tiempo [s] con los motores quietos antes de fijar el cero
#define T_ASENTAR 0.2f

// Ganancias iniciales: las reemplazan las que envía la Raspberry (se ajustan desde la página).
// tPrediccion = 0: la predicción sin objetivo la hace el filtro de Kalman (T_PREDICCION), no el PI
static const ConfigControlPi CONFIG_CONTROL_PAN = {
    .Kp = 2.0f,
    .Ki = 0.5f,
    .omegaMax = 3.0f,
    .ts = TS,
    .tPrediccion = 0.0f,
};

static const ConfigControlPi CONFIG_CONTROL_TILT = {
    .Kp = 2.0f,
    .Ki = 0.5f,
    .omegaMax = 3.0f,
    .ts = TS,
    .tPrediccion = 0.0f,
};

// ---------------- Instancias ----------------
static ReceptorPi receptor;
static Camara camara;
static MotorPasos motorPan;
static MotorPasos motorTilt;
static EncoderAs5600 encoderPan;
static EncoderAs5600 encoderTilt;
static Kalman kalmanPan;
static Kalman kalmanTilt;
static ControlPi controlPan;
static ControlPi controlTilt;
static Barrido barridoPan;
static Barrido barridoTilt;

// ---------------- Ángulos ----------------
// Ángulo comandado a cada eje desde el cero [rad]: la integral de la velocidad pedida al motor
static float thetaCmdPan;
static float thetaCmdTilt;

// Fija la velocidad de los dos ejes [rad/s] hasta el próximo paso del lazo de control
static void moverEjes(float omegaPan, float omegaTilt)
{
    motorPasosFijarVelocidad(&motorPan, omegaPan);
    motorPasosFijarVelocidad(&motorTilt, omegaTilt);
    thetaCmdPan += omegaPan * TS;
    thetaCmdTilt += omegaTilt * TS;
}

// Ángulo actual de un eje de la plataforma [rad]: el que mide el encoder o, si el sensor
// no está, el comandado. El cero (α = β = 0) es la posición de arranque o la que se dejó
// a mano en la etapa Calibrar.
static float anguloEje(EncoderAs5600 *enc, float thetaCmd)
{
    float angulo;
    if (encoderAs5600Leer(enc, &angulo)) {
        return angulo;
    }
    return thetaCmd;
}

// Tiempo que toma camaraDireccionObjetivo [µs], acumulado entre impresiones del monitor.
// Si una interrupción u otra tarea corta el cálculo, esa muestra sale más larga (se nota en el máximo).
static portMUX_TYPE muxTiempoCalculo = portMUX_INITIALIZER_UNLOCKED;
static uint32_t tiempoCalculoSuma;
static uint32_t tiempoCalculoMax;
static uint32_t tiempoCalculoN;

// Pan y tilt a los que está el objetivo respecto del gimbal, con la plataforma en (alfa, beta)
static void direccionObjetivo(const ErrorObjetivo *medicion, float alfa, float beta, float *alfaObj, float *betaObj)
{
    int64_t t0 = esp_timer_get_time();
    camaraDireccionObjetivo(&camara, medicion->ex, medicion->ey, alfa, beta, alfaObj, betaObj);
    uint32_t dt = (uint32_t)(esp_timer_get_time() - t0);

    taskENTER_CRITICAL(&muxTiempoCalculo);
    tiempoCalculoSuma += dt;
    tiempoCalculoN++;
    if (dt > tiempoCalculoMax) {
        tiempoCalculoMax = dt;
    }
    taskEXIT_CRITICAL(&muxTiempoCalculo);
}

// ---------------- Lazo de control ----------------
static const char *TAG = "control";
static TaskHandle_t tareaControlHandle;

// Aplica al PI los parámetros que envía la Raspberry y devuelve la velocidad máxima vigente [rad/s].
// Mientras no llegue ninguno valen los de CONFIG_CONTROL_PAN y CONFIG_CONTROL_TILT.
static float aplicarParametros(void)
{
    static ParametrosControl aplicados;
    static bool hayAplicados = false;

    ParametrosControl p;
    if (receptorPiParametros(&receptor, &p)) {
        bool cambio = !hayAplicados || p.omegaMax != aplicados.omegaMax ||
                      p.kpPan != aplicados.kpPan || p.kiPan != aplicados.kiPan ||
                      p.kpTilt != aplicados.kpTilt || p.kiTilt != aplicados.kiTilt;
        if (cambio) {
            controlPiAjustar(&controlPan, p.kpPan, p.kiPan, p.omegaMax);
            controlPiAjustar(&controlTilt, p.kpTilt, p.kiTilt, p.omegaMax);
            aplicados = p;
            hayAplicados = true;
            ESP_LOGI(TAG, "Parametros: omegaMax %.3f rad/s | pan Kp %.3f Ki %.3f | tilt Kp %.3f Ki %.3f",
                     p.omegaMax, p.kpPan, p.kiPan, p.kpTilt, p.kiTilt);
        }
    }

    return controlPan.config.omegaMax;
}

// Seguimiento: el filtro de Kalman estima la dirección del objetivo y el PI lleva la plataforma hacia ella
static void pasoSeguimiento(bool entrando, const ErrorObjetivo *medicion, bool visible, float alfa, float beta)
{
    static int64_t tMedicionUsada = -1;  // t_us del último paquete que se le entregó al filtro
    static float tSinMedicion = 0.0f;    // tiempo desde ese paquete [s]

    if (entrando) {
        kalmanReiniciar(&kalmanPan);
        kalmanReiniciar(&kalmanTilt);
        controlPiReiniciar(&controlPan);
        controlPiReiniciar(&controlTilt);
        tSinMedicion = 0.0f;
    }

    // El receptor entrega el último paquete mientras no caduque: al filtro solo se le
    // pasa una vez, cuando es nuevo
    bool nueva = visible && medicion->t_us != tMedicionUsada;

    float alfaObj = 0.0f;
    float betaObj = 0.0f;
    if (nueva) {
        direccionObjetivo(medicion, alfa, beta, &alfaObj, &betaObj);
        tMedicionUsada = medicion->t_us;
        tSinMedicion = 0.0f;
    } else {
        tSinMedicion += TS;
    }

    // Tras mucho tiempo sin ver al objetivo no hay nada que estimar
    if (tSinMedicion > T_PREDICCION) {
        kalmanReiniciar(&kalmanPan);
        kalmanReiniciar(&kalmanTilt);
    }

    // Con medición nueva el filtro corrige; sin ella predice con la última velocidad estimada
    float alfaEst = kalmanPaso(&kalmanPan, alfaObj, nueva);
    float betaEst = kalmanPaso(&kalmanTilt, betaObj, nueva);
    bool estimado = kalmanPan.iniciado;

    // Error de cada eje: objetivo estimado - plataforma. El pan de la plataforma
    // acumula vueltas, así que se toma el camino más corto
    float ePan = estimado ? remainderf(alfaEst - alfa, DOS_PI) : 0.0f;
    float eTilt = estimado ? betaEst - beta : 0.0f;

    moverEjes(controlPiPaso(&controlPan, ePan, estimado), controlPiPaso(&controlTilt, eTilt, estimado));
}

// Calibrar: los motores quedan liberados para acomodar la plataforma a mano, y la posición en
// que queda al salir de la etapa pasa a ser el cero (α = β = 0). Se llama en todos los pasos
// del lazo; devuelve true mientras la calibración no termina (el resto del control no corre).
static bool pasoCalibrar(EtapaPrograma etapa)
{
    static bool liberados = false;
    static int pasosAsentar = 0;  // pasos del lazo que faltan para fijar el cero

    if (etapa == ETAPA_CALIBRAR) {
        if (!liberados) {
            motorPasosHabilitar(&motorPan, false);
            motorPasosHabilitar(&motorTilt, false);
            liberados = true;
            ESP_LOGI(TAG, "Calibrar: motores liberados, acomodar la plataforma a mano");
        }
        return true;
    }

    if (liberados) {  // se salió de Calibrar: los motores vuelven a sujetar la plataforma
        motorPasosHabilitar(&motorPan, true);
        motorPasosHabilitar(&motorTilt, true);
        liberados = false;
        pasosAsentar = (int)(T_ASENTAR / TS);
    }

    if (pasosAsentar > 0) {
        moverEjes(0.0f, 0.0f);
        pasosAsentar--;
        if (pasosAsentar == 0) {
            encoderAs5600FijarCero(&encoderPan);
            encoderAs5600FijarCero(&encoderTilt);
            thetaCmdPan = 0.0f;
            thetaCmdTilt = 0.0f;
            ESP_LOGI(TAG, "Calibrar: esta posicion es el angulo cero de pan y tilt");
        }
        return true;
    }

    return false;
}

// Búsqueda: barrido suave y periódico de los dos ejes, partiendo desde donde está la plataforma
static void pasoBusqueda(bool entrando, float alfa, float beta, float omegaMax)
{
    if (entrando) {
        barridoReiniciar(&barridoPan, alfa);
        barridoReiniciar(&barridoTilt, beta);
    }

    moverEjes(barridoPaso(&barridoPan, alfa, omegaMax), barridoPaso(&barridoTilt, beta, omegaMax));
}

static void tareaControl(void *arg)
{
    EtapaPrograma etapaAnterior = ETAPA_DETENIDO;

    while (true) {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // la despierta el temporizador cada TS

        ErrorObjetivo medicion;
        bool visible = receptorPiLeer(&receptor, &medicion);
        EtapaPrograma etapa = receptorPiEtapa(&receptor);
        float omegaMax = aplicarParametros();

        // En Búsqueda llega un paquete por frame: sin paquetes recientes se cortó la comunicación.
        // (En Seguimiento no se puede saber así, pero ahí los motores paran solos tras T_PREDICCION.)
        if (etapa == ETAPA_BUSQUEDA && (esp_timer_get_time() - medicion.t_us) > (int64_t)(T_SIN_ENLACE * 1e6f)) {
            etapa = ETAPA_DETENIDO;
        }

        // Calibrar (y el momento en que se fija el cero al salir): el resto del control espera
        if (pasoCalibrar(etapa)) {
            etapaAnterior = ETAPA_CALIBRAR;
            continue;
        }

        float alfa = anguloEje(&encoderPan, thetaCmdPan);
        float beta = anguloEje(&encoderTilt, thetaCmdTilt);
        bool entrando = etapa != etapaAnterior;
        etapaAnterior = etapa;

        // Lo que hace el control depende de la etapa en que está la interfaz
        switch (etapa) {

            case ETAPA_BUSQUEDA:
                pasoBusqueda(entrando, alfa, beta, omegaMax);
                break;

            case ETAPA_SEGUIMIENTO:
                pasoSeguimiento(entrando, &medicion, visible, alfa, beta);
                break;

            case ETAPA_DETENIDO:
            default:
                moverEjes(0.0f, 0.0f);
                break;
        }
    }
}

static void despertarControl(void *arg)
{
    xTaskNotifyGive(tareaControlHandle);
}

// Crea la tarea de control y el temporizador que la despierta cada TS
static void iniciarLazoControl(void)
{
    xTaskCreate(tareaControl, "control", 4096, NULL, 8, &tareaControlHandle);

    esp_timer_handle_t timer;
    esp_timer_create_args_t args = {
        .callback = despertarControl,
        .name = "control",
    };
    ESP_ERROR_CHECK(esp_timer_create(&args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, (uint64_t)(TS * 1e6f)));
}

// ---------------- Monitor ----------------
// Ángulo del encoder en grados, o "--" si el sensor no respondió al iniciar
static void textoEncoder(EncoderAs5600 *enc, char *texto, size_t largo)
{
    float angulo;
    if (encoderAs5600Leer(enc, &angulo)) {
        snprintf(texto, largo, "%.1f°", angulo * RAD_A_GRADOS);
    } else {
        snprintf(texto, largo, "--");
    }
}

// ---------------- Programa ----------------
void app_main(void)
{
    receptorPiIniciar(&receptor, &CONFIG_RECEPTOR);
    camaraIniciar(&camara, &CONFIG_CAMARA);
    encoderAs5600Iniciar(&encoderPan, &CONFIG_ENCODER_PAN);
    encoderAs5600Iniciar(&encoderTilt, &CONFIG_ENCODER_TILT);

    if (ACTIVAR_CONTROL) {
        motorPasosIniciar(&motorPan, &CONFIG_MOTOR_PAN);
        motorPasosIniciar(&motorTilt, &CONFIG_MOTOR_TILT);
        controlPiIniciar(&controlPan, &CONFIG_CONTROL_PAN);
        controlPiIniciar(&controlTilt, &CONFIG_CONTROL_TILT);
        kalmanIniciar(&kalmanPan, &CONFIG_KALMAN_PAN);
        kalmanIniciar(&kalmanTilt, &CONFIG_KALMAN_TILT);
        barridoIniciar(&barridoPan, &CONFIG_BARRIDO_PAN);
        barridoIniciar(&barridoTilt, &CONFIG_BARRIDO_TILT);
        iniciarLazoControl();
    }

    // Impresión de depuración a 10 Hz
    uint32_t paquetesAntes = 0;
    int64_t tAntes = esp_timer_get_time();

    while (true) {

        vTaskDelay(pdMS_TO_TICKS(PERIODO_IMPRESION_MS));

        ErrorObjetivo e;
        bool visible = receptorPiLeer(&receptor, &e);

        uint32_t ok, descartados;
        receptorPiEstadisticas(&receptor, &ok, &descartados);

        int64_t ahora = esp_timer_get_time();
        float tasa = (ok - paquetesAntes) * 1e6f / (ahora - tAntes);
        paquetesAntes = ok;
        tAntes = ahora;

        static const char *const NOMBRES_ETAPA[] = {"?", "Detenido", "Calibrar", "Busqueda", "Seguimiento"};
        printf("%s | ", NOMBRES_ETAPA[receptorPiEtapa(&receptor)]);

        if (visible) {
            float alfaObj, betaObj;
            direccionObjetivo(&e, anguloEje(&encoderPan, thetaCmdPan), anguloEje(&encoderTilt, thetaCmdTilt),
                              &alfaObj, &betaObj);
            printf("ex: %d px | ey: %d px | objetivo: pan %.1f° tilt %.1f°", e.ex, e.ey,
                   alfaObj * RAD_A_GRADOS, betaObj * RAD_A_GRADOS);
        } else {
            printf("Sin objetivo");
        }

        // Estimación del filtro de Kalman (solo en Seguimiento y con el control activo): posición y velocidad
        if (receptorPiEtapa(&receptor) == ETAPA_SEGUIMIENTO && kalmanPan.iniciado) {
            printf(" | kalman: pan %.1f° (%.1f°/s) tilt %.1f° (%.1f°/s)",
                   kalmanPan.theta * RAD_A_GRADOS, kalmanPan.omega * RAD_A_GRADOS,
                   kalmanTilt.theta * RAD_A_GRADOS, kalmanTilt.omega * RAD_A_GRADOS);
        }
        printf(" | %.1f Hz | descartados: %" PRIu32, tasa, descartados);

        // Tiempo de camaraDireccionObjetivo desde la impresión anterior (solo se calcula con objetivo)
        taskENTER_CRITICAL(&muxTiempoCalculo);
        uint32_t suma = tiempoCalculoSuma, maximo = tiempoCalculoMax, n = tiempoCalculoN;
        tiempoCalculoSuma = 0;
        tiempoCalculoMax = 0;
        tiempoCalculoN = 0;
        taskEXIT_CRITICAL(&muxTiempoCalculo);

        // if (n > 0) {
        //     printf(" | calculo: prom %.1f us max %" PRIu32 " us (n=%" PRIu32 ")", (float)suma / n, maximo, n);
        // }

        // Ángulo comandado por el control y medido por el encoder: si se separan, el motor perdió pasos
        char encPan[16], encTilt[16];
        textoEncoder(&encoderPan, encPan, sizeof(encPan));
        textoEncoder(&encoderTilt, encTilt, sizeof(encTilt));
        uint32_t erroresI2c = encoderAs5600Errores(&encoderPan) + encoderAs5600Errores(&encoderTilt);

        printf(" | pan: cmd %.1f° enc %s | tilt: cmd %.1f° enc %s | errores I2C: %" PRIu32 "\n",
               thetaCmdPan * RAD_A_GRADOS, encPan, thetaCmdTilt * RAD_A_GRADOS, encTilt, erroresI2c);
    }
}
