/*
 * Controlador pan-tilt: recibe de la Raspberry Pi el error del objetivo en la imagen y mueve
 * los dos motores paso a paso con un PI de velocidad por eje.
 *
 * Módulos (cada uno con su header y sin depender de los demás):
 *     receptor_pi  recibe por UART el error ex, ey [px] que envía interfaz.py
 *     camara       convierte el error en píxeles a la dirección del objetivo respecto del gimbal
 *     control_pi   PI de velocidad de un eje
 *     motor_pasos  genera los pasos de un motor con DRV8825
 *     encoder_as5600  lee el ángulo del eje de un motor
 * Este archivo los configura, crea una instancia de cada uno y los conecta en la tarea de control.
 *
 * La UART0 (USB) queda para el monitor a 115200.
 */

#include <inttypes.h>
#include <math.h>
#include <stdio.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "camara.h"
#include "control_pi.h"
#include "encoder_as5600.h"
#include "motor_pasos.h"
#include "receptor_pi.h"

// ---------------- Configuración ----------------
#define ACTIVAR_CONTROL false  // false: solo recepción, los motores quedan sin habilitar
#define TS (1.0f / 50.0f)      // período de control [s], a la par de la cámara (FPS_CAMARA = 50 en interfaz.py)
#define PERIODO_IMPRESION_MS 100
#define DOS_PI 6.28318531f
#define RAD_A_GRADOS (360.0f / DOS_PI)

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
    .micropasos = 1,
    .reduccion = 1.0f,
    .signo = 1.0f,
};

static const ConfigMotorPasos CONFIG_MOTOR_TILT = {
    .pines = {.en = GPIO_NUM_32, .step = GPIO_NUM_12, .dir = GPIO_NUM_13,
              .m0 = GPIO_NUM_26, .m1 = GPIO_NUM_27, .m2 = GPIO_NUM_14},
    .micropasos = 1,
    .reduccion = 1.0f,
    .signo = 1.0f,
};

// Un AS5600 por motor, cada uno en su bus I2C (pull-ups de 4.7 kΩ en la PCB)
static const ConfigEncoderAs5600 CONFIG_ENCODER_PAN = {
    .puerto = I2C_NUM_0,
    .sda = GPIO_NUM_21,
    .scl = GPIO_NUM_22,
    .reduccion = 1.0f,  // la misma que la del motor del pan
    .signo = 1.0f,
};

static const ConfigEncoderAs5600 CONFIG_ENCODER_TILT = {
    .puerto = I2C_NUM_1,
    .sda = GPIO_NUM_25,
    .scl = GPIO_NUM_33,
    .reduccion = 1.0f,  // la misma que la del motor del tilt
    .signo = 1.0f,
};

// Ganancias iniciales: ajustarlas en la plataforma real
static const ConfigControlPi CONFIG_CONTROL_PAN = {
    .Kp = 2.0f,
    .Ki = 0.5f,
    .omegaMax = 3.0f,
    .ts = TS,
    .tPrediccion = 1.0f,
};

static const ConfigControlPi CONFIG_CONTROL_TILT = {
    .Kp = 2.0f,
    .Ki = 0.5f,
    .omegaMax = 3.0f,
    .ts = TS,
    .tPrediccion = 1.0f,
};

// ---------------- Instancias ----------------
static ReceptorPi receptor;
static Camara camara;
static MotorPasos motorPan;
static MotorPasos motorTilt;
static EncoderAs5600 encoderPan;
static EncoderAs5600 encoderTilt;
static ControlPi controlPan;
static ControlPi controlTilt;

// ---------------- Ángulos ----------------
// Ángulo actual de un eje de la plataforma [rad]: el que mide el encoder o, si el sensor
// no está, el comandado por el control. El cero (α = β = 0) es la posición de arranque.
static float anguloEje(EncoderAs5600 *enc, const ControlPi *control)
{
    float angulo;
    if (encoderAs5600Leer(enc, &angulo)) {
        return angulo;
    }
    return control->theta;
}

// Pan y tilt a los que está el objetivo respecto del gimbal y error de cada eje (objetivo - plataforma)
static void errorObjetivo(const ErrorObjetivo *medicion, float *alfaObj, float *betaObj, float *ePan, float *eTilt)
{
    float alfa = anguloEje(&encoderPan, &controlPan);
    float beta = anguloEje(&encoderTilt, &controlTilt);

    camaraDireccionObjetivo(&camara, medicion->ex, medicion->ey, alfa, beta, alfaObj, betaObj);

    // αobj está entre -π y π y el pan de la plataforma acumula vueltas: se toma el camino más corto
    *ePan = remainderf(*alfaObj - alfa, DOS_PI);
    *eTilt = *betaObj - beta;
}

// ---------------- Lazo de control ----------------
static TaskHandle_t tareaControlHandle;

static void tareaControl(void *arg)
{
    while (true) {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // la despierta el temporizador cada TS

        ErrorObjetivo medicion;
        bool visible = receptorPiLeer(&receptor, &medicion);

        float ePan = 0.0f;
        float eTilt = 0.0f;
        if (visible) {
            float alfaObj, betaObj;
            errorObjetivo(&medicion, &alfaObj, &betaObj, &ePan, &eTilt);
        }

        motorPasosFijarVelocidad(&motorPan, controlPiPaso(&controlPan, ePan, visible));
        motorPasosFijarVelocidad(&motorTilt, controlPiPaso(&controlTilt, eTilt, visible));
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
            float alfaObj, betaObj, ePan, eTilt;
            errorObjetivo(&e, &alfaObj, &betaObj, &ePan, &eTilt);
            printf("ex: %d px | ey: %d px | objetivo: pan %.1f° tilt %.1f°", e.ex, e.ey,
                   alfaObj * RAD_A_GRADOS, betaObj * RAD_A_GRADOS);
        } else {
            printf("Sin objetivo");
        }
        printf(" | %.1f Hz | descartados: %" PRIu32, tasa, descartados);

        // Ángulo comandado por el control y medido por el encoder: si se separan, el motor perdió pasos
        char encPan[16], encTilt[16];
        textoEncoder(&encoderPan, encPan, sizeof(encPan));
        textoEncoder(&encoderTilt, encTilt, sizeof(encTilt));
        uint32_t erroresI2c = encoderAs5600Errores(&encoderPan) + encoderAs5600Errores(&encoderTilt);

        printf(" | pan: cmd %.1f° enc %s | tilt: cmd %.1f° enc %s | errores I2C: %" PRIu32 "\n",
               controlPan.theta * RAD_A_GRADOS, encPan, controlTilt.theta * RAD_A_GRADOS, encTilt, erroresI2c);
    }
}
