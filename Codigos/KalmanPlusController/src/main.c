/*
 * Controlador pan-tilt: recibe de la Raspberry Pi el error del objetivo en la imagen y mueve
 * los dos motores paso a paso con un PI de velocidad por eje.
 *
 * Módulos (cada uno con su header y sin depender de los demás):
 *     receptor_pi  recibe por UART el error ex, ey [px] que envía interfaz.py
 *     camara       convierte el error de píxeles a ángulo
 *     control_pi   PI de velocidad de un eje
 *     motor_pasos  genera los pasos de un motor con DRV8825
 * Este archivo los configura, crea una instancia de cada uno y los conecta en la tarea de control.
 *
 * La UART0 (USB) queda para el monitor a 115200.
 */

#include <inttypes.h>
#include <stdio.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "camara.h"
#include "control_pi.h"
#include "motor_pasos.h"
#include "receptor_pi.h"

// ---------------- Configuración ----------------
#define ACTIVAR_CONTROL false  // false: solo recepción, los motores quedan sin habilitar
#define TS (1.0f / 50.0f)      // período de control [s], a la par de la cámara (FPS_CAMARA = 50 en interfaz.py)
#define PERIODO_IMPRESION_MS 100
#define RAD_A_GRADOS (360.0f / 6.28318531f)

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
static ControlPi controlPan;
static ControlPi controlTilt;

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
            camaraErrorAngular(&camara, medicion.ex, medicion.ey, &ePan, &eTilt);
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

// ---------------- Programa ----------------
void app_main(void)
{
    receptorPiIniciar(&receptor, &CONFIG_RECEPTOR);
    camaraIniciar(&camara, &CONFIG_CAMARA);

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

        if (visible) {
            printf("ex: %d px | ey: %d px", e.ex, e.ey);
        } else {
            printf("Sin objetivo");
        }
        printf(" | %.1f Hz | descartados: %" PRIu32, tasa, descartados);
        printf(" | pan: %.1f° tilt: %.1f°\n", controlPan.theta * RAD_A_GRADOS, controlTilt.theta * RAD_A_GRADOS);
    }
}
