/*
 * Receptor UART del error de seguimiento que envía la Raspberry Pi (Codigos/Raspberry/interfaz.py)
 * Versión ESP-IDF del sketch Codigos/Arduino/receptor_uart.
 *
 * Paquete de 6 bytes por cada frame con objetivo detectado (~50 Hz):
 *     0xAA 0x55 | ex (int16) | ey (int16)   — little-endian, con signo, en [px]
 * Convención (igual que lqr_pan_tilt.py): ex + hacia la derecha, ey + hacia abajo.
 * Si no hay objetivo la Raspberry no envía nada, por eso se detecta su pérdida por timeout.
 *
 * Conexión (solo hace falta Raspberry -> ESP32, ambos a 3.3 V):
 *     Raspberry GPIO14 / TX (pin 8)  ->  GPIO16 (RX de la UART2)
 *     Raspberry GND (pin 6)          ->  GND
 *
 * La UART0 (USB) queda para el monitor a 115200.
 */

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// ---------------- Parámetros (deben coincidir con interfaz.py) ----------------
constexpr uart_port_t UART_PI = UART_NUM_2;
constexpr int PIN_RX = 16;
constexpr int PIN_TX = 17;  // no se usa: la Raspberry solo transmite
constexpr int BAUD_RATE = 115200;
constexpr uint8_t CABECERA_1 = 0xAA;
constexpr uint8_t CABECERA_2 = 0x55;
constexpr int16_t ANCHO_IMG = 640;
constexpr int16_t ALTO_IMG = 480;

constexpr int64_t TIMEOUT_OBJETIVO_US = 100 * 1000;  // ~5 frames a 50 FPS sin paquetes -> objetivo perdido
constexpr int PERIODO_IMPRESION_MS = 100;

constexpr int TAMANO_BUFFER_RX = 1024;
constexpr int TAMANO_COLA_EVENTOS = 20;

static const char *TAG = "receptor_pi";

// ---------------- Recepción ----------------
struct ErrorObjetivo {
    int16_t ex;    // [px] + derecha
    int16_t ey;    // [px] + abajo
    int64_t t_us;  // esp_timer_get_time() al recibirlo
};

enum EstadoRx : uint8_t { ESPERA_CAB1, ESPERA_CAB2, LEYENDO_DATOS };

// Estado del parser: solo lo usa la tarea de recepción
static EstadoRx estadoRx = ESPERA_CAB1;
static uint8_t datos[4];
static uint8_t nDatos = 0;

// Compartido con las demás tareas: se accede siempre dentro de la sección crítica
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static ErrorObjetivo ultimo = {0, 0, 0};
static bool hayDatos = false;
static uint32_t paquetesOk = 0;
static uint32_t paquetesDescartados = 0;

static QueueHandle_t colaUart;

static int16_t leerInt16LE(const uint8_t *p)
{
    return static_cast<int16_t>(p[0] | (p[1] << 8));
}

// Máquina de estados: procesa un byte y devuelve true al completar un paquete válido
static bool procesarByte(uint8_t b, ErrorObjetivo &salida)
{
    switch (estadoRx) {

        case ESPERA_CAB1:
            if (b == CABECERA_1) {
                estadoRx = ESPERA_CAB2;
            }
            return false;

        case ESPERA_CAB2:
            if (b == CABECERA_2) {
                estadoRx = LEYENDO_DATOS;
                nDatos = 0;
            } else if (b != CABECERA_1) {  // con 0xAA 0xAA 0x55 el segundo 0xAA puede ser la cabecera real
                estadoRx = ESPERA_CAB1;
            }
            return false;

        case LEYENDO_DATOS: {
            datos[nDatos++] = b;
            if (nDatos < 4) {
                return false;
            }
            estadoRx = ESPERA_CAB1;

            int16_t ex = leerInt16LE(&datos[0]);
            int16_t ey = leerInt16LE(&datos[2]);

            // No hay checksum: un error fuera de la imagen indica un paquete corrupto o desalineado
            if (ex < -ANCHO_IMG / 2 || ex > ANCHO_IMG / 2 || ey < -ALTO_IMG / 2 || ey > ALTO_IMG / 2) {
                taskENTER_CRITICAL(&mux);
                paquetesDescartados++;
                taskEXIT_CRITICAL(&mux);
                return false;
            }

            salida.ex = ex;
            salida.ey = ey;
            salida.t_us = esp_timer_get_time();
            return true;
        }
    }

    return false;
}

// Despierta con cada evento de la UART: el driver avisa ~1 ms después de que termina
// de llegar un paquete (timeout de RX), así que no hace falta leer por encuesta.
static void tareaUartPi(void *)
{
    uart_event_t evento;
    uint8_t buf[128];

    while (true) {

        if (xQueueReceive(colaUart, &evento, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        switch (evento.type) {

            case UART_DATA: {
                // Se lee todo lo acumulado y se guarda solo el paquete más reciente
                ErrorObjetivo paquete;
                uint32_t nuevos = 0;
                int n;

                while ((n = uart_read_bytes(UART_PI, buf, sizeof(buf), 0)) > 0) {
                    for (int i = 0; i < n; i++) {
                        if (procesarByte(buf[i], paquete)) {
                            nuevos++;
                        }
                    }
                }

                if (nuevos > 0) {
                    taskENTER_CRITICAL(&mux);
                    ultimo = paquete;
                    hayDatos = true;
                    paquetesOk += nuevos;
                    taskEXIT_CRITICAL(&mux);
                    // Aquí se puede avisar a la tarea de control (p. ej. con xTaskNotifyGive)
                }
                break;
            }

            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                // Se perdieron bytes: se descarta lo acumulado y se vuelve a buscar la cabecera
                ESP_LOGW(TAG, "Desborde en la UART, se vacía el buffer");
                uart_flush_input(UART_PI);
                xQueueReset(colaUart);
                estadoRx = ESPERA_CAB1;
                break;

            default:
                break;
        }
    }
}

static void iniciarUartPi()
{
    // En C++ es más simple asignar campo por campo: los inicializadores designados
    // de los ejemplos en C de IDF deben seguir aquí el orden exacto del struct
    uart_config_t config = {};
    config.baud_rate = BAUD_RATE;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(UART_PI, TAMANO_BUFFER_RX, 0, TAMANO_COLA_EVENTOS, &colaUart, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PI, &config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PI, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(tareaUartPi, "uart_pi", 4096, nullptr, 10, nullptr);
}

// Copia el último error recibido. Devuelve true si el objetivo sigue visible
// (llegó un paquete hace menos de TIMEOUT_OBJETIVO_US). Se puede llamar desde cualquier tarea.
static bool leerError(ErrorObjetivo &salida)
{
    taskENTER_CRITICAL(&mux);
    salida = ultimo;
    bool valido = hayDatos;
    taskEXIT_CRITICAL(&mux);

    return valido && (esp_timer_get_time() - salida.t_us) < TIMEOUT_OBJETIVO_US;
}

// ---------------- Motores (DRV8825) ----------------


struct PinesDriver {
    gpio_num_t en, step, dir, m0, m1, m2;
};

constexpr PinesDriver MOTOR_1 = {GPIO_NUM_23, GPIO_NUM_2, GPIO_NUM_15, GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_5};
constexpr PinesDriver MOTOR_2 = {GPIO_NUM_32, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_14};

constexpr int PASOS_POR_VUELTA = 200;  // NEMA 17 (1.8°) por paso

static void iniciarDriver(const PinesDriver &p)
{
    gpio_config_t config = {};
    config.pin_bit_mask = (1ULL << p.en) | (1ULL << p.step) | (1ULL << p.dir) |
                          (1ULL << p.m0) | (1ULL << p.m1) | (1ULL << p.m2);
    config.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&config));

    // M2 M1 M0 = 0 0 0 -> paso completo (si se cambia, ajustar PASOS_POR_VUELTA)
    gpio_set_level(p.m0, 0);
    gpio_set_level(p.m1, 0);
    gpio_set_level(p.m2, 0);
    gpio_set_level(p.en, 0);  // EN es activo en bajo: 0 habilita el driver
}


// ---------------- Generador de pasos ----------------
// Un temporizador a 20 kHz reparte los pasos con un acumulador de fase por motor, así
// la velocidad de cada motor se puede cambiar en cualquier momento sin bloquear al control.
constexpr uint32_t FRECUENCIA_TICK_HZ = 20000;
constexpr float MAX_PASOS_S = FRECUENCIA_TICK_HZ / 2;  // cada pulso ocupa 2 ticks (alto y bajo): no se puede pedir más
constexpr uint32_t UN_PASO_Q16 = 1 << 16;
constexpr float DOS_PI = 6.28318531f;

struct GeneradorPasos {
    PinesDriver pines;
    float reduccion;              // vueltas del motor por vuelta del eje (p. ej. 114/20 con correa y polea de 20 dientes)
    float signo;                  // -1 si el eje gira al revés de la convención (pan + izquierda, tilt + arriba)
    volatile int32_t incremento;  // pasos por tick en Q16, con signo (lo escribe la tarea de control)
    uint32_t fase;
    bool stepAlto;
    bool sentido;                 // nivel actual de DIR
};

// El motor 1 mueve el pan y el motor 2 el tilt
enum { PAN, TILT };
static GeneradorPasos generadores[2] = {
    {MOTOR_1, 1.0f, 1.0f, 0, 0, false, false},
    {MOTOR_2, 1.0f, 1.0f, 0, 0, false, false},
};

static bool tickPasos(gptimer_handle_t, const gptimer_alarm_event_data_t *, void *)
{
    for (GeneradorPasos &g : generadores) {

        // La fase se acumula en todos los ticks; solo el pulso espera cuando no puede salir en este
        bool puedePasar = true;

        if (g.stepAlto) {  // segundo tick del pulso
            gpio_set_level(g.pines.step, 0);
            g.stepAlto = false;
            puedePasar = false;
        }

        int32_t inc = g.incremento;
        bool sentido = inc >= 0;
        if (sentido != g.sentido) {  // DIR se cambia al menos un tick antes del próximo paso (el DRV8825 pide 650 ns)
            gpio_set_level(g.pines.dir, sentido);
            g.sentido = sentido;
            g.fase = 0;
            puedePasar = false;
        }

        g.fase += sentido ? inc : -inc;
        if (puedePasar && g.fase >= UN_PASO_Q16) {
            g.fase -= UN_PASO_Q16;
            gpio_set_level(g.pines.step, 1);
            g.stepAlto = true;
        }
    }
    return false;
}

// Fija la velocidad de un motor a partir de la del eje de la plataforma [rad/s]
static void fijarVelocidad(GeneradorPasos &g, float omega)
{
    float pasosS = std::clamp(g.signo * omega * g.reduccion * PASOS_POR_VUELTA / DOS_PI, -MAX_PASOS_S, MAX_PASOS_S);
    g.incremento = static_cast<int32_t>(pasosS / FRECUENCIA_TICK_HZ * UN_PASO_Q16);
}

static void iniciarGeneradorPasos()
{
    for (GeneradorPasos &g : generadores) {
        iniciarDriver(g.pines);
    }

    gptimer_handle_t timer;
    gptimer_config_t config = {};
    config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    config.direction = GPTIMER_COUNT_UP;
    config.resolution_hz = 1000000;
    ESP_ERROR_CHECK(gptimer_new_timer(&config, &timer));

    gptimer_alarm_config_t alarma = {};
    alarma.alarm_count = 1000000 / FRECUENCIA_TICK_HZ;
    alarma.flags.auto_reload_on_alarm = true;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarma));

    gptimer_event_callbacks_t callbacks = {};
    callbacks.on_alarm = tickPasos;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &callbacks, nullptr));
    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));
}

// ---------------- Control PI ----------------
// Por eje, con e = θo - θ medido por la cámara y ξ = ∫e dt:
//     ω = Kp·e + Ki·ξ,  saturada a ±ωmax
// Como los motores son paso a paso, la salida del PI es directamente la velocidad del eje.
constexpr float TS = 1.0f / 50.0f;      // período de control [s], a la par de la cámara (FPS_CAMARA = 50 en interfaz.py)
constexpr float T_PREDICCION = 1.0f;    // sin objetivo sigue a la última velocidad hasta este tiempo [s], luego se detiene

constexpr float GRADOS_A_RAD = DOS_PI / 360.0f;
static const float F_X = (ANCHO_IMG / 2) / tanf(62.2f / 2 * GRADOS_A_RAD);  // focal [px] con el FOV completo
static const float F_Y = (ALTO_IMG / 2) / tanf(48.8f / 2 * GRADOS_A_RAD);

struct ParamEje {
    float Kp;        // ganancia proporcional [1/s]
    float Ki;        // ganancia integral [1/s²]
    float omegaMax;  // saturación de la velocidad del eje [rad/s]
};

// Ganancias iniciales: ajustarlas en la plataforma real
constexpr ParamEje PARAM_PAN = {2.0f, 0.5f, 3.0f};
constexpr ParamEje PARAM_TILT = {2.0f, 0.5f, 3.0f};

struct ControlEje {
    ParamEje p;
    float theta;     // ángulo de la plataforma desde el arranque [rad]
    float xi;        // integral del error [rad·s]
    float tPerdido;  // tiempo sin ver el objetivo [s]
};

static ControlEje controlPan = {PARAM_PAN, 0, 0, 0};
static ControlEje controlTilt = {PARAM_TILT, 0, 0, 0};

static TaskHandle_t tareaControlHandle;

// errores_desde_pixeles() del script: e [rad], pan + a la izquierda, tilt + hacia arriba
static void erroresDesdePixeles(int16_t ex, int16_t ey, float thetaTilt, float &ePan, float &eTilt)
{
    ePan = -atanf(ex / F_X);
    eTilt = -atanf(ey / F_Y);
}

// Un paso del PI. Devuelve la velocidad [rad/s] que debe tener la plataforma hasta el próximo paso.
static float pasoControl(ControlEje &c, float e, bool visible)
{
    const ParamEje &p = c.p;

    if (visible) {
        c.xi += TS * e;  // el integrador solo acumula con el objetivo visible
        c.tPerdido = 0.0f;
    } else {
        e = 0.0f;  // sin medición: el integrador mantiene la última velocidad de seguimiento
        c.tPerdido += TS;
    }

    float omega = 0.0f;
    if (visible || c.tPerdido <= T_PREDICCION) {
        omega = p.Kp * e + p.Ki * c.xi;
    } else {
        c.xi = 0.0f;  // objetivo perdido: se detiene y se olvida la integral
    }

    float omegaSat = std::clamp(omega, -p.omegaMax, p.omegaMax);
    if (omegaSat != omega && visible) {
        c.xi -= TS * e;  // anti-windup: si satura, se deshace la integración de este paso
    }

    c.theta += omegaSat * TS;
    return omegaSat;
}

static void tareaControl(void *)
{
    while (true) {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // la despierta el temporizador cada TS

        ErrorObjetivo medicion;
        bool visible = leerError(medicion);

        float ePan = 0.0f;
        float eTilt = 0.0f;
        if (visible) {
            erroresDesdePixeles(medicion.ex, medicion.ey, controlTilt.theta, ePan, eTilt);
        }

        fijarVelocidad(generadores[PAN], pasoControl(controlPan, ePan, visible));
        fijarVelocidad(generadores[TILT], pasoControl(controlTilt, eTilt, visible));
    }
}

static void iniciarControl()
{
    iniciarGeneradorPasos();
    xTaskCreate(tareaControl, "control", 4096, nullptr, 8, &tareaControlHandle);

    esp_timer_handle_t timer;
    esp_timer_create_args_t args = {};
    args.callback = [](void *) { xTaskNotifyGive(tareaControlHandle); };
    args.name = "control";
    ESP_ERROR_CHECK(esp_timer_create(&args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, static_cast<uint64_t>(TS * 1e6f)));
}

// ---------------- Programa ----------------
extern "C" void app_main(void)
{
    iniciarUartPi();
    // iniciarControl();

    ESP_LOGI(TAG, "Receptor UART iniciado (RX = GPIO%d, %d baudios)", PIN_RX, BAUD_RATE);

    // Impresión de depuración a 10 Hz
    uint32_t paquetesAntes = 0;
    int64_t tAntes = esp_timer_get_time();

    while (true) {

        vTaskDelay(pdMS_TO_TICKS(PERIODO_IMPRESION_MS));

        ErrorObjetivo e;
        bool visible = leerError(e);

        taskENTER_CRITICAL(&mux);
        uint32_t ok = paquetesOk;
        uint32_t descartados = paquetesDescartados;
        taskEXIT_CRITICAL(&mux);

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
        printf(" | pan: %.1f° tilt: %.1f°\n", controlPan.theta / GRADOS_A_RAD, controlTilt.theta / GRADOS_A_RAD);
    }
}
