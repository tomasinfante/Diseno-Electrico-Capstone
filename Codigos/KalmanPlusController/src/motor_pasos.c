#include "motor_pasos.h"

#include <math.h>
#include <stdlib.h>

#include "driver/gptimer.h"
#include "esp_log.h"

#define PASOS_COMPLETOS_POR_VUELTA 200  // NEMA 17: 1.8° por paso
#define FRECUENCIA_TICK_HZ 20000
#define MAX_PASOS_S (FRECUENCIA_TICK_HZ / 2)  // cada pulso ocupa 2 ticks (alto y bajo): no se puede pedir más
#define UN_PASO_Q16 (1 << 16)
#define DOS_PI 6.28318531f
#define MAX_MOTORES 4

static const char *TAG = "motor_pasos";

// Motores que atiende la interrupción. Primero se guarda el puntero y después se incrementa
// nMotores, así la interrupción nunca ve un motor a medio agregar.
static MotorPasos *motores[MAX_MOTORES];
static volatile int nMotores = 0;

static bool tickPasos(gptimer_handle_t timer, const gptimer_alarm_event_data_t *datos, void *ctx)
{
    for (int i = 0; i < nMotores; i++) {
        MotorPasos *m = motores[i];

        // La fase se acumula en todos los ticks; solo el pulso espera cuando no puede salir en este
        bool puedePasar = true;

        if (m->stepAlto) {  // segundo tick del pulso
            gpio_set_level(m->config.pines.step, 0);
            m->stepAlto = false;
            puedePasar = false;
        }

        int32_t inc = m->incremento;
        bool sentido = inc >= 0;
        if (sentido != m->sentido) {  // DIR se cambia al menos un tick antes del próximo paso (el DRV8825 pide 650 ns)
            gpio_set_level(m->config.pines.dir, sentido);
            m->sentido = sentido;
            m->fase = 0;
            puedePasar = false;
        }

        m->fase += sentido ? inc : -inc;
        if (puedePasar && m->fase >= UN_PASO_Q16) {
            m->fase -= UN_PASO_Q16;
            gpio_set_level(m->config.pines.step, 1);
            m->stepAlto = true;
        }
    }
    return false;
}

static void iniciarTemporizador(void)
{
    gptimer_handle_t timer;
    gptimer_config_t config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&config, &timer));

    gptimer_alarm_config_t alarma = {
        .alarm_count = 1000000 / FRECUENCIA_TICK_HZ,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarma));

    gptimer_event_callbacks_t callbacks = {
        .on_alarm = tickPasos,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &callbacks, NULL));
    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));
}

// Niveles de M2 M1 M0 del DRV8825: 000 paso completo, 001 1/2, 010 1/4, 011 1/8, 100 1/16, 101 1/32.
// Coinciden con log2(micropasos).
static int modoMicropasos(int micropasos)
{
    for (int modo = 0; modo <= 5; modo++) {
        if ((1 << modo) == micropasos) {
            return modo;
        }
    }
    ESP_LOGE(TAG, "micropasos = %d no es válido (1, 2, 4, 8, 16 o 32)", micropasos);
    abort();
}

void motorPasosIniciar(MotorPasos *m, const ConfigMotorPasos *config)
{
    if (nMotores >= MAX_MOTORES) {
        ESP_LOGE(TAG, "No caben más de %d motores", MAX_MOTORES);
        abort();
    }

    m->config = *config;
    m->pasosPorRad = PASOS_COMPLETOS_POR_VUELTA * config->micropasos * config->reduccion / DOS_PI;
    m->incremento = 0;
    m->fase = 0;
    m->stepAlto = false;
    m->sentido = false;

    const PinesDriver *p = &config->pines;
    gpio_config_t pines = {
        .pin_bit_mask = (1ULL << p->en) | (1ULL << p->step) | (1ULL << p->dir) |
                        (1ULL << p->m0) | (1ULL << p->m1) | (1ULL << p->m2),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&pines));

    int modo = modoMicropasos(config->micropasos);
    gpio_set_level(p->m0, modo & 1);
    gpio_set_level(p->m1, (modo >> 1) & 1);
    gpio_set_level(p->m2, (modo >> 2) & 1);
    gpio_set_level(p->step, 0);
    gpio_set_level(p->dir, 0);
    gpio_set_level(p->en, 0);  // EN es activo en bajo: 0 habilita el driver

    if (nMotores == 0) {
        iniciarTemporizador();
    }
    motores[nMotores] = m;
    nMotores++;
}

void motorPasosFijarVelocidad(MotorPasos *m, float omega)
{
    float pasosS = fminf(fmaxf(m->config.signo * omega * m->pasosPorRad, -MAX_PASOS_S), MAX_PASOS_S);
    m->incremento = (int32_t)(pasosS / FRECUENCIA_TICK_HZ * UN_PASO_Q16);
}

void motorPasosHabilitar(MotorPasos *m, bool habilitar)
{
    m->incremento = 0;
    gpio_set_level(m->config.pines.en, habilitar ? 0 : 1);  // EN es activo en bajo
}
