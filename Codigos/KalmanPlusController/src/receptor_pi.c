#include "receptor_pi.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"

#define CABECERA_1 0xAA      // el segundo byte de la cabecera es la etapa o TIPO_PARAMETROS
#define TIPO_PARAMETROS 0x10
#define LARGO_PARAMETROS 11  // 5 valores uint16 + suma de comprobación
#define TAMANO_BUFFER_RX 1024
#define TAMANO_COLA_EVENTOS 20

enum { ESPERA_CAB1, ESPERA_CAB2, LEYENDO_DATOS, LEYENDO_PARAMETROS };

static const char *TAG = "receptor_pi";

static int16_t leerInt16LE(const uint8_t *p)
{
    return (int16_t)(p[0] | (p[1] << 8));
}

static void contarDescartado(ReceptorPi *r)
{
    taskENTER_CRITICAL(&r->mux);
    r->paquetesDescartados++;
    taskEXIT_CRITICAL(&r->mux);
}

// Revisa la suma de comprobación del paquete de parámetros en r->datos y, si está bien, los guarda
static void guardarParametros(ReceptorPi *r)
{
    uint8_t suma = TIPO_PARAMETROS;
    for (int i = 0; i < LARGO_PARAMETROS - 1; i++) {
        suma += r->datos[i];
    }
    if (suma != r->datos[LARGO_PARAMETROS - 1]) {
        contarDescartado(r);
        return;
    }

    ParametrosControl p = {
        .omegaMax = (uint16_t)leerInt16LE(&r->datos[0]) / 1000.0f,
        .kpPan = (uint16_t)leerInt16LE(&r->datos[2]) / 1000.0f,
        .kiPan = (uint16_t)leerInt16LE(&r->datos[4]) / 1000.0f,
        .kpTilt = (uint16_t)leerInt16LE(&r->datos[6]) / 1000.0f,
        .kiTilt = (uint16_t)leerInt16LE(&r->datos[8]) / 1000.0f,
    };

    taskENTER_CRITICAL(&r->mux);
    r->parametros = p;
    r->hayParametros = true;
    taskEXIT_CRITICAL(&r->mux);
}

// Máquina de estados: procesa un byte y devuelve true al completar un paquete de error válido
static bool procesarByte(ReceptorPi *r, uint8_t b, ErrorObjetivo *salida)
{
    switch (r->estadoRx) {

        case ESPERA_CAB1:
            if (b == CABECERA_1) {
                r->estadoRx = ESPERA_CAB2;
            }
            return false;

        case ESPERA_CAB2:
            if (b >= ETAPA_DETENIDO && b <= ETAPA_SEGUIMIENTO) {
                r->etapaRx = b;
                r->estadoRx = LEYENDO_DATOS;
                r->nDatos = 0;
            } else if (b == TIPO_PARAMETROS) {
                r->estadoRx = LEYENDO_PARAMETROS;
                r->nDatos = 0;
            } else if (b != CABECERA_1) {  // con 0xAA 0xAA etapa el segundo 0xAA puede ser la cabecera real
                r->estadoRx = ESPERA_CAB1;
            }
            return false;

        case LEYENDO_DATOS: {
            r->datos[r->nDatos++] = b;
            if (r->nDatos < 4) {
                return false;
            }
            r->estadoRx = ESPERA_CAB1;

            int16_t ex = leerInt16LE(&r->datos[0]);
            int16_t ey = leerInt16LE(&r->datos[2]);
            int16_t maxEx = r->config.anchoImg / 2;
            int16_t maxEy = r->config.altoImg / 2;

            // No hay checksum: un error fuera de la imagen indica un paquete corrupto o desalineado
            if (ex < -maxEx || ex > maxEx || ey < -maxEy || ey > maxEy) {
                contarDescartado(r);
                return false;
            }

            salida->ex = ex;
            salida->ey = ey;
            salida->t_us = esp_timer_get_time();
            salida->etapa = (EtapaPrograma)r->etapaRx;
            return true;
        }

        case LEYENDO_PARAMETROS:
            r->datos[r->nDatos++] = b;
            if (r->nDatos == LARGO_PARAMETROS) {
                r->estadoRx = ESPERA_CAB1;
                guardarParametros(r);
            }
            return false;
    }

    return false;
}

// Despierta con cada evento de la UART: el driver avisa ~1 ms después de que termina
// de llegar un paquete (timeout de RX), así que no hace falta leer por encuesta.
static void tareaUartPi(void *arg)
{
    ReceptorPi *r = arg;
    uart_event_t evento;
    uint8_t buf[128];

    while (true) {

        if (xQueueReceive(r->colaUart, &evento, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        switch (evento.type) {

            case UART_DATA: {
                // Se lee todo lo acumulado y se guarda solo el paquete más reciente
                ErrorObjetivo paquete;
                uint32_t nuevos = 0;
                int n;

                while ((n = uart_read_bytes(r->config.puerto, buf, sizeof(buf), 0)) > 0) {
                    for (int i = 0; i < n; i++) {
                        if (procesarByte(r, buf[i], &paquete)) {
                            nuevos++;
                        }
                    }
                }

                if (nuevos > 0) {
                    taskENTER_CRITICAL(&r->mux);
                    r->ultimo = paquete;
                    r->hayDatos = true;
                    r->paquetesOk += nuevos;
                    taskEXIT_CRITICAL(&r->mux);
                }
                break;
            }

            case UART_FIFO_OVF:
            case UART_BUFFER_FULL:
                // Se perdieron bytes: se descarta lo acumulado y se vuelve a buscar la cabecera
                ESP_LOGW(TAG, "Desborde en la UART, se vacía el buffer");
                uart_flush_input(r->config.puerto);
                xQueueReset(r->colaUart);
                r->estadoRx = ESPERA_CAB1;
                break;

            default:
                break;
        }
    }
}

void receptorPiIniciar(ReceptorPi *r, const ConfigReceptorPi *config)
{
    r->config = *config;
    portMUX_INITIALIZE(&r->mux);
    r->estadoRx = ESPERA_CAB1;
    r->nDatos = 0;
    r->hayDatos = false;
    r->hayParametros = false;
    r->paquetesOk = 0;
    r->paquetesDescartados = 0;

    uart_config_t uart = {
        .baud_rate = config->baudios,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(config->puerto, TAMANO_BUFFER_RX, 0, TAMANO_COLA_EVENTOS, &r->colaUart, 0));
    ESP_ERROR_CHECK(uart_param_config(config->puerto, &uart));
    ESP_ERROR_CHECK(uart_set_pin(config->puerto, config->pinTx, config->pinRx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(tareaUartPi, "uart_pi", 4096, r, 10, NULL);

    ESP_LOGI(TAG, "Iniciado (RX = GPIO%d, %d baudios)", config->pinRx, config->baudios);
}

bool receptorPiLeer(ReceptorPi *r, ErrorObjetivo *salida)
{
    taskENTER_CRITICAL(&r->mux);
    *salida = r->ultimo;
    bool valido = r->hayDatos;
    taskEXIT_CRITICAL(&r->mux);

    // Fuera de Seguimiento los paquetes solo informan la etapa (ex = ey = 0): no hay objetivo
    return valido && salida->etapa == ETAPA_SEGUIMIENTO &&
           (esp_timer_get_time() - salida->t_us) < r->config.timeoutUs;
}

EtapaPrograma receptorPiEtapa(ReceptorPi *r)
{
    taskENTER_CRITICAL(&r->mux);
    EtapaPrograma etapa = r->hayDatos ? r->ultimo.etapa : ETAPA_DETENIDO;
    taskEXIT_CRITICAL(&r->mux);

    return etapa;
}

bool receptorPiParametros(ReceptorPi *r, ParametrosControl *salida)
{
    taskENTER_CRITICAL(&r->mux);
    bool hay = r->hayParametros;
    if (hay) {
        *salida = r->parametros;
    }
    taskEXIT_CRITICAL(&r->mux);

    return hay;
}

void receptorPiEstadisticas(ReceptorPi *r, uint32_t *ok, uint32_t *descartados)
{
    taskENTER_CRITICAL(&r->mux);
    *ok = r->paquetesOk;
    *descartados = r->paquetesDescartados;
    taskEXIT_CRITICAL(&r->mux);
}
