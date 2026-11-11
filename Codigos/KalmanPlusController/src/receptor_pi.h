/*
 * Receptor UART del error de seguimiento que envía la Raspberry Pi (Codigos/Raspberry/interfaz.py)
 *
 * Paquete de 6 bytes:
 *     0xAA | etapa (0x01..0x04) | ex (int16) | ey (int16)   — little-endian, con signo, en [px]
 * Convención: ex + hacia la derecha, ey + hacia abajo.
 *
 * El segundo byte de la cabecera es la etapa del programa, que decide la Raspberry:
 *     Detenido, Calibrar y Búsqueda: un paquete por frame con ex = ey = 0 (no hay objetivo).
 *     Seguimiento: un paquete por cada frame con objetivo detectado. Si no hay objetivo la
 *     Raspberry no envía nada, por eso se detecta su pérdida por timeout.
 *
 * Conexión (solo hace falta Raspberry -> ESP32, ambos a 3.3 V):
 *     Raspberry GPIO14 / TX (pin 8)  ->  pinRx del ESP32
 *     Raspberry GND (pin 6)          ->  GND
 *
 * Uso:
 *     static ReceptorPi receptor;
 *     receptorPiIniciar(&receptor, &config);   // crea la tarea que lee la UART
 *     ErrorObjetivo e;
 *     if (receptorPiLeer(&receptor, &e)) { ... }  // desde cualquier tarea
 *     EtapaPrograma etapa = receptorPiEtapa(&receptor);
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Etapas del programa: deben coincidir con las de interfaz.py
typedef enum {
    ETAPA_DETENIDO = 1,
    ETAPA_CALIBRAR = 2,
    ETAPA_BUSQUEDA = 3,
    ETAPA_SEGUIMIENTO = 4,
} EtapaPrograma;

typedef struct {
    int16_t ex;           // [px] + derecha
    int16_t ey;           // [px] + abajo
    int64_t t_us;         // esp_timer_get_time() al recibirlo
    EtapaPrograma etapa;  // etapa que venía en la cabecera del paquete
} ErrorObjetivo;

typedef struct {
    uart_port_t puerto;
    int pinRx;
    int pinTx;          // no se usa: la Raspberry solo transmite
    int baudios;        // debe coincidir con UART_BAUDIOS de interfaz.py
    int16_t anchoImg;   // resolución de la cámara: un error fuera de la imagen se descarta
    int16_t altoImg;
    int64_t timeoutUs;  // sin paquetes durante este tiempo el objetivo se da por perdido
} ConfigReceptorPi;

typedef struct {
    ConfigReceptorPi config;

    // Estado interno: no tocar desde fuera del módulo
    QueueHandle_t colaUart;
    portMUX_TYPE mux;
    int estadoRx;
    uint8_t etapaRx;  // etapa del paquete que se está recibiendo
    uint8_t datos[4];
    uint8_t nDatos;
    ErrorObjetivo ultimo;
    bool hayDatos;
    uint32_t paquetesOk;
    uint32_t paquetesDescartados;
} ReceptorPi;

// Instala el driver de la UART y crea la tarea de recepción
void receptorPiIniciar(ReceptorPi *r, const ConfigReceptorPi *config);

// Copia el último paquete recibido. Devuelve true si el objetivo sigue visible: el paquete
// es de Seguimiento y llegó hace menos de timeoutUs. Se puede llamar desde cualquier tarea.
bool receptorPiLeer(ReceptorPi *r, ErrorObjetivo *salida);

// Última etapa informada por la Raspberry (ETAPA_DETENIDO si todavía no llegó ningún paquete).
// No caduca: en Seguimiento sin objetivo no llegan paquetes y la etapa sigue siendo la misma.
EtapaPrograma receptorPiEtapa(ReceptorPi *r);

// Paquetes válidos y descartados desde el arranque
void receptorPiEstadisticas(ReceptorPi *r, uint32_t *ok, uint32_t *descartados);
