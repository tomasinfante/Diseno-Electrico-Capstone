/*
 * Encoder magnético AS5600 en el eje de un motor, por I2C.
 *
 * Todos los AS5600 tienen la dirección fija 0x36, así que cada uno va en su propio bus I2C
 * (el ESP32 tiene dos: I2C_NUM_0 e I2C_NUM_1). Una tarea lee el ángulo cada 10 ms y cuenta las
 * vueltas, así el ángulo sigue más allá de 360°. El cero es la posición de arranque.
 * El sensor mide el eje del motor: el ángulo del eje de la plataforma es ese dividido por la reducción.
 *
 * Uso (una instancia por encoder):
 *     static EncoderAs5600 encoder;
 *     encoderAs5600Iniciar(&encoder, &config);   // si el sensor no responde, queda sin encoder
 *     float angulo;
 *     if (encoderAs5600Leer(&encoder, &angulo)) { ... }   // desde cualquier tarea
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"

typedef struct {
    i2c_port_num_t puerto;  // I2C_NUM_0 o I2C_NUM_1, uno distinto por encoder
    gpio_num_t sda;
    gpio_num_t scl;
    float reduccion;        // vueltas del motor por vuelta del eje (la misma que la del motor)
    float signo;            // -1 si el ángulo crece al revés que el del eje
} ConfigEncoderAs5600;

typedef struct {
    ConfigEncoderAs5600 config;
    bool presente;  // false si el sensor no respondió al iniciar

    // Estado interno: no tocar desde fuera del módulo
    i2c_master_dev_handle_t dispositivo;
    portMUX_TYPE mux;
    uint16_t rawAnterior;
    int32_t cuentas;   // posición del motor desde el arranque, con vueltas
    uint32_t errores;  // lecturas fallidas
} EncoderAs5600;

// Crea el bus I2C, revisa el sensor y el imán, fija el cero y crea la tarea de lectura
void encoderAs5600Iniciar(EncoderAs5600 *enc, const ConfigEncoderAs5600 *config);

// Ángulo del eje de la plataforma desde el arranque [rad]. Devuelve false si el sensor no está.
// Se puede llamar desde cualquier tarea.
bool encoderAs5600Leer(EncoderAs5600 *enc, float *angulo);

// Lecturas fallidas desde el arranque
uint32_t encoderAs5600Errores(EncoderAs5600 *enc);
