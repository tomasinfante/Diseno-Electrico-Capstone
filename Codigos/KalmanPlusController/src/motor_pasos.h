/*
 * Motor paso a paso (NEMA 17) con driver DRV8825, comandado en velocidad.
 *
 * Un temporizador común a 20 kHz reparte los pasos de todos los motores con un acumulador
 * de fase por motor, así la velocidad se puede cambiar en cualquier momento sin bloquear
 * a quien la fija. El temporizador arranca al iniciar el primer motor.
 *
 * Uso (una instancia por motor):
 *     static MotorPasos motor;
 *     motorPasosIniciar(&motor, &config);         // configura los pines y habilita el driver
 *     motorPasosFijarVelocidad(&motor, omega);    // [rad/s] del eje de la plataforma
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"

typedef struct {
    gpio_num_t en, step, dir, m0, m1, m2;
} PinesDriver;

typedef struct {
    PinesDriver pines;
    int micropasos;   // 1 (paso completo), 2, 4, 8, 16 o 32: fija M0, M1 y M2
    float reduccion;  // vueltas del motor por vuelta del eje (p. ej. 114/20 con correa y polea de 20 dientes)
    float signo;      // -1 si el eje gira al revés de la convención (pan + izquierda, tilt + arriba)
} ConfigMotorPasos;

typedef struct {
    ConfigMotorPasos config;
    float pasosPorRad;  // pasos por radián del eje, con micropasos y reducción

    // Estado interno, lo usa la interrupción del temporizador: no tocar desde fuera del módulo
    volatile int32_t incremento;  // pasos por tick en Q16, con signo
    uint32_t fase;
    bool stepAlto;
    bool sentido;  // nivel actual de DIR
} MotorPasos;

// Configura los pines, habilita el driver y agrega el motor al temporizador común
void motorPasosIniciar(MotorPasos *m, const ConfigMotorPasos *config);

// Fija la velocidad del eje de la plataforma [rad/s]. Se puede llamar desde cualquier tarea.
void motorPasosFijarVelocidad(MotorPasos *m, float omega);
