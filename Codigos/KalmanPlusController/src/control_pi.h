/*
 * Control PI de velocidad para un eje.
 *
 * Con e = θo - θ medido por la cámara y ξ = ∫e dt:
 *     ω = Kp·e + Ki·ξ,  saturada a ±omegaMax
 * Como los motores son paso a paso, la salida del PI es directamente la velocidad del eje.
 *
 * Uso (una instancia por eje):
 *     static ControlPi control;
 *     controlPiIniciar(&control, &config);
 *     float omega = controlPiPaso(&control, e, visible);   // una vez cada config.ts
 */

#pragma once

#include <stdbool.h>

typedef struct {
    float Kp;           // ganancia proporcional [1/s]
    float Ki;           // ganancia integral [1/s²]
    float omegaMax;     // saturación de la velocidad del eje [rad/s]
    float ts;           // período con que se llama a controlPiPaso [s]
    float tPrediccion;  // sin objetivo sigue a la última velocidad hasta este tiempo [s], luego se detiene
} ConfigControlPi;

typedef struct {
    ConfigControlPi config;
    float theta;     // ángulo comandado desde el arranque [rad]
    float xi;        // integral del error [rad·s]
    float tPerdido;  // tiempo sin ver el objetivo [s]
} ControlPi;

void controlPiIniciar(ControlPi *c, const ConfigControlPi *config);

// Cambia las ganancias y la saturación con el control andando
void controlPiAjustar(ControlPi *c, float Kp, float Ki, float omegaMax);

// Olvida la integral del error. Para cuando el PI deja de usarse (otra etapa del programa).
void controlPiReiniciar(ControlPi *c);

// Un paso del PI. Devuelve la velocidad [rad/s] que debe tener el eje hasta el próximo paso.
float controlPiPaso(ControlPi *c, float e, bool visible);
