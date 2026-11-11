/*
 * Filtro de Kalman para estimar la posición angular del objetivo en un eje (pan o tilt).
 *
 * Estado: x = [θ, ω]ᵀ, la dirección del objetivo respecto del gimbal [rad] y su velocidad [rad/s].
 * Modelo de velocidad constante, perturbado por una aceleración aleatoria a (ruido blanco):
 *     θ[k+1] = θ[k] + T·ω[k] + (T²/2)·a
 *     ω[k+1] = ω[k] + T·a
 * Medición: z = θ + v, el ángulo que entrega camaraDireccionObjetivo (αobj o βobj).
 *
 * En cada paso el filtro predice con el modelo y, si hay una medición nueva, la corrige.
 * Sin medición solo predice: la estimación sigue a la última velocidad estimada y su
 * incertidumbre crece, lo que sirve para seguir al objetivo mientras la cámara no lo ve.
 *
 * Uso (una instancia por eje):
 *     static Kalman kalman;
 *     kalmanIniciar(&kalman, &config);
 *     float theta = kalmanPaso(&kalman, alfaObj, hayMedicion);   // una vez cada config.ts
 *     // kalman.omega es la velocidad estimada del objetivo
 */

#pragma once

#include <stdbool.h>

typedef struct {
    float ts;                // período con que se llama a kalmanPaso [s]
    float sigmaMedicion;     // desviación estándar del ruido de la medición [rad]
    float sigmaAceleracion;  // aceleración típica del objetivo [rad/s²]: mayor = sigue más rápido, filtra menos
    float sigmaOmegaInicial; // incertidumbre de la velocidad al recibir la primera medición [rad/s]
} ConfigKalman;

typedef struct {
    ConfigKalman config;
    bool iniciado;  // false hasta que llega la primera medición
    float theta;    // posición estimada del objetivo [rad]
    float omega;    // velocidad estimada del objetivo [rad/s]

    // Covarianza del error de estimación, P = [p00 p01; p01 p11]
    float p00;  // varianza de θ [rad²]
    float p01;
    float p11;  // varianza de ω [rad²/s²]
} Kalman;

void kalmanIniciar(Kalman *k, const ConfigKalman *config);

// Olvida la estimación: el filtro parte de nuevo con la siguiente medición.
// Para cuando el objetivo se perdió por mucho tiempo o se va a seguir a otro.
void kalmanReiniciar(Kalman *k);

// Un paso del filtro. `medicion` [rad] solo se usa si hayMedicion es true; debe ser una
// medición nueva (no repetir la del paso anterior). Devuelve la posición estimada [rad].
// Antes de la primera medición devuelve 0 y k->iniciado es false.
float kalmanPaso(Kalman *k, float medicion, bool hayMedicion);
