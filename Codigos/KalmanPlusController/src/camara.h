/*
 * Modelo de la cámara: convierte el error en píxeles a error angular.
 *
 * Uso:
 *     static Camara camara;
 *     camaraIniciar(&camara, &config);
 *     camaraErrorAngular(&camara, ex, ey, &ePan, &eTilt);
 */

#pragma once

#include <stdint.h>

typedef struct {
    int ancho;    // resolución de la imagen [px]
    int alto;
    float fovH;   // campo de visión horizontal y vertical [°]
    float fovV;
} ConfigCamara;

typedef struct {
    float fx;  // focales [px], calculadas con el campo de visión
    float fy;
} Camara;

void camaraIniciar(Camara *c, const ConfigCamara *config);

// Error del objetivo respecto al centro [rad]: pan + a la izquierda, tilt + hacia arriba
void camaraErrorAngular(const Camara *c, int16_t ex, int16_t ey, float *ePan, float *eTilt);
