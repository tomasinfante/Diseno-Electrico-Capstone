/*
 * Modelo de la cámara: convierte la posición del objetivo en la imagen a su dirección
 * respecto del gimbal (ángulos de pan y tilt a los que está el objetivo).
 *
 * Con el modelo pinhole, el objetivo está en la dirección (marco de la cámara)
 *     rc = [ex/fx, ey/fy, 1]ᵀ        con ex = u - cx, ey = v - cy
 * que se lleva al marco del gimbal con los ángulos α (pan) y β (tilt) de la plataforma:
 *     rg = RTot(α, β) · Pᵀ · rc,     RTot = RPan(α) · RTilt(β)
 * y de ahí
 *     αobj = atan2(rg_y, rg_x)
 *     βobj = atan2(rg_z, √(rg_x² + rg_y²))
 *
 * Uso:
 *     static Camara camara;
 *     camaraIniciar(&camara, &config);
 *     camaraDireccionObjetivo(&camara, ex, ey, alfa, beta, &alfaObj, &betaObj);
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

// Dirección del objetivo en el marco del gimbal [rad]: alfaObj es el pan (+ a la izquierda,
// entre -π y π) y betaObj el tilt (+ hacia arriba, entre -π/2 y π/2).
//     ex, ey      error del objetivo respecto al centro de la imagen [px]: + derecha, + abajo
//     alfa, beta  pan y tilt de la plataforma al capturar la imagen [rad]
void camaraDireccionObjetivo(const Camara *c, int16_t ex, int16_t ey, float alfa, float beta,
                             float *alfaObj, float *betaObj);
