#include "camara.h"

#include <math.h>

#define GRADOS_A_RAD (6.28318531f / 360.0f)

void camaraIniciar(Camara *c, const ConfigCamara *config)
{
    c->fx = (config->ancho / 2) / tanf(config->fovH / 2 * GRADOS_A_RAD);
    c->fy = (config->alto / 2) / tanf(config->fovV / 2 * GRADOS_A_RAD);
}

void camaraErrorAngular(const Camara *c, int16_t ex, int16_t ey, float *ePan, float *eTilt)
{
    *ePan = -atanf(ex / c->fx);
    *eTilt = -atanf(ey / c->fy);
}
