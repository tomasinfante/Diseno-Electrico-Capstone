#include "camara.h"

#include <math.h>

#define GRADOS_A_RAD (6.28318531f / 360.0f)

void camaraIniciar(Camara *c, const ConfigCamara *config)
{
    c->fx = (config->ancho / 2) / tanf(config->fovH / 2 * GRADOS_A_RAD);
    c->fy = (config->alto / 2) / tanf(config->fovV / 2 * GRADOS_A_RAD);
}

void camaraDireccionObjetivo(const Camara *c, int16_t ex, int16_t ey, float alfa, float beta,
                             float *alfaObj, float *betaObj)
{
    // Dirección en el marco de la cámara (pinhole invertido): rc = [ex/fx, ey/fy, 1]
    float rcX = ex / c->fx;
    float rcY = ey / c->fy;
    float rcZ = 1.0f;

    // p = Pᵀ·rc: ejes de la cámara (x derecha, y abajo, z al frente) a los del gimbal
    // sin rotar (x al frente, y izquierda, z arriba)
    float pX = rcZ;
    float pY = -rcX;
    float pZ = -rcY;

    // rg = RTot(α, β)·p, con RTot = RPan(α)·RTilt(β)
    float ca = cosf(alfa), sa = sinf(alfa);
    float cb = cosf(beta), sb = sinf(beta);

    float rgX = ca * cb * pX - sa * pY - ca * sb * pZ;
    float rgY = sa * cb * pX + ca * pY - sa * sb * pZ;
    float rgZ = sb * pX + cb * pZ;

    *alfaObj = atan2f(rgY, rgX);
    *betaObj = atan2f(rgZ, sqrtf(rgX * rgX + rgY * rgY));
}
