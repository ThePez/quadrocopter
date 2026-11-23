/*
 *****************************************************************************
 * File: attitude3d.h
 * Author: Claude (Anthropic)
 * Date: 20-11-2026
 * Brief: Interface for the 3D drone view.
 *****************************************************************************
 */

/*
 * 3D drone model driven by pitch/roll/yaw, rendered into an offscreen texture
 * so it can sit inside any rectangle of the window.
 */
#ifndef ATTITUDE3D_H
#define ATTITUDE3D_H

#include "raylib.h"

typedef struct {
    RenderTexture2D target;
    int w, h;
    float azimuth, elevation, distance; /* orbit camera, degrees / scene units */
    float pitch, roll, yaw;             /* degrees */
} Attitude3D;

void att_init(Attitude3D *a);
void att_free(Attitude3D *a);
void att_set(Attitude3D *a, float pitch, float roll, float yaw);
/* Draws a titled card with the 3D view inside it. */
void att_draw(Attitude3D *a, Rectangle card, const char *title);

#endif
