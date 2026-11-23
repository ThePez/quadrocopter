/*
 *****************************************************************************
 * File: attitude3d.c
 * Author: Claude (Anthropic)
 * Date: 20-11-2026
 * Brief: 3D drone model driven by pitch, roll and yaw.
 *****************************************************************************
 */

/*
 * The 3D drone view. The model is drawn into an offscreen RenderTexture each frame and that
 * texture is then drawn inside the card, so the 3D scene is clipped to the card and can sit
 * anywhere in the 2D layout. Orientation comes from the drone's pitch / roll / yaw.
 */

#include "attitude3d.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ui.h"

/*
 * Scene axes (same as the Python app): X = right, Y = forward (nose), Z = up.
 * raylib is Y-up with -Z forward, so scene (x, y, z) is drawn at (x, z, -y).
 */
typedef struct {
    float x, y, z;
} V3;

static Vector3 to_rl(V3 v)
{
    return (Vector3){v.x, v.z, -v.y};
}

/* Roll about Y, then pitch about X, then yaw about Z (aerospace ZYX). */
static V3 rotate(V3 p, float pitch, float roll, float yaw)
{
    float cr = cosf(roll * DEG2RAD), sr = sinf(roll * DEG2RAD);
    /* Positive pitch is front DOWN (the firmware's convention), so the nose goes down. */
    float cp = cosf(pitch * DEG2RAD), sp = -sinf(pitch * DEG2RAD);
    float cy = cosf(yaw * DEG2RAD), sy = sinf(yaw * DEG2RAD);

    V3 r = {p.x * cr + p.z * sr, p.y, -p.x * sr + p.z * cr};
    V3 q = {r.x, r.y * cp - r.z * sp, r.y * sp + r.z * cp};
    return (V3){q.x * cy - q.y * sy, q.x * sy + q.y * cy, q.z};
}

void att_init(Attitude3D *a)
{
    memset(a, 0, sizeof(*a));
    a->azimuth = 35;
    a->elevation = 28;
    a->distance = 5.5f;
}

void att_free(Attitude3D *a)
{
    if (a->target.id != 0) {
        UnloadRenderTexture(a->target);
    }

    a->target.id = 0;
}

void att_set(Attitude3D *a, float pitch, float roll, float yaw)
{
    a->pitch = pitch;
    a->roll = roll;
    a->yaw = yaw;
}

static void draw_scene(const Attitude3D *a)
{
    float az = a->azimuth * DEG2RAD, el = a->elevation * DEG2RAD;
    V3 eye = {a->distance * cosf(el) * sinf(az), -a->distance * cosf(el) * cosf(az),
              a->distance * sinf(el)};
    Camera3D cam = {
        .position = to_rl(eye),
        .target = (Vector3){0, 0, 0},
        .up = (Vector3){0, 1, 0},
        .fovy = 45.0f,
        .projection = CAMERA_PERSPECTIVE,
    };

    BeginMode3D(cam);

    /* Ground reference grid */
    for (int i = -5; i <= 5; i++) {
        Color c = i == 0 ? (Color){72, 82, 104, 255} : (Color){38, 44, 58, 255};
        DrawLine3D((Vector3){(float)i, -1.5f, -5}, (Vector3){(float)i, -1.5f, 5}, c);
        DrawLine3D((Vector3){-5, -1.5f, (float)i}, (Vector3){5, -1.5f, (float)i}, c);
    }

    const float s = 0.7071f; /* arms at 45 degrees */
    V3 fl = {-s, s, 0}, fr = {s, s, 0}, bl = {-s, -s, 0}, br = {s, -s, 0};
    V3 *tips[4] = {&fl, &bl, &br, &fr};
    for (int i = 0; i < 4; i++) {
        *tips[i] = rotate(*tips[i], a->pitch, a->roll, a->yaw);
    }

    Vector3 origin = to_rl(rotate((V3){0, 0, 0}, a->pitch, a->roll, a->yaw));
    Color rear = {150, 157, 172, 255};
    DrawCylinderEx(origin, to_rl(fl), 0.055f, 0.055f, 10, UI_SERIES_RED);
    DrawCylinderEx(origin, to_rl(fr), 0.055f, 0.055f, 10, UI_SERIES_RED);
    DrawCylinderEx(origin, to_rl(bl), 0.055f, 0.055f, 10, rear);
    DrawCylinderEx(origin, to_rl(br), 0.055f, 0.055f, 10, rear);

    /* Motor markers use the same colours as the Motors plot (tips are FL, BL, BR, FR). */
    const Color motor_colors[4] = {UI_SERIES_RED, UI_SERIES_CYAN, UI_SERIES_ORANGE,
                                   UI_SERIES_PURPLE};
    for (int i = 0; i < 4; i++) {
        DrawSphere(to_rl(*tips[i]), 0.14f, motor_colors[i]);
    }

    /* Nose marker so heading reads clearly while yawing */
    V3 nose = rotate((V3){0, 0.95f, 0}, a->pitch, a->roll, a->yaw);
    DrawSphere(to_rl(nose), 0.07f, UI_SERIES_GREEN);
    DrawCube(origin, 0.3f, 0.12f, 0.3f, (Color){210, 216, 228, 255});

    EndMode3D();
}

void att_draw(Attitude3D *a, Rectangle card, const char *title)
{
    ui_card(card, title);
    Rectangle b = ui_card_body(card, true);
    b.height += UI_PAD - 4; /* let the view run closer to the bottom edge */
    int w = (int)b.width, h = (int)b.height;
    if (w < 16 || h < 16) {
        return;
    }

    if (a->target.id == 0 || a->w != w || a->h != h) {
        if (a->target.id != 0) {
            UnloadRenderTexture(a->target);
        }

        a->target = LoadRenderTexture(w, h);
        SetTextureFilter(a->target.texture, TEXTURE_FILTER_BILINEAR);
        a->w = w;
        a->h = h;
    }

    /* Orbit with left-drag, zoom with the wheel. */
    Vector2 m = GetMousePosition();
    if (!GuiIsLocked() && CheckCollisionPointRec(m, b)) {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 d = GetMouseDelta();
            a->azimuth -= d.x * 0.4f;
            a->elevation += d.y * 0.4f;
            if (a->elevation > 89) {
                a->elevation = 89;
            }

            if (a->elevation < -89) {
                a->elevation = -89;
            }
        }

        a->distance -= GetMouseWheelMove() * 0.4f;
        if (a->distance < 2) {
            a->distance = 2;
        }

        if (a->distance > 20) {
            a->distance = 20;
        }
    }

    BeginTextureMode(a->target);
    ClearBackground(UI_FIELD);
    draw_scene(a);
    EndTextureMode();

    /* Render textures are stored upside down. */
    DrawTextureRec(a->target.texture, (Rectangle){0, 0, (float)w, -(float)h}, (Vector2){b.x, b.y},
                   WHITE);
    DrawRectangleLinesEx(b, 1, UI_BORDER);

    char info[96];
    snprintf(info, sizeof(info), "P %.1f   R %.1f   Y %.1f", (double)a->pitch, (double)a->roll,
             (double)a->yaw);
    ui_text(info, b.x + 10, b.y + 8, UI_SMALL + 1, UI_TEXT);
    ui_text("drag to orbit", b.x + 10, b.y + b.height - 22, UI_SMALL - 1, UI_MUTED);
}
