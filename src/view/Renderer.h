// Renderer.h - drives the 3D part of every frame (and the colour-ID pick pass).
#pragma once
#include "../math/Vec3.h"

void setupLighting();
// Render the 3D scene into the given viewport. eyeShift != 0 for stereo eyes.
void renderView(int vx, int vy, int vw, int vh, float eyeShift, bool pickPass = false);
// Full frame: 3D view(s) + HUD + mini-map, then swap.
void renderFrame(bool swap = true);

struct PickResult {
    int id = 0;            // 0 nothing, 1..N part index+1, PICK_TABLE, PICK_FLOOR
    bool hasPoint = false;
    Vec3 point;            // world point under the cursor (gluUnProject)
};
// Colour picking: render IDs into the back buffer, read the pixel, never shown.
PickResult pickAt(int mouseX, int mouseY);
