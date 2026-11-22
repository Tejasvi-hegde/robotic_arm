// Force.cpp
#include "Force.h"
#include "../math/Vec3.h"

void updateForce(ForceState& f, float tipY, float surfaceY) {
    f.penetration = tipY < surfaceY ? (surfaceY - tipY) : 0.0f;
    f.raw = f.k * f.penetration;
    f.smooth = f.alpha * f.raw + (1.0f - f.alpha) * f.smooth;
    if (f.smooth < 0.01f) f.smooth = 0;
}

void forceColour(const ForceState& f, float rgb[3]) {
    float u = clampf(f.smooth / f.maxForce(), 0.0f, 1.0f);
    if (u < 0.5f) { rgb[0] = u * 2; rgb[1] = 0.85f; rgb[2] = 0.1f; }
    else          { rgb[0] = 1.0f; rgb[1] = 0.85f * (1 - (u - 0.5f) * 2); rgb[2] = 0.1f; }
}
