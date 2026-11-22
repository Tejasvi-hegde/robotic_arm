// Force.h - contact force computation, smoothing and mapping (Unit V).
//   F = k * d               (spring "virtual wall", d = penetration into the table)
//   Fs <- a*F + (1-a)*Fs    (exponential smoothing so the bar does not jitter)
#pragma once

struct ForceState {
    float k = 400.0f;        // stiffness (N per world unit)
    float alpha = 0.2f;      // smoothing factor
    float maxPenetration = 0.12f;
    float penetration = 0;
    float raw = 0;
    float smooth = 0;
    float maxForce() const { return k * maxPenetration; }
};

// tipY = fingertip height, surfaceY = table top height.
void updateForce(ForceState& f, float tipY, float surfaceY);
// Map force to a colour (green -> yellow -> red) for the haptic stand-in bar.
void forceColour(const ForceState& f, float rgb[3]);
