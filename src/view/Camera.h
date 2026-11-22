// Camera.h - viewing pipeline and projections (Unit IV).
//   Orbit   : perspective projection (gluPerspective) + gluLookAt on a sphere
//   Top     : orthographic (parallel) projection (glOrtho) looking straight down
//   Gripper : perspective from the end effector ("operator eye" / first person)
#pragma once
#include "../math/Vec3.h"

enum CameraMode { CAM_ORBIT = 0, CAM_TOP, CAM_GRIPPER, CAM_COUNT };

struct Camera {
    int   mode = CAM_ORBIT;
    float yaw = 40.0f, pitch = 26.0f, dist = 13.5f;   // orbit (degrees / units)
    Vec3  center{0.5f, 0.6f, 0.0f};
    float fovy = 45.0f;
    float nearP = 0.1f, farP = 120.0f;
    float topHalf = 7.0f;                              // half height of the ortho window

    // Load GL_PROJECTION for the given viewport aspect ratio.
    void applyProjection(float aspect) const;
    // Load GL_MODELVIEW; eyeShift moves the eye sideways (stereo rendering).
    void applyView(float eyeShift = 0.0f) const;
    Vec3 eye() const;
};

const char* cameraName(int mode);
