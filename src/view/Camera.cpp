// Camera.cpp
#include "Camera.h"
#include "../GLInclude.h"
#include "../App.h"
#include <cmath>

const char* cameraName(int mode) {
    switch (mode) {
        case CAM_ORBIT: return "Orbit (perspective)";
        case CAM_TOP: return "Top (orthographic)";
        case CAM_GRIPPER: return "Gripper cam (first person)";
    }
    return "?";
}

static void orbitEye(const Camera& c, Vec3& eye) {
    float y = deg2rad(c.yaw), p = deg2rad(c.pitch);
    eye = c.center + Vec3(c.dist * std::cos(p) * std::cos(y),
                          c.dist * std::sin(p),
                          c.dist * std::cos(p) * std::sin(y));
}

static void gripperEye(Vec3& eye, Vec3& look, Vec3& up) {
    const Mat4& f = g.pose.frame[4];
    Vec3 axis = f.column(0).normalized();     // gripper points along its local +X
    up = f.column(1).normalized();
    // wrist-mounted camera: above the palm, looking past the fingers
    eye = f.translation() - axis * 0.55f + up * 0.38f;
    look = f.translation() + axis * 1.2f;
}

Vec3 Camera::eye() const {
    Vec3 e, l, u;
    switch (mode) {
        case CAM_TOP: return Vec3(center.x, 25.0f, center.z);
        case CAM_GRIPPER: gripperEye(e, l, u); return e;
        default: orbitEye(*this, e); return e;
    }
}

void Camera::applyProjection(float aspect) const {
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (mode == CAM_TOP) {
        glOrtho(-topHalf * aspect, topHalf * aspect, -topHalf, topHalf, nearP, farP);
    } else if (mode == CAM_GRIPPER) {
        gluPerspective(70.0, aspect, std::fmin(nearP, 0.05f), farP);
    } else {
        gluPerspective(fovy, aspect, nearP, farP);
    }
    glMatrixMode(GL_MODELVIEW);
}

void Camera::applyView(float eyeShift) const {
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    Vec3 e, l, u;
    if (mode == CAM_TOP) {
        e = Vec3(center.x, 25.0f, center.z);
        l = Vec3(center.x, 0, center.z);
        u = Vec3(0, 0, -1);
    } else if (mode == CAM_GRIPPER) {
        gripperEye(e, l, u);
    } else {
        orbitEye(*this, e);
        l = center;
        u = Vec3(0, 1, 0);
    }
    if (eyeShift != 0.0f) {                     // parallel-axis stereo: shift eye and target
        Vec3 fwd = (l - e).normalized();
        Vec3 right = fwd.cross(u).normalized();
        e += right * eyeShift;
        l += right * eyeShift;
    }
    gluLookAt(e.x, e.y, e.z, l.x, l.y, l.z, u.x, u.y, u.z);
}
