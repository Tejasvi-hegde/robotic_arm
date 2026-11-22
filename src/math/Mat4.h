// Mat4.h - 4x4 homogeneous transformation matrix (Unit III/IV/V).
//
// Stored COLUMN-MAJOR exactly like OpenGL: m[col*4 + row].
// So a Mat4 can be passed straight to glMultMatrixf / glLoadMatrixf,
// and the composite we build here is the same one glRotatef/glTranslatef
// build on the matrix stack when the arm is drawn.
#pragma once
#include "Vec3.h"
#include <cstring>

struct Mat4 {
    float m[16];

    Mat4() { setIdentity(); }

    void setIdentity() {
        std::memset(m, 0, sizeof(m));
        m[0] = m[5] = m[10] = m[15] = 1.0f;
    }

    float& at(int row, int col) { return m[col * 4 + row]; }
    float at(int row, int col) const { return m[col * 4 + row]; }

    // C = A * B  (apply B first, then A - same order as the GL matrix stack)
    Mat4 operator*(const Mat4& b) const {
        Mat4 c;
        for (int r = 0; r < 4; ++r)
            for (int col = 0; col < 4; ++col) {
                float s = 0;
                for (int k = 0; k < 4; ++k) s += at(r, k) * b.at(k, col);
                c.at(r, col) = s;
            }
        return c;
    }

    // Transform a point (w = 1): includes translation.
    Vec3 transformPoint(const Vec3& p) const {
        return {at(0, 0) * p.x + at(0, 1) * p.y + at(0, 2) * p.z + at(0, 3),
                at(1, 0) * p.x + at(1, 1) * p.y + at(1, 2) * p.z + at(1, 3),
                at(2, 0) * p.x + at(2, 1) * p.y + at(2, 2) * p.z + at(2, 3)};
    }
    // Transform a direction (w = 0): rotation only.
    Vec3 transformDir(const Vec3& d) const {
        return {at(0, 0) * d.x + at(0, 1) * d.y + at(0, 2) * d.z,
                at(1, 0) * d.x + at(1, 1) * d.y + at(1, 2) * d.z,
                at(2, 0) * d.x + at(2, 1) * d.y + at(2, 2) * d.z};
    }
    Vec3 translation() const { return {at(0, 3), at(1, 3), at(2, 3)}; }
    Vec3 column(int c) const { return {at(0, c), at(1, c), at(2, c)}; }

    // ---- Basic transforms (homogeneous form) ----
    static Mat4 translate(float tx, float ty, float tz) {
        Mat4 t;
        t.at(0, 3) = tx; t.at(1, 3) = ty; t.at(2, 3) = tz;
        return t;
    }
    static Mat4 scale(float sx, float sy, float sz) {
        Mat4 s;
        s.at(0, 0) = sx; s.at(1, 1) = sy; s.at(2, 2) = sz;
        return s;
    }
    // Rotation about X axis by angle in degrees (right-handed, like glRotatef).
    static Mat4 rotateX(float deg) {
        float c = std::cos(deg2rad(deg)), s = std::sin(deg2rad(deg));
        Mat4 r;
        r.at(1, 1) = c; r.at(1, 2) = -s;
        r.at(2, 1) = s; r.at(2, 2) = c;
        return r;
    }
    static Mat4 rotateY(float deg) {
        float c = std::cos(deg2rad(deg)), s = std::sin(deg2rad(deg));
        Mat4 r;
        r.at(0, 0) = c;  r.at(0, 2) = s;
        r.at(2, 0) = -s; r.at(2, 2) = c;
        return r;
    }
    static Mat4 rotateZ(float deg) {
        float c = std::cos(deg2rad(deg)), s = std::sin(deg2rad(deg));
        Mat4 r;
        r.at(0, 0) = c; r.at(0, 1) = -s;
        r.at(1, 0) = s; r.at(1, 1) = c;
        return r;
    }
};
