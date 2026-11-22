// Primitives.cpp - hand-built primitives (Unit I: primitives and attributes).
#include "Primitives.h"
#include "../GLInclude.h"
#include "../math/Vec3.h"
#include <cmath>

void drawBox(float sx, float sy, float sz) {
    float x = sx / 2, y = sy / 2, z = sz / 2;
    glBegin(GL_QUADS);
    // +X
    glNormal3f(1, 0, 0);
    glVertex3f(x, -y, -z); glVertex3f(x, y, -z); glVertex3f(x, y, z); glVertex3f(x, -y, z);
    // -X
    glNormal3f(-1, 0, 0);
    glVertex3f(-x, -y, z); glVertex3f(-x, y, z); glVertex3f(-x, y, -z); glVertex3f(-x, -y, -z);
    // +Y
    glNormal3f(0, 1, 0);
    glVertex3f(-x, y, -z); glVertex3f(-x, y, z); glVertex3f(x, y, z); glVertex3f(x, y, -z);
    // -Y
    glNormal3f(0, -1, 0);
    glVertex3f(-x, -y, -z); glVertex3f(x, -y, -z); glVertex3f(x, -y, z); glVertex3f(-x, -y, z);
    // +Z
    glNormal3f(0, 0, 1);
    glVertex3f(-x, -y, z); glVertex3f(x, -y, z); glVertex3f(x, y, z); glVertex3f(-x, y, z);
    // -Z
    glNormal3f(0, 0, -1);
    glVertex3f(-x, -y, -z); glVertex3f(-x, y, -z); glVertex3f(x, y, -z); glVertex3f(x, -y, -z);
    glEnd();
}

void drawWireBox(float sx, float sy, float sz) {
    float x = sx / 2, y = sy / 2, z = sz / 2;
    glBegin(GL_LINE_LOOP);
    glVertex3f(-x, -y, -z); glVertex3f(x, -y, -z); glVertex3f(x, -y, z); glVertex3f(-x, -y, z);
    glEnd();
    glBegin(GL_LINE_LOOP);
    glVertex3f(-x, y, -z); glVertex3f(x, y, -z); glVertex3f(x, y, z); glVertex3f(-x, y, z);
    glEnd();
    glBegin(GL_LINES);
    glVertex3f(-x, -y, -z); glVertex3f(-x, y, -z);
    glVertex3f(x, -y, -z);  glVertex3f(x, y, -z);
    glVertex3f(x, -y, z);   glVertex3f(x, y, z);
    glVertex3f(-x, -y, z);  glVertex3f(-x, y, z);
    glEnd();
}

void drawCylinderX(float len, float r, int slices) {
    // side
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= slices; ++i) {
        float a = 2 * PI_F * i / slices;
        float cy = std::cos(a), cz = std::sin(a);
        glNormal3f(0, cy, cz);
        glVertex3f(len, r * cy, r * cz);   // order chosen so faces are CCW from outside
        glVertex3f(0, r * cy, r * cz);
    }
    glEnd();
    // end caps
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(-1, 0, 0);
    glVertex3f(0, 0, 0);
    for (int i = slices; i >= 0; --i) {
        float a = 2 * PI_F * i / slices;
        glVertex3f(0, r * std::cos(a), r * std::sin(a));
    }
    glEnd();
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(1, 0, 0);
    glVertex3f(len, 0, 0);
    for (int i = 0; i <= slices; ++i) {
        float a = 2 * PI_F * i / slices;
        glVertex3f(len, r * std::cos(a), r * std::sin(a));
    }
    glEnd();
}

void drawCylinderY(float h, float r, int slices) {
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= slices; ++i) {
        float a = 2 * PI_F * i / slices;
        float cx = std::cos(a), cz = std::sin(a);
        glNormal3f(cx, 0, cz);
        glVertex3f(r * cx, 0, r * cz);     // CCW from outside (back-face culling safe)
        glVertex3f(r * cx, h, r * cz);
    }
    glEnd();
    glBegin(GL_TRIANGLE_FAN);   // top
    glNormal3f(0, 1, 0);
    glVertex3f(0, h, 0);
    for (int i = slices; i >= 0; --i) {
        float a = 2 * PI_F * i / slices;
        glVertex3f(r * std::cos(a), h, r * std::sin(a));
    }
    glEnd();
    glBegin(GL_TRIANGLE_FAN);   // bottom
    glNormal3f(0, -1, 0);
    glVertex3f(0, 0, 0);
    for (int i = 0; i <= slices; ++i) {
        float a = 2 * PI_F * i / slices;
        glVertex3f(r * std::cos(a), 0, r * std::sin(a));
    }
    glEnd();
}

void drawSphere(float r, int slices, int stacks) {
    static GLUquadric* q = nullptr;
    if (!q) q = gluNewQuadric();
    gluSphere(q, r, slices, stacks);
}

void drawGrid(float half, float step, float y) {
    glBegin(GL_LINES);
    for (float v = -half; v <= half + 1e-4f; v += step) {
        glVertex3f(v, y, -half); glVertex3f(v, y, half);
        glVertex3f(-half, y, v); glVertex3f(half, y, v);
    }
    glEnd();
}

void drawAxes(float len) {
    GLboolean lit = glIsEnabled(GL_LIGHTING);
    glDisable(GL_LIGHTING);
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1, 0, 0); glVertex3f(0, 0, 0); glVertex3f(len, 0, 0);
    glColor3f(0, 1, 0); glVertex3f(0, 0, 0); glVertex3f(0, len, 0);
    glColor3f(0, 0, 1); glVertex3f(0, 0, 0); glVertex3f(0, 0, len);
    glEnd();
    glLineWidth(1.0f);
    if (lit) glEnable(GL_LIGHTING);
}
