// Sierpinski.cpp - recursive subdivision of a tetrahedron (the classic Angel example).
#include "Sierpinski.h"
#include "../GLInclude.h"
#include <cmath>

static Vec3 g_centroid;

static void triangle(const Vec3& a, const Vec3& b, const Vec3& c) {
    Vec3 n = (b - a).cross(c - a).normalized();
    Vec3 mid = (a + b + c) / 3.0f;
    glNormal3f(n.x, n.y, n.z);
    if (n.dot(mid - g_centroid) >= 0) {       // already counter-clockwise from outside
        glVertex3f(a.x, a.y, a.z); glVertex3f(b.x, b.y, b.z); glVertex3f(c.x, c.y, c.z);
    } else {                                  // flip winding + normal so culling keeps it
        glNormal3f(-n.x, -n.y, -n.z);
        glVertex3f(a.x, a.y, a.z); glVertex3f(c.x, c.y, c.z); glVertex3f(b.x, b.y, b.z);
    }
}

static void tetra(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
    g_centroid = (a + b + c + d) / 4.0f;
    triangle(a, b, c);
    triangle(a, c, d);
    triangle(a, d, b);
    triangle(b, d, c);
}

static void divide(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, int depth) {
    if (depth <= 0) { tetra(a, b, c, d); return; }
    Vec3 ab = (a + b) * 0.5f, ac = (a + c) * 0.5f, ad = (a + d) * 0.5f;
    Vec3 bc = (b + c) * 0.5f, bd = (b + d) * 0.5f, cd = (c + d) * 0.5f;
    divide(a, ab, ac, ad, depth - 1);
    divide(ab, b, bc, bd, depth - 1);
    divide(ac, bc, c, cd, depth - 1);
    divide(ad, bd, cd, d, depth - 1);
}

void drawSierpinski(float height, int depth) {
    float s = height / std::sqrt(2.0f / 3.0f);       // edge length from height
    float r = s / std::sqrt(3.0f);                   // circumradius of the base triangle
    Vec3 v0(r, 0, 0);
    Vec3 v1(-r / 2, 0, s / 2);
    Vec3 v2(-r / 2, 0, -s / 2);
    Vec3 apex(0, height, 0);
    glBegin(GL_TRIANGLES);
    divide(v0, v1, v2, apex, depth);
    glEnd();
}
