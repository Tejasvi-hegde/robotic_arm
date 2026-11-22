// Collision.cpp
#include "Collision.h"
#include <cmath>
#include <algorithm>

Vec3 closestPointOnAABB(const AABB& box, const Vec3& p) {
    Vec3 mn = box.min(), mx = box.max();
    return {clampf(p.x, mn.x, mx.x), clampf(p.y, mn.y, mx.y), clampf(p.z, mn.z, mx.z)};
}

float distPointAABB(const AABB& box, const Vec3& p) {
    return (p - closestPointOnAABB(box, p)).length();
}

bool sphereVsAABB(const Vec3& centre, float r, const AABB& box) {
    return distPointAABB(box, centre) < r;
}

float distSegmentAABB(const Vec3& a, const Vec3& b, const AABB& box) {
    // f(t) = dist(a + t(b-a), box) is convex on [0,1] -> ternary search.
    float lo = 0, hi = 1;
    for (int i = 0; i < 40; ++i) {
        float m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3;
        float f1 = distPointAABB(box, lerp(a, b, m1));
        float f2 = distPointAABB(box, lerp(a, b, m2));
        if (f1 < f2) hi = m2; else lo = m1;
    }
    return distPointAABB(box, lerp(a, b, (lo + hi) / 2));
}

bool capsuleVsAABB(const Capsule& c, const AABB& box) {
    return distSegmentAABB(c.a, c.b, box) < c.r;
}

bool aabbVsAABB(const AABB& a, const AABB& b) {
    return std::fabs(a.c.x - b.c.x) <= a.h.x + b.h.x &&
           std::fabs(a.c.y - b.c.y) <= a.h.y + b.h.y &&
           std::fabs(a.c.z - b.c.z) <= a.h.z + b.h.z;
}

bool circleXZvsAABB(float cx, float cz, float r, const AABB& box) {
    float px = clampf(cx, box.c.x - box.h.x, box.c.x + box.h.x);
    float pz = clampf(cz, box.c.z - box.h.z, box.c.z + box.h.z);
    float dx = cx - px, dz = cz - pz;
    return dx * dx + dz * dz < r * r;
}
