// Collision.h - collision detection (Unit V: physical modelling).
// Links are capsules (segment + radius); the world is made of AABBs.
#pragma once
#include "../math/Vec3.h"

struct AABB {
    Vec3 c;   // centre
    Vec3 h;   // half extents
    Vec3 min() const { return c - h; }
    Vec3 max() const { return c + h; }
};

struct Capsule {
    Vec3 a, b;
    float r;
};

// Closest point on the box to p (clamp each coordinate).
Vec3  closestPointOnAABB(const AABB& box, const Vec3& p);
float distPointAABB(const AABB& box, const Vec3& p);
bool  sphereVsAABB(const Vec3& centre, float r, const AABB& box);
// Exact distance between a segment and a box: the distance is a convex
// function of the segment parameter t, so a ternary search finds its minimum.
float distSegmentAABB(const Vec3& a, const Vec3& b, const AABB& box);
bool  capsuleVsAABB(const Capsule& c, const AABB& box);
bool  aabbVsAABB(const AABB& a, const AABB& b);
// Circle on the floor (XZ plane) vs the box footprint - used for the safety zone.
bool  circleXZvsAABB(float cx, float cz, float r, const AABB& box);
