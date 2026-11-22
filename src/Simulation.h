// Simulation.h - per-frame update: motion, collision gating, safety zone, parts.
#pragma once
#include <string>
#include "App.h"
#include "arm/IK.h"

constexpr float PART_SIZE = 0.32f;
constexpr float TABLE_HALF = 4.5f;     // table top spans [-4.5, 4.5] in x and z, at y = 0
constexpr float FLOOR_Y = -2.5f;

Vec3 partGraspPoint(const Part& p);
AABB binBox(const Bin& b);

// Would the arm collide in this pose? Fills hits[3] (per link) and a reason.
bool armPoseCollides(const App& a, const float th[J_COUNT], bool hits[3], std::string* why);
// Apply angles only if collision-free; otherwise flag the links red and beep.
bool tryApplyAngles(App& a, const float th[J_COUNT]);
// Solve IK for a world target and start a smooth move. Returns the IK result.
IKResult commandIK(App& a, const Vec3& target, bool quiet = false);

void resetWorld(App& a);
void updateSimulation(App& a, float dt);
