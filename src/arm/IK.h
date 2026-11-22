// IK.h - closed-form inverse kinematics (Unit V: object position).
#pragma once
#include "Arm.h"

enum IKResult { IK_OK = 0, IK_UNREACHABLE, IK_JOINT_LIMIT };

// Solve joint angles that put the fingertip centre at `target` (world),
// with the gripper axis at world pitch `approachDeg` (-90 = pointing straight down).
// Keeps the current gripper opening. Writes into out[] only on IK_OK.
IKResult solveIK(const ArmState& s, const Vec3& target, float approachDeg,
                 bool elbowUp, float out[J_COUNT]);

// Same, but tries the requested approach first and then a few tilted
// approaches (-90, -70, -50, -30, 0) so far targets are still reachable.
IKResult solveIKFlexible(const ArmState& s, const Vec3& target, bool elbowUp,
                         float out[J_COUNT], float* usedApproach = nullptr);

const char* ikResultText(IKResult r);
