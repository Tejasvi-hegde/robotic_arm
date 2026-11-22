// IK.cpp - base yaw + 2-link planar solve (law of cosines) + wrist compensation.
//
//   th0 = atan2(-z, x)          (glRotatef about +Y turns +X toward -Z)
//   r   = sqrt(x^2+z^2) - L3 cos(phi),   y' = y - h - L3 sin(phi)
//   cos th2 = (r^2 + y'^2 - L1^2 - L2^2) / (2 L1 L2)
//   th1 = atan2(y', r) - atan2(L2 sin th2, L1 + L2 cos th2)
//   th3 = phi - th1 - th2
// Reachable only if |L1 - L2| <= sqrt(r^2 + y'^2) <= L1 + L2.
#include "IK.h"
#include <cmath>

IKResult solveIK(const ArmState& s, const Vec3& target, float approachDeg,
                 bool elbowUp, float out[J_COUNT]) {
    const float L1 = s.L1, L2 = s.L2, L3 = s.L3;
    const float phi = deg2rad(approachDeg);

    // 1) base yaw points the arm's vertical plane at the target
    float th0 = std::atan2(-target.z, target.x);

    // 2) wrist position inside that plane (subtract the fixed wrist+gripper segment)
    float rTip = std::sqrt(target.x * target.x + target.z * target.z);
    float r  = rTip - L3 * std::cos(phi);
    float yp = target.y - s.baseH - L3 * std::sin(phi);

    float d = std::sqrt(r * r + yp * yp);
    if (d > L1 + L2 - 1e-4f || d < std::fabs(L1 - L2) + 1e-4f) return IK_UNREACHABLE;

    // 3) law of cosines for the elbow (clamped so acos never returns NaN)
    float c2 = (r * r + yp * yp - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    c2 = clampf(c2, -1.0f, 1.0f);
    float th2 = std::acos(c2);
    if (elbowUp) th2 = -th2;          // negative elbow angle = elbow above the line

    float th1 = std::atan2(yp, r) - std::atan2(L2 * std::sin(th2), L1 + L2 * std::cos(th2));
    float th3 = phi - th1 - th2;

    float sol[J_COUNT];
    sol[J_BASE]     = rad2deg(th0);
    sol[J_SHOULDER] = rad2deg(th1);
    sol[J_ELBOW]    = rad2deg(th2);
    sol[J_WRIST]    = rad2deg(th3);
    sol[J_GRIPPER]  = s.theta[J_GRIPPER];

    // normalise wrist into [-180,180]
    while (sol[J_WRIST] > 180) sol[J_WRIST] -= 360;
    while (sol[J_WRIST] < -180) sol[J_WRIST] += 360;

    if (!withinLimits(sol)) return IK_JOINT_LIMIT;
    for (int i = 0; i < J_COUNT; ++i) out[i] = sol[i];
    return IK_OK;
}

IKResult solveIKFlexible(const ArmState& s, const Vec3& target, bool elbowUp,
                         float out[J_COUNT], float* usedApproach) {
    static const float approaches[] = {-90, -75, -60, -45, -30, -15, 0};
    IKResult last = IK_UNREACHABLE;
    for (float a : approaches) {
        IKResult r = solveIK(s, target, a, elbowUp, out);
        if (r == IK_OK) { if (usedApproach) *usedApproach = a; return IK_OK; }
        if (r == IK_JOINT_LIMIT) last = IK_JOINT_LIMIT;
    }
    // try the other elbow configuration before giving up
    for (float a : approaches) {
        if (solveIK(s, target, a, !elbowUp, out) == IK_OK) {
            if (usedApproach) *usedApproach = a;
            return IK_OK;
        }
    }
    return last;
}

const char* ikResultText(IKResult r) {
    switch (r) {
        case IK_OK: return "OK";
        case IK_UNREACHABLE: return "Target unreachable (outside |L1-L2| .. L1+L2)";
        case IK_JOINT_LIMIT: return "Solution violates joint limits";
    }
    return "?";
}
