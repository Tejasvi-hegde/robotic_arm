// Arm.h - the robot arm: shared state, joint limits, forward kinematics, drawing.
// (Unit V: kinematics modelling, homogeneous transformation matrices, object hierarchies)
#pragma once
#include "../math/Mat4.h"

enum Joint { J_BASE = 0, J_SHOULDER, J_ELBOW, J_WRIST, J_GRIPPER, J_COUNT };
enum ArmMode { MODE_MANUAL = 0, MODE_IK, MODE_AUTO, MODE_COUNT };

// ArmState is the ONLY structure both team members write to.
struct ArmState {
    // Joint values: base yaw, shoulder pitch, elbow pitch, wrist pitch in degrees.
    // theta[J_GRIPPER] is the gripper opening 0 (closed) .. 1 (open).
    float theta[J_COUNT] = {0, 60, -100, -50, 1};

    // Interpolation (smoothstep) from start[] to goal[] over duration seconds.
    float start[J_COUNT] = {0};
    float goal[J_COUNT]  = {0};
    float t = 0, duration = 0;
    bool  moving = false;

    // Geometry (world units). L3 = wrist joint to fingertip centre.
    float baseH = 0.6f, L1 = 2.0f, L2 = 1.8f, L3 = 0.6f;

    int   mode = MODE_MANUAL;
    int   selectedJoint = J_SHOULDER;
    bool  elbowUp = true;

    bool  linkHit[3] = {false, false, false};  // link currently in collision (drawn red)
    bool  colliding = false;
    bool  halted = false;                      // safety zone breach
    int   heldPart = -1;
};

// Joint limits (degrees; gripper 0..1)
extern const float JOINT_MIN[J_COUNT];
extern const float JOINT_MAX[J_COUNT];
extern const char* JOINT_NAMES[J_COUNT];

// World pose of every joint frame, from forward kinematics.
struct ArmPose {
    Mat4 frame[5];   // 0 base(after yaw) 1 shoulder 2 elbow 3 wrist 4 tip/gripper
    Mat4 local[5];   // local transform of each joint (for the HUD matrix panel)
    Vec3 shoulder, elbow, wrist, tip;
    float pitch;     // world pitch of the gripper axis (deg) = th1+th2+th3
};

// Forward kinematics: T(0->e) = Ry(th0) T(0,h,0) Rz(th1) T(L1,0,0) Rz(th2) T(L2,0,0) Rz(th3) T(L3,0,0)
ArmPose computeFK(const ArmState& s, const float theta[J_COUNT]);
inline ArmPose computeFK(const ArmState& s) { return computeFK(s, s.theta); }

bool withinLimits(const float theta[J_COUNT]);
void clampToLimits(float theta[J_COUNT]);

// Draw arm with the GL matrix stack (glPushMatrix/glPopMatrix hierarchy).
void drawArm(const ArmState& s, bool showAxes);

// Begin a smooth move to new goal angles.
void armMoveTo(ArmState& s, const float goal[J_COUNT], float minDuration = 0.4f);
// Compute next angles of the current interpolation without applying them.
// Returns false when not moving.
bool armInterpolate(const ArmState& s, float dt, float out[J_COUNT], float& newT);

// Home pose (arm folded up, gripper open)
extern const float HOME_POSE[J_COUNT];
