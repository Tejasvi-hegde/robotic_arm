// Arm.cpp - hierarchical model + forward kinematics + smooth interpolation.
#include "Arm.h"
#include "../GLInclude.h"
#include "../scene/Primitives.h"
#include "../scene/Scene.h"
#include <cmath>
#include <algorithm>

const float JOINT_MIN[J_COUNT] = {-180.0f, -20.0f, -165.0f, -135.0f, 0.0f};
const float JOINT_MAX[J_COUNT] = { 180.0f, 160.0f,  165.0f,  135.0f, 1.0f};
const char* JOINT_NAMES[J_COUNT] = {"Base yaw", "Shoulder", "Elbow", "Wrist", "Gripper"};
const float HOME_POSE[J_COUNT] = {0.0f, 75.0f, -110.0f, -55.0f, 1.0f};

static const float GRIP_LEN = 0.25f;   // part of L3 taken by the gripper fingers

ArmPose computeFK(const ArmState& s, const float th[J_COUNT]) {
    ArmPose p;
    // Local transform of each joint relative to its parent.
    p.local[0] = Mat4::rotateY(th[J_BASE]);
    p.local[1] = Mat4::translate(0, s.baseH, 0) * Mat4::rotateZ(th[J_SHOULDER]);
    p.local[2] = Mat4::translate(s.L1, 0, 0) * Mat4::rotateZ(th[J_ELBOW]);
    p.local[3] = Mat4::translate(s.L2, 0, 0) * Mat4::rotateZ(th[J_WRIST]);
    p.local[4] = Mat4::translate(s.L3, 0, 0);

    // Composite (world) transform = product of locals from the root down.
    p.frame[0] = p.local[0];
    for (int i = 1; i < 5; ++i) p.frame[i] = p.frame[i - 1] * p.local[i];

    p.shoulder = p.frame[1].translation();
    p.elbow    = p.frame[2].translation();
    p.wrist    = p.frame[3].translation();
    p.tip      = p.frame[4].translation();
    p.pitch    = th[J_SHOULDER] + th[J_ELBOW] + th[J_WRIST];
    return p;
}

bool withinLimits(const float th[J_COUNT]) {
    for (int i = 0; i < J_COUNT; ++i)
        if (th[i] < JOINT_MIN[i] - 1e-3f || th[i] > JOINT_MAX[i] + 1e-3f) return false;
    return true;
}

void clampToLimits(float th[J_COUNT]) {
    for (int i = 0; i < J_COUNT; ++i) th[i] = clampf(th[i], JOINT_MIN[i], JOINT_MAX[i]);
}

// ---------------------------------------------------------------- drawing
static void linkColour(bool hit, float r, float g, float b) {
    if (hit) glColor3f(0.95f, 0.1f, 0.1f);
    else glColor3f(r, g, b);
}

static void drawGripper(float opening, bool hit) {
    // palm
    linkColour(hit, 0.25f, 0.25f, 0.28f);
    glPushMatrix();
    glTranslatef(0.03f, 0, 0);
    drawBox(0.06f, 0.12f, 0.36f);
    glPopMatrix();
    // two fingers that slide apart along local Z
    float sep = 0.05f + 0.13f * opening;
    for (int side = -1; side <= 1; side += 2) {
        glPushMatrix();
        glTranslatef(GRIP_LEN / 2 + 0.03f, 0, side * sep);
        linkColour(hit, 0.85f, 0.85f, 0.88f);
        drawBox(GRIP_LEN - 0.04f, 0.08f, 0.04f);
        glPopMatrix();
    }
}

void drawArm(const ArmState& s, bool showAxes) {
    const float* th = s.theta;
    glPushMatrix();
        // ---- base (yaw about Y)
        glRotatef(th[J_BASE], 0, 1, 0);
        glColor3f(0.30f, 0.32f, 0.36f);
        Scene::callList(DL_BASE);
        if (showAxes) drawAxes(0.8f);

        // ---- shoulder
        glTranslatef(0, s.baseH, 0);
        glRotatef(th[J_SHOULDER], 0, 0, 1);
        glColor3f(0.2f, 0.2f, 0.22f);
        drawSphere(0.22f);
        linkColour(s.linkHit[0], 0.98f, 0.62f, 0.10f);      // industrial orange
        Scene::callList(DL_LINK1);
        if (showAxes) drawAxes(0.7f);

        // ---- elbow
        glTranslatef(s.L1, 0, 0);
        glRotatef(th[J_ELBOW], 0, 0, 1);
        glColor3f(0.2f, 0.2f, 0.22f);
        drawSphere(0.18f);
        linkColour(s.linkHit[1], 0.98f, 0.62f, 0.10f);
        Scene::callList(DL_LINK2);
        if (showAxes) drawAxes(0.6f);

        // ---- wrist
        glTranslatef(s.L2, 0, 0);
        glRotatef(th[J_WRIST], 0, 0, 1);
        glColor3f(0.2f, 0.2f, 0.22f);
        drawSphere(0.13f);
        linkColour(s.linkHit[2], 0.55f, 0.57f, 0.62f);
        Scene::callList(DL_LINK3);
        if (showAxes) drawAxes(0.5f);

        // ---- gripper sits at the end of the wrist link
        glTranslatef(s.L3 - GRIP_LEN, 0, 0);
        drawGripper(th[J_GRIPPER], s.linkHit[2]);
        if (showAxes) { glTranslatef(GRIP_LEN, 0, 0); drawAxes(0.4f); }
    glPopMatrix();
}

// ---------------------------------------------------------------- motion
void armMoveTo(ArmState& s, const float goal[J_COUNT], float minDuration) {
    float maxDiff = 0;
    for (int i = 0; i < J_COUNT; ++i) {
        s.start[i] = s.theta[i];
        s.goal[i] = goal[i];
        float d = std::fabs(goal[i] - s.theta[i]);
        if (i == J_GRIPPER) d *= 60.0f;          // gripper 0..1 ~ 60 degrees of "work"
        maxDiff = std::max(maxDiff, d);
    }
    s.duration = std::max(minDuration, maxDiff / 90.0f * 0.9f);  // ~100 deg/s
    s.t = 0;
    s.moving = true;
}

bool armInterpolate(const ArmState& s, float dt, float out[J_COUNT], float& newT) {
    if (!s.moving) return false;
    newT = std::min(s.duration, s.t + dt);
    float u = s.duration > 0 ? newT / s.duration : 1.0f;
    float k = u * u * (3.0f - 2.0f * u);          // smoothstep 3u^2 - 2u^3
    for (int i = 0; i < J_COUNT; ++i) out[i] = s.start[i] + (s.goal[i] - s.start[i]) * k;
    return true;
}
