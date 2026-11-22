// Simulation.cpp
#include "Simulation.h"
#include <cmath>
#include <cstdio>

Vec3 partGraspPoint(const Part& p) { return p.pos + Vec3(0, PART_SIZE * 0.5f, 0); }

AABB binBox(const Bin& b) { return {b.pos + Vec3(0, 0.2f, 0), Vec3(0.42f, 0.2f, 0.42f)}; }

static float surfaceUnder(const Vec3& p) {
    bool overTable = std::fabs(p.x) <= TABLE_HALF && std::fabs(p.z) <= TABLE_HALF;
    return overTable ? 0.0f : FLOOR_Y;
}

bool armPoseCollides(const App& a, const float th[J_COUNT], bool hits[3], std::string* why) {
    hits[0] = hits[1] = hits[2] = false;
    ArmPose p = computeFK(a.arm, th);
    Capsule caps[3] = {{p.shoulder, p.elbow, 0.15f},
                       {p.elbow, p.wrist, 0.12f},
                       {p.wrist, p.tip, 0.09f}};
    const char* linkName[3] = {"upper arm", "forearm", "wrist/gripper"};
    char buf[160];
    bool any = false;

    auto report = [&](int link, const char* what) {
        hits[link] = true;
        if (!any && why) {
            std::snprintf(buf, sizeof(buf), "Collision: %s hits %s", linkName[link], what);
            *why = buf;
        }
        any = true;
    };

    for (int l = 0; l < 3; ++l) {
        for (size_t i = 0; i < a.obstacles.size(); ++i)
            if (capsuleVsAABB(caps[l], a.obstacles[i])) report(l, "an obstacle pillar");
        for (size_t i = 0; i < a.bins.size(); ++i)
            if (capsuleVsAABB(caps[l], binBox(a.bins[i]))) report(l, "a bin");
        if (capsuleVsAABB(caps[l], a.op.box())) report(l, "the OPERATOR");
    }
    // Table / floor: joints may not go through the surface.
    if (p.elbow.y - 0.12f < surfaceUnder(p.elbow)) report(0, "the table");
    if (p.wrist.y - 0.10f < surfaceUnder(p.wrist)) report(1, "the table");
    // The fingertip may press into the table a little (virtual wall -> force bar).
    if (p.tip.y < surfaceUnder(p.tip) - a.force.maxPenetration) report(2, "the table (force limit)");
    return any;
}

bool tryApplyAngles(App& a, const float thIn[J_COUNT]) {
    float th[J_COUNT];
    for (int i = 0; i < J_COUNT; ++i) th[i] = thIn[i];
    clampToLimits(th);
    bool hits[3];
    std::string why;
    if (armPoseCollides(a, th, hits, &why)) {
        for (int i = 0; i < 3; ++i) a.arm.linkHit[i] = hits[i];
        if (!a.arm.colliding) { a.collisionCount++; showMessage(why); }
        a.arm.colliding = true;
        playBeep();
        return false;
    }
    for (int i = 0; i < J_COUNT; ++i) a.arm.theta[i] = th[i];
    for (int i = 0; i < 3; ++i) a.arm.linkHit[i] = false;
    a.arm.colliding = false;
    a.pose = computeFK(a.arm);
    return true;
}

IKResult commandIK(App& a, const Vec3& target, bool quiet) {
    float sol[J_COUNT];
    float approach = -90;
    IKResult r = solveIKFlexible(a.arm, target, a.arm.elbowUp, sol, &approach);
    a.target = target;
    a.hasTarget = true;
    if (r != IK_OK) {
        if (!quiet) showMessage(ikResultText(r));
        return r;
    }
    armMoveTo(a.arm, sol);
    if (!quiet) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "IK solved: target (%.2f, %.2f, %.2f), approach %.0f deg",
                      target.x, target.y, target.z, approach);
        showMessage(buf, 2.0f);
    }
    return IK_OK;
}

void resetWorld(App& a) {
    // Parts: 3x3 grid on the +X side of the table, colours mixed (palette indices 0,1,2).
    a.parts.clear();
    const int colours[9] = {0, 1, 2, 2, 0, 1, 1, 2, 0};
    int k = 0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            Part p;
            p.pos = Vec3(2.0f + 0.6f * i, 0.0f, -1.0f + 1.0f * j);
            p.colorIndex = colours[k++];
            a.parts.push_back(p);
        }
    // Bins: one per colour, on the -Z side.
    a.bins.clear();
    a.bins.push_back({Vec3(0.3f, 0, -2.9f), 0});
    a.bins.push_back({Vec3(-1.4f, 0, -2.6f), 1});
    a.bins.push_back({Vec3(-2.6f, 0, -1.4f), 2});
    // Obstacles: two pillars on the +Z side.
    a.obstacles.clear();
    a.obstacles.push_back({Vec3(-2.0f, 0.7f, 2.0f), Vec3(0.35f, 0.7f, 0.35f)});
    a.obstacles.push_back({Vec3(1.2f, 0.55f, 2.9f), Vec3(0.3f, 0.55f, 0.3f)});
    // Operator outside the safety zone.
    a.op = Operator();
    // Arm
    for (int i = 0; i < J_COUNT; ++i) a.arm.theta[i] = HOME_POSE[i];
    a.arm.moving = false;
    a.arm.heldPart = -1;
    a.arm.colliding = false;
    for (bool& h : a.arm.linkHit) h = false;
    a.arm.halted = false;
    a.pose = computeFK(a.arm);
    a.fsm = Behaviour();
    a.teach.playing = false;
    a.hasTarget = false;
    a.force = ForceState();
    a.safetyRadius = a.arm.L1 + a.arm.L2 + 0.3f;
}

void updateSimulation(App& a, float dt) {
    if (a.messageTime > 0) a.messageTime -= dt;
    if (a.beepCooldown > 0) a.beepCooldown -= dt;

    // ---- operator patrol behaviour: walks a line that cuts into the safety zone
    if (a.op.patrol) {
        const Vec3 P0(5.5f, FLOOR_Y, -4.5f), P1(2.8f, FLOOR_Y, 4.5f);
        a.op.patrolT += a.op.dir * dt * 0.07f;
        if (a.op.patrolT > 1) { a.op.patrolT = 1; a.op.dir = -1; }
        if (a.op.patrolT < 0) { a.op.patrolT = 0; a.op.dir = 1; }
        a.op.pos = lerp(P0, P1, a.op.patrolT);
    }

    // ---- safety zone
    a.zoneBreached = circleXZvsAABB(0, 0, a.safetyRadius, a.op.box());
    bool halt = a.set.safetyEnabled && a.zoneBreached;
    if (halt && !a.arm.halted) { showMessage("SAFETY STOP: operator inside the safety zone"); playBeep(); }
    if (!halt && a.arm.halted) showMessage("Safety zone clear - resuming", 2.0f);
    a.arm.halted = halt;

    // ---- smooth motion, gated by collision detection
    if (!a.arm.halted && a.arm.moving) {
        float next[J_COUNT], newT;
        if (armInterpolate(a.arm, dt, next, newT)) {
            if (tryApplyAngles(a, next)) {
                a.arm.t = newT;
                if (a.arm.t >= a.arm.duration) a.arm.moving = false;
            } else {
                a.arm.moving = false;          // drop the step, stop
                behaviourOnCollision(a);
                a.teach.playing = false;
            }
        }
    }

    // ---- teach-mode playback
    if (a.teach.playing && !a.arm.moving && !a.arm.halted) {
        if (a.teach.index < (int)a.teach.waypoints.size()) {
            armMoveTo(a.arm, a.teach.waypoints[a.teach.index].data());
            a.teach.index++;
        } else {
            a.teach.playing = false;
            showMessage("Teach playback finished", 2.0f);
        }
    }

    // ---- behaviour FSM
    behaviourUpdate(a, dt);

    a.pose = computeFK(a.arm);

    // ---- contact force (fingertip against the table)
    float surf = (std::fabs(a.pose.tip.x) <= TABLE_HALF && std::fabs(a.pose.tip.z) <= TABLE_HALF) ? 0.0f : FLOOR_Y;
    updateForce(a.force, a.pose.tip.y, surf);

    // ---- parts
    for (size_t i = 0; i < a.parts.size(); ++i) {
        Part& p = a.parts[i];
        if (p.state == PART_HELD) {
            p.pos = a.pose.tip + p.holdOffset;
        } else if (p.state == PART_FALLING) {
            p.vy -= 9.8f * dt;
            p.pos.y += p.vy * dt;
            if (p.pos.y <= p.landY) { p.pos.y = p.landY; p.vy = 0; p.state = PART_BIN; }
        }
    }
}
