// Behaviour.cpp - autonomous pick-and-place finite-state machine.
//
// IDLE -> APPROACH -> DESCEND -> GRASP -> LIFT -> TRANSFER -> RELEASE -> RETRACT -> IDLE
// any state -> HALT (safety zone) -> back to the same state
// any motion state -> RECOVER (collision) -> home pose -> retry the interrupted state
#include "Behaviour.h"
#include "../App.h"
#include "../Simulation.h"
#include <cmath>

const char* fsmStateName(int s) {
    static const char* names[FSM_COUNT] = {"IDLE", "APPROACH", "DESCEND", "GRASP", "LIFT",
                                           "TRANSFER", "RELEASE", "RETRACT", "RECOVER",
                                           "HALT (safety)", "DONE"};
    return (s >= 0 && s < FSM_COUNT) ? names[s] : "?";
}

void behaviourStart(App& a) {
    a.fsm.running = true;
    a.fsm.recoverAttempts = 0;
    if (a.fsm.state == FSM_DONE || a.fsm.state == FSM_HALT) a.fsm.state = FSM_IDLE;
    a.fsm.issued = false;
    a.arm.mode = MODE_AUTO;
    showMessage("Auto mode: pick-and-place started", 2.0f);
}

void behaviourStop(App& a) {
    a.fsm.running = false;
    showMessage("Auto mode paused", 2.0f);
}

void behaviourOnCollision(App& a) {
    Behaviour& f = a.fsm;
    if (!f.running) return;
    if (f.state == FSM_RECOVER || f.recoverAttempts >= 3) {
        f.running = false;
        showMessage("Auto stopped: path blocked. Clear the obstacle or use Manual mode.", 5.0f);
        return;
    }
    f.resumeState = f.state;
    f.state = FSM_RECOVER;
    f.issued = false;
    f.recoverAttempts++;
}

static int findBin(const App& a, int colorIndex) {
    for (size_t i = 0; i < a.bins.size(); ++i)
        if (a.bins[i].colorIndex == colorIndex) return (int)i;
    return -1;
}

// Command an IK move; on failure mark the part unreachable and retract.
static bool moveIK(App& a, const Vec3& target, float gripper) {
    float saved = a.arm.theta[J_GRIPPER];
    a.arm.theta[J_GRIPPER] = gripper;           // IK keeps the current gripper value
    IKResult r = commandIK(a, target, true);
    a.arm.theta[J_GRIPPER] = saved;
    if (r != IK_OK) return false;
    a.arm.start[J_GRIPPER] = saved;             // interpolate the gripper from where it is
    a.arm.goal[J_GRIPPER] = gripper;
    return true;
}

static void moveGripperOnly(App& a, float gripper) {
    float goal[J_COUNT];
    for (int i = 0; i < J_COUNT; ++i) goal[i] = a.arm.theta[i];
    goal[J_GRIPPER] = gripper;
    armMoveTo(a.arm, goal, 0.35f);
}

static void go(Behaviour& f, int s) { f.state = s; f.issued = false; f.wait = 0; }

void behaviourUpdate(App& a, float dt) {
    Behaviour& f = a.fsm;
    if (!f.running) return;

    // Safety zone: pause in HALT, motion is frozen by the simulation.
    if (a.arm.halted) {
        if (f.state != FSM_HALT) { f.resumeState = f.state; f.state = FSM_HALT; }
        return;
    }
    if (f.state == FSM_HALT) f.state = f.resumeState;   // resume, keep 'issued'

    bool arrived = f.issued && !a.arm.moving;

    switch (f.state) {
    case FSM_IDLE: {
        f.part = -1;
        for (size_t i = 0; i < a.parts.size(); ++i)
            if (a.parts[i].state == PART_TABLE && !a.parts[i].unreachable) { f.part = (int)i; break; }
        if (f.part < 0) {
            f.running = false;
            f.state = FSM_DONE;
            armMoveTo(a.arm, HOME_POSE);
            showMessage("All reachable parts sorted. Press R to reset the cell.", 5.0f);
            return;
        }
        f.bin = findBin(a, a.parts[f.part].colorIndex);
        f.recoverAttempts = 0;
        go(f, FSM_APPROACH);
        break;
    }
    case FSM_APPROACH:
        if (!f.issued) {
            if (!moveIK(a, partGraspPoint(a.parts[f.part]) + Vec3(0, 0.6f, 0), 1.0f)) {
                a.parts[f.part].unreachable = true;
                showMessage("Part unreachable - skipped", 2.5f);
                go(f, FSM_RETRACT);
            } else f.issued = true;
        } else if (arrived) go(f, FSM_DESCEND);
        break;
    case FSM_DESCEND:
        if (!f.issued) {
            if (!moveIK(a, partGraspPoint(a.parts[f.part]), 1.0f)) {
                a.parts[f.part].unreachable = true;
                go(f, FSM_RETRACT);
            } else f.issued = true;
        } else if (arrived) go(f, FSM_GRASP);
        break;
    case FSM_GRASP:
        if (!f.issued) { moveGripperOnly(a, 0.15f); f.issued = true; }
        else if (arrived) {
            f.wait += dt;
            if (f.wait > 0.15f) {
                Part& p = a.parts[f.part];
                p.state = PART_HELD;
                p.holdOffset = p.pos - a.pose.tip;
                a.arm.heldPart = f.part;
                go(f, FSM_LIFT);
            }
        }
        break;
    case FSM_LIFT:
        if (!f.issued) {
            if (!moveIK(a, partGraspPoint(a.parts[f.part]) + Vec3(0, 0.8f, 0), 0.15f)) go(f, FSM_TRANSFER);
            else f.issued = true;
        } else if (arrived) go(f, FSM_TRANSFER);
        break;
    case FSM_TRANSFER:
        if (!f.issued) {
            if (f.bin < 0 || !moveIK(a, a.bins[f.bin].pos + Vec3(0, 1.1f, 0), 0.15f)) {
                showMessage("Bin unreachable - stopping", 3.0f);
                f.running = false;
            } else f.issued = true;
        } else if (arrived) go(f, FSM_RELEASE);
        break;
    case FSM_RELEASE:
        if (!f.issued) { moveGripperOnly(a, 1.0f); f.issued = true; }
        else if (arrived) {
            Part& p = a.parts[f.part];
            Bin& b = a.bins[f.bin];
            static const float off[4][2] = {{-0.17f, -0.17f}, {0.17f, 0.17f}, {0.17f, -0.17f}, {-0.17f, 0.17f}};
            int slot = b.count % 4;
            p.state = PART_FALLING;
            p.vy = 0;
            p.pos.x = b.pos.x + off[slot][0];
            p.pos.z = b.pos.z + off[slot][1];
            p.landY = b.pos.y + 0.02f + (b.count / 4) * PART_SIZE;
            b.count++;
            a.arm.heldPart = -1;
            go(f, FSM_RETRACT);
        }
        break;
    case FSM_RETRACT:
        if (!f.issued) { armMoveTo(a.arm, HOME_POSE); f.issued = true; }
        else if (arrived) go(f, FSM_IDLE);
        break;
    case FSM_RECOVER:
        if (!f.issued) {
            float home[J_COUNT];
            for (int i = 0; i < J_COUNT; ++i) home[i] = HOME_POSE[i];
            home[J_GRIPPER] = a.arm.theta[J_GRIPPER];
            armMoveTo(a.arm, home);
            f.issued = true;
        } else if (arrived) {
            go(f, f.resumeState);    // retry the interrupted state from a safe pose
        }
        break;
    default:
        break;
    }
}
