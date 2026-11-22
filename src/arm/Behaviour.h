// Behaviour.h - behaviour modelling: the autonomous pick-and-place FSM (Unit V).
#pragma once

enum FsmState {
    FSM_IDLE = 0,   // choose next part
    FSM_APPROACH,   // above the part, gripper open
    FSM_DESCEND,    // down to the part
    FSM_GRASP,      // close gripper, attach part
    FSM_LIFT,       // back up
    FSM_TRANSFER,   // above the matching bin
    FSM_RELEASE,    // open gripper, part drops
    FSM_RETRACT,    // back to home pose
    FSM_RECOVER,    // collision happened: go home, then retry the interrupted state
    FSM_HALT,       // safety zone breached: wait
    FSM_DONE,       // all parts sorted
    FSM_COUNT
};

struct Behaviour {
    bool running = false;
    int  state = FSM_IDLE;
    int  resumeState = FSM_IDLE;  // state to resume after HALT / RECOVER
    int  part = -1;               // part being handled
    int  bin = -1;
    bool issued = false;          // has the motion for this state been commanded?
    float wait = 0;               // small dwell timer (grasp / release)
    int  recoverAttempts = 0;
};

struct App;
const char* fsmStateName(int s);
void behaviourStart(App& a);
void behaviourStop(App& a);
void behaviourUpdate(App& a, float dt);
// Called by the simulation when a planned motion step collides.
void behaviourOnCollision(App& a);
