// App.h - the whole application state in one place (owned by main.cpp).
#pragma once
#include <vector>
#include <string>
#include "arm/Arm.h"
#include "arm/Behaviour.h"
#include "physics/Collision.h"
#include "physics/Force.h"
#include "view/Camera.h"

enum PartState { PART_TABLE = 0, PART_HELD, PART_FALLING, PART_BIN };

struct Part {
    Vec3 pos;            // bottom centre of the Sierpinski pyramid
    int  colorIndex;     // index into the colour palette (indexed colour, Unit I)
    int  state = PART_TABLE;
    float vy = 0;        // vertical speed while falling
    Vec3 holdOffset;     // offset from fingertip while held
    float landY = 0;     // y where a falling part stops
    bool unreachable = false;
};

struct Bin {
    Vec3 pos;            // bottom centre
    int  colorIndex;
    int  count = 0;
};

struct Operator {
    Vec3 pos{6.0f, -2.5f, 3.5f};  // feet position on the floor
    bool patrol = false;
    float patrolT = 0;            // 0..1 along the patrol line
    int dir = 1;
    AABB box() const {            // body: 0.6 x 3.4 x 0.6, head included
        return {pos + Vec3(0, 1.7f, 0), Vec3(0.3f, 1.7f, 0.3f)};
    }
};

struct Settings {
    bool depthTest = true, cullFace = true, wireframe = false, bounds = false;
    bool axes = false, lighting = true, useLists = true, showPick = false;
    bool safetyEnabled = true, stereo = false, stress = false, help = true;
    bool sound = true;
    int  sierpinskiDepth = 2;
    int  clipAlgo = 0;   // 0 Cohen-Sutherland, 1 Liang-Barsky
    int  lineAlgo = 0;   // 0 Bresenham, 1 DDA
};

struct TeachState {
    std::vector<std::vector<float>> waypoints;
    bool playing = false;
    int  index = 0;
};

struct App {
    ArmState arm;
    ArmPose  pose;
    std::vector<Part> parts;
    std::vector<Bin>  bins;
    std::vector<AABB> obstacles;    // fixed pillars
    Operator op;
    Camera   cam;
    Settings set;
    Behaviour fsm;
    ForceState force;
    TeachState teach;

    float safetyRadius = 4.1f;      // L1 + L2 + 0.3
    bool  zoneBreached = false;

    // mini-map world window (x,z ranges) - can be zoomed and panned
    float mmCx = 0, mmCz = 0, mmHalf = 6.5f;

    bool hasTarget = false;
    Vec3 target;

    int winW = 1280, winH = 760;
    float fps = 0;
    std::string message;
    float messageTime = 0;
    float beepCooldown = 0;
    int   collisionCount = 0;
};

extern App g;

void showMessage(const std::string& m, float seconds = 3.0f);
void playBeep();
