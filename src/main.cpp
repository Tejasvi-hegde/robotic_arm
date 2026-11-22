// main.cpp - RoboCell: Interactive Robotic Arm Workcell Simulator
// CS373IA Computer Graphics and Virtual Reality - mini project
//
// Builds the window, registers the GLUT callbacks and runs the main loop.
// A fixed 60 Hz timer (glutTimerFunc) advances the simulation; display()
// draws the 3D view(s), the HUD and the mini-map.
//
// Optional capture flags (for report screenshots / automated tests):
//   --shot out.ppm     render one frame to a PPM image and exit
//   --sim  <seconds>   fast-forward the simulation before the shot
//   --auto             start the pick-and-place FSM
//   --cam  orbit|top|gripper     --stereo   --bounds   --axes   --wire
//   --operator-in      put the operator inside the safety zone
//   --pickview         show the colour-ID pick buffer
//   --manual-hit       drive the arm into a pillar (collision demo)
//   --size WxH         window size
//   --pick X Y         IK mode: colour-pick the pixel (X,Y) and send the arm there
#include "GLInclude.h"
#include "App.h"
#include "Simulation.h"
#include "scene/Scene.h"
#include "view/Renderer.h"
#include "input/Input.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>

App g;

void showMessage(const std::string& m, float seconds) {
    g.message = m;
    g.messageTime = seconds;
}

void playBeep() {
    if (!g.set.sound || g.beepCooldown > 0) return;
    g.beepCooldown = 0.7f;
#ifdef _WIN32
    MessageBeep(MB_ICONEXCLAMATION);       // non-blocking system sound (sound display)
#else
    std::fputc('\a', stdout);
    std::fflush(stdout);
#endif
}

// ------------------------------------------------------------------ timing
static int g_lastMs = 0;
static int g_frames = 0, g_fpsMs = 0;

static void onTimer(int) {
    int now = glutGet(GLUT_ELAPSED_TIME);
    float dt = (now - g_lastMs) / 1000.0f;
    g_lastMs = now;
    if (dt > 0.05f) dt = 0.05f;           // avoid huge jumps after a pause / window drag
    updateSimulation(g, dt);
    glutPostRedisplay();
    glutTimerFunc(16, onTimer, 0);
}

static void onDisplay() {
    renderFrame();
    ++g_frames;
    int now = glutGet(GLUT_ELAPSED_TIME);
    if (now - g_fpsMs >= 500) {
        g.fps = g_frames * 1000.0f / (now - g_fpsMs);
        g_frames = 0;
        g_fpsMs = now;
    }
}

// ------------------------------------------------------------------ capture mode
struct CaptureOptions {
    std::string shot;
    float simSeconds = 0;
    bool autoMode = false, operatorIn = false, manualHit = false;
    int pickX = -1, pickY = -1;
};

static void writePPM(const std::string& path) {
    std::vector<unsigned char> px((size_t)g.winW * g.winH * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_BACK);
    glReadPixels(0, 0, g.winW, g.winH, GL_RGB, GL_UNSIGNED_BYTE, px.data());
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) { std::perror("ppm"); return; }
    std::fprintf(f, "P6\n%d %d\n255\n", g.winW, g.winH);
    for (int y = g.winH - 1; y >= 0; --y) std::fwrite(&px[(size_t)y * g.winW * 3], 1, (size_t)g.winW * 3, f);
    std::fclose(f);
    std::printf("Saved %s\n", path.c_str());
}

static CaptureOptions g_cap;

static void captureAndExit() {
    if (g_cap.autoMode) behaviourStart(g);
    if (g_cap.manualHit) {
        // face the +Z pillar, level the gripper and lower the arm until the collision gate stops it
        float th[J_COUNT];
        for (int k = 0; k < 200; ++k) {
            for (int i = 0; i < J_COUNT; ++i) th[i] = g.arm.theta[i];
            if (th[J_BASE] > -66) th[J_BASE] -= 3;                 // 1) face the +Z pillar
            else if (th[J_WRIST] < 35) th[J_WRIST] += 5;          // 2) gripper horizontal
            else { th[J_SHOULDER] -= 1; th[J_WRIST] += 1; }       // 3) lower, keep it level
            if (!tryApplyAngles(g, th)) break;
        }
    }
    if (g_cap.operatorIn) { g.op.pos = Vec3(3.2f, FLOOR_Y, 1.5f); }
    if (g_cap.pickX >= 0) {
        g.arm.mode = MODE_IK;
        PickResult r = pickAt(g_cap.pickX, g_cap.pickY);
        std::printf("pick (%d,%d) -> id %d  point %s (%.2f %.2f %.2f)\n", g_cap.pickX, g_cap.pickY, r.id,
                    r.hasPoint ? "yes" : "no", r.point.x, r.point.y, r.point.z);
        if (r.id >= 1 && r.id <= (int)g.parts.size()) commandIK(g, partGraspPoint(g.parts[r.id - 1]) + Vec3(0, 0.02f, 0));
        else if (r.hasPoint) commandIK(g, Vec3(r.point.x, 0.3f, r.point.z));
    }
    int steps = (int)(g_cap.simSeconds * 60);
    for (int i = 0; i < steps; ++i) updateSimulation(g, 1.0f / 60.0f);
    updateSimulation(g, 1.0f / 60.0f);
    g.fps = 60;
    renderFrame(false);   // draw into the back buffer without swapping
    glFinish();
    writePPM(g_cap.shot);
    std::exit(0);
}

static void parseArgs(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string { return i + 1 < argc ? argv[++i] : ""; };
        if (a == "--shot") g_cap.shot = next();
        else if (a == "--sim") g_cap.simSeconds = (float)std::atof(next().c_str());
        else if (a == "--auto") g_cap.autoMode = true;
        else if (a == "--operator-in") g_cap.operatorIn = true;
        else if (a == "--manual-hit") g_cap.manualHit = true;
        else if (a == "--stereo") g.set.stereo = true;
        else if (a == "--bounds") g.set.bounds = true;
        else if (a == "--axes") g.set.axes = true;
        else if (a == "--wire") g.set.wireframe = true;
        else if (a == "--pickview") g.set.showPick = true;
        else if (a == "--nohelp") g.set.help = false;
        else if (a == "--pick") { g_cap.pickX = std::atoi(next().c_str()); g_cap.pickY = std::atoi(next().c_str()); }
        else if (a == "--cam") {
            std::string c = next();
            g.cam.mode = c == "top" ? CAM_TOP : c == "gripper" ? CAM_GRIPPER : CAM_ORBIT;
        } else if (a == "--size") {
            int w, h;
            if (std::sscanf(next().c_str(), "%dx%d", &w, &h) == 2) { g.winW = w; g.winH = h; }
        }
    }
}

// ------------------------------------------------------------------ main
int main(int argc, char** argv) {
    glutInit(&argc, argv);
    parseArgs(argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(g.winW, g.winH);
    glutInitWindowPosition(60, 40);
    glutCreateWindow("RoboCell - Robotic Arm Workcell Simulator (CGVR)");
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

    setupLighting();
    Scene::init();          // compile display lists
    resetWorld(g);

    glutDisplayFunc(onDisplay);
    Input::registerCallbacks();
    Input::createMenus();

    if (!g_cap.shot.empty()) { captureAndExit(); return 0; }

    showMessage("Welcome! Right-click for the menu, M to change mode, Space for auto.", 6.0f);
    g_lastMs = g_fpsMs = glutGet(GLUT_ELAPSED_TIME);
    glutTimerFunc(16, onTimer, 0);
    glutMainLoop();
    return 0;
}
