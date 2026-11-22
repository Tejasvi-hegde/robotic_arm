// Input.cpp
//
// Logical input devices (Unit II):
//   LOCATOR - mouse position (measure) ; PICK - colour-ID picking on click (trigger)
//   STRING/CHOICE - keyboard and the pop-up menu
// GLUT works in EVENT mode: each device puts events in a queue and our callbacks run.
#include "Input.h"
#include "../GLInclude.h"
#include "../App.h"
#include "../Simulation.h"
#include "../scene/Scene.h"
#include "../view/Renderer.h"
#include "../ui/Hud.h"
#include <cstdio>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <string>

// ------------------------------------------------------------------ helpers
static void enterManual() {
    if (g.arm.mode != MODE_MANUAL) {
        g.arm.mode = MODE_MANUAL;
        g.fsm.running = false;
        g.arm.moving = false;
        g.teach.playing = false;
    }
}

static void stepJoint(int j, float delta) {
    enterManual();
    if (g.arm.halted) { showMessage("Safety stop active - move the operator out first"); return; }
    float th[J_COUNT];
    for (int i = 0; i < J_COUNT; ++i) th[i] = g.arm.theta[i];
    th[j] += delta;
    tryApplyAngles(g, th);
}

// Manual gripper: closing near a part grabs it, opening drops it (into a bin if above one).
static void manualGripperToggle() {
    enterManual();
    bool closing = g.arm.theta[J_GRIPPER] > 0.5f;
    float goal[J_COUNT];
    for (int i = 0; i < J_COUNT; ++i) goal[i] = g.arm.theta[i];
    goal[J_GRIPPER] = closing ? 0.15f : 1.0f;
    for (int i = 0; i < J_COUNT; ++i) g.arm.theta[i] = goal[i];   // instant in manual mode

    if (closing && g.arm.heldPart < 0) {
        int best = -1;
        float bestD = 0.3f;
        for (size_t i = 0; i < g.parts.size(); ++i) {
            if (g.parts[i].state == PART_HELD || g.parts[i].state == PART_FALLING) continue;
            float d = (partGraspPoint(g.parts[i]) - g.pose.tip).length();
            if (d < bestD) { bestD = d; best = (int)i; }
        }
        if (best >= 0) {
            Part& p = g.parts[best];
            if (p.state == PART_BIN) {
                for (Bin& b : g.bins) if ((Vec3(b.pos.x, 0, b.pos.z) - Vec3(p.pos.x, 0, p.pos.z)).length() < 0.6f && b.count > 0) b.count--;
            }
            p.state = PART_HELD;
            p.holdOffset = p.pos - g.pose.tip;
            g.arm.heldPart = best;
            showMessage("Part grasped", 1.5f);
        }
    } else if (!closing && g.arm.heldPart >= 0) {
        Part& p = g.parts[g.arm.heldPart];
        p.state = PART_FALLING;
        p.vy = 0;
        p.landY = (std::fabs(p.pos.x) <= TABLE_HALF && std::fabs(p.pos.z) <= TABLE_HALF) ? 0.0f : FLOOR_Y;
        for (Bin& b : g.bins) {
            if (std::fabs(p.pos.x - b.pos.x) < 0.42f && std::fabs(p.pos.z - b.pos.z) < 0.42f) {
                p.landY = b.pos.y + 0.02f + (b.count / 4) * PART_SIZE;
                b.count++;
                showMessage(b.colorIndex == p.colorIndex ? "Dropped in the correct bin" : "Dropped in the WRONG bin", 2.0f);
                break;
            }
        }
        g.arm.heldPart = -1;
    }
    g.pose = computeFK(g.arm);
}

static void toggle(bool& flag, const char* name) {
    flag = !flag;
    showMessage(std::string(name) + (flag ? ": ON" : ": OFF"), 1.5f);
    Input::refreshMenuLabels();
}

static void cycleMode() {
    int m = (g.arm.mode + 1) % MODE_COUNT;
    g.arm.mode = m;
    g.teach.playing = false;
    if (m == MODE_AUTO) behaviourStart(g);
    else {
        g.fsm.running = false;
        showMessage(m == MODE_IK ? "IK mode: click a part or a point on the table" : "Manual mode: 1-5 select, arrows rotate", 2.5f);
    }
}

static void doReset() {
    resetWorld(g);
    showMessage("Cell reset", 1.5f);
}

static void setSierpinski(int d) {
    g.set.sierpinskiDepth = d;
    Scene::rebuildPartList();
    showMessage("Sierpinski depth " + std::to_string(d) + " (" + std::to_string((int)std::pow(4, d)) + " tetrahedra per part)", 2.0f);
}

static void teachRecord() {
    g.teach.waypoints.push_back(std::vector<float>(g.arm.theta, g.arm.theta + J_COUNT));
    showMessage("Teach: waypoint " + std::to_string(g.teach.waypoints.size()) + " recorded", 1.5f);
}
static void teachPlay() {
    if (g.teach.waypoints.empty()) { showMessage("Teach: no waypoints - press T to record"); return; }
    g.arm.mode = MODE_MANUAL;
    g.fsm.running = false;
    g.teach.playing = true;
    g.teach.index = 0;
    showMessage("Teach: playing " + std::to_string(g.teach.waypoints.size()) + " waypoints", 2.0f);
}

// ------------------------------------------------------------------ keyboard
static void onKeyboard(unsigned char key, int, int) {
    switch (std::tolower(key)) {
        case 27: case 'q': glutLeaveMainLoop(); return;
        case '1': case '2': case '3': case '4': case '5':
            g.arm.selectedJoint = key - '1'; break;
        case 'g': manualGripperToggle(); break;
        case 'm': cycleMode(); break;
        case ' ':
            if (g.fsm.running) behaviourStop(g); else behaviourStart(g);
            break;
        case 'e': g.arm.elbowUp = !g.arm.elbowUp;
            showMessage(g.arm.elbowUp ? "IK: elbow-up solution" : "IK: elbow-down solution", 1.5f); break;
        case 'r': doReset(); break;
        case 'h': g.set.help = !g.set.help; break;
        case 'c': g.cam.mode = (g.cam.mode + 1) % CAM_COUNT; showMessage(cameraName(g.cam.mode), 1.5f); break;
        case 'v': toggle(g.set.stereo, "Stereo (VR) view"); break;
        case '[': g.cam.nearP = std::fmax(0.05f, g.cam.nearP - 0.5f);
            showMessage("Near plane " + std::to_string(g.cam.nearP).substr(0, 4), 1.0f); break;
        case ']': g.cam.nearP = std::fmin(20.0f, g.cam.nearP + 0.5f);
            showMessage("Near plane " + std::to_string(g.cam.nearP).substr(0, 4) + " (geometry closer than this is clipped)", 1.5f); break;
        case 'i': g.op.pos.z -= 0.25f; g.op.patrol = false; break;
        case 'k': g.op.pos.z += 0.25f; g.op.patrol = false; break;
        case 'j': g.op.pos.x -= 0.25f; g.op.patrol = false; break;
        case 'l': g.op.pos.x += 0.25f; g.op.patrol = false; break;
        case 'o': toggle(g.op.patrol, "Operator patrol"); break;
        case 'z': toggle(g.set.safetyEnabled, "Safety zone"); break;
        case 't': teachRecord(); break;
        case 'y': teachPlay(); break;
        case 'u': g.teach.waypoints.clear(); g.teach.playing = false; showMessage("Teach: cleared", 1.5f); break;
        case 'w': toggle(g.set.wireframe, "Wireframe"); break;
        case 'b': toggle(g.set.bounds, "Bounding volumes"); break;
        case 'a': toggle(g.set.axes, "Joint axes"); break;
        case 'd': toggle(g.set.depthTest, "Depth buffer (z-buffer)"); break;
        case 'x': toggle(g.set.cullFace, "Back-face culling"); break;
        case 'p': toggle(g.set.showPick, "Pick buffer view"); break;
        case 'f': toggle(g.set.lighting, "Lighting"); break;
        case 'n': toggle(g.set.useLists, "Display lists"); break;
        case 's': toggle(g.set.stress, "Stress test (225 pyramids)"); break;
        case '-': g.mmHalf = std::fmin(30.0f, g.mmHalf * 1.15f); break;
        case '=': case '+': g.mmHalf = std::fmax(1.5f, g.mmHalf / 1.15f); break;
        default: break;
    }
    glutPostRedisplay();
}

static void onSpecial(int key, int, int) {
    bool shift = (glutGetModifiers() & GLUT_ACTIVE_SHIFT) != 0;
    if (shift) {                         // pan the mini-map world window
        float s = g.mmHalf * 0.1f;
        if (key == GLUT_KEY_LEFT) g.mmCx -= s;
        if (key == GLUT_KEY_RIGHT) g.mmCx += s;
        if (key == GLUT_KEY_UP) g.mmCz -= s;
        if (key == GLUT_KEY_DOWN) g.mmCz += s;
        glutPostRedisplay();
        return;
    }
    int j = g.arm.selectedJoint;
    float step = (j == J_GRIPPER) ? 0.05f : 2.0f;
    switch (key) {
        case GLUT_KEY_LEFT:  stepJoint(j, -step); break;
        case GLUT_KEY_RIGHT: stepJoint(j, +step); break;
        case GLUT_KEY_UP:    g.arm.selectedJoint = (j + J_COUNT - 1) % J_COUNT; break;
        case GLUT_KEY_DOWN:  g.arm.selectedJoint = (j + 1) % J_COUNT; break;
        case GLUT_KEY_PAGE_UP: g.cam.dist = std::fmax(4.0f, g.cam.dist - 0.8f); break;
        case GLUT_KEY_PAGE_DOWN: g.cam.dist = std::fmin(40.0f, g.cam.dist + 0.8f); break;
        default: break;
    }
    glutPostRedisplay();
}

// ------------------------------------------------------------------ mouse
static int g_downX = 0, g_downY = 0, g_lastX = 0, g_lastY = 0;
static bool g_leftDown = false, g_dragging = false;

static void handlePick(int x, int y) {
    if (g.set.stereo) { showMessage("Picking is disabled in stereo view", 1.5f); return; }
    PickResult r = pickAt(x, y);
    int nParts = (int)g.parts.size();
    if (r.id >= 1 && r.id <= nParts) {
        int idx = r.id - 1;
        const Part& p = g.parts[idx];
        if (g.arm.mode == MODE_IK) {
            if (g.arm.halted) { showMessage("Safety stop active"); return; }
            commandIK(g, partGraspPoint(p) + Vec3(0, 0.02f, 0));
        } else {
            showMessage("Picked part " + std::to_string(r.id) + " (" + PALETTE[p.colorIndex].name +
                        ") - press M for IK mode to reach it", 2.5f);
        }
    } else if ((r.id == Scene::PICK_TABLE || r.id == Scene::PICK_FLOOR) && r.hasPoint) {
        if (g.arm.mode == MODE_IK) {
            if (g.arm.halted) { showMessage("Safety stop active"); return; }
            Vec3 t = r.point;
            t.y = (r.id == Scene::PICK_TABLE ? 0.0f : FLOOR_Y) + 0.3f;   // hover just above the surface
            commandIK(g, t);
        } else {
            char buf[96];
            std::snprintf(buf, sizeof(buf), "Clicked %s at (%.2f, %.2f, %.2f)",
                          r.id == Scene::PICK_TABLE ? "table" : "floor", r.point.x, r.point.y, r.point.z);
            showMessage(buf, 2.0f);
        }
    }
}

static void onMouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        if (state == GLUT_DOWN) {
            g_leftDown = true; g_dragging = false;
            g_downX = g_lastX = x; g_downY = g_lastY = y;
        } else {
            g_leftDown = false;
            if (!g_dragging && !Hud::insideMiniMap(x, y)) handlePick(x, y);
        }
    }
    // wheel (buttons 3/4 on GLUT implementations without glutMouseWheelFunc)
    if ((button == 3 || button == 4) && state == GLUT_DOWN) {
        float f = button == 3 ? 0.9f : 1.1f;
        if (g.cam.mode == CAM_TOP) g.cam.topHalf = clampf(g.cam.topHalf * f, 2.0f, 30.0f);
        else g.cam.dist = clampf(g.cam.dist * f, 4.0f, 40.0f);
    }
    glutPostRedisplay();
}

static void onMotion(int x, int y) {
    if (!g_leftDown) return;
    if (std::abs(x - g_downX) + std::abs(y - g_downY) > 4) g_dragging = true;
    if (!g_dragging) return;
    int dx = x - g_lastX, dy = y - g_lastY;
    g_lastX = x; g_lastY = y;
    if (g.cam.mode == CAM_TOP) {
        float s = 2 * g.cam.topHalf / g.winH;
        g.cam.center.x -= dx * s;
        g.cam.center.z -= dy * s;
    } else {
        g.cam.yaw += dx * 0.4f;
        g.cam.pitch = clampf(g.cam.pitch + dy * 0.3f, -10.0f, 89.0f);
    }
    glutPostRedisplay();
}

static void onWheel(int, int dir, int, int) {
    float f = dir > 0 ? 0.9f : 1.1f;
    if (g.cam.mode == CAM_TOP) g.cam.topHalf = clampf(g.cam.topHalf * f, 2.0f, 30.0f);
    else g.cam.dist = clampf(g.cam.dist * f, 4.0f, 40.0f);
    glutPostRedisplay();
}

// ------------------------------------------------------------------ window events
static void onReshape(int w, int h) {
    g.winW = w > 1 ? w : 1;
    g.winH = h > 1 ? h : 1;
    glViewport(0, 0, g.winW, g.winH);     // aspect ratio is recomputed every frame
    glutPostRedisplay();
}

// ------------------------------------------------------------------ menus
enum MenuItem {
    MI_MANUAL = 1, MI_IK, MI_AUTO_START, MI_AUTO_PAUSE,
    MI_CAM_ORBIT, MI_CAM_TOP, MI_CAM_GRIP,
    MI_DEPTH, MI_CULL, MI_WIRE, MI_BOUNDS, MI_AXES, MI_LIGHT, MI_LISTS, MI_PICKVIEW, MI_STEREO, MI_STRESS,
    MI_HELP, MI_SOUND,
    MI_CLIP_CS, MI_CLIP_LB, MI_LINE_BRES, MI_LINE_DDA,
    MI_SIER0, MI_SIER1, MI_SIER2, MI_SIER3, MI_SIER4,
    MI_SAFETY, MI_PATROL, MI_ELBOW,
    MI_TEACH_REC, MI_TEACH_PLAY, MI_TEACH_CLEAR,
    MI_RESET, MI_QUIT
};

static int g_displayMenu = 0, g_safetyMenu = 0;

static std::string label(const char* name, bool on) { return std::string(on ? "[x] " : "[ ] ") + name; }

void Input::refreshMenuLabels() {
    if (!g_displayMenu) return;
    int cur = glutGetMenu();
    glutSetMenu(g_displayMenu);
    glutChangeToMenuEntry(1, label("Depth buffer (z-buffer)", g.set.depthTest).c_str(), MI_DEPTH);
    glutChangeToMenuEntry(2, label("Back-face culling", g.set.cullFace).c_str(), MI_CULL);
    glutChangeToMenuEntry(3, label("Wireframe", g.set.wireframe).c_str(), MI_WIRE);
    glutChangeToMenuEntry(4, label("Bounding volumes", g.set.bounds).c_str(), MI_BOUNDS);
    glutChangeToMenuEntry(5, label("Joint axes", g.set.axes).c_str(), MI_AXES);
    glutChangeToMenuEntry(6, label("Lighting", g.set.lighting).c_str(), MI_LIGHT);
    glutChangeToMenuEntry(7, label("Display lists", g.set.useLists).c_str(), MI_LISTS);
    glutChangeToMenuEntry(8, label("Show pick buffer", g.set.showPick).c_str(), MI_PICKVIEW);
    glutChangeToMenuEntry(9, label("Stereo VR view", g.set.stereo).c_str(), MI_STEREO);
    glutChangeToMenuEntry(10, label("Stress test", g.set.stress).c_str(), MI_STRESS);
    glutChangeToMenuEntry(11, label("Help overlay", g.set.help).c_str(), MI_HELP);
    glutChangeToMenuEntry(12, label("Collision sound", g.set.sound).c_str(), MI_SOUND);
    glutSetMenu(g_safetyMenu);
    glutChangeToMenuEntry(1, label("Safety zone enabled", g.set.safetyEnabled).c_str(), MI_SAFETY);
    glutChangeToMenuEntry(2, label("Operator patrol", g.op.patrol).c_str(), MI_PATROL);
    if (cur) glutSetMenu(cur);
}

static void onMenu(int item) {
    switch (item) {
        case MI_MANUAL: enterManual(); g.arm.mode = MODE_MANUAL; break;
        case MI_IK: g.fsm.running = false; g.arm.mode = MODE_IK; showMessage("IK mode: click a part or the table", 2.0f); break;
        case MI_AUTO_START: behaviourStart(g); break;
        case MI_AUTO_PAUSE: behaviourStop(g); break;
        case MI_CAM_ORBIT: g.cam.mode = CAM_ORBIT; break;
        case MI_CAM_TOP: g.cam.mode = CAM_TOP; break;
        case MI_CAM_GRIP: g.cam.mode = CAM_GRIPPER; break;
        case MI_DEPTH: toggle(g.set.depthTest, "Depth buffer (z-buffer)"); break;
        case MI_CULL: toggle(g.set.cullFace, "Back-face culling"); break;
        case MI_WIRE: toggle(g.set.wireframe, "Wireframe"); break;
        case MI_BOUNDS: toggle(g.set.bounds, "Bounding volumes"); break;
        case MI_AXES: toggle(g.set.axes, "Joint axes"); break;
        case MI_LIGHT: toggle(g.set.lighting, "Lighting"); break;
        case MI_LISTS: toggle(g.set.useLists, "Display lists"); break;
        case MI_PICKVIEW: toggle(g.set.showPick, "Pick buffer view"); break;
        case MI_STEREO: toggle(g.set.stereo, "Stereo (VR) view"); break;
        case MI_STRESS: toggle(g.set.stress, "Stress test"); break;
        case MI_HELP: toggle(g.set.help, "Help overlay"); break;
        case MI_SOUND: toggle(g.set.sound, "Collision sound"); break;
        case MI_CLIP_CS: g.set.clipAlgo = 0; showMessage("Line clipping: Cohen-Sutherland", 1.5f); break;
        case MI_CLIP_LB: g.set.clipAlgo = 1; showMessage("Line clipping: Liang-Barsky", 1.5f); break;
        case MI_LINE_BRES: g.set.lineAlgo = 0; showMessage("Line drawing: Bresenham", 1.5f); break;
        case MI_LINE_DDA: g.set.lineAlgo = 1; showMessage("Line drawing: DDA", 1.5f); break;
        case MI_SIER0: case MI_SIER1: case MI_SIER2: case MI_SIER3: case MI_SIER4:
            setSierpinski(item - MI_SIER0); break;
        case MI_SAFETY: toggle(g.set.safetyEnabled, "Safety zone"); break;
        case MI_PATROL: toggle(g.op.patrol, "Operator patrol"); break;
        case MI_ELBOW: g.arm.elbowUp = !g.arm.elbowUp;
            showMessage(g.arm.elbowUp ? "IK: elbow-up" : "IK: elbow-down", 1.5f); break;
        case MI_TEACH_REC: teachRecord(); break;
        case MI_TEACH_PLAY: teachPlay(); break;
        case MI_TEACH_CLEAR: g.teach.waypoints.clear(); break;
        case MI_RESET: doReset(); break;
        case MI_QUIT: glutLeaveMainLoop(); return;
    }
    glutPostRedisplay();
}

void Input::createMenus() {
    int mode = glutCreateMenu(onMenu);
    glutAddMenuEntry("Manual (keyboard)", MI_MANUAL);
    glutAddMenuEntry("IK (click target)", MI_IK);
    glutAddMenuEntry("Auto: start pick-and-place", MI_AUTO_START);
    glutAddMenuEntry("Auto: pause", MI_AUTO_PAUSE);

    int cam = glutCreateMenu(onMenu);
    glutAddMenuEntry("Orbit - perspective", MI_CAM_ORBIT);
    glutAddMenuEntry("Top - orthographic", MI_CAM_TOP);
    glutAddMenuEntry("Gripper cam - first person", MI_CAM_GRIP);

    g_displayMenu = glutCreateMenu(onMenu);
    for (int i = 0; i < 12; ++i) glutAddMenuEntry("-", MI_DEPTH + i);   // labels filled below

    int clip = glutCreateMenu(onMenu);
    glutAddMenuEntry("Cohen-Sutherland", MI_CLIP_CS);
    glutAddMenuEntry("Liang-Barsky", MI_CLIP_LB);

    int line = glutCreateMenu(onMenu);
    glutAddMenuEntry("Bresenham", MI_LINE_BRES);
    glutAddMenuEntry("DDA", MI_LINE_DDA);

    int sier = glutCreateMenu(onMenu);
    glutAddMenuEntry("Depth 0 (1 tetrahedron)", MI_SIER0);
    glutAddMenuEntry("Depth 1 (4)", MI_SIER1);
    glutAddMenuEntry("Depth 2 (16)", MI_SIER2);
    glutAddMenuEntry("Depth 3 (64)", MI_SIER3);
    glutAddMenuEntry("Depth 4 (256)", MI_SIER4);

    g_safetyMenu = glutCreateMenu(onMenu);
    glutAddMenuEntry("-", MI_SAFETY);
    glutAddMenuEntry("-", MI_PATROL);

    int teach = glutCreateMenu(onMenu);
    glutAddMenuEntry("Record waypoint (T)", MI_TEACH_REC);
    glutAddMenuEntry("Play waypoints (Y)", MI_TEACH_PLAY);
    glutAddMenuEntry("Clear waypoints (U)", MI_TEACH_CLEAR);

    glutCreateMenu(onMenu);
    glutAddSubMenu("Mode", mode);
    glutAddSubMenu("Camera / projection", cam);
    glutAddSubMenu("Display toggles", g_displayMenu);
    glutAddSubMenu("Mini-map line clipping", clip);
    glutAddSubMenu("Mini-map line drawing", line);
    glutAddSubMenu("Sierpinski part depth", sier);
    glutAddSubMenu("Safety", g_safetyMenu);
    glutAddSubMenu("Teach mode", teach);
    glutAddMenuEntry("Toggle elbow up/down", MI_ELBOW);
    glutAddMenuEntry("Reset cell", MI_RESET);
    glutAddMenuEntry("Quit", MI_QUIT);
    glutAttachMenu(GLUT_RIGHT_BUTTON);

    refreshMenuLabels();
}

void Input::registerCallbacks() {
    glutKeyboardFunc(onKeyboard);
    glutSpecialFunc(onSpecial);
    glutMouseFunc(onMouse);
    glutMotionFunc(onMotion);
    glutMouseWheelFunc(onWheel);
    glutReshapeFunc(onReshape);
}
