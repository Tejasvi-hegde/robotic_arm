// Hud.cpp - everything drawn in 2D on top of the 3D view.
//
// The MINI-MAP is the Unit III showcase. It is a top-down plan of the cell drawn
// pixel by pixel into a software Canvas with our own algorithms:
//   window-to-viewport mapping, DDA / Bresenham lines, midpoint circles,
//   boundary fill, flood fill, scan-line polygon fill,
//   Cohen-Sutherland / Liang-Barsky line clipping, Sutherland-Hodgman polygon clipping.
// Lines are first drawn in dim grey unclipped, then the clipped part in colour,
// so the effect of clipping against the inner window is visible.
#include "Hud.h"
#include "../GLInclude.h"
#include "../App.h"
#include "../Simulation.h"
#include "../scene/Scene.h"
#include "../raster/Raster2D.h"
#include "../raster/Clip2D.h"
#include <cstdio>
#include <cstdarg>
#include <cmath>
#include <algorithm>
#include <string>

// ------------------------------------------------------------------ text helpers
static void text(int x, int y, const std::string& s, float r = 0.92f, float gg = 0.93f, float b = 0.95f,
                 void* font = GLUT_BITMAP_8_BY_13) {
    glColor3f(r, gg, b);
    glRasterPos2i(x, y);
    glutBitmapString(font, (const unsigned char*)s.c_str());
}

static void panel(int x, int y, int w, int h, float a = 0.62f) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.05f, 0.06f, 0.08f, a);
    glBegin(GL_QUADS);
    glVertex2i(x, y); glVertex2i(x + w, y); glVertex2i(x + w, y + h); glVertex2i(x, y + h);
    glEnd();
    glDisable(GL_BLEND);
}

static std::string fmt(const char* f, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof(buf), f, ap);
    va_end(ap);
    return buf;
}

// ------------------------------------------------------------------ mini-map
static Canvas g_canvas(Hud::MM_SIZE, Hud::MM_SIZE);

struct ClipStats { int accepted = 0, rejected = 0, cut = 0; } g_clip;

static const Pixel BG = rgb(24, 27, 33);
static const int INSET = 18;

// World (x, z) -> map plane (u = x, v = -z so that "far" (-z) is up, like the top camera)
static Rect2 worldWindow() {
    return {g.mmCx - g.mmHalf, -g.mmCz - g.mmHalf, g.mmCx + g.mmHalf, -g.mmCz + g.mmHalf};
}
static Rect2 viewportRect() { return {0, 0, (float)Hud::MM_SIZE - 1, (float)Hud::MM_SIZE - 1}; }
static Rect2 clipRect() {
    return {(float)INSET, (float)INSET, (float)(Hud::MM_SIZE - 1 - INSET), (float)(Hud::MM_SIZE - 1 - INSET)};
}
static Pt2 toMap(float wx, float wz) {
    return windowToViewport({wx, -wz}, worldWindow(), viewportRect());
}
static float worldToPixels(float d) { return d * (Hud::MM_SIZE - 1) / (2 * g.mmHalf); }

static void rasterLine(int x0, int y0, int x1, int y1, Pixel c) {
    if (g.set.lineAlgo == 0) lineBresenham(g_canvas, x0, y0, x1, y1, c);
    else lineDDA(g_canvas, x0, y0, x1, y1, c);
}

// Draw a map-space segment: dim unclipped copy, then the clipped part in colour.
static void clippedLine(Pt2 a, Pt2 b, Pixel c, bool ghost = true) {
    if (ghost) rasterLine((int)std::lround(a.x), (int)std::lround(a.y), (int)std::lround(b.x),
                          (int)std::lround(b.y), rgb(c.r / 4 + 20, c.g / 4 + 20, c.b / 4 + 24));
    float x0 = a.x, y0 = a.y, x1 = b.x, y1 = b.y;
    bool ok = g.set.clipAlgo == 0 ? clipCohenSutherland(x0, y0, x1, y1, clipRect())
                                  : clipLiangBarsky(x0, y0, x1, y1, clipRect());
    if (!ok) { g_clip.rejected++; return; }
    bool changed = std::fabs(x0 - a.x) + std::fabs(y0 - a.y) + std::fabs(x1 - b.x) + std::fabs(y1 - b.y) > 0.01f;
    if (changed) g_clip.cut++; else g_clip.accepted++;
    rasterLine((int)std::lround(x0), (int)std::lround(y0), (int)std::lround(x1), (int)std::lround(y1), c);
}

static void worldRectOutline(float x0, float z0, float x1, float z1, Pixel c) {
    Pt2 p0 = toMap(x0, z0), p1 = toMap(x1, z0), p2 = toMap(x1, z1), p3 = toMap(x0, z1);
    clippedLine(p0, p1, c); clippedLine(p1, p2, c); clippedLine(p2, p3, c); clippedLine(p3, p0, c);
}

// Outline + boundary fill, only if the whole rectangle is inside the clip window
// (otherwise the fill could leak out through a clipped-away edge).
static void filledWorldRect(float x0, float z0, float x1, float z1, Pixel edge, Pixel fill) {
    worldRectOutline(x0, z0, x1, z1, edge);
    Pt2 a = toMap(x0, z0), b = toMap(x1, z1);
    Rect2 cr = clipRect();
    float minx = std::fmin(a.x, b.x), maxx = std::fmax(a.x, b.x);
    float miny = std::fmin(a.y, b.y), maxy = std::fmax(a.y, b.y);
    if (minx > cr.xmin && maxx < cr.xmax && miny > cr.ymin && maxy < cr.ymax && maxx - minx > 2 && maxy - miny > 2) {
        IRect area{(int)cr.xmin, (int)cr.ymin, (int)cr.xmax, (int)cr.ymax};
        boundaryFill4(g_canvas, (int)std::lround((minx + maxx) / 2), (int)std::lround((miny + maxy) / 2),
                      fill, edge, area);
    }
}

static void buildMiniMap() {
    g_clip = ClipStats();
    g_canvas.clear(BG);
    Rect2 cr = clipRect();
    IRect area{(int)cr.xmin + 1, (int)cr.ymin + 1, (int)cr.xmax - 1, (int)cr.ymax - 1};

    // 1. table outline
    worldRectOutline(-TABLE_HALF, -TABLE_HALF, TABLE_HALF, TABLE_HALF, rgb(150, 155, 165));

    // 2. reach envelope rings (midpoint circle): inner |L1-L2| and outer L1+L2
    Pt2 base = toMap(0, 0);
    int bx = (int)std::lround(base.x), by = (int)std::lround(base.y);
    float rIn = std::fabs(g.arm.L1 - g.arm.L2), rOut = g.arm.L1 + g.arm.L2;
    Pixel ring = rgb(70, 200, 110);
    circleMidpoint(g_canvas, bx, by, (int)std::lround(worldToPixels(rOut)), ring);
    circleMidpoint(g_canvas, bx, by, std::max(2, (int)std::lround(worldToPixels(rIn))), ring);

    // 3. obstacles and bins: outline + boundary fill
    for (const AABB& o : g.obstacles)
        filledWorldRect(o.c.x - o.h.x, o.c.z - o.h.z, o.c.x + o.h.x, o.c.z + o.h.z,
                        rgb(200, 130, 70), rgb(120, 75, 40));
    for (const Bin& b : g.bins) {
        const PaletteColour& pc = PALETTE[b.colorIndex];
        Pixel edge = rgb((int)(pc.r * 255), (int)(pc.g * 255), (int)(pc.b * 255));
        filledWorldRect(b.pos.x - 0.42f, b.pos.z - 0.42f, b.pos.x + 0.42f, b.pos.z + 0.42f,
                        edge, rgb(edge.r / 3, edge.g / 3, edge.b / 3));
    }

    // 4. flood fill the reachable annulus (seed between the two rings, on background)
    int rMid = (int)std::lround(worldToPixels((rIn + rOut) / 2));
    for (int k = 0; k < 16; ++k) {
        float ang = 2 * PI_F * k / 16;
        int sx = bx + (int)(rMid * std::cos(ang)), sy = by + (int)(rMid * std::sin(ang));
        if (area.contains(sx, sy) && g_canvas.get(sx, sy) == BG) {
            floodFill4(g_canvas, sx, sy, rgb(30, 52, 40), area);
            break;
        }
    }

    // 5. safety zone: 32-gon -> Sutherland-Hodgman clip -> scan-line fill (blended)
    if (g.set.safetyEnabled) {
        std::vector<Pt2> zone;
        for (int i = 0; i < 32; ++i) {
            float a = 2 * PI_F * i / 32;
            zone.push_back(toMap(g.safetyRadius * std::cos(a), g.safetyRadius * std::sin(a)));
        }
        std::vector<Pt2> clipped = clipSutherlandHodgman(zone, cr);
        Pixel zc = g.zoneBreached ? rgb(255, 40, 40) : rgb(255, 210, 40);
        scanlineFill(g_canvas, clipped, zc, g.zoneBreached ? 0.35f : 0.12f);
        for (size_t i = 0; i < zone.size(); ++i) clippedLine(zone[i], zone[(i + 1) % zone.size()], zc, false);
    }

    // 6. parts (small squares in their palette colour)
    for (const Part& p : g.parts) {
        Pt2 m = toMap(p.pos.x, p.pos.z);
        const PaletteColour& pc = PALETTE[p.colorIndex];
        Pixel c = rgb((int)(pc.r * 255), (int)(pc.g * 255), (int)(pc.b * 255));
        for (int dx = -2; dx <= 2; ++dx)
            for (int dy = -2; dy <= 2; ++dy)
                if (area.contains((int)m.x + dx, (int)m.y + dy)) g_canvas.set((int)m.x + dx, (int)m.y + dy, c);
    }

    // 7. operator footprint
    AABB ob = g.op.box();
    worldRectOutline(ob.c.x - ob.h.x, ob.c.z - ob.h.z, ob.c.x + ob.h.x, ob.c.z + ob.h.z,
                     g.zoneBreached ? rgb(255, 60, 60) : rgb(255, 230, 60));

    // 8. arm projected onto the floor
    const ArmPose& P = g.pose;
    Pixel armC = rgb(255, 160, 30);
    clippedLine(toMap(0, 0), toMap(P.elbow.x, P.elbow.z), armC);
    clippedLine(toMap(P.elbow.x, P.elbow.z), toMap(P.wrist.x, P.wrist.z), armC);
    clippedLine(toMap(P.wrist.x, P.wrist.z), toMap(P.tip.x, P.tip.z), rgb(230, 230, 240));

    // 9. planned path: fingertip -> IK target
    if (g.hasTarget) {
        Pt2 t = toMap(g.target.x, g.target.z);
        clippedLine(toMap(P.tip.x, P.tip.z), t, rgb(60, 230, 240));
        circleMidpoint(g_canvas, (int)std::lround(t.x), (int)std::lround(t.y), 4, rgb(60, 230, 240));
    }

    // 10. clip window border (Bresenham)
    Pixel w = rgb(235, 235, 240);
    int x0 = (int)cr.xmin, y0 = (int)cr.ymin, x1 = (int)cr.xmax, y1 = (int)cr.ymax;
    lineBresenham(g_canvas, x0, y0, x1, y0, w);
    lineBresenham(g_canvas, x1, y0, x1, y1, w);
    lineBresenham(g_canvas, x1, y1, x0, y1, w);
    lineBresenham(g_canvas, x0, y1, x0, y0, w);
}

bool Hud::insideMiniMap(int mx, int my) {
    int x0 = g.winW - MM_SIZE - MM_MARGIN;
    int yTop = MM_MARGIN + 18;                 // window y (top-down) of the map's top edge
    int myGL = g.winH - my;
    (void)yTop;
    return mx >= x0 && mx < x0 + MM_SIZE && myGL >= MM_MARGIN + 20 && myGL < MM_MARGIN + 20 + MM_SIZE;
}

static void drawMiniMap() {
    buildMiniMap();
    int x0 = g.winW - Hud::MM_SIZE - Hud::MM_MARGIN;
    int y0 = Hud::MM_MARGIN + 20;
    panel(x0 - 6, y0 - 24, Hud::MM_SIZE + 12, Hud::MM_SIZE + 46, 0.75f);
    glRasterPos2i(x0, y0);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glDrawPixels(Hud::MM_SIZE, Hud::MM_SIZE, GL_RGB, GL_UNSIGNED_BYTE, g_canvas.data());
    text(x0, y0 + Hud::MM_SIZE + 6, fmt("Mini-map  %s + %s", g.set.lineAlgo == 0 ? "Bresenham" : "DDA",
                                         g.set.clipAlgo == 0 ? "Cohen-Suth." : "Liang-Barsky"),
         0.75f, 0.9f, 1.0f);
    text(x0, y0 - 16, fmt("clip  in:%d cut:%d out:%d  -/= zoom", g_clip.accepted, g_clip.cut, g_clip.rejected),
         0.7f, 0.72f, 0.78f);
}

// ------------------------------------------------------------------ panels
static void drawStatusPanel() {
    const ArmState& a = g.arm;
    int x = 12, y = g.winH - 22, lh = 15;
    panel(4, g.winH - 352, 336, 348);
    text(x, y, "RoboCell - Robotic Arm Workcell Simulator", 1.0f, 0.75f, 0.25f, GLUT_BITMAP_HELVETICA_12);
    y -= lh + 4;
    static const char* modeNames[MODE_COUNT] = {"MANUAL (keys)", "IK (click target)", "AUTO (pick & place)"};
    text(x, y, fmt("Mode   : %s", modeNames[a.mode])); y -= lh;
    text(x, y, fmt("FSM    : %s%s", fsmStateName(g.fsm.state), g.fsm.running ? "" : "  [stopped]")); y -= lh;
    text(x, y, fmt("Camera : %s", cameraName(g.cam.mode))); y -= lh;
    text(x, y, fmt("Elbow  : %s     Approach: auto", a.elbowUp ? "UP" : "DOWN")); y -= lh + 4;

    for (int j = 0; j < J_COUNT; ++j) {
        bool sel = j == a.selectedJoint;
        std::string v = j == J_GRIPPER ? fmt("%5.2f (0..1)", a.theta[j]) : fmt("%7.1f deg", a.theta[j]);
        text(x, y, fmt("%s %d %-9s %s", sel ? ">" : " ", j + 1, JOINT_NAMES[j], v.c_str()),
             sel ? 1.0f : 0.85f, sel ? 0.85f : 0.86f, sel ? 0.3f : 0.9f);
        y -= lh;
    }
    y -= 4;
    const Vec3& t = g.pose.tip;
    text(x, y, fmt("Effector (FK): %6.2f %6.2f %6.2f", t.x, t.y, t.z), 0.6f, 1.0f, 0.9f); y -= lh;
    if (g.hasTarget) { text(x, y, fmt("IK target    : %6.2f %6.2f %6.2f", g.target.x, g.target.y, g.target.z), 0.4f, 0.95f, 1.0f); }
    y -= lh;
    int sorted = 0;
    for (const Part& p : g.parts) if (p.state == PART_BIN) sorted++;
    text(x, y, fmt("Parts sorted : %d / %d    Collisions: %d", sorted, (int)g.parts.size(), g.collisionCount)); y -= lh;
    text(x, y, fmt("FPS %.0f   Display lists: %s%s", g.fps, g.set.useLists ? "ON" : "OFF (immediate)",
                   g.set.stress ? "  +stress" : "")); y -= lh + 4;

    // status flags
    if (a.halted) text(x, y, "SAFETY STOP - operator in zone", 1.0f, 0.25f, 0.25f);
    else if (a.colliding) text(x, y, "COLLISION - motion blocked", 1.0f, 0.35f, 0.35f);
    else if (a.moving) text(x, y, "Moving...", 0.6f, 0.9f, 0.6f);
    else text(x, y, "Ready", 0.6f, 0.9f, 0.6f);
    y -= lh;

    // force bar (haptic feedback stand-in)
    y -= 6;
    text(x, y, "Contact force");
    float rgbc[3];
    forceColour(g.force, rgbc);
    float u = clampf(g.force.smooth / g.force.maxForce(), 0, 1);
    int bx = x + 112, bw = 200;
    glColor3f(0.25f, 0.26f, 0.3f);
    glBegin(GL_QUADS);
    glVertex2i(bx, y - 3); glVertex2i(bx + bw, y - 3); glVertex2i(bx + bw, y + 10); glVertex2i(bx, y + 10);
    glColor3f(rgbc[0], rgbc[1], rgbc[2]);
    glVertex2i(bx, y - 3); glVertex2i(bx + (int)(bw * u), y - 3);
    glVertex2i(bx + (int)(bw * u), y + 10); glVertex2i(bx, y + 10);
    glEnd();
    y -= lh;
    text(x, y, fmt("F=k*d = %.1f N   smoothed %.1f N", g.force.raw, g.force.smooth), 0.75f, 0.76f, 0.8f);
}

static void drawMatrixPanel() {
    int j = g.arm.selectedJoint;
    if (j > J_WRIST) j = 4;      // gripper -> tip frame
    const Mat4& W = g.pose.frame[j];
    const Mat4& L = g.pose.local[j];
    int w = 372, x = g.winW - w - 8, y = g.winH - 22, lh = 14;
    panel(x - 4, g.winH - 312, w + 8, 308);
    text(x, y, fmt("Joint %d (%s) - local transform", j + 1, j == 4 ? "Tip" : JOINT_NAMES[j]),
         1.0f, 0.75f, 0.25f, GLUT_BITMAP_HELVETICA_12);
    y -= lh + 2;
    for (int r = 0; r < 4; ++r) {
        text(x, y, fmt("[%7.3f %7.3f %7.3f %7.3f]", L.at(r, 0), L.at(r, 1), L.at(r, 2), L.at(r, 3)), 0.8f, 0.9f, 1.0f);
        y -= lh;
    }
    y -= 6;
    text(x, y, "Composite world matrix (root -> joint)", 1.0f, 0.75f, 0.25f, GLUT_BITMAP_HELVETICA_12);
    y -= lh + 2;
    for (int r = 0; r < 4; ++r) {
        text(x, y, fmt("[%7.3f %7.3f %7.3f %7.3f]", W.at(r, 0), W.at(r, 1), W.at(r, 2), W.at(r, 3)), 0.6f, 1.0f, 0.8f);
        y -= lh;
    }
    y -= 6;
    // 2D homogeneous rotation of the base seen from the top (Unit III composite 2D transforms)
    float c = std::cos(deg2rad(g.arm.theta[J_BASE])), s = std::sin(deg2rad(g.arm.theta[J_BASE]));
    text(x, y, "Top view 2D base rotation (3x3 homogeneous)", 1.0f, 0.75f, 0.25f, GLUT_BITMAP_HELVETICA_12);
    y -= lh + 2;
    text(x, y, fmt("[%6.3f %6.3f  0.000]", c, -s), 0.85f, 0.85f, 0.95f); y -= lh;
    text(x, y, fmt("[%6.3f %6.3f  0.000]", s, c), 0.85f, 0.85f, 0.95f); y -= lh;
    text(x, y, "[ 0.000  0.000  1.000]", 0.85f, 0.85f, 0.95f); y -= lh + 6;
    text(x, y, "T = Ry(t0) T(0,h,0) Rz(t1) T(L1,0,0) Rz(t2)", 0.7f, 0.72f, 0.78f); y -= lh;
    text(x, y, "    T(L2,0,0) Rz(t3) T(L3,0,0)", 0.7f, 0.72f, 0.78f);
}

static void drawHelp() {
    static const char* lines[] = {
        "KEYS (H hides)                      MOUSE",
        "M mode  Space auto start/pause      L-drag orbit / L-click pick",
        "1-5 joint  <-/-> rotate  Up/Dn sel  wheel zoom   R-click MENU",
        "G gripper  E elbow up/down  R reset",
        "C camera  V stereo  [ ] near plane  I J K L move operator",
        "O operator patrol  Z safety zone    T record  Y play  U clear teach",
        "W wireframe  B bounds  A axes  D depth  X cull  P pick view",
        "F lighting  N display lists  S stress  -/= map zoom  Shift+arrows pan",
    };
    int n = sizeof(lines) / sizeof(lines[0]);
    int x = 12, lh = 14, h = n * lh + 12;
    panel(4, 6, 540, h);
    int y = 6 + h - 16;
    for (int i = 0; i < n; ++i) { text(x, y, lines[i], 0.82f, 0.85f, 0.9f); y -= lh; }
}

static void drawMessage() {
    if (g.messageTime <= 0 || g.message.empty()) return;
    int w = (int)g.message.size() * 9 + 24;
    int x = (g.winW - w) / 2, y = g.winH - 44;
    panel(x, y - 8, w, 28, 0.8f);
    bool warn = g.message.find("SAFETY") != std::string::npos || g.message.find("Collision") != std::string::npos ||
                g.message.find("unreachable") != std::string::npos || g.message.find("blocked") != std::string::npos;
    text(x + 12, y + 2, g.message, warn ? 1.0f : 0.8f, warn ? 0.4f : 1.0f, warn ? 0.4f : 0.85f, GLUT_BITMAP_9_BY_15);
}

void Hud::draw() {
    glViewport(0, 0, g.winW, g.winH);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, g.winW, 0, g.winH);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_CULL_FACE);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    drawStatusPanel();
    drawMatrixPanel();
    drawMiniMap();
    if (g.set.help) drawHelp();
    drawMessage();
    if (g.set.stereo) text(g.winW / 2 - 70, 12, "STEREO  L-eye | R-eye", 0.9f, 0.9f, 0.4f);
    if (g.set.showPick) text(g.winW / 2 - 90, 28, "PICK BUFFER VIEW (colour IDs)", 1.0f, 0.5f, 0.5f);
}
