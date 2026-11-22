// Scene.cpp
#include "Scene.h"
#include "Primitives.h"
#include "Sierpinski.h"
#include "../GLInclude.h"
#include "../App.h"
#include "../Simulation.h"
#include <vector>
#include <algorithm>
#include <cmath>

const PaletteColour PALETTE[3] = {
    {0.90f, 0.18f, 0.18f, "Red"},
    {0.15f, 0.75f, 0.30f, "Green"},
    {0.20f, 0.45f, 0.95f, "Blue"},
};

static GLuint g_listBase = 0;

// ------------------------------------------------------------ geometry builders
static void buildFloor() {
    glPushAttrib(GL_ENABLE_BIT);       // restores the lighting state afterwards
    glDisable(GL_LIGHTING);
    glColor3f(0.32f, 0.34f, 0.38f);
    drawGrid(12.0f, 1.0f, FLOOR_Y + 0.001f);
    glPopAttrib();
    // solid floor slab just under the grid
    glColor3f(0.20f, 0.21f, 0.24f);
    glPushMatrix();
    glTranslatef(0, FLOOR_Y - 0.05f, 0);
    drawBox(24.0f, 0.1f, 24.0f);
    glPopMatrix();
}

static void buildTable() {
    glColor3f(0.62f, 0.64f, 0.68f);
    glPushMatrix();
    glTranslatef(0, -0.1f, 0);
    drawBox(TABLE_HALF * 2, 0.2f, TABLE_HALF * 2);
    glPopMatrix();
    glColor3f(0.35f, 0.36f, 0.40f);
    float legH = -FLOOR_Y - 0.2f;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sz = -1; sz <= 1; sz += 2) {
            glPushMatrix();
            glTranslatef(sx * (TABLE_HALF - 0.3f), -0.2f - legH / 2, sz * (TABLE_HALF - 0.3f));
            drawBox(0.3f, legH, 0.3f);
            glPopMatrix();
        }
    // painted lines on the table top (GL_LINES)
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glColor3f(0.95f, 0.8f, 0.1f);
    glBegin(GL_LINE_LOOP);
    float e = TABLE_HALF - 0.1f;
    glVertex3f(-e, 0.002f, -e); glVertex3f(e, 0.002f, -e);
    glVertex3f(e, 0.002f, e);   glVertex3f(-e, 0.002f, e);
    glEnd();
    glPopAttrib();
}

static void buildBase() {
    drawCylinderY(0.12f, 0.75f, 32);          // floor plate
    glPushMatrix();
    glTranslatef(0, 0.12f, 0);
    drawCylinderY(g.arm.baseH - 0.12f, 0.35f, 28);
    glPopMatrix();
}

static void buildLink1() { drawCylinderX(g.arm.L1, 0.14f); }
static void buildLink2() { drawCylinderX(g.arm.L2, 0.11f); }
static void buildLink3() { drawCylinderX(g.arm.L3 - 0.25f, 0.08f); }

static void buildBin() {
    // open box: floor + 4 walls, 0.84 x 0.4 x 0.84
    const float w = 0.84f, h = 0.4f, t = 0.04f;
    glPushMatrix(); glTranslatef(0, t / 2, 0); drawBox(w, t, w); glPopMatrix();
    glPushMatrix(); glTranslatef(0, h / 2, w / 2 - t / 2); drawBox(w, h, t); glPopMatrix();
    glPushMatrix(); glTranslatef(0, h / 2, -w / 2 + t / 2); drawBox(w, h, t); glPopMatrix();
    glPushMatrix(); glTranslatef(w / 2 - t / 2, h / 2, 0); drawBox(t, h, w); glPopMatrix();
    glPushMatrix(); glTranslatef(-w / 2 + t / 2, h / 2, 0); drawBox(t, h, w); glPopMatrix();
}

static void buildPart() { drawSierpinski(PART_SIZE, g.set.sierpinskiDepth); }

static void buildOperator() {
    // legs
    glColor3f(0.15f, 0.2f, 0.35f);
    for (int s = -1; s <= 1; s += 2) {
        glPushMatrix(); glTranslatef(s * 0.12f, 0.75f, 0); drawBox(0.18f, 1.5f, 0.22f); glPopMatrix();
    }
    // body (hi-vis vest)
    glColor3f(1.0f, 0.75f, 0.05f);
    glPushMatrix(); glTranslatef(0, 2.2f, 0); drawBox(0.56f, 1.4f, 0.32f); glPopMatrix();
    // arms
    glColor3f(0.9f, 0.7f, 0.55f);
    for (int s = -1; s <= 1; s += 2) {
        glPushMatrix(); glTranslatef(s * 0.36f, 2.15f, 0); drawBox(0.14f, 1.2f, 0.16f); glPopMatrix();
    }
    // head + helmet
    glPushMatrix(); glTranslatef(0, 3.12f, 0); drawSphere(0.2f); glPopMatrix();
    glColor3f(1.0f, 1.0f, 1.0f);
    glPushMatrix(); glTranslatef(0, 3.22f, 0); drawCylinderY(0.08f, 0.24f); glPopMatrix();
}

static void buildCube() { drawBox(1, 1, 1); }

typedef void (*Builder)();
static Builder g_builders[DL_COUNT] = {buildFloor, buildTable, buildBase, buildLink1, buildLink2,
                                       buildLink3, buildBin, buildPart, buildOperator, buildCube};

// ------------------------------------------------------------ display lists (Unit I)
void Scene::init() {
    g_listBase = glGenLists(DL_COUNT);
    for (int i = 0; i < DL_COUNT; ++i) {
        glNewList(g_listBase + i, GL_COMPILE);   // record the commands once...
        g_builders[i]();
        glEndList();
    }
}

void Scene::rebuildPartList() {
    glNewList(g_listBase + DL_PART, GL_COMPILE);
    buildPart();
    glEndList();
}

void Scene::callList(int id) {
    if (g.set.useLists) glCallList(g_listBase + id);   // ...and replay them every frame
    else g_builders[id]();                             // or re-issue every vertex (immediate mode)
}

// ------------------------------------------------------------ world
static void colour(int idx, float k = 1.0f) {
    glColor3f(PALETTE[idx].r * k, PALETTE[idx].g * k, PALETTE[idx].b * k);
}

void Scene::drawWorld() {
    callList(DL_FLOOR);
    callList(DL_TABLE);

    // bins
    for (const Bin& b : g.bins) {
        glPushMatrix();
        glTranslatef(b.pos.x, b.pos.y, b.pos.z);
        colour(b.colorIndex, 0.75f);
        callList(DL_BIN);
        glPopMatrix();
    }
    // obstacle pillars (unit cube scaled to the AABB)
    for (const AABB& o : g.obstacles) {
        glPushMatrix();
        glTranslatef(o.c.x, o.c.y, o.c.z);
        glScalef(o.h.x * 2, o.h.y * 2, o.h.z * 2);
        glColor3f(0.45f, 0.30f, 0.20f);
        callList(DL_CUBE);
        glPopMatrix();
        // hazard stripe on top
        glPushMatrix();
        glTranslatef(o.c.x, o.c.y + o.h.y + 0.01f, o.c.z);
        glColor3f(0.95f, 0.8f, 0.1f);
        drawBox(o.h.x * 2, 0.02f, o.h.z * 2);
        glPopMatrix();
    }
    // parts
    for (size_t i = 0; i < g.parts.size(); ++i) {
        const Part& p = g.parts[i];
        glPushMatrix();
        glTranslatef(p.pos.x, p.pos.y, p.pos.z);
        colour(p.colorIndex, p.unreachable ? 0.4f : 1.0f);
        callList(DL_PART);
        glPopMatrix();
    }
    // operator
    glPushMatrix();
    glTranslatef(g.op.pos.x, g.op.pos.y, g.op.pos.z);
    callList(DL_OPERATOR);
    glPopMatrix();

    if (g.set.stress) drawStress();
}

void Scene::drawStress() {
    for (int i = -7; i <= 7; ++i)
        for (int j = -7; j <= 7; ++j) {
            glPushMatrix();
            glTranslatef(i * 1.4f, FLOOR_Y, j * 1.4f + 0.0f);
            colour(((i + j) % 3 + 3) % 3, 0.6f);
            callList(DL_PART);
            glPopMatrix();
        }
}

// ------------------------------------------------------------ colour-ID pick render
// The real ID goes in the red channel (1..N parts, 200 table, 201 floor).
// In "show pick buffer" mode the IDs are mapped to false colours so they are visible.
static bool g_visualizePick = false;
static void pickColour(int id) {
    if (!g_visualizePick) { glColor3ub((GLubyte)id, 0, 0); return; }
    if (id == 0) glColor3ub(0, 0, 0);
    else if (id == Scene::PICK_TABLE) glColor3ub(90, 20, 20);
    else if (id == Scene::PICK_FLOOR) glColor3ub(20, 20, 70);
    else glColor3ub((GLubyte)(60 + id * 67 % 196), (GLubyte)(60 + id * 113 % 196), (GLubyte)(60 + id * 29 % 196));
}

void Scene::drawPickScene(bool visualize) {
    g_visualizePick = visualize;
    // Flat colours only: no lighting, no dithering, no smoothing, so the ID read back is exact.
    glDisable(GL_LIGHTING);
    glDisable(GL_DITHER);
    glShadeModel(GL_FLAT);

    pickColour(PICK_FLOOR);
    glPushMatrix(); glTranslatef(0, FLOOR_Y - 0.05f, 0); drawBox(24, 0.1f, 24); glPopMatrix();
    pickColour(PICK_TABLE);
    glPushMatrix(); glTranslatef(0, -0.1f, 0); drawBox(TABLE_HALF * 2, 0.2f, TABLE_HALF * 2); glPopMatrix();

    pickColour(0);  // occluders: bins, pillars, arm, operator
    for (const Bin& b : g.bins) { glPushMatrix(); glTranslatef(b.pos.x, b.pos.y, b.pos.z); buildBin(); glPopMatrix(); }
    for (const AABB& o : g.obstacles) {
        glPushMatrix(); glTranslatef(o.c.x, o.c.y, o.c.z); drawBox(o.h.x * 2, o.h.y * 2, o.h.z * 2); glPopMatrix();
    }
    glPushMatrix(); glTranslatef(g.op.pos.x, g.op.pos.y, g.op.pos.z); buildOperator(); glPopMatrix();

    for (size_t i = 0; i < g.parts.size(); ++i) {
        const Part& p = g.parts[i];
        // slightly enlarged pick proxy so small pyramids are easy to click
        pickColour((int)i + 1);
        glPushMatrix();
        glTranslatef(p.pos.x, p.pos.y, p.pos.z);
        drawSierpinski(PART_SIZE * 1.1f, 0);
        glPopMatrix();
    }
    glShadeModel(GL_SMOOTH);
    glEnable(GL_DITHER);
}

// ------------------------------------------------------------ translucent safety zone
// Painter's algorithm (depth sort): the cylinder's faces are sorted far-to-near from
// the eye every frame and drawn with blending and depth writes OFF.
void Scene::drawSafetyZone(const Vec3& eye) {
    if (!g.set.safetyEnabled) return;
    const int N = 40;
    const float R = g.safetyRadius, y0 = FLOOR_Y + 0.01f, y1 = 1.6f;
    struct Face { Vec3 v[4]; float dist; };
    std::vector<Face> faces;
    faces.reserve(N);
    for (int i = 0; i < N; ++i) {
        float a0 = 2 * PI_F * i / N, a1 = 2 * PI_F * (i + 1) / N;
        Face f;
        f.v[0] = Vec3(R * std::cos(a0), y0, R * std::sin(a0));
        f.v[1] = Vec3(R * std::cos(a1), y0, R * std::sin(a1));
        f.v[2] = Vec3(R * std::cos(a1), y1, R * std::sin(a1));
        f.v[3] = Vec3(R * std::cos(a0), y1, R * std::sin(a0));
        Vec3 c = (f.v[0] + f.v[1] + f.v[2] + f.v[3]) / 4.0f;
        f.dist = (c - eye).length();
        faces.push_back(f);
    }
    std::sort(faces.begin(), faces.end(), [](const Face& a, const Face& b) { return a.dist > b.dist; });

    bool flash = g.zoneBreached && (glutGet(GLUT_ELAPSED_TIME) / 250) % 2 == 0;
    glDisable(GL_LIGHTING);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    for (const Face& f : faces) {
        if (g.zoneBreached) glColor4f(1.0f, 0.1f, 0.1f, flash ? 0.35f : 0.18f);
        else glColor4f(1.0f, 0.85f, 0.1f, 0.10f);
        glBegin(GL_QUADS);
        for (int k = 0; k < 4; ++k) glVertex3f(f.v[k].x, f.v[k].y, f.v[k].z);
        glEnd();
    }
    // outline ring on the floor
    glColor4f(1.0f, g.zoneBreached ? 0.1f : 0.8f, 0.1f, 0.8f);
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < N; ++i) {
        float a = 2 * PI_F * i / N;
        glVertex3f(R * std::cos(a), y0, R * std::sin(a));
    }
    glEnd();
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    if (g.set.cullFace) glEnable(GL_CULL_FACE);
    if (g.set.lighting) glEnable(GL_LIGHTING);
}

// ------------------------------------------------------------ bounding volumes
static void wireCapsule(const Vec3& a, const Vec3& b, float r) {
    const int n = 6;
    for (int i = 0; i <= n; ++i) {
        Vec3 p = lerp(a, b, (float)i / n);
        glPushMatrix();
        glTranslatef(p.x, p.y, p.z);
        glutWireSphere(r, 10, 6);
        glPopMatrix();
    }
}

static void wireAABB(const AABB& b) {
    glPushMatrix();
    glTranslatef(b.c.x, b.c.y, b.c.z);
    drawWireBox(b.h.x * 2, b.h.y * 2, b.h.z * 2);
    glPopMatrix();
}

void Scene::drawBounds() {
    glDisable(GL_LIGHTING);
    const ArmPose& p = g.pose;
    glColor3f(g.arm.linkHit[0] ? 1.f : 0.2f, g.arm.linkHit[0] ? 0.f : 1.f, 1.f);
    wireCapsule(p.shoulder, p.elbow, 0.15f);
    glColor3f(g.arm.linkHit[1] ? 1.f : 0.2f, g.arm.linkHit[1] ? 0.f : 1.f, 1.f);
    wireCapsule(p.elbow, p.wrist, 0.12f);
    glColor3f(g.arm.linkHit[2] ? 1.f : 0.2f, g.arm.linkHit[2] ? 0.f : 1.f, 1.f);
    wireCapsule(p.wrist, p.tip, 0.09f);
    glColor3f(1.0f, 0.4f, 1.0f);
    for (const AABB& o : g.obstacles) wireAABB(o);
    for (const Bin& b : g.bins) wireAABB(binBox(b));
    glColor3f(g.zoneBreached ? 1.f : 1.f, g.zoneBreached ? 0.1f : 1.f, 0.1f);
    wireAABB(g.op.box());
    if (g.set.lighting) glEnable(GL_LIGHTING);
}

void Scene::drawTargetMarker() {
    if (!g.hasTarget) return;
    glDisable(GL_LIGHTING);
    glColor3f(0.1f, 1.0f, 1.0f);
    glLineWidth(2);
    const Vec3& t = g.target;
    float s = 0.18f;
    glBegin(GL_LINES);
    glVertex3f(t.x - s, t.y, t.z); glVertex3f(t.x + s, t.y, t.z);
    glVertex3f(t.x, t.y - s, t.z); glVertex3f(t.x, t.y + s, t.z);
    glVertex3f(t.x, t.y, t.z - s); glVertex3f(t.x, t.y, t.z + s);
    glEnd();
    // dashed drop line to the table
    glEnable(GL_LINE_STIPPLE);
    glLineStipple(2, 0x00FF);
    glBegin(GL_LINES);
    glVertex3f(t.x, t.y, t.z); glVertex3f(t.x, 0.0f, t.z);
    glEnd();
    glDisable(GL_LINE_STIPPLE);
    glLineWidth(1);
    if (g.set.lighting) glEnable(GL_LIGHTING);
}
