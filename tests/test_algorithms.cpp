// test_algorithms.cpp - checks every algorithm against a known answer.
// Run: RoboCellTests   (prints PASS/FAIL per check, exit code 0 when all pass)
#include "App.h"
#include "Simulation.h"
#include "arm/IK.h"
#include "raster/Raster2D.h"
#include "raster/Clip2D.h"
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <cstdlib>

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, name) do { if (cond) { ++g_pass; std::printf("PASS  %s\n", name); } \
                               else { ++g_fail; std::printf("FAIL  %s\n", name); } } while (0)

static void testFK() {
    ArmState s;
    float th[J_COUNT] = {0, 0, 0, 0, 1};
    ArmPose p = computeFK(s, th);
    Vec3 e(s.L1 + s.L2 + s.L3, s.baseH, 0);
    CHECK((p.tip - e).length() < 1e-4f, "FK: all joints 0 -> tip at (L1+L2+L3, h, 0)");
    float th2[J_COUNT] = {90, 90, 0, 0, 1};
    p = computeFK(s, th2);
    CHECK((p.tip - Vec3(0, s.baseH + s.L1 + s.L2 + s.L3, 0)).length() < 1e-4f, "FK: shoulder 90 -> arm straight up");
    float th3[J_COUNT] = {90, 0, 0, 0, 1};
    p = computeFK(s, th3);
    CHECK((p.tip - Vec3(0, s.baseH, -(s.L1 + s.L2 + s.L3))).length() < 1e-4f, "FK: base +90 turns +X to -Z");
}

static void testIK() {
    ArmState s;
    std::srand(7);
    int ok = 0, tried = 0;
    float worst = 0;
    for (int n = 0; n < 2000 && tried < 300; ++n) {
        float th[J_COUNT];
        th[J_BASE] = -170 + std::rand() % 340;
        th[J_SHOULDER] = 5 + std::rand() % 140;
        th[J_ELBOW] = -(10 + std::rand() % 150);     // elbow-up family
        th[J_WRIST] = 0;
        th[J_GRIPPER] = 1;
        ArmPose p = computeFK(s, th);
        float phi = p.pitch;
        while (phi > 180) phi -= 360;
        while (phi < -180) phi += 360;
        th[J_WRIST] = 0;
        float sol[J_COUNT];
        if (solveIK(s, p.tip, phi, true, sol) != IK_OK) continue;
        ++tried;
        ArmPose q = computeFK(s, sol);
        float err = (q.tip - p.tip).length();
        worst = std::fmax(worst, err);
        if (err < 1e-3f) ++ok;
    }
    char buf[128];
    std::snprintf(buf, sizeof(buf), "IK round trip: %d/%d random poses reproduced (worst error %.2e)", ok, tried, worst);
    CHECK(tried > 100 && ok == tried, buf);

    float out[J_COUNT];
    CHECK(solveIK(s, Vec3(10, 0.5f, 0), -90, true, out) == IK_UNREACHABLE, "IK: target beyond L1+L2 is unreachable");
    // picking pose: straight down onto a part on the table
    CHECK(solveIK(s, Vec3(2.6f, 0.16f, 0.0f), -90, true, out) == IK_OK, "IK: part at (2.6, 0.16, 0) reachable from above");
    ArmPose q = computeFK(s, out);
    CHECK(std::fabs(q.pitch + 90) < 0.01f, "IK: gripper pitch is -90 (pointing down)");
}

static int countPixels(const Canvas& c, Pixel col) {
    int n = 0;
    for (int y = 0; y < c.height(); ++y)
        for (int x = 0; x < c.width(); ++x) if (c.get(x, y) == col) ++n;
    return n;
}

static void testRaster() {
    Canvas c(64, 64);
    Pixel w = rgb(255, 255, 255), k = rgb(0, 0, 0);
    // Bresenham in all 8 octants: pixel count = max(|dx|,|dy|)+1 and endpoints set
    int pts[8][2] = {{20, 5}, {5, 20}, {-5, 20}, {-20, 5}, {-20, -5}, {-5, -20}, {5, -20}, {20, -5}};
    bool allOk = true;
    for (auto& d : pts) {
        c.clear(k);
        lineBresenham(c, 32, 32, 32 + d[0], 32 + d[1], w);
        int expect = std::max(std::abs(d[0]), std::abs(d[1])) + 1;
        if (countPixels(c, w) != expect || c.get(32 + d[0], 32 + d[1]) != w) allOk = false;
    }
    CHECK(allOk, "Bresenham: gap-free in all 8 octants, endpoints included");
    allOk = true;
    for (auto& d : pts) {
        c.clear(k);
        lineDDA(c, 32, 32, 32 + d[0], 32 + d[1], w);
        int expect = std::max(std::abs(d[0]), std::abs(d[1])) + 1;
        if (countPixels(c, w) != expect) allOk = false;
    }
    CHECK(allOk, "DDA: one pixel per step along the major axis");

    // Trace (2,3) -> (10,7) like the viva question
    c.clear(k);
    lineBresenham(c, 2, 3, 10, 7, w);
    int expY[9] = {3, 3, 4, 4, 5, 5, 6, 6, 7};
    bool trace = true;
    for (int i = 0; i <= 8; ++i) if (c.get(2 + i, expY[i]) != w) trace = false;
    CHECK(trace, "Bresenham trace (2,3)->(10,7) = y 3,3,4,4,5,5,6,6,7");

    // midpoint circle symmetric and closed
    c.clear(k);
    circleMidpoint(c, 32, 32, 20, w);
    bool sym = true;
    for (int y = 0; y < 64; ++y)
        for (int x = 0; x < 64; ++x)
            if ((c.get(x, y) == w) != (c.get(64 - x, y) == w) || (c.get(x, y) == w) != (c.get(y, x) == w)) sym = false;
    CHECK(sym, "Midpoint circle: 8-way symmetric");
    // flood fill inside the circle must not leak (circle is closed for 4-connected fill)
    IRect all{0, 0, 63, 63};
    int painted = floodFill4(c, 32, 32, rgb(0, 255, 0), all);
    CHECK(painted > 1000 && c.get(1, 1) == k, "Flood fill (4-conn.) stays inside the circle");

    // boundary fill of a rectangle
    c.clear(k);
    Pixel edge = rgb(255, 0, 0);
    lineBresenham(c, 10, 10, 30, 10, edge); lineBresenham(c, 30, 10, 30, 25, edge);
    lineBresenham(c, 30, 25, 10, 25, edge); lineBresenham(c, 10, 25, 10, 10, edge);
    int bf = boundaryFill4(c, 20, 18, rgb(0, 0, 255), edge, all);
    CHECK(bf == 19 * 14, "Boundary fill fills exactly the 19x14 interior");

    // scan-line fill of an axis aligned square with corners on pixel edges
    c.clear(k);
    scanlineFill(c, {{10, 10}, {20, 10}, {20, 20}, {10, 20}}, w);
    CHECK(countPixels(c, w) == 100, "Scan-line fill: 10x10 square -> 100 pixels");
}

static void testClip() {
    Rect2 r{0, 0, 10, 10};
    struct L { float x0, y0, x1, y1; bool accept; };
    L cases[] = {{2, 2, 8, 8, true}, {-5, -5, -1, 12, false}, {-5, 5, 15, 5, true},
                 {5, -5, 5, 15, true}, {-2, 8, 4, 14, true}, {11, 11, 20, 20, false}, {-1, 5, 5, 12, true}};
    bool same = true;
    for (const L& c : cases) {
        float a0 = c.x0, b0 = c.y0, a1 = c.x1, b1 = c.y1;
        float p0 = c.x0, q0 = c.y0, p1 = c.x1, q1 = c.y1;
        bool cs = clipCohenSutherland(a0, b0, a1, b1, r);
        bool lb = clipLiangBarsky(p0, q0, p1, q1, r);
        if (cs != c.accept || lb != c.accept) same = false;
        if (cs && lb) {
            // same endpoints (order preserved by both algorithms)
            if (std::fabs(a0 - p0) + std::fabs(b0 - q0) + std::fabs(a1 - p1) + std::fabs(b1 - q1) > 1e-3f) same = false;
            if (csOutcode(a0, b0, r) || csOutcode(a1, b1, r)) same = false;
        }
    }
    CHECK(same, "Cohen-Sutherland and Liang-Barsky agree on 7 test lines");
    CHECK(csOutcode(-1, 11, r) == (CS_LEFT | CS_TOP), "Cohen-Sutherland region code for (-1,11) = TOP|LEFT (1001)");

    // Sutherland-Hodgman: triangle over a corner gains vertices at the corner
    std::vector<Pt2> tri = {{5, 5}, {15, 5}, {5, 15}};
    auto out = clipSutherlandHodgman(tri, r);
    bool inside = !out.empty();
    for (auto& p : out) if (p.x < -1e-4f || p.x > 10 + 1e-4f || p.y < -1e-4f || p.y > 10 + 1e-4f) inside = false;
    CHECK(inside && out.size() == 5, "Sutherland-Hodgman: triangle over a corner -> 5 vertices, all inside");

    Pt2 v = windowToViewport({0, 0}, Rect2{-5, -5, 5, 5}, Rect2{0, 0, 100, 100});
    CHECK(std::fabs(v.x - 50) < 1e-4f && std::fabs(v.y - 50) < 1e-4f, "Window-to-viewport: window centre -> viewport centre");
}

static void testCollision() {
    AABB box{Vec3(0, 0, 0), Vec3(1, 1, 1)};
    CHECK(capsuleVsAABB({Vec3(-3, 0, 1.1f), Vec3(3, 0, 1.1f), 0.2f}, box), "Capsule passing 0.1 from a face collides (r=0.2)");
    CHECK(!capsuleVsAABB({Vec3(-3, 0, 1.5f), Vec3(3, 0, 1.5f), 0.2f}, box), "Capsule 0.5 away does not collide");
    CHECK(std::fabs(distSegmentAABB(Vec3(2, 2, 0), Vec3(4, 4, 0), box) - std::sqrt(2.0f)) < 1e-3f,
          "Segment-AABB distance to a corner = sqrt(2)");
    CHECK(circleXZvsAABB(0, 0, 4.1f, AABB{Vec3(4.2f, 0, 0), Vec3(0.3f, 1, 0.3f)}), "Safety circle detects operator at x=4.2");
}

static void testSimulation() {
    resetWorld(g);
    bool hits[3];
    CHECK(!armPoseCollides(g, g.arm.theta, hits, nullptr), "Home pose is collision free");
    // run the full autonomous cycle (simulated time) and check every part is sorted
    behaviourStart(g);
    for (int i = 0; i < 60 * 240 && g.fsm.running; ++i) updateSimulation(g, 1.0f / 60.0f);
    for (int i = 0; i < 120; ++i) updateSimulation(g, 1.0f / 60.0f);
    int sorted = 0, correct = 0;
    for (const Part& p : g.parts) {
        if (p.state == PART_BIN) {
            ++sorted;
            for (const Bin& b : g.bins)
                if (b.colorIndex == p.colorIndex && std::fabs(p.pos.x - b.pos.x) < 0.42f && std::fabs(p.pos.z - b.pos.z) < 0.42f) ++correct;
        }
    }
    char buf[96];
    std::snprintf(buf, sizeof(buf), "Auto pick-and-place sorts all parts: %d/%d in bins, %d in the right bin, %d collisions",
                  sorted, (int)g.parts.size(), correct, g.collisionCount);
    CHECK(sorted == (int)g.parts.size() && correct == sorted, buf);

    // safety stop: operator inside the zone freezes the arm
    resetWorld(g);
    behaviourStart(g);
    for (int i = 0; i < 30; ++i) updateSimulation(g, 1.0f / 60.0f);
    g.op.pos = Vec3(3.0f, FLOOR_Y, 0.5f);
    updateSimulation(g, 1.0f / 60.0f);
    float before[J_COUNT];
    for (int i = 0; i < J_COUNT; ++i) before[i] = g.arm.theta[i];
    for (int i = 0; i < 60; ++i) updateSimulation(g, 1.0f / 60.0f);
    bool frozen = true;
    for (int i = 0; i < J_COUNT; ++i) if (std::fabs(before[i] - g.arm.theta[i]) > 1e-5f) frozen = false;
    CHECK(g.arm.halted && frozen && g.fsm.state == FSM_HALT, "Operator in zone -> arm halted, FSM in HALT");
    g.op.pos = Vec3(7.0f, FLOOR_Y, 5.0f);
    for (int i = 0; i < 60; ++i) updateSimulation(g, 1.0f / 60.0f);
    CHECK(!g.arm.halted && g.fsm.state != FSM_HALT, "Operator leaves -> FSM resumes");

    // manual collision: drive into the +Z pillar
    resetWorld(g);
    float th[J_COUNT];
    bool blocked = false;
    for (int k = 0; k < 200 && !blocked; ++k) {
        for (int i = 0; i < J_COUNT; ++i) th[i] = g.arm.theta[i];
            if (th[J_BASE] > -66) th[J_BASE] -= 3;                 // 1) face the +Z pillar
            else if (th[J_WRIST] < 35) th[J_WRIST] += 5;          // 2) gripper horizontal
            else { th[J_SHOULDER] -= 1; th[J_WRIST] += 1; }       // 3) lower, keep it level
        blocked = !tryApplyAngles(g, th);
    }
    CHECK(blocked && g.arm.linkHit[2] && g.message.find("pillar") != std::string::npos,
          "Manual drive into pillar is blocked, gripper link flagged red");

    // force: tip pressed 0.05 into the table
    ForceState f;
    for (int i = 0; i < 60; ++i) updateForce(f, -0.05f, 0.0f);
    CHECK(std::fabs(f.raw - 20.0f) < 1e-3f && std::fabs(f.smooth - 20.0f) < 0.1f, "Force F=k*d = 400*0.05 = 20, smoothed converges");
}

int main() {
    testFK();
    testIK();
    testRaster();
    testClip();
    testCollision();
    testSimulation();
    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
