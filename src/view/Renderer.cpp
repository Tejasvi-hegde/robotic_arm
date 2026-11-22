// Renderer.cpp
#include "Renderer.h"
#include "../GLInclude.h"
#include "../App.h"
#include "../scene/Scene.h"
#include "../ui/Hud.h"

void setupLighting() {
    GLfloat amb[] = {0.32f, 0.32f, 0.35f, 1.0f};
    GLfloat dif[] = {0.85f, 0.85f, 0.80f, 1.0f};
    GLfloat spec[] = {0.35f, 0.35f, 0.35f, 1.0f};
    glLightfv(GL_LIGHT0, GL_AMBIENT, amb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, dif);
    glLightfv(GL_LIGHT0, GL_SPECULAR, spec);
    glEnable(GL_LIGHT0);
    // fill light from the other side
    GLfloat dif1[] = {0.25f, 0.27f, 0.32f, 1.0f};
    glLightfv(GL_LIGHT1, GL_DIFFUSE, dif1);
    glEnable(GL_LIGHT1);

    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    GLfloat mspec[] = {0.25f, 0.25f, 0.25f, 1.0f};
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mspec);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 32.0f);
    glEnable(GL_NORMALIZE);          // links are drawn with glScalef -> renormalise
    glShadeModel(GL_SMOOTH);
}

static void applyRenderState() {
    if (g.set.depthTest) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST);
    if (g.set.cullFace) { glEnable(GL_CULL_FACE); glCullFace(GL_BACK); } else glDisable(GL_CULL_FACE);
    if (g.set.lighting) glEnable(GL_LIGHTING); else glDisable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, g.set.wireframe ? GL_LINE : GL_FILL);
}

void renderView(int vx, int vy, int vw, int vh, float eyeShift, bool pickPass) {
    glViewport(vx, vy, vw, vh);
    float aspect = vh > 0 ? (float)vw / vh : 1.0f;     // keep the aspect ratio on resize
    g.cam.applyProjection(aspect);
    g.cam.applyView(eyeShift);

    // lights are positioned in world space (after the view transform)
    GLfloat lp0[] = {0.45f, 1.0f, 0.65f, 0.0f};
    GLfloat lp1[] = {-0.6f, 0.4f, -0.5f, 0.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, lp0);
    glLightfv(GL_LIGHT1, GL_POSITION, lp1);

    if (pickPass || g.set.showPick) {
        glEnable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        Scene::drawPickScene(!pickPass);      // false colours only when shown on screen
        // the arm also occludes the parts (its flat colours never fall in the part ID range)
        drawArm(g.arm, false);
        return;
    }

    applyRenderState();
    Scene::drawWorld();
    drawArm(g.arm, g.set.axes);
    if (g.set.bounds) Scene::drawBounds();
    Scene::drawTargetMarker();
    Scene::drawSafetyZone(g.cam.eye());      // translucent: always last
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void renderFrame(bool swap) {
    glClearColor(0.11f, 0.12f, 0.15f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (g.set.stereo) {
        // side-by-side stereo pair (simulated HMD display, Unit I five components)
        int half = g.winW / 2;
        renderView(0, 0, half, g.winH, -0.18f);
        renderView(half, 0, g.winW - half, g.winH, +0.18f);
    } else {
        renderView(0, 0, g.winW, g.winH, 0.0f);
    }
    Hud::draw();
    if (swap) glutSwapBuffers();
}

PickResult pickAt(int mx, int my) {
    PickResult r;
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    renderView(0, 0, g.winW, g.winH, 0.0f, true);
    glFinish();

    int glY = g.winH - my - 1;               // window y is top-down, GL is bottom-up
    GLubyte px[3] = {0, 0, 0};
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(mx, glY, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, px);
    r.id = px[0];

    GLfloat depth = 1.0f;
    glReadPixels(mx, glY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
    if (depth < 1.0f) {
        GLdouble mv[16], pr[16];
        GLint vp[4];
        glGetDoublev(GL_MODELVIEW_MATRIX, mv);
        glGetDoublev(GL_PROJECTION_MATRIX, pr);
        glGetIntegerv(GL_VIEWPORT, vp);
        GLdouble wx, wy, wz;
        if (gluUnProject(mx, glY, depth, mv, pr, vp, &wx, &wy, &wz)) {
            r.hasPoint = true;
            r.point = Vec3((float)wx, (float)wy, (float)wz);
        }
    }
    return r;   // nothing swapped: the user never sees the ID image
}
