// Primitives.h - geometry built only from OpenGL primitives (Unit I).
#pragma once

// Axis-aligned box centred at the origin, size sx * sy * sz (GL_QUADS + normals).
void drawBox(float sx, float sy, float sz);
// Wire box (GL_LINE_LOOP / GL_LINES) used for bounding volumes.
void drawWireBox(float sx, float sy, float sz);
// Solid cylinder along +X from x=0 to x=len (GL_QUAD_STRIP side + GL_TRIANGLE_FAN caps).
void drawCylinderX(float len, float radius, int slices = 20);
// Solid cylinder along +Y from y=0 to y=h.
void drawCylinderY(float h, float radius, int slices = 24);
// Sphere via GLU quadric (joints, operator head).
void drawSphere(float r, int slices = 16, int stacks = 12);
// Ground grid of GL_LINES.
void drawGrid(float half, float step, float y);
// RGB axes (X red, Y green, Z blue) of given length - debug view of a joint frame.
void drawAxes(float len);
