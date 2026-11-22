// Sierpinski.h - the 3D Sierpinski gasket (Unit I) used as the shape of every part.
#pragma once
#include "../math/Vec3.h"

// Draw a Sierpinski tetrahedron with its base on y = 0 and the given height,
// subdivided `depth` times (depth 0 = a single tetrahedron, 4^depth tetrahedra).
void drawSierpinski(float height, int depth);
