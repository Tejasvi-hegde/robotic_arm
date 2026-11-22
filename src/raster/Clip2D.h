// Clip2D.h - 2D viewing: window-to-viewport mapping and clipping (Unit III).
#pragma once
#include <vector>
#include "Raster2D.h"

struct Rect2 { float xmin, ymin, xmax, ymax; };

// Window-to-viewport transformation:
//   xv = xvmin + (xw - xwmin) * (xvmax - xvmin) / (xwmax - xwmin)   (same for y)
Pt2 windowToViewport(const Pt2& w, const Rect2& window, const Rect2& viewport);

// Cohen-Sutherland line clipping with 4-bit region codes. Returns false if rejected.
enum { CS_INSIDE = 0, CS_LEFT = 1, CS_RIGHT = 2, CS_BOTTOM = 4, CS_TOP = 8 };
int  csOutcode(float x, float y, const Rect2& r);
bool clipCohenSutherland(float& x0, float& y0, float& x1, float& y1, const Rect2& r);

// Liang-Barsky parametric line clipping. Returns false if rejected.
bool clipLiangBarsky(float& x0, float& y0, float& x1, float& y1, const Rect2& r);

// Sutherland-Hodgman polygon clipping against the 4 edges of a rectangle.
std::vector<Pt2> clipSutherlandHodgman(const std::vector<Pt2>& poly, const Rect2& r);
