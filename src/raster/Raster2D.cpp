// Raster2D.cpp
#include "Raster2D.h"
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <utility>

void Canvas::clear(Pixel c) {
    for (size_t i = 0; i < px.size(); i += 3) { px[i] = c.r; px[i + 1] = c.g; px[i + 2] = c.b; }
}

void Canvas::set(int x, int y, Pixel c) {
    if (!inside(x, y)) return;                       // point clipping
    size_t i = ((size_t)y * W + x) * 3;
    px[i] = c.r; px[i + 1] = c.g; px[i + 2] = c.b;
}

void Canvas::blend(int x, int y, Pixel c, float a) {
    if (!inside(x, y)) return;
    size_t i = ((size_t)y * W + x) * 3;
    px[i]     = (uint8_t)(px[i]     * (1 - a) + c.r * a);
    px[i + 1] = (uint8_t)(px[i + 1] * (1 - a) + c.g * a);
    px[i + 2] = (uint8_t)(px[i + 2] * (1 - a) + c.b * a);
}

Pixel Canvas::get(int x, int y) const {
    if (!inside(x, y)) return {};
    size_t i = ((size_t)y * W + x) * 3;
    return {px[i], px[i + 1], px[i + 2]};
}

// ------------------------------------------------------------------ DDA
// Step along the major axis one pixel at a time, add the slope to the minor axis.
void lineDDA(Canvas& c, int x0, int y0, int x1, int y1, Pixel col) {
    int dx = x1 - x0, dy = y1 - y0;
    int steps = std::max(std::abs(dx), std::abs(dy));
    if (steps == 0) { c.set(x0, y0, col); return; }
    float xInc = (float)dx / steps, yInc = (float)dy / steps;
    float x = (float)x0, y = (float)y0;
    for (int i = 0; i <= steps; ++i) {
        c.set((int)std::lround(x), (int)std::lround(y), col);
        x += xInc;
        y += yInc;
    }
}

// ------------------------------------------------------------------ Bresenham
// Integer-only, works in all 8 octants. Decision parameter err = dx - dy.
void lineBresenham(Canvas& c, int x0, int y0, int x1, int y1, Pixel col) {
    int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    while (true) {
        c.set(x0, y0, col);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx)  { err += dx; y0 += sy; }
    }
}

// ------------------------------------------------------------------ midpoint circle
// p0 = 1 - r; p<0 -> next pixel E, p += 2x+3 ; else SE, p += 2(x-y)+5
static void plot8(Canvas& c, int xc, int yc, int x, int y, Pixel col) {
    c.set(xc + x, yc + y, col); c.set(xc - x, yc + y, col);
    c.set(xc + x, yc - y, col); c.set(xc - x, yc - y, col);
    c.set(xc + y, yc + x, col); c.set(xc - y, yc + x, col);
    c.set(xc + y, yc - x, col); c.set(xc - y, yc - x, col);
}

void circleMidpoint(Canvas& c, int xc, int yc, int r, Pixel col) {
    if (r <= 0) { c.set(xc, yc, col); return; }
    int x = 0, y = r;
    int p = 1 - r;
    plot8(c, xc, yc, x, y, col);
    while (x < y) {
        ++x;
        if (p < 0) p += 2 * x + 1;
        else { --y; p += 2 * (x - y) + 1; }
        plot8(c, xc, yc, x, y, col);
    }
}

// ------------------------------------------------------------------ scan-line fill
// For every scan line: intersect with all edges (half-open rule ymin <= y < ymax so
// vertices are not counted twice), sort the x-intersections, fill between pairs.
void scanlineFill(Canvas& c, const std::vector<Pt2>& poly, Pixel col, float alpha) {
    if (poly.size() < 3) return;
    float ymin = poly[0].y, ymax = poly[0].y;
    for (const Pt2& p : poly) { ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y); }
    int y0 = std::max(0, (int)std::ceil(ymin)), y1 = std::min(c.height() - 1, (int)std::floor(ymax));
    std::vector<float> xs;
    for (int y = y0; y <= y1; ++y) {
        xs.clear();
        float sy = y + 0.5f;                         // sample at pixel centre
        for (size_t i = 0; i < poly.size(); ++i) {
            Pt2 a = poly[i], b = poly[(i + 1) % poly.size()];
            if (a.y == b.y) continue;                // horizontal edges add nothing
            if (a.y > b.y) std::swap(a, b);
            if (sy < a.y || sy >= b.y) continue;
            xs.push_back(a.x + (sy - a.y) * (b.x - a.x) / (b.y - a.y));
        }
        std::sort(xs.begin(), xs.end());
        for (size_t k = 0; k + 1 < xs.size(); k += 2) {
            int xa = (int)std::ceil(xs[k] - 0.5f), xb = (int)std::floor(xs[k + 1] - 0.5f);
            for (int x = xa; x <= xb; ++x) {
                if (alpha >= 1.0f) c.set(x, y, col); else c.blend(x, y, col, alpha);
            }
        }
    }
}

// ------------------------------------------------------------------ boundary / flood fill
// Iterative (explicit stack) so large areas cannot overflow the call stack.
int boundaryFill4(Canvas& c, int x, int y, Pixel fill, Pixel boundary, const IRect& area) {
    int painted = 0;
    std::vector<std::pair<int, int>> st;
    st.push_back({x, y});
    while (!st.empty()) {
        auto [px, py] = st.back();
        st.pop_back();
        if (!area.contains(px, py) || !c.inside(px, py)) continue;
        Pixel cur = c.get(px, py);
        if (cur == boundary || cur == fill) continue;
        c.set(px, py, fill);
        ++painted;
        st.push_back({px + 1, py}); st.push_back({px - 1, py});
        st.push_back({px, py + 1}); st.push_back({px, py - 1});
    }
    return painted;
}

int floodFill4(Canvas& c, int x, int y, Pixel fill, const IRect& area) {
    if (!c.inside(x, y) || !area.contains(x, y)) return 0;
    Pixel old = c.get(x, y);
    if (old == fill) return 0;
    int painted = 0;
    std::vector<std::pair<int, int>> st;
    st.push_back({x, y});
    while (!st.empty()) {
        auto [px, py] = st.back();
        st.pop_back();
        if (!area.contains(px, py) || !c.inside(px, py)) continue;
        if (c.get(px, py) != old) continue;
        c.set(px, py, fill);
        ++painted;
        st.push_back({px + 1, py}); st.push_back({px - 1, py});
        st.push_back({px, py + 1}); st.push_back({px, py - 1});
    }
    return painted;
}
