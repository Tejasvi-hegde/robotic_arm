// Clip2D.cpp
#include "Clip2D.h"

Pt2 windowToViewport(const Pt2& w, const Rect2& win, const Rect2& vp) {
    float sx = (vp.xmax - vp.xmin) / (win.xmax - win.xmin);
    float sy = (vp.ymax - vp.ymin) / (win.ymax - win.ymin);
    return {vp.xmin + (w.x - win.xmin) * sx, vp.ymin + (w.y - win.ymin) * sy};
}

// ------------------------------------------------------------ Cohen-Sutherland
int csOutcode(float x, float y, const Rect2& r) {
    int code = CS_INSIDE;
    if (x < r.xmin) code |= CS_LEFT;
    else if (x > r.xmax) code |= CS_RIGHT;
    if (y < r.ymin) code |= CS_BOTTOM;
    else if (y > r.ymax) code |= CS_TOP;
    return code;
}

bool clipCohenSutherland(float& x0, float& y0, float& x1, float& y1, const Rect2& r) {
    int c0 = csOutcode(x0, y0, r), c1 = csOutcode(x1, y1, r);
    while (true) {
        if (!(c0 | c1)) return true;          // both inside: trivially accept
        if (c0 & c1) return false;            // share an outside region: trivially reject
        int out = c0 ? c0 : c1;               // pick an endpoint that is outside
        float x = 0, y = 0;
        if (out & CS_TOP)         { x = x0 + (x1 - x0) * (r.ymax - y0) / (y1 - y0); y = r.ymax; }
        else if (out & CS_BOTTOM) { x = x0 + (x1 - x0) * (r.ymin - y0) / (y1 - y0); y = r.ymin; }
        else if (out & CS_RIGHT)  { y = y0 + (y1 - y0) * (r.xmax - x0) / (x1 - x0); x = r.xmax; }
        else                      { y = y0 + (y1 - y0) * (r.xmin - x0) / (x1 - x0); x = r.xmin; }
        if (out == c0) { x0 = x; y0 = y; c0 = csOutcode(x0, y0, r); }
        else           { x1 = x; y1 = y; c1 = csOutcode(x1, y1, r); }
    }
}

// ------------------------------------------------------------ Liang-Barsky
// P(t) = P0 + t (P1 - P0), 0 <= t <= 1.  For each edge: p_k t <= q_k
//   p = {-dx, dx, -dy, dy},  q = {x0-xmin, xmax-x0, y0-ymin, ymax-y0}
bool clipLiangBarsky(float& x0, float& y0, float& x1, float& y1, const Rect2& r) {
    float dx = x1 - x0, dy = y1 - y0;
    float p[4] = {-dx, dx, -dy, dy};
    float q[4] = {x0 - r.xmin, r.xmax - x0, y0 - r.ymin, r.ymax - y0};
    float u1 = 0.0f, u2 = 1.0f;
    for (int k = 0; k < 4; ++k) {
        if (p[k] == 0) {
            if (q[k] < 0) return false;        // parallel and outside
        } else {
            float t = q[k] / p[k];
            if (p[k] < 0) { if (t > u1) u1 = t; }   // entering
            else          { if (t < u2) u2 = t; }   // leaving
        }
    }
    if (u1 > u2) return false;
    float nx0 = x0 + u1 * dx, ny0 = y0 + u1 * dy;
    float nx1 = x0 + u2 * dx, ny1 = y0 + u2 * dy;
    x0 = nx0; y0 = ny0; x1 = nx1; y1 = ny1;
    return true;
}

// ------------------------------------------------------------ Sutherland-Hodgman
// Clip the polygon against one edge at a time; 4 cases per edge (in->in, in->out,
// out->in, out->out).
static bool insideEdge(const Pt2& p, int edge, const Rect2& r) {
    switch (edge) {
        case 0: return p.x >= r.xmin;
        case 1: return p.x <= r.xmax;
        case 2: return p.y >= r.ymin;
        default: return p.y <= r.ymax;
    }
}

static Pt2 intersectEdge(const Pt2& a, const Pt2& b, int edge, const Rect2& r) {
    float t;
    switch (edge) {
        case 0: t = (r.xmin - a.x) / (b.x - a.x); return {r.xmin, a.y + t * (b.y - a.y)};
        case 1: t = (r.xmax - a.x) / (b.x - a.x); return {r.xmax, a.y + t * (b.y - a.y)};
        case 2: t = (r.ymin - a.y) / (b.y - a.y); return {a.x + t * (b.x - a.x), r.ymin};
        default: t = (r.ymax - a.y) / (b.y - a.y); return {a.x + t * (b.x - a.x), r.ymax};
    }
}

std::vector<Pt2> clipSutherlandHodgman(const std::vector<Pt2>& poly, const Rect2& r) {
    std::vector<Pt2> out = poly;
    for (int edge = 0; edge < 4 && !out.empty(); ++edge) {
        std::vector<Pt2> in = out;
        out.clear();
        for (size_t i = 0; i < in.size(); ++i) {
            const Pt2& cur = in[i];
            const Pt2& prev = in[(i + in.size() - 1) % in.size()];
            bool cIn = insideEdge(cur, edge, r), pIn = insideEdge(prev, edge, r);
            if (cIn) {
                if (!pIn) out.push_back(intersectEdge(prev, cur, edge, r));  // out -> in
                out.push_back(cur);                                         // in -> in
            } else if (pIn) {
                out.push_back(intersectEdge(prev, cur, edge, r));           // in -> out
            }                                                               // out -> out: nothing
        }
    }
    return out;
}
