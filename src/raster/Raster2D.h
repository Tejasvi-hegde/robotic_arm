// Raster2D.h - hand-written raster algorithms (Unit III), drawing into a
// software frame buffer ("Canvas") one pixel at a time. The canvas is then
// copied to the screen with glDrawPixels - no GL line/polygon calls are used,
// so every pixel of the mini-map comes from these algorithms.
#pragma once
#include <vector>
#include <cstdint>

struct Pixel {
    uint8_t r = 0, g = 0, b = 0;
    bool operator==(const Pixel& o) const { return r == o.r && g == o.g && b == o.b; }
    bool operator!=(const Pixel& o) const { return !(*this == o); }
};
inline Pixel rgb(int r, int g, int b) { return {(uint8_t)r, (uint8_t)g, (uint8_t)b}; }

struct Pt2 { float x, y; };

struct IRect { int xmin, ymin, xmax, ymax; bool contains(int x, int y) const {
    return x >= xmin && x <= xmax && y >= ymin && y <= ymax; } };

class Canvas {
public:
    Canvas(int w = 1, int h = 1) { resize(w, h); }
    void resize(int w, int h) { W = w; H = h; px.assign((size_t)w * h * 3, 0); }
    int width() const { return W; }
    int height() const { return H; }
    void clear(Pixel c);
    // setPixel with POINT CLIPPING: pixels outside the canvas are ignored.
    void set(int x, int y, Pixel c);
    void blend(int x, int y, Pixel c, float alpha);
    Pixel  get(int x, int y) const;
    bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < W && y < H; }
    const uint8_t* data() const { return px.data(); }
private:
    int W = 1, H = 1;
    std::vector<uint8_t> px;   // row 0 = bottom row (OpenGL convention)
};

// ---- line drawing
void lineDDA(Canvas& c, int x0, int y0, int x1, int y1, Pixel col);
void lineBresenham(Canvas& c, int x0, int y0, int x1, int y1, Pixel col);
// ---- circle drawing (8-way symmetry, integer decision parameter)
void circleMidpoint(Canvas& c, int xc, int yc, int r, Pixel col);
// ---- filled-area primitives
// Scan-line polygon fill (even-odd rule). alpha < 1 blends over the canvas.
void scanlineFill(Canvas& c, const std::vector<Pt2>& poly, Pixel col, float alpha = 1.0f);
// 4-connected boundary fill: paints until it meets `boundary` colour. Limited to `area`.
int boundaryFill4(Canvas& c, int x, int y, Pixel fill, Pixel boundary, const IRect& area);
// 4-connected flood fill: replaces the seed's colour with `fill`. Limited to `area`.
int floodFill4(Canvas& c, int x, int y, Pixel fill, const IRect& area);
