// Scene.h - the workcell: display lists, colour palette, world drawing, pick buffer,
// translucent safety zone (painter's algorithm) and bounding-volume overlay.
#pragma once
#include "../math/Vec3.h"

enum DisplayListId {
    DL_FLOOR = 0, DL_TABLE, DL_BASE, DL_LINK1, DL_LINK2, DL_LINK3,
    DL_BIN, DL_PART, DL_OPERATOR, DL_CUBE, DL_COUNT
};

// "Indexed colour": parts and bins store an index into this palette.
struct PaletteColour { float r, g, b; const char* name; };
extern const PaletteColour PALETTE[3];

namespace Scene {
    void init();                     // compile all display lists
    void rebuildPartList();          // after the Sierpinski depth changes
    void callList(int id);           // glCallList or immediate mode (toggle)

    void drawWorld();                // everything except the arm
    // flat colour-ID render for picking; visualize=true shows the IDs in false colours
    void drawPickScene(bool visualize = false);
    void drawSafetyZone(const Vec3& eye);
    void drawBounds();               // capsules + AABBs (wire)
    void drawTargetMarker();
    void drawStress();               // 225 extra pyramids for the display-list benchmark

    constexpr int PICK_TABLE = 200;  // id written in the red channel for the table top
    constexpr int PICK_FLOOR = 201;
}
