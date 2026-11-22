// Hud.h - 2D overlay: text panels, matrix panel, force bar and the mini-map.
#pragma once

namespace Hud {
    void draw();
    // Mini-map placement in window pixels (bottom-right corner) - used to ignore clicks.
    bool insideMiniMap(int mouseX, int mouseY);
    constexpr int MM_SIZE = 260;
    constexpr int MM_MARGIN = 12;
}
