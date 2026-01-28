#ifndef MOSS_GUI_INTERN_H
#define MOSS_GUI_INTERN_H

#include <Moss/Moss_GUI.h>
#include <Moss/Moss_Platform.h>

struct VContainer { Rect area; float cursorY; float spacing; };
struct HContainer { Rect area; float cursorX; float spacing; };

static std::vector<VContainer> g_vStack;
static std::vector<HContainer> g_hStack;

// ── optional offset stack for centring ───────────────────────────
static std::vector<Float2> g_offsetStack;     // Vec2 is {float x,y}
static Float2 g_currentOffset   = {0.f, 0.f}; // applied to every widget rect

inline bool rectContains(const Rect& rect, float mouse_x, float mouse_y) { return mouse_x >= rect.x && mouse_x <= rect.x + rect.width && mouse_y >= rect.y && mouse_y <= rect.y + rect.height; }

static bool GetNextWidgetRectV(float requestedHeight, Rect* outRect)
{
    if (g_vStack.empty())
        return false;

    VContainer& c = g_vStack.back();

    // Compute the widget rect at current cursor position inside container
    *outRect = { c.area.x, c.cursorY, c.area.width, requestedHeight };

    // Advance the cursor for next widget, including spacing
    c.cursorY += requestedHeight + c.spacing;

    // Optionally grow container height to fit children (dynamic resizing)
    float used = c.cursorY - c.area.y;
    if (used > c.area.height) 
        c.area.height = used;

    return true;
}



static bool GetNextWidgetRectH(float width, Rect* out)
{
    if (g_hStack.empty()) return false;
    auto& c = g_hStack.back();
    *out = { c.cursorX, c.area.y, width, c.area.height };
    c.cursorX += width + c.spacing;
    return true;
}

void PushOffset(const Float2& offset) {
    g_offsetStack.push_back(offset);
}

void PopOffset() {
    if (g_offsetStack.empty()) return; g_offsetStack.pop_back();
}

Float2 GetCurrentOffset() {
    if (g_offsetStack.empty()) return {0, 0};
    return g_offsetStack.back();
}


inline Rect ApplyOffset(const Rect& r)
{
    return { r.x + g_currentOffset.x,
             r.y + g_currentOffset.y,
             r.width, r.height };
}
#endif // MOSS_GUI_INTERN_H