#include <Moss/Moss_GUI.h>
#include <Moss/GUI/GUI_internal.h>
#include <Moss/Platform/platform_intern.h>

// -----------------------------------------------------------------------------
// Containers
// -----------------------------------------------------------------------------

void BeginVContainer(Rect& rect, float spacing)
{
    if (!g_vStack.empty())
    {
        const auto& parent = g_vStack.back();
        rect.x      = parent.area.x;
        rect.y      = parent.cursorY;
        rect.width  = parent.area.width;
        // rect.height is caller‑supplied (0 if you don't care)
    }
    else {
        rect = ApplyOffset(rect);
    }
    g_vStack.push_back({ rect, rect.y, spacing });
}

void EndVContainer() { if(g_vStack.empty()) { return; } g_vStack.pop_back(); }

MOSS_API void BeginHContainer(Rect& rect, float spacing)
{
    // Inherit from parent V or H if present
    if (!g_hStack.empty())
    {
        const auto& parent = g_hStack.back();
        rect.x     = parent.cursorX;
        rect.y     = parent.area.y;
        rect.height = parent.area.height;
    }
    rect = ApplyOffset(rect);
    g_hStack.push_back({ rect, rect.x, spacing });
}

MOSS_API void EndHContainer() { if(g_hStack.empty()) return; g_hStack.pop_back(); }


MOSS_API void BeginCenterContainer(Rect& rect)
{
    PushOffset({ rect.x + rect.width * 0.5f, rect.y + rect.height * 0.5f });
}

MOSS_API void EndCenterContainer() { PopOffset(); }


// -----------------------------------------------------------------------------
// Widgets
// -----------------------------------------------------------------------------
MOSS_API bool Button(const char* label,
                     const Rect* rectPtr,
                     const Mat44& vpMatrix,
                     float defaultWidth,
                     float defaultHeight)
{
    // ──────────────────────────────────────────────────────────────────
    // 1. Resolve rectangle
    // ──────────────────────────────────────────────────────────────────
    Rect rect;

    if (rectPtr)                                  // explicit placement
    {
        rect = ApplyOffset(*rectPtr);
    }
    else                                          // auto‑layout
    {
        bool ok;
        if (!g_hStack.empty())                    // inside H‑container
        {
            ok = GetNextWidgetRectH(defaultWidth, &rect);
        }
        else                                      // inside V‑container
        {
            ok = GetNextWidgetRectV(defaultHeight, &rect);
        }
        if (!ok) return false;                    // no active container
    }

    // ──────────────────────────────────────────────────────────────────
    // 2. Interaction
    // ──────────────────────────────────────────────────────────────────
    bool hover   = rectContains(rect, io.mouse_x, io.mouse_y);
    bool clicked = hover && IsJustPressed(Mouse::LEFT);

    // ──────────────────────────────────────────────────────────────────
    // 3. Visuals
    // ──────────────────────────────────────────────────────────────────
    Color base   = Color(0.85f, 0.85f, 0.85f, 1.0f); // Light gray-white
    Color hoverC = Color(0.95f, 0.95f, 0.95f, 1.0f); // Very light white
    Color press  = Color(0.75f, 0.75f, 0.75f, 1.0f); // Slightly darker white


    Color drawCol = clicked ? press : (hover ? hoverC : base);

    static Surface quad(rect.x, rect.y, rect.width, rect.height, drawCol);     // reuse one quad
    quad.update();
    quad.draw(vpMatrix);

    Font font;
    if (!font.load("C:/Windows/Fonts/arial.ttf", defaultHeight)) {
        printf("Font failed to load!");
    }
    
    font.renderText(label, rect.x, rect.y, vpMatrix);

    return clicked;
}

bool Checkbox(const char* label, bool* value, const Rect& rect, const Mat44& vpMatrix) {
    bool isHovered = rectContains(rect, io.mouse_x, io.mouse_y);
    bool clicked = false;

    // toggle checkbox state
    if (isHovered && IsJustPressed(Mouse::LEFT)) { clicked = true; *value = !(*value); }

    // Color based on state
    Color baseColor     = Color(0.2f, 0.2f, 0.2f, 1.0f);
    Color hoverColor    = Color(0.3f, 0.3f, 0.3f, 1.0f);
    Color activeColor   = Color(0.0f, 0.6f, 0.2f, 1.0f);
    Color drawColor     = *value ? activeColor : baseColor;
    if (isHovered) drawColor = (*value) ? activeColor : hoverColor;

    Surface checkboxSurface(rect.x, rect.y, rect.width, rect.height, drawColor);
    checkboxSurface.draw(vpMatrix);

    // Optional: Draw check mark or label (requires text rendering system)

    return clicked; // returns true if it was just clicked this frame
}
