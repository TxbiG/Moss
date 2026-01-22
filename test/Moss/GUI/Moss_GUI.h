//                        MIT License
//
//                  Copyright (c) 2025 Toby
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/*!
 * @file Moss_GUI.h
 * @brief Core GUI abstraction layer for the Moss Framework.
 *
 */

#ifndef MOSS_GUI_H
#define MOSS_GUI_H

#include <Moss/Moss_Renderer.h>

// - Plot Graphs 2D & 3D (https://github.com/epezent/Moss_plot) (https://github.com/brenocq/Moss_plot3d)
// - Bezier Curve (https://github.com/ocornut/Moss_gui/issues/786)
// - Add Docking (Using windows)

// Want to make a GUI in C++? Think again. - https://www.youtube.com/watch?v=OcljnBdE3TA
// Goal: To make GUI Easy, Vercitile and Optimised.


MOSS_SUPRESS_WARNINGS_BEGIN

// Forward Declorations
typedef struct Moss_Style Moss_Style;
typedef struct Moss_Theme Moss_Theme;
typedef struct Moss_GUI Moss_GUI;

struct Style {
    float  windowRounding = 4.0f;
    float  itemSpacing    = 4.0f;
    Color  colWindowBg    = Color(40,40,40,230);
    Color  colBtn         = Color(70,70,70,255);
    Color  colBtnHover    = Color(92,92,92,255);
    Color  colBtnActive   = Color(110,110,110,255);
};


struct DrawChannel;               // Temporary storage to output draw commands out of order, used by Moss_DrawListSplitter and Moss_DrawList::ChannelsSplit()
struct DrawCmd;                   // A single draw command within a parent Moss_DrawList (generally maps to 1 GPU draw call, unless it is a callback)
struct DrawData;                  // All draw command lists required to render the frame + pos/size coordinates to use for the projection matrix.
struct DrawList;                  // A single draw command list (generally one per window, conceptually you may see this as a dynamic "mesh" builder)
struct DrawListSharedData;        // Data shared among multiple draw lists (typically owned by parent Moss_Gui context, but you may create one yourself)
struct DrawListSplitter;          // Helper to split a draw list into different layers which can be drawn into out of order, then flattened back.
struct DrawVert;                  // A single vertex (pos + uv + col = 20 bytes by default. Override layout with Moss_GUI_OVERRIDE_DRAWVERT_STRUCT_LAYOUT)

typedef int Moss_GuiWindowFlags;
typedef int Moss_GuiChildFlags;
enum Moss_ButtonFlags {Pressed, Released};

struct GuiContext;

// struct Dialouge; < This should also be put into platform.h
/*
MOSS_API GuiContext*    CreateContext(Moss_FontAtlas* shared_font_atlas = NULL);
MOSS_API void           DestroyContext(GuiContext* ctx = NULL);   // NULL = destroy current context
MOSS_API GuiContext*    GetCurrentContext();
MOSS_API void           SetCurrentContext(GuiContext* ctx);
*/

// Windows
/*! @brief X.*/
MOSS_API bool          BeginWindow(const char* name, bool* p_open = NULL, Moss_GuiWindowFlags flags = 0);
/*! @brief X.*/
MOSS_API void          EndWindow();

// Child Windows
/*! @brief X.*/
MOSS_API bool          BeginWindowChild(const char* str_id, const Float2& size = Float2(0, 0), Moss_GuiChildFlags child_flags = 0, Moss_GuiWindowFlags window_flags = 0);
/*! @brief X.*/
MOSS_API bool          BeginWindowChild(uint32 id, const Float2& size = Float2(0, 0), Moss_GuiChildFlags child_flags = 0, Moss_GuiWindowFlags window_flags = 0);
/*! @brief X.*/
MOSS_API void          EndWindowChild();

// Containers
/*! @brief X. @param x.*/
MOSS_API void          BeginContainer(Rect& rect);
/*! @brief X. @param x.*/
MOSS_API void          EndContainer();
/*! @brief X. @param x.*/
MOSS_API void          BeginVContainer(Rect& rect, float spacing = 50.0f);
/*! @brief X. @param x.*/
MOSS_API void          EndVContainer();
/*! @brief X. @param x.*/
MOSS_API void          BeginHContainer(Rect& rect, float spacing = 50.0f);
/*! @brief X. @param x.*/
MOSS_API void          EndHContainer();
/*! @brief X. @param x.*/
MOSS_API void          BeginCenterContainer(Rect& rect);
/*! @brief X. @param x.*/
MOSS_API void          EndCenterContainer();
/*! @brief X. @param x.*/
MOSS_API void          BeginMarginContainer(float leftMargin = 4.0f, float rightMargin = 4.0f, float topMargin = 4.0f, float bottomMargin = 4.0f);
/*! @brief X. @param x.*/
MOSS_API void          EndMarginContainer();
/*! @brief X. @param x.*/
MOSS_API void          BeginVScrollContainer();
/*! @brief X. @param x.*/
MOSS_API void          EndVScrollContainer();
/*! @brief X. @param x.*/
MOSS_API void          BeginHScrollContainer();
/*! @brief X. @param x.*/
MOSS_API void          EndHScrollContainer();
/*! @brief X. @param x.*/
MOSS_API void          BeginGridContainer();
/*! @brief X. @param x.*/
MOSS_API void          EndGridContainer();

/*! @brief X.  @param SubViewport& a viewport to display on screen .*/
//MOSS_API void          SubViewportContainer(SubViewport& subviewport);

// Widgets: Text
MOSS_API void          Text(const char* fmt, ...);
MOSS_API void          RichText(const char* label);

// Button Types
/*! @brief X.*/
MOSS_API bool          Button(const char* label, const Rect* rectPtr, const Mat44& vpMatrix, float defaultWidth = 100.0f, float defaultHeight = 24.0f);
MOSS_API bool          Checkbox(const char* label, bool* value, const Rect* rectPtr, const Mat44& vpMatrix, float defaultSize = 20.0f);

MOSS_SUPRESS_WARNINGS_END

#endif // MOSS_GUI_H