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

// Rects
// Rects
/*! @brief X.*/
//MOSS_API void          TextureRect(uint64 user_texture_id, const Float2& Moss_age_size, const Float2& uv0 = Float2(0, 0), const Float2& uv1 = Float2(1, 1));
/*! @brief X.*/
MOSS_API void          ColorRect(Color color, const Float2& size);

// Widgets: Text
MOSS_API void          Text(const char* fmt, ...);
MOSS_API void          RichText(const char* label);

MOSS_API void          ProgressBar();

// URLS
/*! @brief Hyperlink text button, return true when clicked.*/
MOSS_API bool          TextLink(const char* label);
/*! @brief Hyperlink text button, automatically open file/url when clicked.*/
MOSS_API void          TextLinkOpenURL(const char* label, const char* url = NULL);
//MOSS_API void          Moss_ImageOpenURL(Texture& texture, const char* url = NULL);

// Button Types
/*! @brief X.*/
MOSS_API bool          Button(const char* label, const Rect* rectPtr, const Mat44& vpMatrix, float defaultWidth = 100.0f, float defaultHeight = 24.0f);
MOSS_API bool          ToggleSwitch(bool active);
MOSS_API bool          RadioButton(const char* label, bool active);
MOSS_API bool          Checkbox(const char* label, bool* value, const Rect* rectPtr, const Mat44& vpMatrix, float defaultSize = 20.0f);


// Sliders
MOSS_API void          SliderFloat(float min_value, float max_value, float value);
MOSS_API void          SliderDouble(double min_value, double max_value, double value);
MOSS_API void          SliderInt(int min_value, int max_value, int value);

MOSS_API void          SliderFloat2(Float2 min_value, Float2 max_value, Float2 value);
MOSS_API void          SliderDouble2(Float2 min_value, Float2 max_value, Float2 value);
MOSS_API void          SliderInt2(Float2 min_value, Float2 max_value, Float2 value);

MOSS_API void          SliderFloat3(Float3 min_value, Float3 max_value, Float3 value);
MOSS_API void          SliderDouble3(Double3 min_value, Double3 max_value, Double3 value);
MOSS_API void          SliderInt3(Int3 min_value, Int3 max_value, Int3 value);

MOSS_API void          SliderFloat4(Float4 min_value, Float4 max_value, Float4 value);
MOSS_API void          SliderDouble4(Double4 min_value, Double4 max_value, Double4 value);
MOSS_API void          SliderInt4(Int4 min_value, Int4 max_value, Int4 value);


// Drag Sliders
/*! @brief X.*/
MOSS_API bool          DragFloat(const char* label, float* value, float v_speed = 1.0f, float v_min = 0.0f, float v_max = 0.0f, const char* format = "%.3f", Moss_GuiSliderFlags flags = 0);     // If v_min >= v_max we have no bound
/*! @brief X.*/
MOSS_API bool          DragFloat2(const char* label, Float2 value, float v_speed = 1.0f, float v_min = 0.0f, float v_max = 0.0f, const char* format = "%.3f", Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          DragFloat3(const char* label, Float3 value, float v_speed = 1.0f, float v_min = 0.0f, float v_max = 0.0f, const char* format = "%.3f", Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          DragFloat4(const char* label, Float4 value, float v_speed = 1.0f, float v_min = 0.0f, float v_max = 0.0f, const char* format = "%.3f", Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          DragFloatRange2(const char* label, float* v_current_min, float* v_current_max, float v_speed = 1.0f, float v_min = 0.0f, float v_max = 0.0f, const char* format = "%.3f", const char* format_max = NULL, Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          DragInt(const char* label, int* v, float v_speed = 1.0f, int v_min = 0, int v_max = 0, const char* format = "%d", Moss_GuiSliderFlags flags = 0);  // If v_min >= v_max we have no bound
/*! @brief X.*/
MOSS_API bool          DragInt2(const char* label, int v[2], float v_speed = 1.0f, int v_min = 0, int v_max = 0, const char* format = "%d", Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          DragInt3(const char* label, int v[3], float v_speed = 1.0f, int v_min = 0, int v_max = 0, const char* format = "%d", Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          DragInt4(const char* label, int v[4], float v_speed = 1.0f, int v_min = 0, int v_max = 0, const char* format = "%d", Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          DragIntRange2(const char* label, int* v_current_min, int* v_current_max, float v_speed = 1.0f, int v_min = 0, int v_max = 0, const char* format = "%d", const char* format_max = NULL, Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          DragScalar(const char* label, Moss_GuiDataType data_type, void* p_data, float v_speed = 1.0f, const void* p_min = NULL, const void* p_max = NULL, const char* format = NULL, Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          DragScalarN(const char* label, Moss_GuiDataType data_type, void* p_data, int components, float v_speed = 1.0f, const void* p_min = NULL, const void* p_max = NULL, const char* format = NULL, Moss_GuiSliderFlags flags = 0);



// Input (Keyboard)
/*! @brief X.*/
MOSS_API bool          InputText(const char* label, char* buf, size_t buf_size, Moss_GuiInputTextFlags flags = 0, Moss_GuiInputTextCallback callback = NULL, void* user_data = NULL);
/*! @brief X.*/
MOSS_API bool          InputTextMultiline(const char* label, char* buf, size_t buf_size, const Float2& size = Float2(0, 0), Moss_GuiInputTextFlags flags = 0, Moss_GuiInputTextCallback callback = NULL, void* user_data = NULL);
/*! @brief X.*/
MOSS_API bool          InputTextWithHint(const char* label, const char* hint, char* buf, size_t buf_size, Moss_GuiInputTextFlags flags = 0, Moss_GuiInputTextCallback callback = NULL, void* user_data = NULL);
/*! @brief X.*/
MOSS_API bool          InputFloat(const char* label, float* v, float step = 0.0f, float step_fast = 0.0f, const char* format = "%.3f", Moss_GuiInputTextFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          InputFloat2(const char* label, float v[2], const char* format = "%.3f", Moss_GuiInputTextFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          InputFloat3(const char* label, float v[3], const char* format = "%.3f", Moss_GuiInputTextFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          InputFloat4(const char* label, float v[4], const char* format = "%.3f", Moss_GuiInputTextFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          InputInt(const char* label, int* v, int step = 1, int step_fast = 100, Moss_GuiInputTextFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          InputInt2(const char* label, int v[2], Moss_GuiInputTextFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          InputInt3(const char* label, int v[3], Moss_GuiInputTextFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          InputInt4(const char* label, int v[4], Moss_GuiInputTextFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          InputDouble(const char* label, double* v, double step = 0.0, double step_fast = 0.0, const char* format = "%.6f", Moss_GuiInputTextFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          InputScalar(const char* label, Moss_GuiDataType data_type, void* p_data, const void* p_step = NULL, const void* p_step_fast = NULL, const char* format = NULL, Moss_GuiInputTextFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          InputScalarN(const char* label, Moss_GuiDataType data_type, void* p_data, int components, const void* p_step = NULL, const void* p_step_fast = NULL, const char* format = NULL, Moss_GuiInputTextFlags flags = 0);




// Color Editor/Picker 


// Tree Nodes

// List Boxes

// Menus

// Popups, Modals

// Tables

//Headers & Columns declaration
// Tab Bars, Tabs
// Drag and Drop
// GraphNode


#endif // MOSS_GUI_H