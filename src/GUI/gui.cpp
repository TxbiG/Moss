#include "Moss_GUI.h"






// Progressbar
/*! @brief X.*/
MOSS_API void          ProgressBar(float fraction, const Float2& size_arg = Float2(-FLT_MIN, 0), const char* overlay = NULL);

// Combo Box (Dropdown box)
/*! @brief X.*/
MOSS_API bool          BeginCombo(const char* label, const char* preview_value, Moss_GuiComboFlags flags = 0);
/*! @brief X.*/
MOSS_API void          EndCombo(); // only call EndCombo() if BeginCombo() returns true!
/*! @brief X.*/
MOSS_API bool          Combo(const char* label, int* current_item, const char* const items[], int items_count, int popup_max_height_in_items = -1);
/*! @brief X.*/
MOSS_API bool          Combo(const char* label, int* current_item, const char* items_separated_by_zeros, int popup_max_height_in_items = -1);      // Separate items with \0 within a string, end item-list with \0\0. e.g. "One\0Two\0Three\0"
/*! @brief X.*/
MOSS_API bool          Combo(const char* label, int* current_item, const char* (*getter)(void* user_data, int idx), void* user_data, int items_count, int popup_max_height_in_items = -1);


// Sliders
/*! @brief X.*/
MOSS_API bool          SliderFloat(const char* label, float* v, float v_min, float v_max, const char* format = "%.3f", Moss_GuiSliderFlags flags = 0);     // adjust format to decorate the value with a prefix or a suffix for in-slider labels or unit display.
/*! @brief X.*/
MOSS_API bool          SliderFloat2(const char* label, float v[2], float v_min, float v_max, const char* format = "%.3f", Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          SliderFloat3(const char* label, float v[3], float v_min, float v_max, const char* format = "%.3f", Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          SliderFloat4(const char* label, float v[4], float v_min, float v_max, const char* format = "%.3f", Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          SliderAngle(const char* label, float* v_rad, float v_degrees_min = -360.0f, float v_degrees_max = +360.0f, const char* format = "%.0f deg", Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          SliderInt(const char* label, int* v, int v_min, int v_max, const char* format = "%d", Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          SliderInt2(const char* label, int v[2], int v_min, int v_max, const char* format = "%d", Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          SliderInt3(const char* label, int v[3], int v_min, int v_max, const char* format = "%d", Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          SliderInt4(const char* label, int v[4], int v_min, int v_max, const char* format = "%d", Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          SliderScalar(const char* label, Moss_GuiDataType data_type, void* p_data, const void* p_min, const void* p_max, const char* format = NULL, Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          SliderScalarN(const char* label, Moss_GuiDataType data_type, void* p_data, int components, const void* p_min, const void* p_max, const char* format = NULL, Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          VSliderFloat(const char* label, const Float2& size, float* v, float v_min, float v_max, const char* format = "%.3f", Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          VSliderInt(const char* label, const Float2& size, int* v, int v_min, int v_max, const char* format = "%d", Moss_GuiSliderFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          VSliderScalar(const char* label, const Float2& size, Moss_GuiDataType data_type, void* p_data, const void* p_min, const void* p_max, const char* format = NULL, Moss_GuiSliderFlags flags = 0);

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
/*! @brief X.*/
MOSS_API bool          ColorEdit3(const char* label, float col[3], Moss_GuiColorEditFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          ColorEdit4(const char* label, float col[4], Moss_GuiColorEditFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          ColorPicker3(const char* label, float col[3], Moss_GuiColorEditFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          ColorPicker4(const char* label, float col[4], Moss_GuiColorEditFlags flags = 0, const float* ref_col = NULL);
/*! @brief X.*/
MOSS_API bool          ColorButton(const char* desc_id, const Float4& col, Moss_GuiColorEditFlags flags = 0, const Float2& size = Float2(0, 0)); // display a color square/button, hover for details, return true when pressed.
/*! @brief X.*/
MOSS_API void          SetColorEditOptions(Moss_GuiColorEditFlags flags);                     // initialize current options (generally on application startup) if you want to select a default format, picker type, etc. User will be able to change many settings, unless you pass the _NoOptions flag to your calls.

// Tree Nodes
/*! @brief X.*/
MOSS_API bool          TreeNode(const char* label);
/*! @brief X.*/
MOSS_API bool          TreeNode(const char* str_id, const char* fmt, ...) Moss__FMTARGS(2);   // helper variation to easily decorelate the id from the displayed string. Read the FAQ about why and how to use ID. to align arbitrary text at the same level as a TreeNode() you can use Bullet().
/*! @brief X.*/
MOSS_API bool          TreeNode(const void* ptr_id, const char* fmt, ...) Moss__FMTARGS(2);   // "
/*! @brief X.*/
MOSS_API bool          TreeNodeV(const char* str_id, const char* fmt, va_list args) Moss__FMTLIST(2);
/*! @brief X.*/
MOSS_API bool          TreeNodeV(const void* ptr_id, const char* fmt, va_list args) Moss__FMTLIST(2);
/*! @brief X.*/
MOSS_API bool          TreeNodeEx(const char* label, Moss_GuiTreeNodeFlags flags = 0);
/*! @brief X.*/
MOSS_API bool          TreeNodeEx(const char* str_id, Moss_GuiTreeNodeFlags flags, const char* fmt, ...) Moss__FMTARGS(3);
/*! @brief X.*/
MOSS_API bool          TreeNodeEx(const void* ptr_id, Moss_GuiTreeNodeFlags flags, const char* fmt, ...) Moss__FMTARGS(3);
/*! @brief X.*/
MOSS_API bool          TreeNodeExV(const char* str_id, Moss_GuiTreeNodeFlags flags, const char* fmt, va_list args) Moss__FMTLIST(3);
/*! @brief X.*/
MOSS_API bool          TreeNodeExV(const void* ptr_id, Moss_GuiTreeNodeFlags flags, const char* fmt, va_list args) Moss__FMTLIST(3);
/*! @brief X.*/
MOSS_API void          TreePush(const char* str_id);                                       // ~ Indent()+PushID(). Already called by TreeNode() when returning true, but you can call TreePush/TreePop yourself if desired.
/*! @brief X.*/
MOSS_API void          TreePush(const void* ptr_id);                                       // "
/*! @brief X.*/
MOSS_API void          TreePop();                                                          // ~ Unindent()+PopID()
/*! @brief X.*/
MOSS_API float         GetTreeNodeToLabelSpacing();                                        // horizontal distance preceding label when using TreeNode*() or Bullet() == (g.FontSize + style.FramePadding.x*2) for a regular unframed TreeNode
/*! @brief X.*/
MOSS_API bool          CollapsingHeader(const char* label, Moss_GuiTreeNodeFlags flags = 0);  // if returning 'true' the header is open. doesn't indent nor push on ID stack. user doesn't have to call TreePop().
/*! @brief X.*/
MOSS_API bool          CollapsingHeader(const char* label, bool* p_visible, Moss_GuiTreeNodeFlags flags = 0); // when 'p_visible != NULL': if '*p_visible==true' display an additional small close button on upper right of the header which will set the bool to false when clicked, if '*p_visible==false' don't display the header.
/*! @brief X.*/
MOSS_API void          SetNextItemOpen(bool is_open, Moss_GuiCond cond = 0);                  // set next TreeNode/CollapsingHeader open state.
/*! @brief X.*/
MOSS_API void          SetNextItemStorageID(uint32 storage_id);                           // set id to use for open/close storage (default to same as item id).

// Widgets: Selectables
    // - A selectable highlights when hovered, and can display another color when selected.
    // - Neighbors selectable extend their highlight bounds in order to leave no gap between them. This is so a series of selected Selectable appear contiguous.
MOSS_API bool          Selectable(const char* label, bool selected = false, Moss_GuiSelectableFlags flags = 0, const Float2& size = Float2(0, 0)); // "bool selected" carry the selection state (read-only). Selectable() is clicked is returns true so you can modify your selection state. size.x==0.0: use remaining width, size.x>0.0: specify width. size.y==0.0: use label height, size.y>0.0: specify height
MOSS_API bool          Selectable(const char* label, bool* p_selected, Moss_GuiSelectableFlags flags = 0, const Float2& size = Float2(0, 0));      // "bool* p_selected" point to the selection state (read-write), as a convenient helper.

    // Multi-selection system for Selectable(), Checkbox(), TreeNode() functions [BETA]
    // - This enables standard multi-selection/range-selection idioms (CTRL+Mouse/Keyboard, SHIFT+Mouse/Keyboard, etc.) in a way that also allow a clipper to be used.
    // - Moss_GuiSelectionUserData is often used to store your item index within the current view (but may store something else).
    // - Read comments near Moss_GuMoss_ultiSelectIO for instructions/details and see 'Demo->Widgets->Selection State & Multi-Select' for demo.
    // - TreeNode() is technically supported but... using this correctly is more complicated. You need some sort of linear/random access to your tree,
    //   which is suited to advanced trees setups already Moss_plementing filters and clipper. We will work sMoss_plifying the current demo.
    // - 'selection_size' and 'items_count' parameters are optional and used by a few features. If they are costly for you to compute, you may avoid them.
MOSS_API Moss_GuiMultiSelectIO*   BeginMultiSelect(Moss_ultiSelectFlags flags, int selection_size = -1, int items_count = -1);
MOSS_API Moss_GuiMultiSelectIO*   EndMultiSelect();
MOSS_API void                  SetNextItemSelectionUserData(Moss_GuiSelectionUserData selection_user_data);
MOSS_API bool                  IsItemToggledSelection();                                   // Was the last item selection state toggled? Useful if you need the per-item information _before_ reaching EndMultiSelect(). We only returns toggle _event_ in order to handle clipping correctly.

// Widgets: List Boxes
    // - This is essentially a thin wrapper to using BeginChild/EndChild with the Moss_GuiChildFlags_FrameStyle flag for stylistic changes + displaying a label.
    // - If you don't need a label you can probably sMoss_ply use BeginChild() with the Moss_GuiChildFlags_FrameStyle flag for the same result.
    // - You can submit contents and manage your selection state however you want it, by creating e.g. Selectable() or any other items.
    // - The sMoss_plified/old ListBox() api are helpers over BeginListBox()/EndListBox() which are kept available for convenience purpose. This is analogous to how Combos are created.
    // - Choose frame width:   size.x > 0.0f: custom  /  size.x < 0.0f or -FLT_MIN: right-align   /  size.x = 0.0f (default): use current ItemWidth
    // - Choose frame height:  size.y > 0.0f: custom  /  size.y < 0.0f or -FLT_MIN: bottom-align  /  size.y = 0.0f (default): arbitrary default height which can fit ~7 items
MOSS_API bool          BeginListBox(const char* label, const Float2& size = Float2(0, 0)); // open a framed scrolling region
MOSS_API void          EndListBox();                                                       // only call EndListBox() if BeginListBox() returned true!
MOSS_API bool          ListBox(const char* label, int* current_item, const char* const items[], int items_count, int height_in_items = -1);
MOSS_API bool          ListBox(const char* label, int* current_item, const char* (*getter)(void* user_data, int idx), void* user_data, int items_count, int height_in_items = -1);

// Widgets: Menus
    // - Use BeginMenuBar() on a window Moss_GuiWindowFlags_MenuBar to append to its menu bar.
    // - Use BeginMainMenuBar() to create a menu bar at the top of the screen and append to it.
    // - Use BeginMenu() to create a menu. You can call BeginMenu() multiple tMoss_e with the same identifier to append more items to it.
    // - Not that MenuItem() keyboardshortcuts are displayed as a convenience but _not processed_ by Dear Moss_Gui at the moment.
MOSS_API bool          BeginMenuBar();                                                     // append to menu-bar of current window (requires Moss_GuiWindowFlags_MenuBar flag set on parent window).
MOSS_API void          EndMenuBar();                                                       // only call EndMenuBar() if BeginMenuBar() returns true!
MOSS_API bool          BeginMainMenuBar();                                                 // create and append to a full screen menu-bar.
MOSS_API void          EndMainMenuBar();                                                   // only call EndMainMenuBar() if BeginMainMenuBar() returns true!
MOSS_API bool          BeginMenu(const char* label, bool enabled = true);                  // create a sub-menu entry. only call EndMenu() if this returns true!
MOSS_API void          EndMenu();                                                          // only call EndMenu() if BeginMenu() returns true!
MOSS_API bool          MenuItem(const char* label, const char* shortcut = NULL, bool selected = false, bool enabled = true);  // return true when activated.
MOSS_API bool          MenuItem(const char* label, const char* shortcut, bool* p_selected, bool enabled = true);              // return true when activated + toggle (*p_selected) if p_selected != NULL

    // Tooltips
    // - Tooltips are windows following the mouse. They do not take focus away.
    // - A tooltip window can contain items of any types.
    // - SetTooltip() is more or less a shortcut for the 'if (BeginTooltip()) { Text(...); EndTooltip(); }' idiom (with a subtlety that it discard any previously submitted tooltip)
MOSS_API bool          BeginTooltip();                                                     // begin/append a tooltip window.
MOSS_API void          EndTooltip();                                                       // only call EndTooltip() if BeginTooltip()/BeginItemTooltip() returns true!
MOSS_API void          SetTooltip(const char* fmt, ...) Moss__FMTARGS(1);                     // set a text-only tooltip. Often used after a Moss_Gui::IsItemHovered() check. Override any previous call to SetTooltip().
MOSS_API void          SetTooltipV(const char* fmt, va_list args) Moss__FMTLIST(1);

    // Tooltips: helpers for showing a tooltip when hovering an item
    // - BeginItemTooltip() is a shortcut for the 'if (IsItemHovered(Moss_GuiHoveredFlags_ForTooltip) && BeginTooltip())' idiom.
    // - SetItemTooltip() is a shortcut for the 'if (IsItemHovered(Moss_GuiHoveredFlags_ForTooltip)) { SetTooltip(...); }' idiom.
    // - Where 'Moss_GuiHoveredFlags_ForTooltip' itself is a shortcut to use 'style.HoverFlagsForTooltipMouse' or 'style.HoverFlagsForTooltipNav' depending on active input type. For mouse it defaults to 'Moss_GuiHoveredFlags_Stationary | Moss_GuiHoveredFlags_DelayShort'.
MOSS_API bool          BeginItemTooltip();                                                 // begin/append a tooltip window if preceding item was hovered.
MOSS_API void          SetItemTooltip(const char* fmt, ...) Moss__FMTARGS(1);                 // set a text-only tooltip if preceding item was hovered. override any previous call to SetTooltip().
MOSS_API void          SetItemTooltipV(const char* fmt, va_list args) Moss__FMTLIST(1);

    // Popups, Modals
    //  - They block normal mouse hovering detection (and therefore most mouse interactions) behind them.
    //  - If not modal: they can be closed by clicking anywhere outside them, or by pressing ESCAPE.
    //  - Their visibility state (~bool) is held internally instead of being held by the programmer as we are used to with regular Begin*() calls.
    //  - The 3 properties above are related: we need to retain popup visibility state in the library because popups may be closed as any tMoss_e.
    //  - You can bypass the hovering restriction by using Moss_GuiHoveredFlags_AllowWhenBlockedByPopup when calling IsItemHovered() or IsWindowHovered().
    //  - Moss_PORTANT: Popup identifiers are relative to the current ID stack, so OpenPopup and BeginPopup generally needs to be at the same level of the stack.
    //    This is sometMoss_es leading to confusing mistakes. May rework this in the future.
    //  - BeginPopup(): query popup state, if open start appending into the window. Call EndPopup() afterwards if returned true. Moss_GuiWindowFlags are forwarded to the window.
    //  - BeginPopupModal(): block every interaction behind the window, cannot be closed by user, add a dMoss_ming background, has a title bar.
MOSS_API bool          BeginPopup(const char* str_id, Moss_GuiWindowFlags flags = 0);                         // return true if the popup is open, and you can start outputting to it.
MOSS_API bool          BeginPopupModal(const char* name, bool* p_open = NULL, Moss_GuiWindowFlags flags = 0); // return true if the modal is open, and you can start outputting to it.
MOSS_API void          EndPopup();                                                                         // only call EndPopup() if BeginPopupXXX() returns true!

    // Popups: open/close functions
    //  - OpenPopup(): set popup state to open. Moss_GuiPopupFlags are available for opening options.
    //  - If not modal: they can be closed by clicking anywhere outside them, or by pressing ESCAPE.
    //  - CloseCurrentPopup(): use inside the BeginPopup()/EndPopup() scope to close manually.
    //  - CloseCurrentPopup() is called by default by Selectable()/MenuItem() when activated (FIXME: need some options).
    //  - Use Moss_GuiPopupFlags_NoOpenOverExistingPopup to avoid opening a popup if there's already one at the same level. This is equivalent to e.g. testing for !IsAnyPopupOpen() prior to OpenPopup().
    //  - Use IsWindowAppearing() after BeginPopup() to tell if a window just opened.
    //  - Moss_PORTANT: Notice that for OpenPopupOnItemClick() we exceptionally default flags to 1 (== Moss_GuiPopupFlags_MouseButtonRight) for backward compatibility with older API taking 'int mouse_button = 1' parameter
MOSS_API void          OpenPopup(const char* str_id, Moss_GuiPopupFlags popup_flags = 0);                     // call to mark popup as open (don't call every frame!).
MOSS_API void          OpenPopup(uint32 id, Moss_GuiPopupFlags popup_flags = 0);                             // id overload to facilitate calling from nested stacks
MOSS_API void          OpenPopupOnItemClick(const char* str_id = NULL, Moss_GuiPopupFlags popup_flags = 1);   // helper to open popup when clicked on last item. Default to Moss_GuiPopupFlags_MouseButtonRight == 1. (note: actually triggers on the mouse _released_ event to be consistent with popup behaviors)
MOSS_API void          CloseCurrentPopup();                                                                // manually close the popup we have begin-ed into.

    // Popups: open+begin combined functions helpers
    //  - Helpers to do OpenPopup+BeginPopup where the Open action is triggered by e.g. hovering an item and right-clicking.
    //  - They are convenient to easily create context menus, hence the name.
    //  - Moss_PORTANT: Notice that BeginPopupContextXXX takes Moss_GuiPopupFlags just like OpenPopup() and unlike BeginPopup(). For full consistency, we may add Moss_GuiWindowFlags to the BeginPopupContextXXX functions in the future.
    //  - Moss_PORTANT: Notice that we exceptionally default their flags to 1 (== Moss_GuiPopupFlags_MouseButtonRight) for backward compatibility with older API taking 'int mouse_button = 1' parameter, so if you add other flags remember to re-add the Moss_GuiPopupFlags_MouseButtonRight.
MOSS_API bool          BeginPopupContextItem(const char* str_id = NULL, Moss_GuiPopupFlags popup_flags = 1);  // open+begin popup when clicked on last item. Use str_id==NULL to associate the popup to previous item. If you want to use that on a non-interactive item such as Text() you need to pass in an explicit ID here. read comments in .cpp!
MOSS_API bool          BeginPopupContextWindow(const char* str_id = NULL, Moss_GuiPopupFlags popup_flags = 1);// open+begin popup when clicked on current window.
MOSS_API bool          BeginPopupContextVoid(const char* str_id = NULL, Moss_GuiPopupFlags popup_flags = 1);  // open+begin popup when clicked in void (where there are no windows).

    // Popups: query functions
    //  - IsPopupOpen(): return true if the popup is open at the current BeginPopup() level of the popup stack.
    //  - IsPopupOpen() with Moss_GuiPopupFlags_AnyPopupId: return true if any popup is open at the current BeginPopup() level of the popup stack.
    //  - IsPopupOpen() with Moss_GuiPopupFlags_AnyPopupId + Moss_GuiPopupFlags_AnyPopupLevel: return true if any popup is open.
MOSS_API bool          IsPopupOpen(const char* str_id, Moss_GuiPopupFlags flags = 0);                         // return true if the popup is open.

    // Tables
    // - Full-featured replacement for old Columns API.
    // - See Demo->Tables for demo code. See top of Moss_gui_tables.cpp for general commentary.
    // - See Moss_GuiTableFlags_ and Moss_GuiTableColumnFlags_ enums for a description of available flags.
    // The typical call flow is:
    // - 1. Call BeginTable(), early out if returning false.
    // - 2. Optionally call TableSetupColumn() to submit column name/flags/defaults.
    // - 3. Optionally call TableSetupScrollFreeze() to request scroll freezing of columns/rows.
    // - 4. Optionally call TableHeadersRow() to submit a header row. Names are pulled from TableSetupColumn() data.
    // - 5. Populate contents:
    //    - In most situations you can use TableNextRow() + TableSetColumnIndex(N) to start appending into a column.
    //    - If you are using tables as a sort of grid, where every column is holding the same type of contents,
    //      you may prefer using TableNextColumn() instead of TableNextRow() + TableSetColumnIndex().
    //      TableNextColumn() will automatically wrap-around into the next row if needed.
    //    - Moss_PORTANT: Comparatively to the old Columns() API, we need to call TableNextColumn() for the first column!
    //    - Summary of possible call flow:
    //        - TableNextRow() -> TableSetColumnIndex(0) -> Text("Hello 0") -> TableSetColumnIndex(1) -> Text("Hello 1")  // OK
    //        - TableNextRow() -> TableNextColumn()      -> Text("Hello 0") -> TableNextColumn()      -> Text("Hello 1")  // OK
    //        -                   TableNextColumn()      -> Text("Hello 0") -> TableNextColumn()      -> Text("Hello 1")  // OK: TableNextColumn() automatically gets to next row!
    //        - TableNextRow()                           -> Text("Hello 0")                                               // Not OK! Missing TableSetColumnIndex() or TableNextColumn()! Text will not appear!
    // - 5. Call EndTable()
MOSS_API bool          BeginTable(const char* str_id, int columns, Moss_GuiTableFlags flags = 0, const Float2& outer_size = Float2(0.0f, 0.0f), float inner_width = 0.0f);
MOSS_API void          EndTable();                                         // only call EndTable() if BeginTable() returns true!
MOSS_API void          TableNextRow(Moss_GuiTableRowFlags row_flags = 0, float min_row_height = 0.0f); // append into the first cell of a new row.
MOSS_API bool          TableNextColumn();                                  // append into the next column (or first column of next row if currently in last column). Return true when column is visible.
MOSS_API bool          TableSetColumnIndex(int column_n);                  // append into the specified column. Return true when column is visible.

    // Tables: Headers & Columns declaration
    // - Use TableSetupColumn() to specify label, resizing policy, default width/weight, id, various other flags etc.
    // - Use TableHeadersRow() to create a header row and automatically submit a TableHeader() for each column.
    //   Headers are required to perform: reordering, sorting, and opening the context menu.
    //   The context menu can also be made available in columns body using Moss_GuiTableFlags_ContextMenuInBody.
    // - You may manually submit headers using TableNextRow() + TableHeader() calls, but this is only useful in
    //   some advanced use cases (e.g. adding custom widgets in header row).
    // - Use TableSetupScrollFreeze() to lock columns/rows so they stay visible when scrolled.
MOSS_API void          TableSetupColumn(const char* label, Moss_GuiTableColumnFlags flags = 0, float init_width_or_weight = 0.0f, uint32 user_id = 0);
MOSS_API void          TableSetupScrollFreeze(int cols, int rows);         // lock columns/rows so they stay visible when scrolled.
MOSS_API void          TableHeader(const char* label);                     // submit one header cell manually (rarely used)
MOSS_API void          TableHeadersRow();                                  // submit a row with headers cells based on data provided to TableSetupColumn() + submit context menu
MOSS_API void          TableAngledHeadersRow();                            // submit a row with angled headers for every column with the Moss_GuiTableColumnFlags_AngledHeader flag. MUST BE FIRST ROW.

    // Tables: Sorting & Miscellaneous functions
    // - Sorting: call TableGetSortSpecs() to retrieve latest sort specs for the table. NULL when not sorting.
    //   When 'sort_specs->SpecsDirty == true' you should sort your data. It will be true when sorting specs have
    //   changed since last call, or the first tMoss_e. Make sure to set 'SpecsDirty = false' after sorting,
    //   else you may wastefully sort your data every frame!
    // - Functions args 'int column_n' treat the default value of -1 as the same as passing the current column index.
MOSS_API Moss_GuiTableSortSpecs*  TableGetSortSpecs();                        // get latest sort specs for the table (NULL if not sorting).  LifetMoss_e: don't hold on this pointer over multiple frames or past any subsequent call to BeginTable().
MOSS_API int                   TableGetColumnCount();                      // return number of columns (value passed to BeginTable)
MOSS_API int                   TableGetColumnIndex();                      // return current column index.
MOSS_API int                   TableGetRowIndex();                         // return current row index.
MOSS_API const char*           TableGetColumnName(int column_n = -1);      // return "" if column didn't have a name declared by TableSetupColumn(). Pass -1 to use current column.
MOSS_API Moss_GuiTableColumnFlags TableGetColumnFlags(int column_n = -1);     // return column flags so you can query their Enabled/Visible/Sorted/Hovered status flags. Pass -1 to use current column.
MOSS_API void                  TableSetColumnEnabled(int column_n, bool v);// change user accessible enabled/disabled state of a column. Set to false to hide the column. User can use the context menu to change this themselves (right-click in headers, or right-click in columns body with Moss_GuiTableFlags_ContextMenuInBody)
MOSS_API int                   TableGetHoveredColumn();                    // return hovered column. return -1 when table is not hovered. return columns_count if the unused space at the right of visible columns is hovered. Can also use (TableGetColumnFlags() & Moss_GuiTableColumnFlags_IsHovered) instead.
MOSS_API void                  TableSetBgColor(Moss_GuiTableBgTarget target, uint32 color, int column_n = -1);  // change the color of a cell, row, or column. See Moss_GuiTableBgTarget_ flags for details.

    // Tab Bars, Tabs
    // - Note: Tabs are automatically created by the docking system (when in 'docking' branch). Use this to create tab bars/tabs yourself.
MOSS_API bool          BeginTabBar(const char* str_id, Moss_GuiTabBarFlags flags = 0);        // create and append into a TabBar
MOSS_API void          EndTabBar();                                                        // only call EndTabBar() if BeginTabBar() returns true!
MOSS_API bool          BeginTabItem(const char* label, bool* p_open = NULL, Moss_GuiTabItemFlags flags = 0); // create a Tab. Returns true if the Tab is selected.
MOSS_API void          EndTabItem();                                                       // only call EndTabItem() if BeginTabItem() returns true!
MOSS_API bool          TabItemButton(const char* label, Moss_GuiTabItemFlags flags = 0);      // create a Tab behaving like a button. return true when clicked. cannot be selected in the tab bar.
MOSS_API void          SetTabItemClosed(const char* tab_or_docked_window_label);           // notify TabBar or Docking system of a closed tab/window ahead (useful to reduce visual flicker on reorderable tab bars). For tab-bar: call after BeginTabBar() and before Tab submissions. Otherwise call with a window name.

    // Drag and Drop
    // - On source items, call BeginDragDropSource(), if it returns true also call SetDragDropPayload() + EndDragDropSource().
    // - On target candidates, call BeginDragDropTarget(), if it returns true also call AcceptDragDropPayload() + EndDragDropTarget().
    // - If you stop calling BeginDragDropSource() the payload is preserved however it won't have a preview tooltip (we currently display a fallback "..." tooltip, see #1725)
    // - An item can be both drag source and drop target.
MOSS_API bool          BeginDragDropSource(Moss_GuiDragDropFlags flags = 0);                                      // call after submitting an item which may be dragged. when this return true, you can call SetDragDropPayload() + EndDragDropSource()
MOSS_API bool          SetDragDropPayload(const char* type, const void* data, size_t sz, Moss_GuiCond cond = 0);  // type is a user defined string of maxMoss_um 32 characters. Strings starting with '_' are reserved for dear Moss_gui internal types. Data is copied and held by Moss_gui. Return true when payload has been accepted.
MOSS_API void          EndDragDropSource();                                                                    // only call EndDragDropSource() if BeginDragDropSource() returns true!
MOSS_API bool                  BeginDragDropTarget();                                                          // call after submitting an item that may receive a payload. If this returns true, you can call AcceptDragDropPayload() + EndDragDropTarget()
MOSS_API const Moss_GuiPayload*   AcceptDragDropPayload(const char* type, Moss_GuiDragDropFlags flags = 0);          // accept contents of a given type. If Moss_GuiDragDropFlags_AcceptBeforeDelivery is set you can peek into the payload before the mouse button is released.
MOSS_API void                  EndDragDropTarget();                                                            // only call EndDragDropTarget() if BeginDragDropTarget() returns true!
MOSS_API const Moss_GuiPayload*   GetDragDropPayload();                                                           // peek directly into the current payload from anywhere. returns NULL when drag and drop is finished or inactive. use Moss_GuiPayload::IsDataType() to test for the payload type.
    // Clipping
    // - Mouse hovering is affected by Moss_Gui::PushClipRect() calls, unlike direct calls to Moss_DrawList::PushClipRect() which are render only.
MOSS_API void          PushClipRect(const Float2& clip_rect_min, const Float2& clip_rect_max, bool intersect_with_current_clip_rect);
MOSS_API void          PopClipRect();

    // Focus, Activation
MOSS_API void          SetItemDefaultFocus();                                              // make last item the default focused item of a newly appearing window.
MOSS_API void          SetKeyboardFocusHere(int offset = 0);                               // focus keyboard on the next widget. Use positive 'offset' to access sub components of a multiple component widget. Use -1 to access previous widget.

    // Keyboard/Gamepad Navigation
MOSS_API void          SetNavCursorVisible(bool visible);                                  // alter visibility of keyboard/gamepad cursor. by default: show when using an arrow key, hide when clicking with mouse.

    // Overlapping mode
MOSS_API void          SetNextItemAllowOverlap();                                          // allow next item to be overlapped by a subsequent item. Useful with invisible buttons, selectable, treenode covering an area where subsequent items may need to be added. Note that both Selectable() and TreeNode() have dedicated flags doing this.

    // Viewports
    // - Currently represents the Platform Window created by the application which is hosting our Dear Moss_Gui windows.
    // - In 'docking' branch with multi-viewport enabled, we extend this concept to have multiple active viewports.
    // - In the future we will extend this concept further to also represent Platform Monitor and support a "no main platform window" operation mode.
MOSS_API Moss_GuiViewport* GetMainViewport();                                                 // return prMoss_ary/default viewport. This can never be NULL.

MOSS_API bool          Shortcut(Moss_GuiKeyChord key_chord, Moss_GuiInputFlags flags = 0);
MOSS_API void          SetNextItemShortcut(Moss_GuiKeyChord key_chord, Moss_GuiInputFlags flags = 0);

// GraphNode
MOSS_API void Graph();
MOSS_API void GraphNode();


// AnMoss_ated Icon
MOSS_API void AnMoss_atedIcon();

// Dialogue
MOSS_API bool Dialogue();

MOSS_API void CodeEditor();
MOSS_API void HexEditor();

/*! DatePicker that allows you to select a date these can be Days, Months, and Years*/
MOSS_API void DatePicker();





// Utility

/*
    // Color Utilities
MOSS_API Float4        ColorConvertU32ToFloat4(uint32 in);
MOSS_API uint32        ColorConvertFloat4ToU32(const Float4& in);
MOSS_API void          ColorConvertRGBtoHSV(float r, float g, float b, float& out_h, float& out_s, float& out_v);
MOSS_API void          ColorConvertHSVtoRGB(float h, float s, float v, float& out_r, float& out_g, float& out_b);
//==========================================================
// Main
MOSS_API Moss_GuiIO&      GetIO();                                    // access the Moss_GuiIO structure (mouse/keyboard/gamepad inputs, tMoss_e, various configuration options/flags)
MOSS_API Moss_GuiPlatformIO& GetPlatformIO();                         // access the Moss_GuiPlatformIO structure (mostly hooks/functions to connect to platform/renderer and OS Clipboard, Moss_E etc.)
MOSS_API Moss_Style&   GetStyle();                                 // access the Style structure (colors, sizes). Always use PushStyleColor(), PushStyleVar() to modify style mid-frame!
MOSS_API void          Moss_GUIBeginFrame();                       // start a new Dear Moss_Gui frame, you can submit any command from this point until Render()/EndFrame().
MOSS_API void          Moss_GUIEndFrame();                         // ends the Dear Moss_Gui frame. automatically called by Render(). If you don't need to render data (skipping rendering) you may call EndFrame() without Render()... but you'll have wasted CPU already! If you don't need to render, better to not create any windows and not call NewFrame() at all!
MOSS_API void          Render();                                   // ends the Dear Moss_Gui frame, finalize the draw data. You can then get call GetDrawData().
MOSS_API Moss_DrawData*   GetDrawData();                              // valid after Render() and until the next call to NewFrame(). this is what you have to render.

// Styles
MOSS_API void          StyleColorsDark(Moss_Theme* theme = NULL);    // (default)
MOSS_API void          StyleColorsLight(Moss_Theme* theme = NULL);   // OH GOD MY EYES

// Scrolling
MOSS_API float         GetScrollX();                                                   // get scrolling amount [0 .. GetScrollMaxX()]
MOSS_API float         GetScrollY();                                                   // get scrolling amount [0 .. GetScrollMaxY()]
MOSS_API void          SetScrollX(float scroll_x);                                     // set scrolling amount [0 .. GetScrollMaxX()]
MOSS_API void          SetScrollY(float scroll_y);                                     // set scrolling amount [0 .. GetScrollMaxY()]
MOSS_API float         GetScrollMaxX();                                                // get maxMoss_um scrolling amount ~~ ContentSize.x - WindowSize.x - DecorationsSize.x
MOSS_API float         GetScrollMaxY();                                                // get maxMoss_um scrolling amount ~~ ContentSize.y - WindowSize.y - DecorationsSize.y
MOSS_API void          SetScrollHereX(float center_x_ratio = 0.5f);                    // adjust scrolling amount to make current cursor position visible. center_x_ratio=0.0: left, 0.5: center, 1.0: right. When using to make a "default/current item" visible, consider using SetItemDefaultFocus() instead.
MOSS_API void          SetScrollHereY(float center_y_ratio = 0.5f);                    // adjust scrolling amount to make current cursor position visible. center_y_ratio=0.0: top, 0.5: center, 1.0: bottom. When using to make a "default/current item" visible, consider using SetItemDefaultFocus() instead.
MOSS_API void          SetScrollFromPosX(float local_x, float center_x_ratio = 0.5f);  // adjust scrolling amount to make given position visible. Generally GetCursorStartPos() + offset to compute a valid position.
MOSS_API void          SetScrollFromPosY(float local_y, float center_y_ratio = 0.5f);  // adjust scrolling amount to make given position visible. Generally GetCursorStartPos() + offset to compute a valid position.

// Parameters stacks (shared)
MOSS_API void          PushFont(Moss_Font* font);                                         // use NULL as a shortcut to push default font
MOSS_API void          PopFont();
MOSS_API void          PushStyleColor(Moss_GuiCol idx, uint32 col);                        // modify a style color. always use this if you modify the style after NewFrame().
MOSS_API void          PushStyleColor(Moss_GuiCol idx, const Float4& col);
MOSS_API void          PopStyleColor(int count = 1);
MOSS_API void          PushStyleVar(int idx, float val);                     // modify a style float variable. always use this if you modify the style after NewFrame()!
MOSS_API void          PushStyleVar(int idx, const Float2& val);             // modify a style Float2 variable. "
MOSS_API void          PushStyleVarX(int idx, float val_x);                  // modify X component of a style Float2 variable. "
MOSS_API void          PushStyleVarY(int idx, float val_y);                  // modify Y component of a style Float2 variable. "
MOSS_API void          PopStyleVar(int count = 1);
MOSS_API void          PushItemFlag(Moss_GuiItemFlags option, bool enabled);              // modify specified shared item flag, e.g. PushItemFlag(Moss_GuiItemFlags_NoTabStop, true)
MOSS_API void          PopItemFlag();

// Parameters stacks (current window)
MOSS_API void          PushItemWidth(float item_width);                                // push width of items for common large "item+label" widgets. >0.0f: width in pixels, <0.0f align xx pixels to the right of window (so -FLT_MIN always align width to the right side).
MOSS_API void          PopItemWidth();
MOSS_API void          SetNextItemWidth(float item_width);                             // set width of the _next_ common large "item+label" widget. >0.0f: width in pixels, <0.0f align xx pixels to the right of window (so -FLT_MIN always align width to the right side)
MOSS_API float         CalcItemWidth();                                                // width of item given pushed settings and current cursor position. NOT necessarily the width of last item unlike most 'Item' functions.
MOSS_API void          PushTextWrapPos(float wrap_local_pos_x = 0.0f);                 // push word-wrapping position for Text*() commands. < 0.0f: no wrapping; 0.0f: wrap to end of window (or column); > 0.0f: wrap at 'wrap_pos_x' position in window local space
MOSS_API void          PopTextWrapPos();

// Style read access
    // - Use the ShowStyleEditor() function to interactively see/edit the colors.
MOSS_API Moss_Font*       GetFont();                                                      // get current font
MOSS_API float         GetFontSize();                                                  // get current font size (= height in pixels) of current font with current scale applied
MOSS_API Float2        GetFontTexUvWhitePixel();                                       // get UV coordinate for a white pixel, useful to draw custom shapes via the Moss_DrawList API
MOSS_API uint32         GetColorU32(Moss_GuiCol idx, float alpha_mul = 1.0f);              // retrieve given style color with style alpha applied and optional extra alpha multiplier, packed as a 32-bit value suitable for Moss_DrawList
MOSS_API uint32         GetColorU32(const Float4& col);                                 // retrieve given color with style alpha applied, packed as a 32-bit value suitable for Moss_DrawList
MOSS_API uint32         GetColorU32(uint32 col, float alpha_mul = 1.0f);                 // retrieve given color with style alpha applied, packed as a 32-bit value suitable for Moss_DrawList
MOSS_API const Float4& GetStyleColorVec4(Moss_GuiCol idx);                                // retrieve style color as stored in Moss_GuiStyle structure. use to feed back into PushStyleColor(), otherwise use GetColorU32() to get style color with style alpha baked in.


// Layout cursor positioning
    // - By "cursor" we mean the current output position.
    // - The typical widget behavior is to output themselves at the current cursor position, then move the cursor one line down.
    // - You can call SameLine() between widgets to undo the last carriage return and output at the right of the preceding widget.
    // - YOU CAN DO 99% OF WHAT YOU NEED WITH ONLY GetCursorScreenPos() and GetContentRegionAvail().
    // - Attention! We currently have inconsistencies between window-local and absolute positions we will aMoss_ to fix with future API:
    //    - Absolute coordinate:        GetCursorScreenPos(), SetCursorScreenPos(), all Moss_DrawList:: functions. -> this is the preferred way forward.
    //    - Window-local coordinates:   SameLine(offset), GetCursorPos(), SetCursorPos(), GetCursorStartPos(), PushTextWrapPos()
    //    - Window-local coordinates:   GetContentRegionMax(), GetWindowContentRegionMin(), GetWindowContentRegionMax() --> all obsoleted. YOU DON'T NEED THEM.
    // - GetCursorScreenPos() = GetCursorPos() + GetWindowPos(). GetWindowPos() is almost only ever useful to convert from window-local to absolute coordinates. Try not to use it.
MOSS_API Float2        GetCursorScreenPos();                                           // cursor position, absolute coordinates. THIS IS YOUR BEST FRIEND (prefer using this rather than GetCursorPos(), also more useful to work with Moss_DrawList API).
MOSS_API void          SetCursorScreenPos(const Float2& pos);                          // cursor position, absolute coordinates. THIS IS YOUR BEST FRIEND.
MOSS_API Float2        GetContentRegionAvail();                                        // available space from current position. THIS IS YOUR BEST FRIEND.
MOSS_API Float2        GetCursorPos();                                                 // [window-local] cursor position in window-local coordinates. This is not your best friend.
MOSS_API float         GetCursorPosX();                                                // [window-local] "
MOSS_API float         GetCursorPosY();                                                // [window-local] "
MOSS_API void          SetCursorPos(const Float2& local_pos);                          // [window-local] "
MOSS_API void          SetCursorPosX(float local_x);                                   // [window-local] "
MOSS_API void          SetCursorPosY(float local_y);                                   // [window-local] "
MOSS_API Float2        GetCursorStartPos();                                            // [window-local] initial cursor position, in window-local coordinates. Call GetCursorScreenPos() after Begin() to get the absolute coordinates version.

// ================================================================================================================
// Other layout functions
MOSS_API void          Separator();                                                    // separator, generally horizontal. inside a menu bar or in horizontal layout mode, this becomes a vertical separator.
MOSS_API void          SameLine(float offset_from_start_x=0.0f, float spacing=-1.0f);  // call between widgets or groups to layout them horizontally. X position given in window coordinates.
MOSS_API void          NewLine();                                                      // undo a SameLine() or force a new line when in a horizontal-layout context.
MOSS_API void          Spacing();                                                      // add vertical spacing.
MOSS_API void          Dummy(const Float2& size);                                      // add a dummy item of given size. unlike InvisibleButton(), Dummy() won't take the mouse click or be navigable into.
MOSS_API void          Indent(float indent_w = 0.0f);                                  // move content position toward the right, by indent_w, or style.IndentSpacing if indent_w <= 0
MOSS_API void          Unindent(float indent_w = 0.0f);                                // move content position back to the left, by indent_w, or style.IndentSpacing if indent_w <= 0
MOSS_API void          BeginGroup();                                                   // lock horizontal starting position
MOSS_API void          EndGroup();                                                     // unlock horizontal starting position + capture the whole group bounding box into one "item" (so you can use IsItemHovered() or layout prMoss_itives such as SameLine() on whole group, etc.)
MOSS_API void          AlignTextToFramePadding();                                      // vertically align upcoming text baseline to FramePadding.y so that it will align properly to regularly framed items (call if you have text on a line before a framed item)
MOSS_API float         GetTextLineHeight();                                            // ~ FontSize
MOSS_API float         GetTextLineHeightWithSpacing();                                 // ~ FontSize + style.ItemSpacing.y (distance in pixels between 2 consecutive lines of text)
MOSS_API float         GetFrameHeight();                                               // ~ FontSize + style.FramePadding.y * 2
MOSS_API float         GetFrameHeightWithSpacing();                                    // ~ FontSize + style.FramePadding.y * 2 + style.ItemSpacing.y (distance in pixels between 2 consecutive lines of framed widgets)

// ID stack/scopes
    // Read the FAQ (docs/FAQ.md or http://dearMoss_gui.com/faq) for more details about how ID are handled in dear Moss_gui.
    // - Those questions are answered and Moss_pacted by understanding of the ID stack system:
    //   - "Q: Why is my widget not reacting when I click on it?"
    //   - "Q: How can I have widgets with an empty label?"
    //   - "Q: How can I have multiple widgets with the same label?"
    // - Short version: ID are hashes of the entire ID stack. If you are creating widgets in a loop you most likely
    //   want to push a unique identifier (e.g. object pointer, loop index) to uniquely differentiate them.
    // - You can also use the "Label##foobar" syntax within widget label to distinguish them from each others.
    // - In this header file we use the "label"/"name" terminology to denote a string that will be displayed + used as an ID,
    //   whereas "str_id" denote a string that is only used as an ID and not normally displayed.
MOSS_API void          PushID(const char* str_id);                                     // push string into the ID stack (will hash string).
MOSS_API void          PushID(const char* str_id_begin, const char* str_id_end);       // push string into the ID stack (will hash string).
MOSS_API void          PushID(const void* ptr_id);                                     // push pointer into the ID stack (will hash pointer).
MOSS_API void          PushID(int int_id);                                             // push integer into the ID stack (will hash integer).
MOSS_API void          PopID();                                                        // pop from the ID stack.
MOSS_API uint32        GetID(const char* str_id);                                      // calculate unique ID (hash of whole ID stack + given parameter). e.g. if you want to query into Moss_GuiStorage yourself
MOSS_API uint32        GetID(const char* str_id_begin, const char* str_id_end);
MOSS_API uint32        GetID(const void* ptr_id);
MOSS_API uint32        GetID(int int_id);
// Windows Utilities
    // - 'current window' = the window we are appending into while inside a Begin()/End() block. 'next window' = next window we will Begin() into.
MOSS_API bool          IsWindowAppearing();
MOSS_API bool          IsWindowCollapsed();
MOSS_API bool          IsWindowFocused(Moss_GuiFocusedFlags flags=0); // is current window focused? or its root/child, depending on flags. see flags for options.
MOSS_API bool          IsWindowHovered(Moss_GuiHoveredFlags flags=0); // is current window hovered and hoverable (e.g. not blocked by a popup/modal)? See Moss_GuiHoveredFlags_ for options. Moss_PORTANT: If you are trying to check whether your mouse should be dispatched to Dear Moss_Gui or to your underlying app, you should not use this function! Use the 'io.WantCaptureMouse' boolean for that! Refer to FAQ entry "How can I tell whether to dispatch mouse/keyboard to Dear Moss_Gui or my application?" for details.
MOSS_API Moss_DrawList*   GetWindowDrawList();                        // get draw list associated to the current window, to append your own drawing prMoss_itives
MOSS_API Float2        GetWindowPos();                             // get current window position in screen space (IT IS UNLIKELY YOU EVER NEED TO USE THIS. Consider always using GetCursorScreenPos() and GetContentRegionAvail() instead)
MOSS_API Float2        GetWindowSize();                            // get current window size (IT IS UNLIKELY YOU EVER NEED TO USE THIS. Consider always using GetCursorScreenPos() and GetContentRegionAvail() instead)
MOSS_API float         GetWindowWidth();                           // get current window width (IT IS UNLIKELY YOU EVER NEED TO USE THIS). Shortcut for GetWindowSize().x.
MOSS_API float         GetWindowHeight();                          // get current window height (IT IS UNLIKELY YOU EVER NEED TO USE THIS). Shortcut for GetWindowSize().y.

// Window manipulation
    // - Prefer using SetNextXXX functions (before Begin) rather that SetXXX functions (after Begin).
MOSS_API void          SetNextWindowPos(const Float2& pos, Moss_GuiCond cond = 0, const Float2& pivot = Float2(0, 0)); // set next window position. call before Begin(). use pivot=(0.5f,0.5f) to center on given point, etc.
MOSS_API void          SetNextWindowSize(const Float2& size, Moss_GuiCond cond = 0);                  // set next window size. set axis to 0.0f to force an auto-fit on this axis. call before Begin()
MOSS_API void          SetNextWindowSizeConstraints(const Float2& size_min, const Float2& size_max, Moss_GuiSizeCallback custom_callback = NULL, void* custom_callback_data = NULL); // set next window size lMoss_its. use 0.0f or FLT_MAX if you don't want lMoss_its. Use -1 for both min and max of same axis to preserve current size (which itself is a constraint). Use callback to apply non-trivial programmatic constraints.
MOSS_API void          SetNextWindowContentSize(const Float2& size);                               // set next window content size (~ scrollable client area, which enforce the range of scrollbars). Not including window decorations (title bar, menu bar, etc.) nor WindowPadding. set an axis to 0.0f to leave it automatic. call before Begin()
MOSS_API void          SetNextWindowCollapsed(bool collapsed, Moss_GuiCond cond = 0);                 // set next window collapsed state. call before Begin()
MOSS_API void          SetNextWindowFocus();                                                       // set next window to be focused / top-most. call before Begin()
MOSS_API void          SetNextWindowScroll(const Float2& scroll);                                  // set next window scrolling value (use < 0.0f to not affect a given axis).
MOSS_API void          SetNextWindowBgAlpha(float alpha);                                          // set next window background color alpha. helper to easily override the Alpha component of Moss_GuiCol_WindowBg/ChildBg/PopupBg. you may also use Moss_GuiWindowFlags_NoBackground.
MOSS_API void          SetWindowPos(const Float2& pos, Moss_GuiCond cond = 0);                        // (not recommended) set current window position - call within Begin()/End(). prefer using SetNextWindowPos(), as this may incur tearing and side-effects.
MOSS_API void          SetWindowSize(const Float2& size, Moss_GuiCond cond = 0);                      // (not recommended) set current window size - call within Begin()/End(). set to Float2(0, 0) to force an auto-fit. prefer using SetNextWindowSize(), as this may incur tearing and minor side-effects.
MOSS_API void          SetWindowCollapsed(bool collapsed, Moss_GuiCond cond = 0);                     // (not recommended) set current window collapsed state. prefer using SetNextWindowCollapsed().
MOSS_API void          SetWindowFocus();                                                           // (not recommended) set current window to be focused / top-most. prefer using SetNextWindowFocus().
MOSS_API void          SetWindowFontScale(float scale);                                            // [OBSOLETE] set font scale. Adjust IO.FontGlobalScale if you want to scale all windows. This is an old API! For correct scaling, prefer to reload font + rebuild Moss_FontAtlas + call style.ScaleAllSizes().
MOSS_API void          SetWindowPos(const char* name, const Float2& pos, Moss_GuiCond cond = 0);      // set named window position.
MOSS_API void          SetWindowSize(const char* name, const Float2& size, Moss_GuiCond cond = 0);    // set named window size. set axis to 0.0f to force an auto-fit on this axis.
MOSS_API void          SetWindowCollapsed(const char* name, bool collapsed, Moss_GuiCond cond = 0);   // set named window collapsed state
MOSS_API void          SetWindowFocus(const char* name);                                           // set named window to be focused / top-most. use NULL to remove focus.

// Memory Allocators
    // - Those functions are not reliant on the current context.
    // - DLL users: heaps and globals are not shared across DLL boundaries! You will need to call SetCurrentContext() + SetAllocatorFunctions()
    //   for each static/DLL boundary you are calling from. Read "Context and Memory Allocators" section of Moss_gui.cpp for more details.
MOSS_API void          SetAllocatorFunctions(Moss_GuMoss_emAllocFunc alloc_func, Moss_GuMoss_emFreeFunc free_func, void* user_data = NULL);
MOSS_API void          GetAllocatorFunctions(Moss_GuMoss_emAllocFunc* p_alloc_func, Moss_GuMoss_emFreeFunc* p_free_func, void** p_user_data);
MOSS_API void*         MemAlloc(size_t size);
MOSS_API void          MemFree(void* ptr);

// Inputs Utilities: Mouse
    // - To refer to a mouse button, you may use named enums in your code e.g. Moss_GuMoss_ouseButton_Left, Moss_GuMoss_ouseButton_Right.
    // - You can also use regular integer: it is forever guaranteed that 0=Left, 1=Right, 2=Middle.
    // - Dragging operations are only reported after mouse has moved a certain distance away from the initial clicking position (see 'lock_threshold' and 'io.MouseDraggingThreshold')
MOSS_API bool          IsMouseDown(Moss_GuMoss_ouseButton button);                               // is mouse button held?
MOSS_API bool          IsMouseClicked(Moss_GuMoss_ouseButton button, bool repeat = false);       // did mouse button clicked? (went from !Down to Down). Same as GetMouseClickedCount() == 1.
MOSS_API bool          IsMouseReleased(Moss_GuMoss_ouseButton button);                           // did mouse button released? (went from Down to !Down)
MOSS_API bool          IsMouseDoubleClicked(Moss_GuMoss_ouseButton button);                      // did mouse button double-clicked? Same as GetMouseClickedCount() == 2. (note that a double-click will also report IsMouseClicked() == true)
MOSS_API bool          IsMouseReleasedWithDelay(Moss_GuMoss_ouseButton button, float delay);     // delayed mouse release (use very sparingly!). Generally used with 'delay >= io.MouseDoubleClickTMoss_e' + combined with a 'io.MouseClickedLastCount==1' test. This is a very rarely used UI idiom, but some apps use this: e.g. MS Explorer single click on an icon to rename.
MOSS_API int           GetMouseClickedCount(Moss_GuMoss_ouseButton button);                      // return the number of successive mouse-clicks at the tMoss_e where a click happen (otherwise 0).
MOSS_API bool          IsMouseHoveringRect(const Float2& r_min, const Float2& r_max, bool clip = true);// is mouse hovering given bounding rect (in screen space). clipped by current clipping settings, but disregarding of other consideration of focus/window ordering/popup-block.
MOSS_API bool          IsMousePosValid(const Float2* mouse_pos = NULL);                    // by convention we use (-FLT_MAX,-FLT_MAX) to denote that there is no mouse available
MOSS_API bool          IsAnyMouseDown();                                                   // [WILL OBSOLETE] is any mouse button held? This was designed for backends, but prefer having backend maintain a mask of held mouse buttons, because upcoming input queue system will make this invalid.
MOSS_API Float2        GetMousePos();                                                      // shortcut to Moss_Gui::GetIO().MousePos provided by user, to be consistent with other calls
MOSS_API Float2        GetMousePosOnOpeningCurrentPopup();                                 // retrieve mouse position at the tMoss_e of opening popup we have BeginPopup() into (helper to avoid user backing that value themselves)
MOSS_API bool          IsMouseDragging(Moss_GuMoss_ouseButton button, float lock_threshold = -1.0f);         // is mouse dragging? (uses io.MouseDraggingThreshold if lock_threshold < 0.0f)
MOSS_API Float2        GetMouseDragDelta(Moss_GuMoss_ouseButton button = 0, float lock_threshold = -1.0f);   // return the delta from the initial clicking position while the mouse button is pressed or was just released. This is locked and return 0.0f until the mouse moves past a distance threshold at least once (uses io.MouseDraggingThreshold if lock_threshold < 0.0f)
MOSS_API void          ResetMouseDragDelta(Moss_GuMoss_ouseButton button = 0);                   //
MOSS_API Moss_GuMoss_ouseCursor GetMouseCursor();                                                // get desired mouse cursor shape. Moss_portant: reset in Moss_Gui::NewFrame(), this is updated during the frame. valid before Render(). If you use software rendering by setting io.MouseDrawCursor Moss_Gui will render those for you
MOSS_API void          SetMouseCursor(Moss_GuMoss_ouseCursor cursor_type);                       // set desired mouse cursor shape
MOSS_API void          SetNextFrameWantCaptureMouse(bool want_capture_mouse);              // Override io.WantCaptureMouse flag next frame (said flag is left for your application to handle, typical when true it instructs your app to ignore inputs). This is equivalent to setting "io.WantCaptureMouse = want_capture_mouse;" after the next NewFrame() call.


    // Miscellaneous Utilities
MOSS_API bool          IsRectVisible(const Float2& size);                                  // test if rectangle (of given size, starting from cursor position) is visible / not clipped.
MOSS_API bool          IsRectVisible(const Float2& rect_min, const Float2& rect_max);      // test if rectangle (in screen space) is visible / not clipped. to perform coarse clipping on user's side.
MOSS_API double        GetTMoss_e();                                                          // get global Moss_gui tMoss_e. incremented by io.DeltaTMoss_e every frame.
MOSS_API int           GetFrameCount();                                                    // get global Moss_gui frame count. incremented by 1 every frame.
MOSS_API Moss_DrawListSharedData* GetDrawListSharedData();                                    // you may use this when creating your own Moss_DrawList instances.
MOSS_API const char*   GetStyleColorName(Moss_GuiCol idx);                                    // get a string corresponding to the enum value (for display, saving, etc.).
MOSS_API void          SetStateStorage(Moss_GuiStorage* storage);                             // replace current window storage with our own (if you want to manipulate it yourself, typically clear subsection of it)
MOSS_API Moss_GuiStorage* GetStateStorage();

    // Text Utilities
MOSS_API Float2        CalcTextSize(const char* text, const char* text_end = NULL, bool hide_text_after_double_hash = false, float wrap_width = -1.0f);

// Inputs Utilities: Keyboard/Mouse/Gamepad
    // - the Moss_GuiKey enum contains all possible keyboard, mouse and gamepad inputs (e.g. Moss_GuiKey_A, Moss_GuiKey_MouseLeft, Moss_GuiKey_GamepadDpadUp...).
    // - (legacy: before v1.87, we used Moss_GuiKey to carry native/user indices as defined by each backends. This was obsoleted in 1.87 (2022-02) and completely removed in 1.91.5 (2024-11). See https://github.com/ocornut/Moss_gui/issues/4921)
    // - (legacy: any use of Moss_GuiKey will assert when key < 512 to detect passing legacy native/user indices)
MOSS_API bool          IsKeyDown(Moss_GuiKey key);                                            // is key being held.
MOSS_API bool          IsKeyPressed(Moss_GuiKey key, bool repeat = true);                     // was key pressed (went from !Down to Down)? if repeat=true, uses io.KeyRepeatDelay / KeyRepeatRate
MOSS_API bool          IsKeyReleased(Moss_GuiKey key);                                        // was key released (went from Down to !Down)?
MOSS_API bool          IsKeyChordPressed(Moss_GuiKeyChord key_chord);                         // was key chord (mods + key) pressed, e.g. you can pass 'Moss_GuMoss_od_Ctrl | Moss_GuiKey_S' as a key-chord. This doesn't do any routing or focus check, please consider using Shortcut() function instead.
MOSS_API int           GetKeyPressedAmount(Moss_GuiKey key, float repeat_delay, float rate);  // uses provided repeat rate/delay. return a count, most often 0 or 1 but might be >1 if RepeatRate is small enough that DeltaTMoss_e > RepeatRate
MOSS_API const char*   GetKeyName(Moss_GuiKey key);                                           // [DEBUG] returns English name of the key. Those names are provided for debugging purpose and are not meant to be saved persistently nor compared.
MOSS_API void          SetNextFrameWantCaptureKeyboard(bool want_capture_keyboard);        // Override io.WantCaptureKeyboard flag next frame (said flag is left for your application to handle, typically when true it instructs your app to ignore inputs). e.g. force capture keyboard when your widget is being hovered. This is equivalent to setting "io.WantCaptureKeyboard = want_capture_keyboard"; after the next NewFrame() call.


   // Item/Widgets Utilities and Query Functions
    // - Most of the functions are referring to the previous Item that has been submitted.
    // - See Demo Window under "Widgets->Querying Status" for an interactive visualization of most of those functions.
MOSS_API bool          IsItemHovered(Moss_GuiHoveredFlags flags = 0);                         // is the last item hovered? (and usable, aka not blocked by a popup, etc.). See Moss_GuiHoveredFlags for more options.
MOSS_API bool          IsItemActive();                                                     // is the last item active? (e.g. button being held, text field being edited. This will continuously return true while holding mouse button on an item. Items that don't interact will always return false)
MOSS_API bool          IsItemFocused();                                                    // is the last item focused for keyboard/gamepad navigation?
MOSS_API bool          IsItemClicked(Moss_GuMoss_ouseButton mouse_button = 0);                   // is the last item hovered and mouse clicked on? (**)  == IsMouseClicked(mouse_button) && IsItemHovered()Moss_portant. (**) this is NOT equivalent to the behavior of e.g. Button(). Read comments in function definition.
MOSS_API bool          IsItemVisible();                                                    // is the last item visible? (items may be out of sight because of clipping/scrolling)
MOSS_API bool          IsItemEdited();                                                     // did the last item modify its underlying value this frame? or was pressed? This is generally the same as the "bool" return value of many widgets.
MOSS_API bool          IsItemActivated();                                                  // was the last item just made active (item was previously inactive).
MOSS_API bool          IsItemDeactivated();                                                // was the last item just made inactive (item was previously active). Useful for Undo/Redo patterns with widgets that require continuous editing.
MOSS_API bool          IsItemDeactivatedAfterEdit();                                       // was the last item just made inactive and made a value change when it was active? (e.g. Slider/Drag moved). Useful for Undo/Redo patterns with widgets that require continuous editing. Note that you may get false positives (some widgets such as Combo()/ListBox()/Selectable() will return true even when clicking an already selected item).
MOSS_API bool          IsItemToggledOpen();                                                // was the last item open state toggled? set by TreeNode().
MOSS_API bool          IsAnyItemHovered();                                                 // is any item hovered?
MOSS_API bool          IsAnyItemActive();                                                  // is any item active?
MOSS_API bool          IsAnyItemFocused();                                                 // is any item focused?
MOSS_API uint32       GetItemID();                                                        // get ID of last item (~~ often same Moss_Gui::GetID(label) beforehand)
MOSS_API Float2        GetItemRectMin();                                                   // get upper-left bounding rectangle of the last item (screen space)
MOSS_API Float2        GetItemRectMax();                                                   // get lower-right bounding rectangle of the last item (screen space)
MOSS_API Float2        GetItemRectSize();                                                  // get size of last item


// Clipboard Utilities
    // - Also see the LogToClipboard() function to capture GUI into clipboard, or easily output text data to the clipboard.
MOSS_API const char*   GetClipboardText();
MOSS_API void          SetClipboardText(const char* text);
*/












enum class Moss_GuiWindowFlags : uint8_t
{
    Moss_GuiWindowFlags_None                   = 0,
    Moss_GuiWindowFlags_NoTitleBar             = 1 << 0,   // Disable title-bar
    Moss_GuiWindowFlags_NoResize               = 1 << 1,   // Disable user resizing with the lower-right grip
    Moss_GuiWindowFlags_NoMove                 = 1 << 2,   // Disable user moving the window
    Moss_GuiWindowFlags_NoScrollbar            = 1 << 3,   // Disable scrollbars (window can still scroll with mouse or programmatically)
    Moss_GuiWindowFlags_NoScrollWithMouse      = 1 << 4,   // Disable user vertically scrolling with mouse wheel. On child window, mouse wheel will be forwarded to the parent unless NoScrollbar is also set.
    Moss_GuiWindowFlags_NoCollapse             = 1 << 5,   // Disable user collapsing window by double-clicking on it. Also referred to as Window Menu Button (e.g. within a docking node).
    Moss_GuiWindowFlags_AlwaysAutoResize       = 1 << 6,   // Resize every window to its content every frame
    Moss_GuiWindowFlags_NoBackground           = 1 << 7,   // Disable drawing background color (WindowBg, etc.) and outside border. SMoss_ilar as using SetNextWindowBgAlpha(0.0f).
    Moss_GuiWindowFlags_NoSavedSettings        = 1 << 8,   // Never load/save settings in .ini file
    Moss_GuiWindowFlags_NoMouseInputs          = 1 << 9,   // Disable catching mouse, hovering test with pass through.
    Moss_GuiWindowFlags_MenuBar                = 1 << 10,  // Has a menu-bar
    Moss_GuiWindowFlags_HorizontalScrollbar    = 1 << 11,  // Allow horizontal scrollbar to appear (off by default). You may use SetNextWindowContentSize(Moss_Vec2(width,0.0f)); prior to calling Begin() to specify width. Read code in Moss_gui_demo in the "Horizontal Scrolling" section.
    Moss_GuiWindowFlags_NoFocusOnAppearing     = 1 << 12,  // Disable taking focus when transitioning from hidden to visible state
    Moss_GuiWindowFlags_NoBringToFrontOnFocus  = 1 << 13,  // Disable bringing window to front when taking focus (e.g. clicking on it or programmatically giving it focus)
    Moss_GuiWindowFlags_AlwaysVerticalScrollbar= 1 << 14,  // Always show vertical scrollbar (even if ContentSize.y < Size.y)
    Moss_GuiWindowFlags_AlwaysHorizontalScrollbar=1<< 15,  // Always show horizontal scrollbar (even if ContentSize.x < Size.x)
    Moss_GuiWindowFlags_NoNavInputs            = 1 << 16,  // No keyboard/gamepad navigation within the window
    Moss_GuiWindowFlags_NoNavFocus             = 1 << 17,  // No focusing toward this window with keyboard/gamepad navigation (e.g. skipped by CTRL+TAB)
    Moss_GuiWindowFlags_UnsavedDocument        = 1 << 18,  // Display a dot next to the title. When used in a tab/docking context, tab is selected when clicking the X + closure is not assumed (will wait for user to stop submitting the tab). Otherwise closure is assumed when pressing the X, so if you keep submitting the tab may reappear at end of tab bar.
    Moss_GuiWindowFlags_NoNav                  = Moss_GuiWindowFlags_NoNavInputs | Moss_GuiWindowFlags_NoNavFocus,
    Moss_GuiWindowFlags_NoDecoration           = Moss_GuiWindowFlags_NoTitleBar | Moss_GuiWindowFlags_NoResize | Moss_GuiWindowFlags_NoScrollbar | Moss_GuiWindowFlags_NoCollapse,
    Moss_GuiWindowFlags_NoInputs               = Moss_GuiWindowFlags_NoMouseInputs | Moss_GuiWindowFlags_NoNavInputs | Moss_GuiWindowFlags_NoNavFocus,

    // [Internal]
    Moss_GuiWindowFlags_ChildWindow            = 1 << 24,  // Don't use! For internal use by BeginChild()
    Moss_GuiWindowFlags_Tooltip                = 1 << 25,  // Don't use! For internal use by BeginTooltip()
    Moss_GuiWindowFlags_Popup                  = 1 << 26,  // Don't use! For internal use by BeginPopup()
    Moss_GuiWindowFlags_Modal                  = 1 << 27,  // Don't use! For internal use by BeginPopupModal()
    Moss_GuiWindowFlags_ChildMenu              = 1 << 28,  // Don't use! For internal use by BeginMenu()
};






// Flags for Moss_Gui::PushItemFlag()
// (Those are shared by all items)
enum Moss_GuiItemFlags_
{
    Moss_GuiItemFlags_None                     = 0,        // (Default)
    Moss_GuiItemFlags_NoTabStop                = 1 << 0,   // false    // Disable keyboard tabbing. This is a "lighter" version of Moss_GuiItemFlags_NoNav.
    Moss_GuiItemFlags_NoNav                    = 1 << 1,   // false    // Disable any form of focusing (keyboard/gamepad directional navigation and SetKeyboardFocusHere() calls).
    Moss_GuiItemFlags_NoNavDefaultFocus        = 1 << 2,   // false    // Disable item being a candidate for default focus (e.g. used by title bar items).
    Moss_GuiItemFlags_ButtonRepeat             = 1 << 3,   // false    // Any button-like behavior will have repeat mode enabled (based on io.KeyRepeatDelay and io.KeyRepeatRate values). Note that you can also call IsItemActive() after any button to tell if it is being held.
    Moss_GuiItemFlags_AutoClosePopups          = 1 << 4,   // true     // MenuItem()/Selectable() automatically close their parent popup window.
    Moss_GuiItemFlags_AllowDuplicateId         = 1 << 5,   // false    // Allow submitting an item with the same identifier as an item already submitted this frame without triggering a warning tooltip if io.ConfigDebugHighlightIdConflicts is set.
};

// Flags for Moss_Gui::InputText()
// (Those are per-item flags. There are shared flags in Moss_GuiIO: io.ConfigInputTextCursorBlink and io.ConfigInputTextEnterKeepActive)
enum Moss_GuiInputTextFlags_
{
    // Basic filters (also see Moss_GuiInputTextFlags_CallbackCharFilter)
    Moss_GuiInputTextFlags_None                = 0,
    Moss_GuiInputTextFlags_CharsDecMoss_al        = 1 << 0,   // Allow 0123456789.+-*/
    Moss_GuiInputTextFlags_CharsHexadecMoss_al    = 1 << 1,   // Allow 0123456789ABCDEFabcdef
    Moss_GuiInputTextFlags_CharsScientific     = 1 << 2,   // Allow 0123456789.+-*/eE (Scientific notation input)
    Moss_GuiInputTextFlags_CharsUppercase      = 1 << 3,   // Turn a..z into A..Z
    Moss_GuiInputTextFlags_CharsNoBlank        = 1 << 4,   // Filter out spaces, tabs

    // Inputs
    Moss_GuiInputTextFlags_AllowTabInput       = 1 << 5,   // Pressing TAB input a '\t' character into the text field
    Moss_GuiInputTextFlags_EnterReturnsTrue    = 1 << 6,   // Return 'true' when Enter is pressed (as opposed to every tMoss_e the value was modified). Consider using IsItemDeactivatedAfterEdit() instead!
    Moss_GuiInputTextFlags_EscapeClearsAll     = 1 << 7,   // Escape key clears content if not empty, and deactivate otherwise (contrast to default behavior of Escape to revert)
    Moss_GuiInputTextFlags_CtrlEnterForNewLine = 1 << 8,   // In multi-line mode, validate with Enter, add new line with Ctrl+Enter (default is opposite: validate with Ctrl+Enter, add line with Enter).

    // Other options
    Moss_GuiInputTextFlags_ReadOnly            = 1 << 9,   // Read-only mode
    Moss_GuiInputTextFlags_Password            = 1 << 10,  // Password mode, display all characters as '*', disable copy
    Moss_GuiInputTextFlags_AlwaysOverwrite     = 1 << 11,  // Overwrite mode
    Moss_GuiInputTextFlags_AutoSelectAll       = 1 << 12,  // Select entire text when first taking mouse focus
    Moss_GuiInputTextFlags_ParseEmptyRefVal    = 1 << 13,  // InputFloat(), InputInt(), InputScalar() etc. only: parse empty string as zero value.
    Moss_GuiInputTextFlags_DisplayEmptyRefVal  = 1 << 14,  // InputFloat(), InputInt(), InputScalar() etc. only: when value is zero, do not display it. Generally used with Moss_GuiInputTextFlags_ParseEmptyRefVal.
    Moss_GuiInputTextFlags_NoHorizontalScroll  = 1 << 15,  // Disable following the cursor horizontally
    Moss_GuiInputTextFlags_NoUndoRedo          = 1 << 16,  // Disable undo/redo. Note that input text owns the text data while active, if you want to provide your own undo/redo stack you need e.g. to call ClearActiveID().

    // Elide display / Alignment
    Moss_GuiInputTextFlags_ElideLeft           = 1 << 17,  // When text doesn't fit, elide left side to ensure right side stays visible. Useful for path/filenames. Single-line only!

    // Callback features
    Moss_GuiInputTextFlags_CallbackCompletion  = 1 << 18,  // Callback on pressing TAB (for completion handling)
    Moss_GuiInputTextFlags_CallbackHistory     = 1 << 19,  // Callback on pressing Up/Down arrows (for history handling)
    Moss_GuiInputTextFlags_CallbackAlways      = 1 << 20,  // Callback on each iteration. User code may query cursor position, modify text buffer.
    Moss_GuiInputTextFlags_CallbackCharFilter  = 1 << 21,  // Callback on character inputs to replace or discard them. Modify 'EventChar' to replace or discard, or return 1 in callback to discard.
    Moss_GuiInputTextFlags_CallbackResize      = 1 << 22,  // Callback on buffer capacity changes request (beyond 'buf_size' parameter value), allowing the string to grow. Notify when the string wants to be resized (for string types which hold a cache of their Size). You will be provided a new BufSize in the callback and NEED to honor it. (see misc/cpp/Moss_gui_stdlib.h for an example of using this)
    Moss_GuiInputTextFlags_CallbackEdit        = 1 << 23,  // Callback on any edit. Note that InputText() already returns true on edit + you can always use IsItemEdited(). The callback is useful to manipulate the underlying buffer while focus is active.

    // Obsolete names
    //Moss_GuiInputTextFlags_AlwaysInsertMode  = Moss_GuiInputTextFlags_AlwaysOverwrite   // [renamed in 1.82] name was not matching behavior
};

// Flags for Moss_Gui::TreeNodeEx(), Moss_Gui::CollapsingHeader*()
enum Moss_GuiTreeNodeFlags_
{
    Moss_GuiTreeNodeFlags_None                 = 0,
    Moss_GuiTreeNodeFlags_Selected             = 1 << 0,   // Draw as selected
    Moss_GuiTreeNodeFlags_Framed               = 1 << 1,   // Draw frame with background (e.g. for CollapsingHeader)
    Moss_GuiTreeNodeFlags_AllowOverlap         = 1 << 2,   // Hit testing to allow subsequent widgets to overlap this one
    Moss_GuiTreeNodeFlags_NoTreePushOnOpen     = 1 << 3,   // Don't do a TreePush() when open (e.g. for CollapsingHeader) = no extra indent nor pushing on ID stack
    Moss_GuiTreeNodeFlags_NoAutoOpenOnLog      = 1 << 4,   // Don't automatically and temporarily open node when Logging is active (by default logging will automatically open tree nodes)
    Moss_GuiTreeNodeFlags_DefaultOpen          = 1 << 5,   // Default node to be open
    Moss_GuiTreeNodeFlags_OpenOnDoubleClick    = 1 << 6,   // Open on double-click instead of sMoss_ple click (default for multi-select unless any _OpenOnXXX behavior is set explicitly). Both behaviors may be combined.
    Moss_GuiTreeNodeFlags_OpenOnArrow          = 1 << 7,   // Open when clicking on the arrow part (default for multi-select unless any _OpenOnXXX behavior is set explicitly). Both behaviors may be combined.
    Moss_GuiTreeNodeFlags_Leaf                 = 1 << 8,   // No collapsing, no arrow (use as a convenience for leaf nodes).
    Moss_GuiTreeNodeFlags_Bullet               = 1 << 9,   // Display a bullet instead of arrow. Moss_PORTANT: node can still be marked open/close if you don't set the _Leaf flag!
    Moss_GuiTreeNodeFlags_FramePadding         = 1 << 10,  // Use FramePadding (even for an unframed text node) to vertically align text baseline to regular widget height. Equivalent to calling AlignTextToFramePadding() before the node.
    Moss_GuiTreeNodeFlags_SpanAvailWidth       = 1 << 11,  // Extend hit box to the right-most edge, even if not framed. This is not the default in order to allow adding other items on the same line without using AllowOverlap mode.
    Moss_GuiTreeNodeFlags_SpanFullWidth        = 1 << 12,  // Extend hit box to the left-most and right-most edges (cover the indent area).
    Moss_GuiTreeNodeFlags_SpanLabelWidth       = 1 << 13,  // Narrow hit box + narrow hovering highlight, will only cover the label text.
    Moss_GuiTreeNodeFlags_SpanAllColumns       = 1 << 14,  // Frame will span all columns of its container table (label will still fit in current column)
    Moss_GuiTreeNodeFlags_LabelSpanAllColumns  = 1 << 15,  // Label will span all columns of its container table
    //Moss_GuiTreeNodeFlags_NoScrollOnOpen     = 1 << 16,  // FIXME: TODO: Disable automatic scroll on TreePop() if node got just open and contents is not visible
    Moss_GuiTreeNodeFlags_NavLeftJumpsToParent = 1 << 17,  // Nav: left arrow moves back to parent. This is processed in TreePop() when there's an unfullfilled Left nav request remaining.
    Moss_GuiTreeNodeFlags_CollapsingHeader     = Moss_GuiTreeNodeFlags_Framed | Moss_GuiTreeNodeFlags_NoTreePushOnOpen | Moss_GuiTreeNodeFlags_NoAutoOpenOnLog,

    // [EXPERMoss_ENTAL] Draw lines connecting TreeNode hierarchy. Discuss in GitHub issue #2920.
    // Default value is pulled from style.TreeLinesFlags. May be overridden in TreeNode calls.
    Moss_GuiTreeNodeFlags_DrawLinesNone        = 1 << 18,  // No lines drawn
    Moss_GuiTreeNodeFlags_DrawLinesFull        = 1 << 19,  // Horizontal lines to child nodes. Vertical line drawn down to TreePop() position: cover full contents. Faster (for large trees).
    Moss_GuiTreeNodeFlags_DrawLinesToNodes     = 1 << 20,  // Horizontal lines to child nodes. Vertical line drawn down to bottom-most child node. Slower (for large trees).

#ifndef Moss_GUI_DISABLE_OBSOLETE_FUNCTIONS
    Moss_GuiTreeNodeFlags_NavLeftJumpsBackHere = Moss_GuiTreeNodeFlags_NavLeftJumpsToParent,  // Renamed in 1.92.0
    Moss_GuiTreeNodeFlags_SpanTextWidth        = Moss_GuiTreeNodeFlags_SpanLabelWidth,        // Renamed in 1.90.7
    Moss_GuiTreeNodeFlags_AllowItemOverlap     = Moss_GuiTreeNodeFlags_AllowOverlap,          // Renamed in 1.89.7
#endif
};

// Flags for OpenPopup*(), BeginPopupContext*(), IsPopupOpen() functions.
// - To be backward compatible with older API which took an 'int mouse_button = 1' argument instead of 'Moss_GuiPopupFlags flags',
//   we need to treat small flags values as a mouse button index, so we encode the mouse button in the first few bits of the flags.
//   It is therefore guaranteed to be legal to pass a mouse button index in Moss_GuiPopupFlags.
// - For the same reason, we exceptionally default the Moss_GuiPopupFlags argument of BeginPopupContextXXX functions to 1 instead of 0.
//   Moss_PORTANT: because the default parameter is 1 (==Moss_GuiPopupFlags_MouseButtonRight), if you rely on the default parameter
//   and want to use another flag, you need to pass in the Moss_GuiPopupFlags_MouseButtonRight flag explicitly.
// - Multiple buttons currently cannot be combined/or-ed in those functions (we could allow it later).
enum Moss_GuiPopupFlags_
{
    Moss_GuiPopupFlags_None                    = 0,
    Moss_GuiPopupFlags_MouseButtonLeft         = 0,        // For BeginPopupContext*(): open on Left Mouse release. Guaranteed to always be == 0 (same as Moss_GuMoss_ouseButton_Left)
    Moss_GuiPopupFlags_MouseButtonRight        = 1,        // For BeginPopupContext*(): open on Right Mouse release. Guaranteed to always be == 1 (same as Moss_GuMoss_ouseButton_Right)
    Moss_GuiPopupFlags_MouseButtonMiddle       = 2,        // For BeginPopupContext*(): open on Middle Mouse release. Guaranteed to always be == 2 (same as Moss_GuMoss_ouseButton_Middle)
    Moss_GuiPopupFlags_MouseButtonMask_        = 0x1F,
    Moss_GuiPopupFlags_MouseButtonDefault_     = 1,
    Moss_GuiPopupFlags_NoReopen                = 1 << 5,   // For OpenPopup*(), BeginPopupContext*(): don't reopen same popup if already open (won't reposition, won't reinitialize navigation)
    //Moss_GuiPopupFlags_NoReopenAlwaysNavInit = 1 << 6,   // For OpenPopup*(), BeginPopupContext*(): focus and initialize navigation even when not reopening.
    Moss_GuiPopupFlags_NoOpenOverExistingPopup = 1 << 7,   // For OpenPopup*(), BeginPopupContext*(): don't open if there's already a popup at the same level of the popup stack
    Moss_GuiPopupFlags_NoOpenOverItems         = 1 << 8,   // For BeginPopupContextWindow(): don't return true when hovering items, only when hovering empty space
    Moss_GuiPopupFlags_AnyPopupId              = 1 << 10,  // For IsPopupOpen(): ignore the Moss_GuiID parameter and test for any popup.
    Moss_GuiPopupFlags_AnyPopupLevel           = 1 << 11,  // For IsPopupOpen(): search/test at any level of the popup stack (default test in the current level)
    Moss_GuiPopupFlags_AnyPopup                = Moss_GuiPopupFlags_AnyPopupId | Moss_GuiPopupFlags_AnyPopupLevel,
};

// Flags for Moss_Gui::Selectable()
enum Moss_GuiSelectableFlags_
{
    Moss_GuiSelectableFlags_None               = 0,
    Moss_GuiSelectableFlags_NoAutoClosePopups  = 1 << 0,   // Clicking this doesn't close parent popup window (overrides Moss_GuiItemFlags_AutoClosePopups)
    Moss_GuiSelectableFlags_SpanAllColumns     = 1 << 1,   // Frame will span all columns of its container table (text will still fit in current column)
    Moss_GuiSelectableFlags_AllowDoubleClick   = 1 << 2,   // Generate press events on double clicks too
    Moss_GuiSelectableFlags_Disabled           = 1 << 3,   // Cannot be selected, display grayed out text
    Moss_GuiSelectableFlags_AllowOverlap       = 1 << 4,   // (WIP) Hit testing to allow subsequent widgets to overlap this one
    Moss_GuiSelectableFlags_Highlight          = 1 << 5,   // Make the item be displayed as if it is hovered

#ifndef Moss_GUI_DISABLE_OBSOLETE_FUNCTIONS
    Moss_GuiSelectableFlags_DontClosePopups    = Moss_GuiSelectableFlags_NoAutoClosePopups,   // Renamed in 1.91.0
    Moss_GuiSelectableFlags_AllowItemOverlap   = Moss_GuiSelectableFlags_AllowOverlap,        // Renamed in 1.89.7
#endif
};

// Flags for Moss_Gui::BeginCombo()
enum Moss_GuiComboFlags_
{
    Moss_GuiComboFlags_None                    = 0,
    Moss_GuiComboFlags_PopupAlignLeft          = 1 << 0,   // Align the popup toward the left by default
    Moss_GuiComboFlags_HeightSmall             = 1 << 1,   // Max ~4 items visible. Tip: If you want your combo popup to be a specific size you can use SetNextWindowSizeConstraints() prior to calling BeginCombo()
    Moss_GuiComboFlags_HeightRegular           = 1 << 2,   // Max ~8 items visible (default)
    Moss_GuiComboFlags_HeightLarge             = 1 << 3,   // Max ~20 items visible
    Moss_GuiComboFlags_HeightLargest           = 1 << 4,   // As many fitting items as possible
    Moss_GuiComboFlags_NoArrowButton           = 1 << 5,   // Display on the preview box without the square arrow button
    Moss_GuiComboFlags_NoPreview               = 1 << 6,   // Display only a square arrow button
    Moss_GuiComboFlags_WidthFitPreview         = 1 << 7,   // Width dynamically calculated from preview contents
    Moss_GuiComboFlags_HeightMask_             = Moss_GuiComboFlags_HeightSmall | Moss_GuiComboFlags_HeightRegular | Moss_GuiComboFlags_HeightLarge | Moss_GuiComboFlags_HeightLargest,
};

// Flags for Moss_Gui::BeginTabBar()
enum Moss_GuiTabBarFlags_
{
    Moss_GuiTabBarFlags_None                           = 0,
    Moss_GuiTabBarFlags_Reorderable                    = 1 << 0,   // Allow manually dragging tabs to re-order them + New tabs are appended at the end of list
    Moss_GuiTabBarFlags_AutoSelectNewTabs              = 1 << 1,   // Automatically select new tabs when they appear
    Moss_GuiTabBarFlags_TabListPopupButton             = 1 << 2,   // Disable buttons to open the tab list popup
    Moss_GuiTabBarFlags_NoCloseWithMiddleMouseButton   = 1 << 3,   // Disable behavior of closing tabs (that are submitted with p_open != NULL) with middle mouse button. You may handle this behavior manually on user's side with if (IsItemHovered() && IsMouseClicked(2)) *p_open = false.
    Moss_GuiTabBarFlags_NoTabListScrollingButtons      = 1 << 4,   // Disable scrolling buttons (apply when fitting policy is Moss_GuiTabBarFlags_FittingPolicyScroll)
    Moss_GuiTabBarFlags_NoTooltip                      = 1 << 5,   // Disable tooltips when hovering a tab
    Moss_GuiTabBarFlags_DrawSelectedOverline           = 1 << 6,   // Draw selected overline markers over selected tab
    Moss_GuiTabBarFlags_FittingPolicyResizeDown        = 1 << 7,   // Resize tabs when they don't fit
    Moss_GuiTabBarFlags_FittingPolicyScroll            = 1 << 8,   // Add scroll buttons when tabs don't fit
    Moss_GuiTabBarFlags_FittingPolicyMask_             = Moss_GuiTabBarFlags_FittingPolicyResizeDown | Moss_GuiTabBarFlags_FittingPolicyScroll,
    Moss_GuiTabBarFlags_FittingPolicyDefault_          = Moss_GuiTabBarFlags_FittingPolicyResizeDown,
};

// Flags for Moss_Gui::BeginTabItem()
enum Moss_GuiTabItemFlags_
{
    Moss_GuiTabItemFlags_None                          = 0,
    Moss_GuiTabItemFlags_UnsavedDocument               = 1 << 0,   // Display a dot next to the title + set Moss_GuiTabItemFlags_NoAssumedClosure.
    Moss_GuiTabItemFlags_SetSelected                   = 1 << 1,   // Trigger flag to programmatically make the tab selected when calling BeginTabItem()
    Moss_GuiTabItemFlags_NoCloseWithMiddleMouseButton  = 1 << 2,   // Disable behavior of closing tabs (that are submitted with p_open != NULL) with middle mouse button. You may handle this behavior manually on user's side with if (IsItemHovered() && IsMouseClicked(2)) *p_open = false.
    Moss_GuiTabItemFlags_NoPushId                      = 1 << 3,   // Don't call PushID()/PopID() on BeginTabItem()/EndTabItem()
    Moss_GuiTabItemFlags_NoTooltip                     = 1 << 4,   // Disable tooltip for the given tab
    Moss_GuiTabItemFlags_NoReorder                     = 1 << 5,   // Disable reordering this tab or having another tab cross over this tab
    Moss_GuiTabItemFlags_Leading                       = 1 << 6,   // Enforce the tab position to the left of the tab bar (after the tab list popup button)
    Moss_GuiTabItemFlags_Trailing                      = 1 << 7,   // Enforce the tab position to the right of the tab bar (before the scrolling buttons)
    Moss_GuiTabItemFlags_NoAssumedClosure              = 1 << 8,   // Tab is selected when trying to close + closure is not Moss_mediately assumed (will wait for user to stop submitting the tab). Otherwise closure is assumed when pressing the X, so if you keep submitting the tab may reappear at end of tab bar.
};

// Flags for Moss_Gui::IsWindowFocused()
enum Moss_GuiFocusedFlags_
{
    Moss_GuiFocusedFlags_None                          = 0,
    Moss_GuiFocusedFlags_ChildWindows                  = 1 << 0,   // Return true if any children of the window is focused
    Moss_GuiFocusedFlags_RootWindow                    = 1 << 1,   // Test from root window (top most parent of the current hierarchy)
    Moss_GuiFocusedFlags_AnyWindow                     = 1 << 2,   // Return true if any window is focused. Moss_portant: If you are trying to tell how to dispatch your low-level inputs, do NOT use this. Use 'io.WantCaptureMouse' instead! Please read the FAQ!
    Moss_GuiFocusedFlags_NoPopupHierarchy              = 1 << 3,   // Do not consider popup hierarchy (do not treat popup emitter as parent of popup) (when used with _ChildWindows or _RootWindow)
    //Moss_GuiFocusedFlags_DockHierarchy               = 1 << 4,   // Consider docking hierarchy (treat dockspace host as parent of docked window) (when used with _ChildWindows or _RootWindow)
    Moss_GuiFocusedFlags_RootAndChildWindows           = Moss_GuiFocusedFlags_RootWindow | Moss_GuiFocusedFlags_ChildWindows,
};

// Flags for Moss_Gui::IsItemHovered(), Moss_Gui::IsWindowHovered()
// Note: if you are trying to check whether your mouse should be dispatched to Dear Moss_Gui or to your app, you should use 'io.WantCaptureMouse' instead! Please read the FAQ!
// Note: windows with the Moss_GuiWindowFlags_NoInputs flag are ignored by IsWindowHovered() calls.
enum Moss_GuiHoveredFlags_
{
    Moss_GuiHoveredFlags_None                          = 0,        // Return true if directly over the item/window, not obstructed by another window, not obstructed by an active popup or modal blocking inputs under them.
    Moss_GuiHoveredFlags_ChildWindows                  = 1 << 0,   // IsWindowHovered() only: Return true if any children of the window is hovered
    Moss_GuiHoveredFlags_RootWindow                    = 1 << 1,   // IsWindowHovered() only: Test from root window (top most parent of the current hierarchy)
    Moss_GuiHoveredFlags_AnyWindow                     = 1 << 2,   // IsWindowHovered() only: Return true if any window is hovered
    Moss_GuiHoveredFlags_NoPopupHierarchy              = 1 << 3,   // IsWindowHovered() only: Do not consider popup hierarchy (do not treat popup emitter as parent of popup) (when used with _ChildWindows or _RootWindow)
    //Moss_GuiHoveredFlags_DockHierarchy               = 1 << 4,   // IsWindowHovered() only: Consider docking hierarchy (treat dockspace host as parent of docked window) (when used with _ChildWindows or _RootWindow)
    Moss_GuiHoveredFlags_AllowWhenBlockedByPopup       = 1 << 5,   // Return true even if a popup window is normally blocking access to this item/window
    //Moss_GuiHoveredFlags_AllowWhenBlockedByModal     = 1 << 6,   // Return true even if a modal popup window is normally blocking access to this item/window. FIXME-TODO: Unavailable yet.
    Moss_GuiHoveredFlags_AllowWhenBlockedByActiveItem  = 1 << 7,   // Return true even if an active item is blocking access to this item/window. Useful for Drag and Drop patterns.
    Moss_GuiHoveredFlags_AllowWhenOverlappedByItem     = 1 << 8,   // IsItemHovered() only: Return true even if the item uses AllowOverlap mode and is overlapped by another hoverable item.
    Moss_GuiHoveredFlags_AllowWhenOverlappedByWindow   = 1 << 9,   // IsItemHovered() only: Return true even if the position is obstructed or overlapped by another window.
    Moss_GuiHoveredFlags_AllowWhenDisabled             = 1 << 10,  // IsItemHovered() only: Return true even if the item is disabled
    Moss_GuiHoveredFlags_NoNavOverride                 = 1 << 11,  // IsItemHovered() only: Disable using keyboard/gamepad navigation state when active, always query mouse
    Moss_GuiHoveredFlags_AllowWhenOverlapped           = Moss_GuiHoveredFlags_AllowWhenOverlappedByItem | Moss_GuiHoveredFlags_AllowWhenOverlappedByWindow,
    Moss_GuiHoveredFlags_RectOnly                      = Moss_GuiHoveredFlags_AllowWhenBlockedByPopup | Moss_GuiHoveredFlags_AllowWhenBlockedByActiveItem | Moss_GuiHoveredFlags_AllowWhenOverlapped,
    Moss_GuiHoveredFlags_RootAndChildWindows           = Moss_GuiHoveredFlags_RootWindow | Moss_GuiHoveredFlags_ChildWindows,

    // Tooltips mode
    // - typically used in IsItemHovered() + SetTooltip() sequence.
    // - this is a shortcut to pull flags from 'style.HoverFlagsForTooltipMouse' or 'style.HoverFlagsForTooltipNav' where you can reconfigure desired behavior.
    //   e.g. 'TooltipHoveredFlagsForMouse' defaults to 'Moss_GuiHoveredFlags_Stationary | Moss_GuiHoveredFlags_DelayShort'.
    // - for frequently actioned or hovered items providing a tooltip, you want may to use Moss_GuiHoveredFlags_ForTooltip (stationary + delay) so the tooltip doesn't show too often.
    // - for items which main purpose is to be hovered, or items with low affordance, or in less consistent apps, prefer no delay or shorter delay.
    Moss_GuiHoveredFlags_ForTooltip                    = 1 << 12,  // Shortcut for standard flags when using IsItemHovered() + SetTooltip() sequence.

    // (Advanced) Mouse Hovering delays.
    // - generally you can use Moss_GuiHoveredFlags_ForTooltip to use application-standardized flags.
    // - use those if you need specific overrides.
    Moss_GuiHoveredFlags_Stationary                    = 1 << 13,  // Require mouse to be stationary for style.HoverStationaryDelay (~0.15 sec) _at least one tMoss_e_. After this, can move on same item/window. Using the stationary test tends to reduces the need for a long delay.
    Moss_GuiHoveredFlags_DelayNone                     = 1 << 14,  // IsItemHovered() only: Return true Moss_mediately (default). As this is the default you generally ignore this.
    Moss_GuiHoveredFlags_DelayShort                    = 1 << 15,  // IsItemHovered() only: Return true after style.HoverDelayShort elapsed (~0.15 sec) (shared between items) + requires mouse to be stationary for style.HoverStationaryDelay (once per item).
    Moss_GuiHoveredFlags_DelayNormal                   = 1 << 16,  // IsItemHovered() only: Return true after style.HoverDelayNormal elapsed (~0.40 sec) (shared between items) + requires mouse to be stationary for style.HoverStationaryDelay (once per item).
    Moss_GuiHoveredFlags_NoSharedDelay                 = 1 << 17,  // IsItemHovered() only: Disable shared delay system where moving from one item to the next keeps the previous tMoss_er for a short tMoss_e (standard for tooltips with long delays)
};

// Flags for Moss_Gui::BeginDragDropSource(), Moss_Gui::AcceptDragDropPayload()
enum Moss_GuiDragDropFlags_
{
    Moss_GuiDragDropFlags_None                         = 0,
    // BeginDragDropSource() flags
    Moss_GuiDragDropFlags_SourceNoPreviewTooltip       = 1 << 0,   // Disable preview tooltip. By default, a successful call to BeginDragDropSource opens a tooltip so you can display a preview or description of the source contents. This flag disables this behavior.
    Moss_GuiDragDropFlags_SourceNoDisableHover         = 1 << 1,   // By default, when dragging we clear data so that IsItemHovered() will return false, to avoid subsequent user code submitting tooltips. This flag disables this behavior so you can still call IsItemHovered() on the source item.
    Moss_GuiDragDropFlags_SourceNoHoldToOpenOthers     = 1 << 2,   // Disable the behavior that allows to open tree nodes and collapsing header by holding over them while dragging a source item.
    Moss_GuiDragDropFlags_SourceAllowNullID            = 1 << 3,   // Allow items such as Text(), Moss_age() that have no unique identifier to be used as drag source, by manufacturing a temporary identifier based on their window-relative position. This is extremely unusual within the dear Moss_gui ecosystem and so we made it explicit.
    Moss_GuiDragDropFlags_SourceExtern                 = 1 << 4,   // External source (from outside of dear Moss_gui), won't attempt to read current item/window info. Will always return true. Only one Extern source can be active sMoss_ultaneously.
    Moss_GuiDragDropFlags_PayloadAutoExpire            = 1 << 5,   // Automatically expire the payload if the source cease to be submitted (otherwise payloads are persisting while being dragged)
    Moss_GuiDragDropFlags_PayloadNoCrossContext        = 1 << 6,   // Hint to specify that the payload may not be copied outside current dear Moss_gui context.
    Moss_GuiDragDropFlags_PayloadNoCrossProcess        = 1 << 7,   // Hint to specify that the payload may not be copied outside current process.
    // AcceptDragDropPayload() flags
    Moss_GuiDragDropFlags_AcceptBeforeDelivery         = 1 << 10,  // AcceptDragDropPayload() will returns true even before the mouse button is released. You can then call IsDelivery() to test if the payload needs to be delivered.
    Moss_GuiDragDropFlags_AcceptNoDrawDefaultRect      = 1 << 11,  // Do not draw the default highlight rectangle when hovering over target.
    Moss_GuiDragDropFlags_AcceptNoPreviewTooltip       = 1 << 12,  // Request hiding the BeginDragDropSource tooltip from the BeginDragDropTarget site.
    Moss_GuiDragDropFlags_AcceptPeekOnly               = Moss_GuiDragDropFlags_AcceptBeforeDelivery | Moss_GuiDragDropFlags_AcceptNoDrawDefaultRect, // For peeking ahead and inspecting the payload before delivery.

#ifndef Moss_GUI_DISABLE_OBSOLETE_FUNCTIONS
    Moss_GuiDragDropFlags_SourceAutoExpirePayload = Moss_GuiDragDropFlags_PayloadAutoExpire, // Renamed in 1.90.9
#endif
};

// Standard Drag and Drop payload types. You can define you own payload types using short strings. Types starting with '_' are defined by Dear Moss_Gui.
#define Moss_GUI_PAYLOAD_TYPE_COLOR_3F     "_COL3F"    // float[3]: Standard type for colors, without alpha. User code may use this type.
#define Moss_GUI_PAYLOAD_TYPE_COLOR_4F     "_COL4F"    // float[4]: Standard type for colors. User code may use this type.

// A prMoss_ary data type
enum Moss_GuiDataType_
{
    Moss_GuiDataType_S8,       // signed char / char (with sensible compilers)
    Moss_GuiDataType_U8,       // unsigned char
    Moss_GuiDataType_S16,      // short
    Moss_GuiDataType_U16,      // unsigned short
    Moss_GuiDataType_S32,      // int
    Moss_GuiDataType_U32,      // unsigned int
    Moss_GuiDataType_S64,      // long long / __int64
    Moss_GuiDataType_U64,      // unsigned long long / unsigned __int64
    Moss_GuiDataType_Float,    // float
    Moss_GuiDataType_Double,   // double
    Moss_GuiDataType_Bool,     // bool (provided for user convenience, not supported by scalar widgets)
    Moss_GuiDataType_String,   // char* (provided for user convenience, not supported by scalar widgets)
    Moss_GuiDataType_COUNT
};

// A cardinal direction
enum Moss_GuiDir : int
{
    Moss_GuiDir_None    = -1,
    Moss_GuiDir_Left    = 0,
    Moss_GuiDir_Right   = 1,
    Moss_GuiDir_Up      = 2,
    Moss_GuiDir_Down    = 3,
    Moss_GuiDir_COUNT
};

// A sorting direction
enum Moss_GuiSortDirection : Moss_U8
{
    Moss_GuiSortDirection_None         = 0,
    Moss_GuiSortDirection_Ascending    = 1,    // Ascending = 0->9, A->Z etc.
    Moss_GuiSortDirection_Descending   = 2     // Descending = 9->0, Z->A etc.
};

// A key identifier (Moss_GuiKey_XXX or Moss_GuMoss_od_XXX value): can represent Keyboard, Mouse and Gamepad values.
// All our named keys are >= 512. Keys value 0 to 511 are left unused and were legacy native/opaque key values (< 1.87).
// Support for legacy keys was completely removed in 1.91.5.
// Read details about the 1.87+ transition : https://github.com/ocornut/Moss_gui/issues/4921
// Note that "Keys" related to physical keys and are not the same concept as input "Characters", the later are submitted via io.AddInputCharacter().
// The keyboard key enum values are named after the keys on a standard US keyboard, and on other keyboard types the keys reported may not match the keycaps.
enum Moss_GuiKey : int
{
    // Keyboard
    Moss_GuiKey_None = 0,
    Moss_GuiKey_NamedKey_BEGIN = 512,  // First valid key value (other than 0)

    Moss_GuiKey_Tab = 512,             // == Moss_GuiKey_NamedKey_BEGIN
    Moss_GuiKey_LeftArrow,
    Moss_GuiKey_RightArrow,
    Moss_GuiKey_UpArrow,
    Moss_GuiKey_DownArrow,
    Moss_GuiKey_PageUp,
    Moss_GuiKey_PageDown,
    Moss_GuiKey_Home,
    Moss_GuiKey_End,
    Moss_GuiKey_Insert,
    Moss_GuiKey_Delete,
    Moss_GuiKey_Backspace,
    Moss_GuiKey_Space,
    Moss_GuiKey_Enter,
    Moss_GuiKey_Escape,
    Moss_GuiKey_LeftCtrl, Moss_GuiKey_LeftShift, Moss_GuiKey_LeftAlt, Moss_GuiKey_LeftSuper,     // Also see Moss_GuMoss_od_Ctrl, Moss_GuMoss_od_Shift, Moss_GuMoss_od_Alt, Moss_GuMoss_od_Super below!
    Moss_GuiKey_RightCtrl, Moss_GuiKey_RightShift, Moss_GuiKey_RightAlt, Moss_GuiKey_RightSuper,
    Moss_GuiKey_Menu,
    Moss_GuiKey_0, Moss_GuiKey_1, Moss_GuiKey_2, Moss_GuiKey_3, Moss_GuiKey_4, Moss_GuiKey_5, Moss_GuiKey_6, Moss_GuiKey_7, Moss_GuiKey_8, Moss_GuiKey_9,
    Moss_GuiKey_A, Moss_GuiKey_B, Moss_GuiKey_C, Moss_GuiKey_D, Moss_GuiKey_E, Moss_GuiKey_F, Moss_GuiKey_G, Moss_GuiKey_H, Moss_GuiKey_I, Moss_GuiKey_J,
    Moss_GuiKey_K, Moss_GuiKey_L, Moss_GuiKey_M, Moss_GuiKey_N, Moss_GuiKey_O, Moss_GuiKey_P, Moss_GuiKey_Q, Moss_GuiKey_R, Moss_GuiKey_S, Moss_GuiKey_T,
    Moss_GuiKey_U, Moss_GuiKey_V, Moss_GuiKey_W, Moss_GuiKey_X, Moss_GuiKey_Y, Moss_GuiKey_Z,
    Moss_GuiKey_F1, Moss_GuiKey_F2, Moss_GuiKey_F3, Moss_GuiKey_F4, Moss_GuiKey_F5, Moss_GuiKey_F6,
    Moss_GuiKey_F7, Moss_GuiKey_F8, Moss_GuiKey_F9, Moss_GuiKey_F10, Moss_GuiKey_F11, Moss_GuiKey_F12,
    Moss_GuiKey_F13, Moss_GuiKey_F14, Moss_GuiKey_F15, Moss_GuiKey_F16, Moss_GuiKey_F17, Moss_GuiKey_F18,
    Moss_GuiKey_F19, Moss_GuiKey_F20, Moss_GuiKey_F21, Moss_GuiKey_F22, Moss_GuiKey_F23, Moss_GuiKey_F24,
    Moss_GuiKey_Apostrophe,        // '
    Moss_GuiKey_Comma,             // ,
    Moss_GuiKey_Minus,             // -
    Moss_GuiKey_Period,            // .
    Moss_GuiKey_Slash,             // /
    Moss_GuiKey_Semicolon,         // ;
    Moss_GuiKey_Equal,             // =
    Moss_GuiKey_LeftBracket,       // [
    Moss_GuiKey_Backslash,         // \ (this text inhibit multiline comment caused by backslash)
    Moss_GuiKey_RightBracket,      // ]
    Moss_GuiKey_GraveAccent,       // `
    Moss_GuiKey_CapsLock,
    Moss_GuiKey_ScrollLock,
    Moss_GuiKey_NumLock,
    Moss_GuiKey_PrintScreen,
    Moss_GuiKey_Pause,
    Moss_GuiKey_Keypad0, Moss_GuiKey_Keypad1, Moss_GuiKey_Keypad2, Moss_GuiKey_Keypad3, Moss_GuiKey_Keypad4,
    Moss_GuiKey_Keypad5, Moss_GuiKey_Keypad6, Moss_GuiKey_Keypad7, Moss_GuiKey_Keypad8, Moss_GuiKey_Keypad9,
    Moss_GuiKey_KeypadDecMoss_al,
    Moss_GuiKey_KeypadDivide,
    Moss_GuiKey_KeypadMultiply,
    Moss_GuiKey_KeypadSubtract,
    Moss_GuiKey_KeypadAdd,
    Moss_GuiKey_KeypadEnter,
    Moss_GuiKey_KeypadEqual,
    Moss_GuiKey_AppBack,               // Available on some keyboard/mouses. Often referred as "Browser Back"
    Moss_GuiKey_AppForward,
    Moss_GuiKey_Oem102,                // Non-US backslash.

    // Gamepad
    // (analog values are 0.0f to 1.0f)
    // (download controller mapping PNG/PSD at http://dearMoss_gui.com/controls_sheets)
    //                              // XBOX        | SWITCH  | PLAYSTA. | -> ACTION
    Moss_GuiKey_GamepadStart,          // Menu        | +       | Options  |
    Moss_GuiKey_GamepadBack,           // View        | -       | Share    |
    Moss_GuiKey_GamepadFaceLeft,       // X           | Y       | Square   | Tap: Toggle Menu. Hold: Windowing mode (Focus/Move/Resize windows)
    Moss_GuiKey_GamepadFaceRight,      // B           | A       | Circle   | Cancel / Close / Exit
    Moss_GuiKey_GamepadFaceUp,         // Y           | X       | Triangle | Text Input / On-screen Keyboard
    Moss_GuiKey_GamepadFaceDown,       // A           | B       | Cross    | Activate / Open / Toggle / Tweak
    Moss_GuiKey_GamepadDpadLeft,       // D-pad Left  | "       | "        | Move / Tweak / Resize Window (in Windowing mode)
    Moss_GuiKey_GamepadDpadRight,      // D-pad Right | "       | "        | Move / Tweak / Resize Window (in Windowing mode)
    Moss_GuiKey_GamepadDpadUp,         // D-pad Up    | "       | "        | Move / Tweak / Resize Window (in Windowing mode)
    Moss_GuiKey_GamepadDpadDown,       // D-pad Down  | "       | "        | Move / Tweak / Resize Window (in Windowing mode)
    Moss_GuiKey_GamepadL1,             // L Bumper    | L       | L1       | Tweak Slower / Focus Previous (in Windowing mode)
    Moss_GuiKey_GamepadR1,             // R Bumper    | R       | R1       | Tweak Faster / Focus Next (in Windowing mode)
    Moss_GuiKey_GamepadL2,             // L Trigger   | ZL      | L2       | [Analog]
    Moss_GuiKey_GamepadR2,             // R Trigger   | ZR      | R2       | [Analog]
    Moss_GuiKey_GamepadL3,             // L Stick     | L3      | L3       |
    Moss_GuiKey_GamepadR3,             // R Stick     | R3      | R3       |
    Moss_GuiKey_GamepadLStickLeft,     //             |         |          | [Analog] Move Window (in Windowing mode)
    Moss_GuiKey_GamepadLStickRight,    //             |         |          | [Analog] Move Window (in Windowing mode)
    Moss_GuiKey_GamepadLStickUp,       //             |         |          | [Analog] Move Window (in Windowing mode)
    Moss_GuiKey_GamepadLStickDown,     //             |         |          | [Analog] Move Window (in Windowing mode)
    Moss_GuiKey_GamepadRStickLeft,     //             |         |          | [Analog]
    Moss_GuiKey_GamepadRStickRight,    //             |         |          | [Analog]
    Moss_GuiKey_GamepadRStickUp,       //             |         |          | [Analog]
    Moss_GuiKey_GamepadRStickDown,     //             |         |          | [Analog]

    // Aliases: Mouse Buttons (auto-submitted from AddMouseButtonEvent() calls)
    // - This is mirroring the data also written to io.MouseDown[], io.MouseWheel, in a format allowing them to be accessed via standard key API.
    Moss_GuiKey_MouseLeft, Moss_GuiKey_MouseRight, Moss_GuiKey_MouseMiddle, Moss_GuiKey_MouseX1, Moss_GuiKey_MouseX2, Moss_GuiKey_MouseWheelX, Moss_GuiKey_MouseWheelY,

    // [Internal] Reserved for mod storage
    Moss_GuiKey_ReservedForModCtrl, Moss_GuiKey_ReservedForModShift, Moss_GuiKey_ReservedForModAlt, Moss_GuiKey_ReservedForModSuper,

    // [Internal] If you need to iterate all keys (for e.g. an input mapper) you may use Moss_GuiKey_NamedKey_BEGIN..Moss_GuiKey_NamedKey_END.
    Moss_GuiKey_NamedKey_END,
    Moss_GuiKey_NamedKey_COUNT = Moss_GuiKey_NamedKey_END - Moss_GuiKey_NamedKey_BEGIN,

    // Keyboard Modifiers (explicitly submitted by backend via AddKeyEvent() calls)
    // - Any functions taking a Moss_GuiKeyChord parameter can binary-or those with regular keys, e.g. Shortcut(Moss_GuMoss_od_Ctrl | Moss_GuiKey_S).
    // - Those are written back into io.KeyCtrl, io.KeyShift, io.KeyAlt, io.KeySuper for convenience,
    //   but may be accessed via standard key API such as IsKeyPressed(), IsKeyReleased(), querying duration etc.
    // - Code polling every key (e.g. an interface to detect a key press for input mapping) might want to ignore those
    //   and prefer using the real keys (e.g. Moss_GuiKey_LeftCtrl, Moss_GuiKey_RightCtrl instead of Moss_GuMoss_od_Ctrl).
    // - In theory the value of keyboard modifiers should be roughly equivalent to a logical or of the equivalent left/right keys.
    //   In practice: it's complicated; mods are often provided from different sources. Keyboard layout, Moss_E, sticky keys and
    //   backends tend to interfere and break that equivalence. The safer decision is to relay that ambiguity down to the end-user...
    // - On macOS, we swap Cmd(Super) and Ctrl keys at the tMoss_e of the io.AddKeyEvent() call.
    Moss_GuMoss_od_None                   = 0,
    Moss_GuMoss_od_Ctrl                   = 1 << 12, // Ctrl (non-macOS), Cmd (macOS)
    Moss_GuMoss_od_Shift                  = 1 << 13, // Shift
    Moss_GuMoss_od_Alt                    = 1 << 14, // Option/Menu
    Moss_GuMoss_od_Super                  = 1 << 15, // Windows/Super (non-macOS), Ctrl (macOS)
    Moss_GuMoss_od_Mask_                  = 0xF000,  // 4-bits

#ifndef Moss_GUI_DISABLE_OBSOLETE_FUNCTIONS
    Moss_GuiKey_COUNT                  = Moss_GuiKey_NamedKey_END,    // Obsoleted in 1.91.5 because it was extremely misleading (since named keys don't start at 0 anymore)
    Moss_GuMoss_od_Shortcut               = Moss_GuMoss_od_Ctrl,            // Removed in 1.90.7, you can now sMoss_ply use Moss_GuMoss_od_Ctrl
    Moss_GuiKey_ModCtrl = Moss_GuMoss_od_Ctrl, Moss_GuiKey_ModShift = Moss_GuMoss_od_Shift, Moss_GuiKey_ModAlt = Moss_GuMoss_od_Alt, Moss_GuiKey_ModSuper = Moss_GuMoss_od_Super, // Renamed in 1.89
    //Moss_GuiKey_KeyPadEnter = Moss_GuiKey_KeypadEnter,              // Renamed in 1.87
#endif
};

// Flags for Shortcut(), SetNextItemShortcut(),
// (and for upcoming extended versions of IsKeyPressed(), IsMouseClicked(), Shortcut(), SetKeyOwner(), SetItemKeyOwner() that are still in Moss_gui_internal.h)
// Don't mistake with Moss_GuiInputTextFlags! (which is for Moss_Gui::InputText() function)
enum Moss_GuiInputFlags_
{
    Moss_GuiInputFlags_None                    = 0,
    Moss_GuiInputFlags_Repeat                  = 1 << 0,   // Enable repeat. Return true on successive repeats. Default for legacy IsKeyPressed(). NOT Default for legacy IsMouseClicked(). MUST BE == 1.

    // Flags for Shortcut(), SetNextItemShortcut()
    // - Routing policies: RouteGlobal+OverActive >> RouteActive or RouteFocused (if owner is active item) >> RouteGlobal+OverFocused >> RouteFocused (if in focused window stack) >> RouteGlobal.
    // - Default policy is RouteFocused. Can select only 1 policy among all available.
    Moss_GuiInputFlags_RouteActive             = 1 << 10,  // Route to active item only.
    Moss_GuiInputFlags_RouteFocused            = 1 << 11,  // Route to windows in the focus stack (DEFAULT). Deep-most focused window takes inputs. Active item takes inputs over deep-most focused window.
    Moss_GuiInputFlags_RouteGlobal             = 1 << 12,  // Global route (unless a focused window or active item registered the route).
    Moss_GuiInputFlags_RouteAlways             = 1 << 13,  // Do not register route, poll keys directly.
    // - Routing options
    Moss_GuiInputFlags_RouteOverFocused        = 1 << 14,  // Option: global route: higher priority than focused route (unless active item in focused route).
    Moss_GuiInputFlags_RouteOverActive         = 1 << 15,  // Option: global route: higher priority than active item. Unlikely you need to use that: will interfere with every active items, e.g. CTRL+A registered by InputText will be overridden by this. May not be fully honored as user/internal code is likely to always assume they can access keys when active.
    Moss_GuiInputFlags_RouteUnlessBgFocused    = 1 << 16,  // Option: global route: will not be applied if underlying background/void is focused (== no Dear Moss_Gui windows are focused). Useful for overlay applications.
    Moss_GuiInputFlags_RouteFromRootWindow     = 1 << 17,  // Option: route evaluated from the point of view of root window rather than current window.

    // Flags for SetNextItemShortcut()
    Moss_GuiInputFlags_Tooltip                 = 1 << 18,  // Automatically display a tooltip when hovering item [BETA] Unsure of right api (opt-in/opt-out)
};

// Configuration flags stored in io.ConfigFlags. Set by user/application.
enum Moss_GuiConfigFlags_
{
    Moss_GuiConfigFlags_None                   = 0,
    Moss_GuiConfigFlags_NavEnableKeyboard      = 1 << 0,   // Master keyboard navigation enable flag. Enable full Tabbing + directional arrows + space/enter to activate.
    Moss_GuiConfigFlags_NavEnableGamepad       = 1 << 1,   // Master gamepad navigation enable flag. Backend also needs to set Moss_GuiBackendFlags_HasGamepad.
    Moss_GuiConfigFlags_NoMouse                = 1 << 4,   // Instruct dear Moss_gui to disable mouse inputs and interactions.
    Moss_GuiConfigFlags_NoMouseCursorChange    = 1 << 5,   // Instruct backend to not alter mouse cursor shape and visibility. Use if the backend cursor changes are interfering with yours and you don't want to use SetMouseCursor() to change mouse cursor. You may want to honor requests from Moss_gui by reading GetMouseCursor() yourself instead.
    Moss_GuiConfigFlags_NoKeyboard             = 1 << 6,   // Instruct dear Moss_gui to disable keyboard inputs and interactions. This is done by ignoring keyboard events and clearing existing states.

    // User storage (to allow your backend/engine to communicate to code that may be shared between multiple projects. Those flags are NOT used by core Dear Moss_Gui)
    Moss_GuiConfigFlags_IsSRGB                 = 1 << 20,  // Application is SRGB-aware.
    Moss_GuiConfigFlags_IsTouchScreen          = 1 << 21,  // Application is using a touch screen instead of a mouse.

#ifndef Moss_GUI_DISABLE_OBSOLETE_FUNCTIONS
    Moss_GuiConfigFlags_NavEnableSetMousePos   = 1 << 2,   // [moved/renamed in 1.91.4] -> use bool io.ConfigNavMoveSetMousePos
    Moss_GuiConfigFlags_NavNoCaptureKeyboard   = 1 << 3,   // [moved/renamed in 1.91.4] -> use bool io.ConfigNavCaptureKeyboard
#endif
};

// Backend capabilities flags stored in io.BackendFlags. Set by Moss_gui_Moss_pl_xxx or custom backend.
enum Moss_GuiBackendFlags_
{
    Moss_GuiBackendFlags_None                  = 0,
    Moss_GuiBackendFlags_HasGamepad            = 1 << 0,   // Backend Platform supports gamepad and currently has one connected.
    Moss_GuiBackendFlags_HasMouseCursors       = 1 << 1,   // Backend Platform supports honoring GetMouseCursor() value to change the OS cursor shape.
    Moss_GuiBackendFlags_HasSetMousePos        = 1 << 2,   // Backend Platform supports io.WantSetMousePos requests to reposition the OS mouse position (only used if io.ConfigNavMoveSetMousePos is set).
    Moss_GuiBackendFlags_RendererHasVtxOffset  = 1 << 3,   // Backend Renderer supports Moss_DrawCmd::VtxOffset. This enables output of large meshes (64K+ vertices) while still using 16-bit indices.
    Moss_GuiBackendFlags_RendererHasTextures   = 1 << 4,   // Backend Renderer supports Moss_TextureData requests to create/update/destroy textures. This enables incremental texture updates and texture reloads. See https://github.com/ocornut/Moss_gui/blob/master/docs/BACKENDS.md for instructions on how to upgrade your custom backend.
};

// Enumeration for PushStyleColor() / PopStyleColor()
enum Moss_GuiCol_
{
    Moss_GuiCol_Text,
    Moss_GuiCol_TextDisabled,
    Moss_GuiCol_WindowBg,              // Background of normal windows
    Moss_GuiCol_ChildBg,               // Background of child windows
    Moss_GuiCol_PopupBg,               // Background of popups, menus, tooltips windows
    Moss_GuiCol_Border,
    Moss_GuiCol_BorderShadow,
    Moss_GuiCol_FrameBg,               // Background of checkbox, radio button, plot, slider, text input
    Moss_GuiCol_FrameBgHovered,
    Moss_GuiCol_FrameBgActive,
    Moss_GuiCol_TitleBg,               // Title bar
    Moss_GuiCol_TitleBgActive,         // Title bar when focused
    Moss_GuiCol_TitleBgCollapsed,      // Title bar when collapsed
    Moss_GuiCol_MenuBarBg,
    Moss_GuiCol_ScrollbarBg,
    Moss_GuiCol_ScrollbarGrab,
    Moss_GuiCol_ScrollbarGrabHovered,
    Moss_GuiCol_ScrollbarGrabActive,
    Moss_GuiCol_CheckMark,             // Checkbox tick and RadioButton circle
    Moss_GuiCol_SliderGrab,
    Moss_GuiCol_SliderGrabActive,
    Moss_GuiCol_Button,
    Moss_GuiCol_ButtonHovered,
    Moss_GuiCol_ButtonActive,
    Moss_GuiCol_Header,                // Header* colors are used for CollapsingHeader, TreeNode, Selectable, MenuItem
    Moss_GuiCol_HeaderHovered,
    Moss_GuiCol_HeaderActive,
    Moss_GuiCol_Separator,
    Moss_GuiCol_SeparatorHovered,
    Moss_GuiCol_SeparatorActive,
    Moss_GuiCol_ResizeGrip,            // Resize grip in lower-right and lower-left corners of windows.
    Moss_GuiCol_ResizeGripHovered,
    Moss_GuiCol_ResizeGripActive,
    Moss_GuiCol_InputTextCursor,       // InputText cursor/caret
    Moss_GuiCol_TabHovered,            // Tab background, when hovered
    Moss_GuiCol_Tab,                   // Tab background, when tab-bar is focused & tab is unselected
    Moss_GuiCol_TabSelected,           // Tab background, when tab-bar is focused & tab is selected
    Moss_GuiCol_TabSelectedOverline,   // Tab horizontal overline, when tab-bar is focused & tab is selected
    Moss_GuiCol_TabDMoss_med,             // Tab background, when tab-bar is unfocused & tab is unselected
    Moss_GuiCol_TabDMoss_medSelected,     // Tab background, when tab-bar is unfocused & tab is selected
    Moss_GuiCol_TabDMoss_medSelectedOverline,//..horizontal overline, when tab-bar is unfocused & tab is selected
    Moss_GuiCol_PlotLines,
    Moss_GuiCol_PlotLinesHovered,
    Moss_GuiCol_PlotHistogram,
    Moss_GuiCol_PlotHistogramHovered,
    Moss_GuiCol_TableHeaderBg,         // Table header background
    Moss_GuiCol_TableBorderStrong,     // Table outer and header borders (prefer using Alpha=1.0 here)
    Moss_GuiCol_TableBorderLight,      // Table inner borders (prefer using Alpha=1.0 here)
    Moss_GuiCol_TableRowBg,            // Table row background (even rows)
    Moss_GuiCol_TableRowBgAlt,         // Table row background (odd rows)
    Moss_GuiCol_TextLink,              // Hyperlink color
    Moss_GuiCol_TextSelectedBg,        // Selected text inside an InputText
    Moss_GuiCol_TreeLines,             // Tree node hierarchy outlines when using Moss_GuiTreeNodeFlags_DrawLines
    Moss_GuiCol_DragDropTarget,        // Rectangle highlighting a drop target
    Moss_GuiCol_NavCursor,             // Color of keyboard/gamepad navigation cursor/rectangle, when visible
    Moss_GuiCol_NavWindowingHighlight, // Highlight window when using CTRL+TAB
    Moss_GuiCol_NavWindowingDMoss_Bg,     // Darken/colorize entire screen behind the CTRL+TAB window list, when active
    Moss_GuiCol_ModalWindowDMoss_Bg,      // Darken/colorize entire screen behind a modal window, when one is active
    Moss_GuiCol_COUNT,

#ifndef Moss_GUI_DISABLE_OBSOLETE_FUNCTIONS
    Moss_GuiCol_TabActive = Moss_GuiCol_TabSelected,                  // [renamed in 1.90.9]
    Moss_GuiCol_TabUnfocused = Moss_GuiCol_TabDMoss_med,                 // [renamed in 1.90.9]
    Moss_GuiCol_TabUnfocusedActive = Moss_GuiCol_TabDMoss_medSelected,   // [renamed in 1.90.9]
    Moss_GuiCol_NavHighlight = Moss_GuiCol_NavCursor,                 // [renamed in 1.91.4]
#endif
};

// Enumeration for PushStyleVar() / PopStyleVar() to temporarily modify the Moss_GuiStyle structure.
// - The enum only refers to fields of Moss_GuiStyle which makes sense to be pushed/popped inside UI code.
//   During initialization or between frames, feel free to just poke into Moss_GuiStyle directly.
// - Tip: Use your programming IDE navigation facilities on the names in the _second column_ below to find the actual members and their description.
//   - In Visual Studio: CTRL+comma ("Edit.GoToAll") can follow symbols inside comments, whereas CTRL+F12 ("Edit.GoToMoss_plementation") cannot.
//   - In Visual Studio w/ Visual Assist installed: ALT+G ("VAssistX.GoToMoss_plementation") can also follow symbols inside comments.
//   - In VS Code, CLion, etc.: CTRL+click can follow symbols inside comments.
// - When changing this enum, you need to update the associated internal table GStyleVarInfo[] accordingly. This is where we link enum values to members offset/type.
enum Moss_GuiStyleVar_
{
    // Enum name -------------------------- // Member in Moss_GuiStyle structure (see Moss_GuiStyle for descriptions)
    Moss_GuiStyleVar_Alpha,                    // float     Alpha
    Moss_GuiStyleVar_DisabledAlpha,            // float     DisabledAlpha
    Moss_GuiStyleVar_WindowPadding,            // Moss_Vec2    WindowPadding
    Moss_GuiStyleVar_WindowRounding,           // float     WindowRounding
    Moss_GuiStyleVar_WindowBorderSize,         // float     WindowBorderSize
    Moss_GuiStyleVar_WindowMinSize,            // Moss_Vec2    WindowMinSize
    Moss_GuiStyleVar_WindowTitleAlign,         // Moss_Vec2    WindowTitleAlign
    Moss_GuiStyleVar_ChildRounding,            // float     ChildRounding
    Moss_GuiStyleVar_ChildBorderSize,          // float     ChildBorderSize
    Moss_GuiStyleVar_PopupRounding,            // float     PopupRounding
    Moss_GuiStyleVar_PopupBorderSize,          // float     PopupBorderSize
    Moss_GuiStyleVar_FramePadding,             // Moss_Vec2    FramePadding
    Moss_GuiStyleVar_FrameRounding,            // float     FrameRounding
    Moss_GuiStyleVar_FrameBorderSize,          // float     FrameBorderSize
    Moss_GuiStyleVar_ItemSpacing,              // Moss_Vec2    ItemSpacing
    Moss_GuiStyleVar_ItemInnerSpacing,         // Moss_Vec2    ItemInnerSpacing
    Moss_GuiStyleVar_IndentSpacing,            // float     IndentSpacing
    Moss_GuiStyleVar_CellPadding,              // Moss_Vec2    CellPadding
    Moss_GuiStyleVar_ScrollbarSize,            // float     ScrollbarSize
    Moss_GuiStyleVar_ScrollbarRounding,        // float     ScrollbarRounding
    Moss_GuiStyleVar_GrabMinSize,              // float     GrabMinSize
    Moss_GuiStyleVar_GrabRounding,             // float     GrabRounding
    Moss_GuiStyleVar_Moss_ageBorderSize,          // float     Moss_ageBorderSize
    Moss_GuiStyleVar_TabRounding,              // float     TabRounding
    Moss_GuiStyleVar_TabBorderSize,            // float     TabBorderSize
    Moss_GuiStyleVar_TabBarBorderSize,         // float     TabBarBorderSize
    Moss_GuiStyleVar_TabBarOverlineSize,       // float     TabBarOverlineSize
    Moss_GuiStyleVar_TableAngledHeadersAngle,  // float     TableAngledHeadersAngle
    Moss_GuiStyleVar_TableAngledHeadersTextAlign,// Moss_Vec2  TableAngledHeadersTextAlign
    Moss_GuiStyleVar_TreeLinesSize,            // float     TreeLinesSize
    Moss_GuiStyleVar_TreeLinesRounding,        // float     TreeLinesRounding
    Moss_GuiStyleVar_ButtonTextAlign,          // Moss_Vec2    ButtonTextAlign
    Moss_GuiStyleVar_SelectableTextAlign,      // Moss_Vec2    SelectableTextAlign
    Moss_GuiStyleVar_SeparatorTextBorderSize,  // float     SeparatorTextBorderSize
    Moss_GuiStyleVar_SeparatorTextAlign,       // Moss_Vec2    SeparatorTextAlign
    Moss_GuiStyleVar_SeparatorTextPadding,     // Moss_Vec2    SeparatorTextPadding
    Moss_GuiStyleVar_COUNT
};

// Flags for InvisibleButton() [extended in Moss_gui_internal.h]
enum Moss_GuiButtonFlags_
{
    Moss_GuiButtonFlags_None                   = 0,
    Moss_GuiButtonFlags_MouseButtonLeft        = 1 << 0,   // React on left mouse button (default)
    Moss_GuiButtonFlags_MouseButtonRight       = 1 << 1,   // React on right mouse button
    Moss_GuiButtonFlags_MouseButtonMiddle      = 1 << 2,   // React on center mouse button
    Moss_GuiButtonFlags_MouseButtonMask_       = Moss_GuiButtonFlags_MouseButtonLeft | Moss_GuiButtonFlags_MouseButtonRight | Moss_GuiButtonFlags_MouseButtonMiddle, // [Internal]
    Moss_GuiButtonFlags_EnableNav              = 1 << 3,   // InvisibleButton(): do not disable navigation/tabbing. Otherwise disabled by default.
};

// Flags for ColorEdit3() / ColorEdit4() / ColorPicker3() / ColorPicker4() / ColorButton()
enum Moss_GuiColorEditFlags_
{
    Moss_GuiColorEditFlags_None            = 0,
    Moss_GuiColorEditFlags_NoAlpha         = 1 << 1,   //              // ColorEdit, ColorPicker, ColorButton: ignore Alpha component (will only read 3 components from the input pointer).
    Moss_GuiColorEditFlags_NoPicker        = 1 << 2,   //              // ColorEdit: disable picker when clicking on color square.
    Moss_GuiColorEditFlags_NoOptions       = 1 << 3,   //              // ColorEdit: disable toggling options menu when right-clicking on inputs/small preview.
    Moss_GuiColorEditFlags_NoSmallPreview  = 1 << 4,   //              // ColorEdit, ColorPicker: disable color square preview next to the inputs. (e.g. to show only the inputs)
    Moss_GuiColorEditFlags_NoInputs        = 1 << 5,   //              // ColorEdit, ColorPicker: disable inputs sliders/text widgets (e.g. to show only the small preview color square).
    Moss_GuiColorEditFlags_NoTooltip       = 1 << 6,   //              // ColorEdit, ColorPicker, ColorButton: disable tooltip when hovering the preview.
    Moss_GuiColorEditFlags_NoLabel         = 1 << 7,   //              // ColorEdit, ColorPicker: disable display of inline text label (the label is still forwarded to the tooltip and picker).
    Moss_GuiColorEditFlags_NoSidePreview   = 1 << 8,   //              // ColorPicker: disable bigger color preview on right side of the picker, use small color square preview instead.
    Moss_GuiColorEditFlags_NoDragDrop      = 1 << 9,   //              // ColorEdit: disable drag and drop target. ColorButton: disable drag and drop source.
    Moss_GuiColorEditFlags_NoBorder        = 1 << 10,  //              // ColorButton: disable border (which is enforced by default)

    // Alpha preview
    // - Prior to 1.91.8 (2025/01/21): alpha was made opaque in the preview by default using old name Moss_GuiColorEditFlags_AlphaPreview.
    // - We now display the preview as transparent by default. You can use Moss_GuiColorEditFlags_AlphaOpaque to use old behavior.
    // - The new flags may be combined better and allow finer controls.
    Moss_GuiColorEditFlags_AlphaOpaque     = 1 << 11,  //              // ColorEdit, ColorPicker, ColorButton: disable alpha in the preview,. Contrary to _NoAlpha it may still be edited when calling ColorEdit4()/ColorPicker4(). For ColorButton() this does the same as _NoAlpha.
    Moss_GuiColorEditFlags_AlphaNoBg       = 1 << 12,  //              // ColorEdit, ColorPicker, ColorButton: disable rendering a checkerboard background behind transparent color.
    Moss_GuiColorEditFlags_AlphaPreviewHalf= 1 << 13,  //              // ColorEdit, ColorPicker, ColorButton: display half opaque / half transparent preview.

    // User Options (right-click on widget to change some of them).
    Moss_GuiColorEditFlags_AlphaBar        = 1 << 16,  //              // ColorEdit, ColorPicker: show vertical alpha bar/gradient in picker.
    Moss_GuiColorEditFlags_HDR             = 1 << 19,  //              // (WIP) ColorEdit: Currently only disable 0.0f..1.0f lMoss_its in RGBA edition (note: you probably want to use Moss_GuiColorEditFlags_Float flag as well).
    Moss_GuiColorEditFlags_DisplayRGB      = 1 << 20,  // [Display]    // ColorEdit: override _display_ type among RGB/HSV/Hex. ColorPicker: select any combination using one or more of RGB/HSV/Hex.
    Moss_GuiColorEditFlags_DisplayHSV      = 1 << 21,  // [Display]    // "
    Moss_GuiColorEditFlags_DisplayHex      = 1 << 22,  // [Display]    // "
    Moss_GuiColorEditFlags_Uint8           = 1 << 23,  // [DataType]   // ColorEdit, ColorPicker, ColorButton: _display_ values formatted as 0..255.
    Moss_GuiColorEditFlags_Float           = 1 << 24,  // [DataType]   // ColorEdit, ColorPicker, ColorButton: _display_ values formatted as 0.0f..1.0f floats instead of 0..255 integers. No round-trip of value via integers.
    Moss_GuiColorEditFlags_PickerHueBar    = 1 << 25,  // [Picker]     // ColorPicker: bar for Hue, rectangle for Sat/Value.
    Moss_GuiColorEditFlags_PickerHueWheel  = 1 << 26,  // [Picker]     // ColorPicker: wheel for Hue, triangle for Sat/Value.
    Moss_GuiColorEditFlags_InputRGB        = 1 << 27,  // [Input]      // ColorEdit, ColorPicker: input and output data in RGB format.
    Moss_GuiColorEditFlags_InputHSV        = 1 << 28,  // [Input]      // ColorEdit, ColorPicker: input and output data in HSV format.

    // Defaults Options. You can set application defaults using SetColorEditOptions(). The intent is that you probably don't want to
    // override them in most of your calls. Let the user choose via the option menu and/or call SetColorEditOptions() once during startup.
    Moss_GuiColorEditFlags_DefaultOptions_ = Moss_GuiColorEditFlags_Uint8 | Moss_GuiColorEditFlags_DisplayRGB | Moss_GuiColorEditFlags_InputRGB | Moss_GuiColorEditFlags_PickerHueBar,

    // [Internal] Masks
    Moss_GuiColorEditFlags_AlphaMask_      = Moss_GuiColorEditFlags_NoAlpha | Moss_GuiColorEditFlags_AlphaOpaque | Moss_GuiColorEditFlags_AlphaNoBg | Moss_GuiColorEditFlags_AlphaPreviewHalf,
    Moss_GuiColorEditFlags_DisplayMask_    = Moss_GuiColorEditFlags_DisplayRGB | Moss_GuiColorEditFlags_DisplayHSV | Moss_GuiColorEditFlags_DisplayHex,
    Moss_GuiColorEditFlags_DataTypeMask_   = Moss_GuiColorEditFlags_Uint8 | Moss_GuiColorEditFlags_Float,
    Moss_GuiColorEditFlags_PickerMask_     = Moss_GuiColorEditFlags_PickerHueWheel | Moss_GuiColorEditFlags_PickerHueBar,
    Moss_GuiColorEditFlags_InputMask_      = Moss_GuiColorEditFlags_InputRGB | Moss_GuiColorEditFlags_InputHSV,

    // Obsolete names
#ifndef Moss_GUI_DISABLE_OBSOLETE_FUNCTIONS
    Moss_GuiColorEditFlags_AlphaPreview = 0,         // [Removed in 1.91.8] This is the default now. Will display a checkerboard unless Moss_GuiColorEditFlags_AlphaNoBg is set.
#endif
    //Moss_GuiColorEditFlags_RGB = Moss_GuiColorEditFlags_DisplayRGB, Moss_GuiColorEditFlags_HSV = Moss_GuiColorEditFlags_DisplayHSV, Moss_GuiColorEditFlags_HEX = Moss_GuiColorEditFlags_DisplayHex  // [renamed in 1.69]
};

// Flags for DragFloat(), DragInt(), SliderFloat(), SliderInt() etc.
// We use the same sets of flags for DragXXX() and SliderXXX() functions as the features are the same and it makes it easier to swap them.
// (Those are per-item flags. There is shared behavior flag too: Moss_GuiIO: io.ConfigDragClickToInputText)
enum Moss_GuiSliderFlags_
{
    Moss_GuiSliderFlags_None               = 0,
    Moss_GuiSliderFlags_Logarithmic        = 1 << 5,       // Make the widget logarithmic (linear otherwise). Consider using Moss_GuiSliderFlags_NoRoundToFormat with this if using a format-string with small amount of digits.
    Moss_GuiSliderFlags_NoRoundToFormat    = 1 << 6,       // Disable rounding underlying value to match precision of the display format string (e.g. %.3f values are rounded to those 3 digits).
    Moss_GuiSliderFlags_NoInput            = 1 << 7,       // Disable CTRL+Click or Enter key allowing to input text directly into the widget.
    Moss_GuiSliderFlags_WrapAround         = 1 << 8,       // Enable wrapping around from max to min and from min to max. Only supported by DragXXX() functions for now.
    Moss_GuiSliderFlags_ClampOnInput       = 1 << 9,       // Clamp value to min/max bounds when input manually with CTRL+Click. By default CTRL+Click allows going out of bounds.
    Moss_GuiSliderFlags_ClampZeroRange     = 1 << 10,      // Clamp even if min==max==0.0f. Otherwise due to legacy reason DragXXX functions don't clamp with those values. When your clamping lMoss_its are dynamic you almost always want to use it.
    Moss_GuiSliderFlags_NoSpeedTweaks      = 1 << 11,      // Disable keyboard modifiers altering tweak speed. Useful if you want to alter tweak speed yourself based on your own logic.
    Moss_GuiSliderFlags_AlwaysClamp        = Moss_GuiSliderFlags_ClampOnInput | Moss_GuiSliderFlags_ClampZeroRange,
    Moss_GuiSliderFlags_InvalidMask_       = 0x7000000F,   // [Internal] We treat using those bits as being potentially a 'float power' argument from the previous API that has got miscast to this enum, and will trigger an assert if needed.
};

// Identify a mouse button.
// Those values are guaranteed to be stable and we frequently use 0/1 directly. Named enums provided for convenience.
enum Moss_GuMoss_ouseButton_
{
    Moss_GuMoss_ouseButton_Left = 0,
    Moss_GuMoss_ouseButton_Right = 1,
    Moss_GuMoss_ouseButton_Middle = 2,
    Moss_GuMoss_ouseButton_COUNT = 5
};

// Enumeration for GetMouseCursor()
// User code may request backend to display given cursor by calling SetMouseCursor(), which is why we have some cursors that are marked unused here
enum Moss_GuMoss_ouseCursor_
{
    Moss_GuMoss_ouseCursor_None = -1,
    Moss_GuMoss_ouseCursor_Arrow = 0,
    Moss_GuMoss_ouseCursor_TextInput,         // When hovering over InputText, etc.
    Moss_GuMoss_ouseCursor_ResizeAll,         // (Unused by Dear Moss_Gui functions)
    Moss_GuMoss_ouseCursor_ResizeNS,          // When hovering over a horizontal border
    Moss_GuMoss_ouseCursor_ResizeEW,          // When hovering over a vertical border or a column
    Moss_GuMoss_ouseCursor_ResizeNESW,        // When hovering over the bottom-left corner of a window
    Moss_GuMoss_ouseCursor_ResizeNWSE,        // When hovering over the bottom-right corner of a window
    Moss_GuMoss_ouseCursor_Hand,              // (Unused by Dear Moss_Gui functions. Use for e.g. hyperlinks)
    Moss_GuMoss_ouseCursor_Wait,              // When waiting for something to process/load.
    Moss_GuMoss_ouseCursor_Progress,          // When waiting for something to process/load, but application is still interactive.
    Moss_GuMoss_ouseCursor_NotAllowed,        // When hovering something with disallowed interaction. Usually a crossed circle.
    Moss_GuMoss_ouseCursor_COUNT
};

// Enumeration for AddMouseSourceEvent() actual source of Mouse Input data.
// Historically we use "Mouse" terminology everywhere to indicate pointer data, e.g. MousePos, IsMousePressed(), io.AddMousePosEvent()
// But that "Mouse" data can come from different source which occasionally may be useful for application to know about.
// You can submit a change of pointer type using io.AddMouseSourceEvent().
enum Moss_GuMoss_ouseSource : int
{
    Moss_GuMoss_ouseSource_Mouse = 0,         // Input is coming from an actual mouse.
    Moss_GuMoss_ouseSource_TouchScreen,       // Input is coming from a touch screen (no hovering prior to initial press, less precise initial press aMoss_ing, dual-axis wheeling possible).
    Moss_GuMoss_ouseSource_Pen,               // Input is coming from a pressure/magnetic pen (often used in conjunction with high-sampling rates).
    Moss_GuMoss_ouseSource_COUNT
};

// Enumeration for Moss_Gui::SetNextWindow***(), SetWindow***(), SetNextItem***() functions
// Represent a condition.
// Moss_portant: Treat as a regular enum! Do NOT combine multiple values using binary operators! All the functions above treat 0 as a shortcut to Moss_GuiCond_Always.
enum Moss_GuiCond_
{
    Moss_GuiCond_None          = 0,        // No condition (always set the variable), same as _Always
    Moss_GuiCond_Always        = 1 << 0,   // No condition (always set the variable), same as _None
    Moss_GuiCond_Once          = 1 << 1,   // Set the variable once per runtMoss_e session (only the first call will succeed)
    Moss_GuiCond_FirstUseEver  = 1 << 2,   // Set the variable if the object/window has no persistently saved data (no entry in .ini file)
    Moss_GuiCond_Appearing     = 1 << 3,   // Set the variable if the object/window is appearing after being hidden/inactive (or the first tMoss_e)
};

//-----------------------------------------------------------------------------
// [SECTION] Tables API flags and structures (Moss_GuiTableFlags, Moss_GuiTableColumnFlags, Moss_GuiTableRowFlags, Moss_GuiTableBgTarget, Moss_GuiTableSortSpecs, Moss_GuiTableColumnSortSpecs)
//-----------------------------------------------------------------------------

// Flags for Moss_Gui::BeginTable()
// - Moss_portant! Sizing policies have complex and subtle side effects, much more so than you would expect.
//   Read comments/demos carefully + experMoss_ent with live demos to get acquainted with them.
// - The DEFAULT sizing policies are:
//    - Default to Moss_GuiTableFlags_SizingFixedFit    if ScrollX is on, or if host window has Moss_GuiWindowFlags_AlwaysAutoResize.
//    - Default to Moss_GuiTableFlags_SizingStretchSame if ScrollX is off.
// - When ScrollX is off:
//    - Table defaults to Moss_GuiTableFlags_SizingStretchSame -> all Columns defaults to Moss_GuiTableColumnFlags_WidthStretch with same weight.
//    - Columns sizing policy allowed: Stretch (default), Fixed/Auto.
//    - Fixed Columns (if any) will generally obtain their requested width (unless the table cannot fit them all).
//    - Stretch Columns will share the remaining width according to their respective weight.
//    - Mixed Fixed/Stretch columns is possible but has various side-effects on resizing behaviors.
//      The typical use of mixing sizing policies is: any number of LEADING Fixed columns, followed by one or two TRAILING Stretch columns.
//      (this is because the visible order of columns have subtle but necessary effects on how they react to manual resizing).
// - When ScrollX is on:
//    - Table defaults to Moss_GuiTableFlags_SizingFixedFit -> all Columns defaults to Moss_GuiTableColumnFlags_WidthFixed
//    - Columns sizing policy allowed: Fixed/Auto mostly.
//    - Fixed Columns can be enlarged as needed. Table will show a horizontal scrollbar if needed.
//    - When using auto-resizing (non-resizable) fixed columns, querying the content width to use item right-alignment e.g. SetNextItemWidth(-FLT_MIN) doesn't make sense, would create a feedback loop.
//    - Using Stretch columns OFTEN DOES NOT MAKE SENSE if ScrollX is on, UNLESS you have specified a value for 'inner_width' in BeginTable().
//      If you specify a value for 'inner_width' then effectively the scrolling space is known and Stretch or mixed Fixed/Stretch columns become meaningful again.
// - Read on documentation at the top of Moss_gui_tables.cpp for details.
enum Moss_GuiTableFlags_
{
    // Features
    Moss_GuiTableFlags_None                       = 0,
    Moss_GuiTableFlags_Resizable                  = 1 << 0,   // Enable resizing columns.
    Moss_GuiTableFlags_Reorderable                = 1 << 1,   // Enable reordering columns in header row (need calling TableSetupColumn() + TableHeadersRow() to display headers)
    Moss_GuiTableFlags_Hideable                   = 1 << 2,   // Enable hiding/disabling columns in context menu.
    Moss_GuiTableFlags_Sortable                   = 1 << 3,   // Enable sorting. Call TableGetSortSpecs() to obtain sort specs. Also see Moss_GuiTableFlags_SortMulti and Moss_GuiTableFlags_SortTristate.
    Moss_GuiTableFlags_NoSavedSettings            = 1 << 4,   // Disable persisting columns order, width and sort settings in the .ini file.
    Moss_GuiTableFlags_ContextMenuInBody          = 1 << 5,   // Right-click on columns body/contents will display table context menu. By default it is available in TableHeadersRow().
    // Decorations
    Moss_GuiTableFlags_RowBg                      = 1 << 6,   // Set each RowBg color with Moss_GuiCol_TableRowBg or Moss_GuiCol_TableRowBgAlt (equivalent of calling TableSetBgColor with Moss_GuiTableBgFlags_RowBg0 on each row manually)
    Moss_GuiTableFlags_BordersInnerH              = 1 << 7,   // Draw horizontal borders between rows.
    Moss_GuiTableFlags_BordersOuterH              = 1 << 8,   // Draw horizontal borders at the top and bottom.
    Moss_GuiTableFlags_BordersInnerV              = 1 << 9,   // Draw vertical borders between columns.
    Moss_GuiTableFlags_BordersOuterV              = 1 << 10,  // Draw vertical borders on the left and right sides.
    Moss_GuiTableFlags_BordersH                   = Moss_GuiTableFlags_BordersInnerH | Moss_GuiTableFlags_BordersOuterH, // Draw horizontal borders.
    Moss_GuiTableFlags_BordersV                   = Moss_GuiTableFlags_BordersInnerV | Moss_GuiTableFlags_BordersOuterV, // Draw vertical borders.
    Moss_GuiTableFlags_BordersInner               = Moss_GuiTableFlags_BordersInnerV | Moss_GuiTableFlags_BordersInnerH, // Draw inner borders.
    Moss_GuiTableFlags_BordersOuter               = Moss_GuiTableFlags_BordersOuterV | Moss_GuiTableFlags_BordersOuterH, // Draw outer borders.
    Moss_GuiTableFlags_Borders                    = Moss_GuiTableFlags_BordersInner | Moss_GuiTableFlags_BordersOuter,   // Draw all borders.
    Moss_GuiTableFlags_NoBordersInBody            = 1 << 11,  // [ALPHA] Disable vertical borders in columns Body (borders will always appear in Headers). -> May move to style
    Moss_GuiTableFlags_NoBordersInBodyUntilResize = 1 << 12,  // [ALPHA] Disable vertical borders in columns Body until hovered for resize (borders will always appear in Headers). -> May move to style
    // Sizing Policy (read above for defaults)
    Moss_GuiTableFlags_SizingFixedFit             = 1 << 13,  // Columns default to _WidthFixed or _WidthAuto (if resizable or not resizable), matching contents width.
    Moss_GuiTableFlags_SizingFixedSame            = 2 << 13,  // Columns default to _WidthFixed or _WidthAuto (if resizable or not resizable), matching the maxMoss_um contents width of all columns. Moss_plicitly enable Moss_GuiTableFlags_NoKeepColumnsVisible.
    Moss_GuiTableFlags_SizingStretchProp          = 3 << 13,  // Columns default to _WidthStretch with default weights proportional to each columns contents widths.
    Moss_GuiTableFlags_SizingStretchSame          = 4 << 13,  // Columns default to _WidthStretch with default weights all equal, unless overridden by TableSetupColumn().
    // Sizing Extra Options
    Moss_GuiTableFlags_NoHostExtendX              = 1 << 16,  // Make outer width auto-fit to columns, overriding outer_size.x value. Only available when ScrollX/ScrollY are disabled and Stretch columns are not used.
    Moss_GuiTableFlags_NoHostExtendY              = 1 << 17,  // Make outer height stop exactly at outer_size.y (prevent auto-extending table past the lMoss_it). Only available when ScrollX/ScrollY are disabled. Data below the lMoss_it will be clipped and not visible.
    Moss_GuiTableFlags_NoKeepColumnsVisible       = 1 << 18,  // Disable keeping column always minMoss_ally visible when ScrollX is off and table gets too small. Not recommended if columns are resizable.
    Moss_GuiTableFlags_PreciseWidths              = 1 << 19,  // Disable distributing remainder width to stretched columns (width allocation on a 100-wide table with 3 columns: Without this flag: 33,33,34. With this flag: 33,33,33). With larger number of columns, resizing will appear to be less smooth.
    // Clipping
    Moss_GuiTableFlags_NoClip                     = 1 << 20,  // Disable clipping rectangle for every individual columns (reduce draw command count, items will be able to overflow into other columns). Generally incompatible with TableSetupScrollFreeze().
    // Padding
    Moss_GuiTableFlags_PadOuterX                  = 1 << 21,  // Default if BordersOuterV is on. Enable outermost padding. Generally desirable if you have headers.
    Moss_GuiTableFlags_NoPadOuterX                = 1 << 22,  // Default if BordersOuterV is off. Disable outermost padding.
    Moss_GuiTableFlags_NoPadInnerX                = 1 << 23,  // Disable inner padding between columns (double inner padding if BordersOuterV is on, single inner padding if BordersOuterV is off).
    // Scrolling
    Moss_GuiTableFlags_ScrollX                    = 1 << 24,  // Enable horizontal scrolling. Require 'outer_size' parameter of BeginTable() to specify the container size. Changes default sizing policy. Because this creates a child window, ScrollY is currently generally recommended when using ScrollX.
    Moss_GuiTableFlags_ScrollY                    = 1 << 25,  // Enable vertical scrolling. Require 'outer_size' parameter of BeginTable() to specify the container size.
    // Sorting
    Moss_GuiTableFlags_SortMulti                  = 1 << 26,  // Hold shift when clicking headers to sort on multiple column. TableGetSortSpecs() may return specs where (SpecsCount > 1).
    Moss_GuiTableFlags_SortTristate               = 1 << 27,  // Allow no sorting, disable default sorting. TableGetSortSpecs() may return specs where (SpecsCount == 0).
    // Miscellaneous
    Moss_GuiTableFlags_HighlightHoveredColumn     = 1 << 28,  // Highlight column headers when hovered (may evolve into a fuller highlight)

    // [Internal] Combinations and masks
    Moss_GuiTableFlags_SizingMask_                = Moss_GuiTableFlags_SizingFixedFit | Moss_GuiTableFlags_SizingFixedSame | Moss_GuiTableFlags_SizingStretchProp | Moss_GuiTableFlags_SizingStretchSame,
};

// Flags for Moss_Gui::TableSetupColumn()
enum Moss_GuiTableColumnFlags_
{
    // Input configuration flags
    Moss_GuiTableColumnFlags_None                  = 0,
    Moss_GuiTableColumnFlags_Disabled              = 1 << 0,   // Overriding/master disable flag: hide column, won't show in context menu (unlike calling TableSetColumnEnabled() which manipulates the user accessible state)
    Moss_GuiTableColumnFlags_DefaultHide           = 1 << 1,   // Default as a hidden/disabled column.
    Moss_GuiTableColumnFlags_DefaultSort           = 1 << 2,   // Default as a sorting column.
    Moss_GuiTableColumnFlags_WidthStretch          = 1 << 3,   // Column will stretch. Preferable with horizontal scrolling disabled (default if table sizing policy is _SizingStretchSame or _SizingStretchProp).
    Moss_GuiTableColumnFlags_WidthFixed            = 1 << 4,   // Column will not stretch. Preferable with horizontal scrolling enabled (default if table sizing policy is _SizingFixedFit and table is resizable).
    Moss_GuiTableColumnFlags_NoResize              = 1 << 5,   // Disable manual resizing.
    Moss_GuiTableColumnFlags_NoReorder             = 1 << 6,   // Disable manual reordering this column, this will also prevent other columns from crossing over this column.
    Moss_GuiTableColumnFlags_NoHide                = 1 << 7,   // Disable ability to hide/disable this column.
    Moss_GuiTableColumnFlags_NoClip                = 1 << 8,   // Disable clipping for this column (all NoClip columns will render in a same draw command).
    Moss_GuiTableColumnFlags_NoSort                = 1 << 9,   // Disable ability to sort on this field (even if Moss_GuiTableFlags_Sortable is set on the table).
    Moss_GuiTableColumnFlags_NoSortAscending       = 1 << 10,  // Disable ability to sort in the ascending direction.
    Moss_GuiTableColumnFlags_NoSortDescending      = 1 << 11,  // Disable ability to sort in the descending direction.
    Moss_GuiTableColumnFlags_NoHeaderLabel         = 1 << 12,  // TableHeadersRow() will submit an empty label for this column. Convenient for some small columns. Name will still appear in context menu or in angled headers. You may append into this cell by calling TableSetColumnIndex() right after the TableHeadersRow() call.
    Moss_GuiTableColumnFlags_NoHeaderWidth         = 1 << 13,  // Disable header text width contribution to automatic column width.
    Moss_GuiTableColumnFlags_PreferSortAscending   = 1 << 14,  // Make the initial sort direction Ascending when first sorting on this column (default).
    Moss_GuiTableColumnFlags_PreferSortDescending  = 1 << 15,  // Make the initial sort direction Descending when first sorting on this column.
    Moss_GuiTableColumnFlags_IndentEnable          = 1 << 16,  // Use current Indent value when entering cell (default for column 0).
    Moss_GuiTableColumnFlags_IndentDisable         = 1 << 17,  // Ignore current Indent value when entering cell (default for columns > 0). Indentation changes _within_ the cell will still be honored.
    Moss_GuiTableColumnFlags_AngledHeader          = 1 << 18,  // TableHeadersRow() will submit an angled header row for this column. Note this will add an extra row.

    // Output status flags, read-only via TableGetColumnFlags()
    Moss_GuiTableColumnFlags_IsEnabled             = 1 << 24,  // Status: is enabled == not hidden by user/api (referred to as "Hide" in _DefaultHide and _NoHide) flags.
    Moss_GuiTableColumnFlags_IsVisible             = 1 << 25,  // Status: is visible == is enabled AND not clipped by scrolling.
    Moss_GuiTableColumnFlags_IsSorted              = 1 << 26,  // Status: is currently part of the sort specs
    Moss_GuiTableColumnFlags_IsHovered             = 1 << 27,  // Status: is hovered by mouse

    // [Internal] Combinations and masks
    Moss_GuiTableColumnFlags_WidthMask_            = Moss_GuiTableColumnFlags_WidthStretch | Moss_GuiTableColumnFlags_WidthFixed,
    Moss_GuiTableColumnFlags_IndentMask_           = Moss_GuiTableColumnFlags_IndentEnable | Moss_GuiTableColumnFlags_IndentDisable,
    Moss_GuiTableColumnFlags_StatusMask_           = Moss_GuiTableColumnFlags_IsEnabled | Moss_GuiTableColumnFlags_IsVisible | Moss_GuiTableColumnFlags_IsSorted | Moss_GuiTableColumnFlags_IsHovered,
    Moss_GuiTableColumnFlags_NoDirectResize_       = 1 << 30,  // [Internal] Disable user resizing this column directly (it may however we resized indirectly from its left edge)
};

// Flags for Moss_Gui::TableNextRow()
enum Moss_GuiTableRowFlags_
{
    Moss_GuiTableRowFlags_None                     = 0,
    Moss_GuiTableRowFlags_Headers                  = 1 << 0,   // Identify header row (set default background color + width of its contents accounted differently for auto column width)
};

// Enum for Moss_Gui::TableSetBgColor()
// Background colors are rendering in 3 layers:
//  - Layer 0: draw with RowBg0 color if set, otherwise draw with ColumnBg0 if set.
//  - Layer 1: draw with RowBg1 color if set, otherwise draw with ColumnBg1 if set.
//  - Layer 2: draw with CellBg color if set.
// The purpose of the two row/columns layers is to let you decide if a background color change should override or blend with the existing color.
// When using Moss_GuiTableFlags_RowBg on the table, each row has the RowBg0 color automatically set for odd/even rows.
// If you set the color of RowBg0 target, your color will override the existing RowBg0 color.
// If you set the color of RowBg1 or ColumnBg1 target, your color will blend over the RowBg0 color.
enum Moss_GuiTableBgTarget_
{
    Moss_GuiTableBgTarget_None                     = 0,
    Moss_GuiTableBgTarget_RowBg0                   = 1,        // Set row background color 0 (generally used for background, automatically set when Moss_GuiTableFlags_RowBg is used)
    Moss_GuiTableBgTarget_RowBg1                   = 2,        // Set row background color 1 (generally used for selection marking)
    Moss_GuiTableBgTarget_CellBg                   = 3,        // Set cell background color (top-most color)
};







struct Moss_GuiStyle
{
    // Font scaling
    // - recap: Moss_Gui::GetFontSize() == FontSizeBase * (FontScaleMain * FontScaleDpi * other_scaling_factors)
    float       FontSizeBase;               // Current base font size before external global factors are applied. Use PushFont(NULL, size) to modify. Use Moss_Gui::GetFontSize() to obtain scaled value.
    float       FontScaleMain;              // Main global scale factor. May be set by application once, or exposed to end-user.
    float       FontScaleDpi;               // Additional global scale factor from viewport/monitor contents scale. When io.ConfigDpiScaleFonts is enabled, this is automatically overwritten when changing monitor DPI.

    float       Alpha;                      // Global alpha applies to everything in Dear Moss_Gui.
    float       DisabledAlpha;              // Additional alpha multiplier applied by BeginDisabled(). Multiply over current value of Alpha.
    Moss_Vec2      WindowPadding;              // Padding within a window.
    float       WindowRounding;             // Radius of window corners rounding. Set to 0.0f to have rectangular windows. Large values tend to lead to variety of artifacts and are not recommended.
    float       WindowBorderSize;           // Thickness of border around windows. Generally set to 0.0f or 1.0f. (Other values are not well tested and more CPU/GPU costly).
    float       WindowBorderHoverPadding;   // Hit-testing extent outside/inside resizing border. Also extend determination of hovered window. Generally meaningfully larger than WindowBorderSize to make it easy to reach borders.
    Moss_Vec2      WindowMinSize;              // MinMoss_um window size. This is a global setting. If you want to constrain individual windows, use SetNextWindowSizeConstraints().
    Moss_Vec2      WindowTitleAlign;           // Alignment for title bar text. Defaults to (0.0f,0.5f) for left-aligned,vertically centered.
    Moss_GuiDir    WindowMenuButtonPosition;   // Side of the collapsing/docking button in the title bar (None/Left/Right). Defaults to Moss_GuiDir_Left.
    float       ChildRounding;              // Radius of child window corners rounding. Set to 0.0f to have rectangular windows.
    float       ChildBorderSize;            // Thickness of border around child windows. Generally set to 0.0f or 1.0f. (Other values are not well tested and more CPU/GPU costly).
    float       PopupRounding;              // Radius of popup window corners rounding. (Note that tooltip windows use WindowRounding)
    float       PopupBorderSize;            // Thickness of border around popup/tooltip windows. Generally set to 0.0f or 1.0f. (Other values are not well tested and more CPU/GPU costly).
    Moss_Vec2      FramePadding;               // Padding within a framed rectangle (used by most widgets).
    float       FrameRounding;              // Radius of frame corners rounding. Set to 0.0f to have rectangular frame (used by most widgets).
    float       FrameBorderSize;            // Thickness of border around frames. Generally set to 0.0f or 1.0f. (Other values are not well tested and more CPU/GPU costly).
    Moss_Vec2      ItemSpacing;                // Horizontal and vertical spacing between widgets/lines.
    Moss_Vec2      ItemInnerSpacing;           // Horizontal and vertical spacing between within elements of a composed widget (e.g. a slider and its label).
    Moss_Vec2      CellPadding;                // Padding within a table cell. Cellpadding.x is locked for entire table. CellPadding.y may be altered between different rows.
    Moss_Vec2      TouchExtraPadding;          // Expand reactive bounding box for touch-based system where touch position is not accurate enough. Unfortunately we don't sort widgets so priority on overlap will always be given to the first widget. So don't grow this too much!
    float       IndentSpacing;              // Horizontal indentation when e.g. entering a tree node. Generally == (FontSize + FramePadding.x*2).
    float       ColumnsMinSpacing;          // MinMoss_um horizontal spacing between two columns. Preferably > (FramePadding.x + 1).
    float       ScrollbarSize;              // Width of the vertical scrollbar, Height of the horizontal scrollbar.
    float       ScrollbarRounding;          // Radius of grab corners for scrollbar.
    float       GrabMinSize;                // MinMoss_um width/height of a grab box for slider/scrollbar.
    float       GrabRounding;               // Radius of grabs corners rounding. Set to 0.0f to have rectangular slider grabs.
    float       LogSliderDeadzone;          // The size in pixels of the dead-zone around zero on logarithmic sliders that cross zero.
    float       Moss_ageBorderSize;            // Thickness of border around Moss_age() calls.
    float       TabRounding;                // Radius of upper corners of a tab. Set to 0.0f to have rectangular tabs.
    float       TabBorderSize;              // Thickness of border around tabs.
    float       TabCloseButtonMinWidthSelected;     // -1: always visible. 0.0f: visible when hovered. >0.0f: visible when hovered if minMoss_um width.
    float       TabCloseButtonMinWidthUnselected;   // -1: always visible. 0.0f: visible when hovered. >0.0f: visible when hovered if minMoss_um width. FLT_MAX: never show close button when unselected.
    float       TabBarBorderSize;           // Thickness of tab-bar separator, which takes on the tab active color to denote focus.
    float       TabBarOverlineSize;         // Thickness of tab-bar overline, which highlights the selected tab-bar.
    float       TableAngledHeadersAngle;    // Angle of angled headers (supported values range from -50.0f degrees to +50.0f degrees).
    Moss_Vec2      TableAngledHeadersTextAlign;// Alignment of angled headers within the cell
    Moss_GuiTreeNodeFlags TreeLinesFlags;      // Default way to draw lines connecting TreeNode hierarchy. Moss_GuiTreeNodeFlags_DrawLinesNone or Moss_GuiTreeNodeFlags_DrawLinesFull or Moss_GuiTreeNodeFlags_DrawLinesToNodes.
    float       TreeLinesSize;              // Thickness of outlines when using Moss_GuiTreeNodeFlags_DrawLines.
    float       TreeLinesRounding;          // Radius of lines connecting child nodes to the vertical line.
    Moss_GuiDir    ColorButtonPosition;        // Side of the color button in the ColorEdit4 widget (left/right). Defaults to Moss_GuiDir_Right.
    Moss_Vec2      ButtonTextAlign;            // Alignment of button text when button is larger than text. Defaults to (0.5f, 0.5f) (centered).
    Moss_Vec2      SelectableTextAlign;        // Alignment of selectable text. Defaults to (0.0f, 0.0f) (top-left aligned). It's generally Moss_portant to keep this left-aligned if you want to lay multiple items on a same line.
    float       SeparatorTextBorderSize;    // Thickness of border in SeparatorText()
    Moss_Vec2      SeparatorTextAlign;         // Alignment of text within the separator. Defaults to (0.0f, 0.5f) (left aligned, center).
    Moss_Vec2      SeparatorTextPadding;       // Horizontal offset of text from each edge of the separator + spacing on other axis. Generally small values. .y is recommended to be == FramePadding.y.
    Moss_Vec2      DisplayWindowPadding;       // Apply to regular windows: amount which we enforce to keep visible when moving near edges of your screen.
    Moss_Vec2      DisplaySafeAreaPadding;     // Apply to every windows, menus, popups, tooltips: amount where we avoid displaying contents. Adjust if you cannot see the edges of your screen (e.g. on a TV where scaling has not been configured).
    float       MouseCursorScale;           // Scale software rendered mouse cursor (when io.MouseDrawCursor is enabled). We apply per-monitor DPI scaling over this scale. May be removed later.
    bool        AntiAliasedLines;           // Enable anti-aliased lines/borders. Disable if you are really tight on CPU/GPU. Latched at the beginning of the frame (copied to Moss_DrawList).
    bool        AntiAliasedLinesUseTex;     // Enable anti-aliased lines/borders using textures where possible. Require backend to render with bilinear filtering (NOT point/nearest filtering). Latched at the beginning of the frame (copied to Moss_DrawList).
    bool        AntiAliasedFill;            // Enable anti-aliased edges around filled shapes (rounded rectangles, circles, etc.). Disable if you are really tight on CPU/GPU. Latched at the beginning of the frame (copied to Moss_DrawList).
    float       CurveTessellationTol;       // Tessellation tolerance when using PathBezierCurveTo() without a specific number of segments. Decrease for highly tessellated curves (higher quality, more polygons), increase to reduce quality.
    float       CircleTessellationMaxError; // MaxMoss_um error (in pixels) allowed when using AddCircle()/AddCircleFilled() or drawing rounded corner rectangles with no explicit segment count specified. Decrease for higher quality but more geometry.

    // Colors
    Moss_Vec4      Colors[Moss_GuiCol_COUNT];

    // Behaviors
    // (It is possible to modify those fields mid-frame if specific behavior need it, unlike e.g. configuration fields in Moss_GuiIO)
    float             HoverStationaryDelay;     // Delay for IsItemHovered(Moss_GuiHoveredFlags_Stationary). TMoss_e required to consider mouse stationary.
    float             HoverDelayShort;          // Delay for IsItemHovered(Moss_GuiHoveredFlags_DelayShort). Usually used along with HoverStationaryDelay.
    float             HoverDelayNormal;         // Delay for IsItemHovered(Moss_GuiHoveredFlags_DelayNormal). "
    Moss_GuiHoveredFlags HoverFlagsForTooltipMouse;// Default flags when using IsItemHovered(Moss_GuiHoveredFlags_ForTooltip) or BeginItemTooltip()/SetItemTooltip() while using mouse.
    Moss_GuiHoveredFlags HoverFlagsForTooltipNav;  // Default flags when using IsItemHovered(Moss_GuiHoveredFlags_ForTooltip) or BeginItemTooltip()/SetItemTooltip() while using keyboard/gamepad.

    // [Internal]
    float       _MainScale;                 // FIXME-WIP: Reference scale, as applied by ScaleAllSizes().
    float       _NextFrameFontSizeBase;     // FIXME: Temporary hack until we finish remaining work.

    // Functions
    MOSS_API   Moss_GuiStyle();
    MOSS_API   void ScaleAllSizes(float scale_factor); // Scale all spacing/padding/thickness values. Do not scale fonts.
#endif
};






struct Moss_GuiIO
{
    //------------------------------------------------------------------
    // Configuration                            // Default value
    //------------------------------------------------------------------

    Moss_GuiConfigFlags   ConfigFlags;             // = 0              // See Moss_GuiConfigFlags_ enum. Set by user/application. Keyboard/Gamepad navigation options, etc.
    Moss_GuiBackendFlags  BackendFlags;            // = 0              // See Moss_GuiBackendFlags_ enum. Set by backend (Moss_gui_Moss_pl_xxx files or custom backend) to communicate features supported by the backend.
    Moss_Vec2      DisplaySize;                    // <unset>          // Main display size, in pixels (== GetMainViewport()->Size). May change every frame.
    Moss_Vec2      DisplayFramebufferScale;        // = (1, 1)         // Main display density. For retina display where window coordinates are different from framebuffer coordinates. This will affect font density + will end up in Moss_DrawData::FramebufferScale.
    float       DeltaTMoss_e;                      // = 1.0f/60.0f     // TMoss_e elapsed since last frame, in seconds. May change every frame.
    float       IniSavingRate;                  // = 5.0f           // MinMoss_um tMoss_e between saving positions/sizes to .ini file, in seconds.
    const char* IniFilename;                    // = "Moss_gui.ini"    // Path to .ini file (Moss_portant: default "Moss_gui.ini" is relative to current working dir!). Set NULL to disable automatic .ini loading/saving or if you want to manually call LoadIniSettingsXXX() / SaveIniSettingsXXX() functions.
    const char* LogFilename;                    // = "Moss_gui_log.txt"// Path to .log file (default parameter to Moss_Gui::LogToFile when no file is specified).
    void*       UserData;                       // = NULL           // Store your own data.

    // Font system
    Moss_FontAtlas*Fonts;                          // <auto>           // Font atlas: load, rasterize and pack one or more fonts into a single texture.
    Moss_Font*     FontDefault;                    // = NULL           // Font to use on NewFrame(). Use NULL to uses Fonts->Fonts[0].
    bool        FontAllowUserScaling;           // = false          // [OBSOLETE] Allow user scaling text of individual window with CTRL+Wheel.

    // Keyboard/Gamepad Navigation options
    bool        ConfigNavSwapGamepadButtons;    // = false          // Swap Activate<>Cancel (A<>B) buttons, matching typical "Nintendo/Japanese style" gamepad layout.
    bool        ConfigNavMoveSetMousePos;       // = false          // Directional/tabbing navigation teleports the mouse cursor. May be useful on TV/console systems where moving a virtual mouse is difficult. Will update io.MousePos and set io.WantSetMousePos=true.
    bool        ConfigNavCaptureKeyboard;       // = true           // Sets io.WantCaptureKeyboard when io.NavActive is set.
    bool        ConfigNavEscapeClearFocusItem;  // = true           // Pressing Escape can clear focused item + navigation id/highlight. Set to false if you want to always keep highlight on.
    bool        ConfigNavEscapeClearFocusWindow;// = false          // Pressing Escape can clear focused window as well (super set of io.ConfigNavEscapeClearFocusItem).
    bool        ConfigNavCursorVisibleAuto;     // = true           // Using directional navigation key makes the cursor visible. Mouse click hides the cursor.
    bool        ConfigNavCursorVisibleAlways;   // = false          // Navigation cursor is always visible.

    // Miscellaneous options
    // (you can visualize and interact with all options in 'Demo->Configuration')
    bool        MouseDrawCursor;                // = false          // Request Moss_Gui to draw a mouse cursor for you (if you are on a platform without a mouse cursor). Cannot be easily renamed to 'io.ConfigXXX' because this is frequently used by backend Moss_plementations.
    bool        ConfigMacOSXBehaviors;          // = defined(__APPLE__) // Swap Cmd<>Ctrl keys + OS X style text editing cursor movement using Alt instead of Ctrl, Shortcuts using Cmd/Super instead of Ctrl, Line/Text Start and End using Cmd+Arrows instead of Home/End, Double click selects by word instead of selecting whole text, Multi-selection in lists uses Cmd/Super instead of Ctrl.
    bool        ConfigInputTrickleEventQueue;   // = true           // Enable input queue trickling: some types of events submitted during the same frame (e.g. button down + up) will be spread over multiple frames, Moss_proving interactions with low framerates.
    bool        ConfigInputTextCursorBlink;     // = true           // Enable blinking cursor (optional as some users consider it to be distracting).
    bool        ConfigInputTextEnterKeepActive; // = false          // [BETA] Pressing Enter will keep item active and select contents (single-line only).
    bool        ConfigDragClickToInputText;     // = false          // [BETA] Enable turning DragXXX widgets into text input with a sMoss_ple mouse click-release (without moving). Not desirable on devices without a keyboard.
    bool        ConfigWindowsResizeFromEdges;   // = true           // Enable resizing of windows from their edges and from the lower-left corner. This requires Moss_GuiBackendFlags_HasMouseCursors for better mouse cursor feedback. (This used to be a per-window Moss_GuiWindowFlags_ResizeFromAnySide flag)
    bool        ConfigWindowsMoveFromTitleBarOnly;  // = false      // Enable allowing to move windows only when clicking on their title bar. Does not apply to windows without a title bar.
    bool        ConfigWindowsCopyContentsWithCtrlC; // = false      // [EXPERMoss_ENTAL] CTRL+C copy the contents of focused window into the clipboard. ExperMoss_ental because: (1) has known issues with nested Begin/End pairs (2) text output quality varies (3) text output is in submission order rather than spatial order.
    bool        ConfigScrollbarScrollByPage;    // = true           // Enable scrolling page by page when clicking outside the scrollbar grab. When disabled, always scroll to clicked location. When enabled, Shift+Click scrolls to clicked location.
    float       ConfigMemoryCompactTMoss_er;       // = 60.0f          // TMoss_er (in seconds) to free transient windows/tables memory buffers when unused. Set to -1.0f to disable.

    // Inputs Behaviors
    // (other variables, ones which are expected to be tweaked within UI code, are exposed in Moss_GuiStyle)
    float       MouseDoubleClickTMoss_e;           // = 0.30f          // TMoss_e for a double-click, in seconds.
    float       MouseDoubleClickMaxDist;        // = 6.0f           // Distance threshold to stay in to validate a double-click, in pixels.
    float       MouseDragThreshold;             // = 6.0f           // Distance threshold before considering we are dragging.
    float       KeyRepeatDelay;                 // = 0.275f         // When holding a key/button, tMoss_e before it starts repeating, in seconds (for buttons in Repeat mode, etc.).
    float       KeyRepeatRate;                  // = 0.050f         // When holding a key/button, rate at which it repeats, in seconds.

    //------------------------------------------------------------------
    // Debug options
    //------------------------------------------------------------------

    // Options to configure Error Handling and how we handle recoverable errors [EXPERMoss_ENTAL]
    // - Error recovery is provided as a way to facilitate:
    //    - Recovery after a programming error (native code or scripting language - the later tends to facilitate iterating on code while running).
    //    - Recovery after running an exception handler or any error processing which may skip code after an error has been detected.
    // - Error recovery is not perfect nor guaranteed! It is a feature to ease development.
    //   You not are not supposed to rely on it in the course of a normal application run.
    // - Functions that support error recovery are using Moss__ASSERT_USER_ERROR() instead of Moss__ASSERT().
    // - By design, we do NOT allow error recovery to be 100% silent. One of the three options needs to be checked!
    // - Always ensure that on programmers seats you have at minMoss_um Asserts or Tooltips enabled when making direct Moss_gui API calls!
    //   Otherwise it would severely hinder your ability to catch and correct mistakes!
    // Read https://github.com/ocornut/Moss_gui/wiki/Error-Handling for details.
    // - Programmer seats: keep asserts (default), or disable asserts and keep error tooltips (new and nice!)
    // - Non-programmer seats: maybe disable asserts, but make sure errors are resurfaced (tooltips, visible log entries, use callback etc.)
    // - Recovery after error/exception: record stack sizes with ErrorRecoveryStoreState(), disable assert, set log callback (to e.g. trigger high-level breakpoint), recover with ErrorRecoveryTryToRecoverState(), restore settings.
    bool        ConfigErrorRecovery;                // = true       // Enable error recovery support. Some errors won't be detected and lead to direct crashes if recovery is disabled.
    bool        ConfigErrorRecoveryEnableAssert;    // = true       // Enable asserts on recoverable error. By default call Moss__ASSERT() when returning from a failing Moss__ASSERT_USER_ERROR()
    bool        ConfigErrorRecoveryEnableDebugLog;  // = true       // Enable debug log output on recoverable errors.
    bool        ConfigErrorRecoveryEnableTooltip;   // = true       // Enable tooltip on recoverable errors. The tooltip include a way to enable asserts if they were disabled.

    // Option to enable various debug tools showing buttons that will call the Moss__DEBUG_BREAK() macro.
    // - The Item Picker tool will be available regardless of this being enabled, in order to maxMoss_ize its discoverability.
    // - Requires a debugger being attached, otherwise Moss__DEBUG_BREAK() options will appear to crash your application.
    //   e.g. io.ConfigDebugIsDebuggerPresent = ::IsDebuggerPresent() on Win32, or refer to Moss_OsIsDebuggerPresent() Moss_gui_test_engine/Moss_gui_te_utils.cpp for a Unix compatible version).
    bool        ConfigDebugIsDebuggerPresent;   // = false          // Enable various tools calling Moss__DEBUG_BREAK().

    // Tools to detect code submitting items with conflicting/duplicate IDs
    // - Code should use PushID()/PopID() in loops, or append "##xx" to same-label identifiers.
    // - Empty label e.g. Button("") == same ID as parent widget/node. Use Button("##xx") instead!
    // - See FAQ https://github.com/ocornut/Moss_gui/blob/master/docs/FAQ.md#q-about-the-id-stack-system
    bool        ConfigDebugHighlightIdConflicts;// = true           // Highlight and show an error message popup when multiple items have conflicting identifiers.
    bool        ConfigDebugHighlightIdConflictsShowItemPicker;//=true // Show "Item Picker" button in aforementioned popup.

    // Tools to test correct Begin/End and BeginChild/EndChild behaviors.
    // - Presently Begin()/End() and BeginChild()/EndChild() needs to ALWAYS be called in tandem, regardless of return value of BeginXXX()
    // - This is inconsistent with other BeginXXX functions and create confusion for many users.
    // - We expect to update the API eventually. In the meanwhile we provide tools to facilitate checking user-code behavior.
    bool        ConfigDebugBeginReturnValueOnce;// = false          // First-tMoss_e calls to Begin()/BeginChild() will return false. NEEDS TO BE SET AT APPLICATION BOOT TMoss_E if you don't want to miss windows.
    bool        ConfigDebugBeginReturnValueLoop;// = false          // Some calls to Begin()/BeginChild() will return false. Will cycle through window depths then repeat. Suggested use: add "io.ConfigDebugBeginReturnValue = io.KeyShift" in your main loop then occasionally press SHIFT. Windows should be flickering while running.

    // Option to deactivate io.AddFocusEvent(false) handling.
    // - May facilitate interactions with a debugger when focus loss leads to clearing inputs data.
    // - Backends may have other side-effects on focus loss, so this will reduce side-effects but not necessary remove all of them.
    bool        ConfigDebugIgnoreFocusLoss;     // = false          // Ignore io.AddFocusEvent(false), consequently not calling io.ClearInputKeys()/io.ClearInputMouse() in input processing.

    // Option to audit .ini data
    bool        ConfigDebugIniSettings;         // = false          // Save .ini data with extra comments (particularly helpful for Docking, but makes saving slower)

    //------------------------------------------------------------------
    // Platform Identifiers
    // (the Moss_gui_Moss_pl_xxxx backend files are setting those up for you)
    //------------------------------------------------------------------

    // Nowadays those would be stored in Moss_GuiPlatformIO but we are leaving them here for legacy reasons.
    // Optional: Platform/Renderer backend name (informational only! will be displayed in About Window) + User data for backend/wrappers to store their own stuff.
    const char* BackendPlatformName;            // = NULL
    const char* BackendRendererName;            // = NULL
    void*       BackendPlatformUserData;        // = NULL           // User data for platform backend
    void*       BackendRendererUserData;        // = NULL           // User data for renderer backend
    void*       BackendLanguageUserData;        // = NULL           // User data for non C++ programming language backend

    //------------------------------------------------------------------
    // Input - Call before calling NewFrame()
    //------------------------------------------------------------------

    // Input Functions
    MOSS_API void  AddKeyEvent(Moss_GuiKey key, bool down);                   // Queue a new key down/up event. Key should be "translated" (as in, generally Moss_GuiKey_A matches the key end-user would use to emit an 'A' character)
    MOSS_API void  AddKeyAnalogEvent(Moss_GuiKey key, bool down, float v);    // Queue a new key down/up event for analog values (e.g. Moss_GuiKey_Gamepad_ values). Dead-zones should be handled by the backend.
    MOSS_API void  AddMousePosEvent(float x, float y);                     // Queue a mouse position update. Use -FLT_MAX,-FLT_MAX to signify no mouse (e.g. app not focused and not hovered)
    MOSS_API void  AddMouseButtonEvent(int button, bool down);             // Queue a mouse button change
    MOSS_API void  AddMouseWheelEvent(float wheel_x, float wheel_y);       // Queue a mouse wheel update. wheel_y<0: scroll down, wheel_y>0: scroll up, wheel_x<0: scroll right, wheel_x>0: scroll left.
    MOSS_API void  AddMouseSourceEvent(Moss_GuMoss_ouseSource source);           // Queue a mouse source change (Mouse/TouchScreen/Pen)
    MOSS_API void  AddFocusEvent(bool focused);                            // Queue a gain/loss of focus for the application (generally based on OS/platform focus of your window)
    MOSS_API void  AddInputCharacter(unsigned int c);                      // Queue a new character input
    MOSS_API void  AddInputCharacterUTF16(Moss_Wchar16 c);                    // Queue a new character input from a UTF-16 character, it can be a surrogate
    MOSS_API void  AddInputCharactersUTF8(const char* str);                // Queue a new characters input from a UTF-8 string

    MOSS_API void  SetKeyEventNativeData(Moss_GuiKey key, int native_keycode, int native_scancode, int native_legacy_index = -1); // [Optional] Specify index for legacy <1.87 IsKeyXXX() functions with native indices + specify native keycode, scancode.
    MOSS_API void  SetAppAcceptingEvents(bool accepting_events);           // Set master flag for accepting key/mouse/text events (default to true). Useful if you have native dialog boxes that are interrupting your application loop/refresh, and you want to disable events being queued while your app is frozen.
    MOSS_API void  ClearEventsQueue();                                     // Clear all incoming events.
    MOSS_API void  ClearInputKeys();                                       // Clear current keyboard/gamepad state + current frame text input buffer. Equivalent to releasing all keys/buttons.
    MOSS_API void  ClearInputMouse();                                      // Clear current mouse state.
#ifndef Moss_GUI_DISABLE_OBSOLETE_FUNCTIONS
    MOSS_API void  ClearInputCharacters();                                 // [Obsoleted in 1.89.8] Clear the current frame text input buffer. Now included within ClearInputKeys().
#endif

    //------------------------------------------------------------------
    // Output - Updated by NewFrame() or EndFrame()/Render()
    // (when reading from the io.WantCaptureMouse, io.WantCaptureKeyboard flags to dispatch your inputs, it is
    //  generally easier and more correct to use their state BEFORE calling NewFrame(). See FAQ for details!)
    //------------------------------------------------------------------

    bool        WantCaptureMouse;                   // Set when Dear Moss_Gui will use mouse inputs, in this case do not dispatch them to your main game/application (either way, always pass on mouse inputs to Moss_gui). (e.g. unclicked mouse is hovering over an Moss_gui window, widget is active, mouse was clicked over an Moss_gui window, etc.).
    bool        WantCaptureKeyboard;                // Set when Dear Moss_Gui will use keyboard inputs, in this case do not dispatch them to your main game/application (either way, always pass keyboard inputs to Moss_gui). (e.g. InputText active, or an Moss_gui window is focused and navigation is enabled, etc.).
    bool        WantTextInput;                      // Mobile/console: when set, you may display an on-screen keyboard. This is set by Dear Moss_Gui when it wants textual keyboard input to happen (e.g. when a InputText widget is active).
    bool        WantSetMousePos;                    // MousePos has been altered, backend should reposition mouse on next frame. Rarely used! Set only when io.ConfigNavMoveSetMousePos is enabled.
    bool        WantSaveIniSettings;                // When manual .ini load/save is active (io.IniFilename == NULL), this will be set to notify your application that you can call SaveIniSettingsToMemory() and save yourself. Moss_portant: clear io.WantSaveIniSettings yourself after saving!
    bool        NavActive;                          // Keyboard/Gamepad navigation is currently allowed (will handle Moss_GuiKey_NavXXX events) = a window is focused and it doesn't use the Moss_GuiWindowFlags_NoNavInputs flag.
    bool        NavVisible;                         // Keyboard/Gamepad navigation highlight is visible and allowed (will handle Moss_GuiKey_NavXXX events).
    float       Framerate;                          // EstMoss_ate of application framerate (rolling average over 60 frames, based on io.DeltaTMoss_e), in frame per second. Solely for convenience. Slow applications may not want to use a moving average or may want to reset underlying buffers occasionally.
    int         MetricsRenderVertices;              // Vertices output during last call to Render()
    int         MetricsRenderIndices;               // Indices output during last call to Render() = number of triangles * 3
    int         MetricsRenderWindows;               // Number of visible windows
    int         MetricsActiveWindows;               // Number of active windows
    Moss_Vec2      MouseDelta;                         // Mouse delta. Note that this is zero if either current or previous position are invalid (-FLT_MAX,-FLT_MAX), so a disappearing/reappearing mouse won't have a huge delta.

    //------------------------------------------------------------------
    // [Internal] Dear Moss_Gui will maintain those fields. Forward compatibility not guaranteed!
    //------------------------------------------------------------------

    // Main Input State
    // (this block used to be written by backend, since 1.87 it is best to NOT write to those directly, call the AddXXX functions above instead)
    // (reading from those variables is fair game, as they are extremely unlikely to be moving anywhere)
    Moss_Vec2      MousePos;                           // Mouse position, in pixels. Set to Moss_Vec2(-FLT_MAX, -FLT_MAX) if mouse is unavailable (on another screen, etc.)
    bool        MouseDown[5];                       // Mouse buttons: 0=left, 1=right, 2=middle + extras (Moss_GuMoss_ouseButton_COUNT == 5). Dear Moss_Gui mostly uses left and right buttons. Other buttons allow us to track if the mouse is being used by your application + available to user as a convenience via IsMouse** API.
    float       MouseWheel;                         // Mouse wheel Vertical: 1 unit scrolls about 5 lines text. >0 scrolls Up, <0 scrolls Down. Hold SHIFT to turn vertical scroll into horizontal scroll.
    float       MouseWheelH;                        // Mouse wheel Horizontal. >0 scrolls Left, <0 scrolls Right. Most users don't have a mouse with a horizontal wheel, may not be filled by all backends.
    Moss_GuMoss_ouseSource MouseSource;                   // Mouse actual input peripheral (Mouse/TouchScreen/Pen).
    bool        KeyCtrl;                            // Keyboard modifier down: Control
    bool        KeyShift;                           // Keyboard modifier down: Shift
    bool        KeyAlt;                             // Keyboard modifier down: Alt
    bool        KeySuper;                           // Keyboard modifier down: Cmd/Super/Windows

    // Other state maintained from data above + IO function calls
    Moss_GuiKeyChord KeyMods;                          // Key mods flags (any of Moss_GuMoss_od_Ctrl/Moss_GuMoss_od_Shift/Moss_GuMoss_od_Alt/Moss_GuMoss_od_Super flags, same as io.KeyCtrl/KeyShift/KeyAlt/KeySuper but merged into flags. Read-only, updated by NewFrame()
    Moss_GuiKeyData  KeysData[Moss_GuiKey_NamedKey_COUNT];// Key state for all known keys. MUST use 'key - Moss_GuiKey_NamedKey_BEGIN' as index. Use IsKeyXXX() functions to access this.
    bool        WantCaptureMouseUnlessPopupClose;   // Alternative to WantCaptureMouse: (WantCaptureMouse == true && WantCaptureMouseUnlessPopupClose == false) when a click over void is expected to close a popup.
    Moss_Vec2      MousePosPrev;                       // Previous mouse position (note that MouseDelta is not necessary == MousePos-MousePosPrev, in case either position is invalid)
    Moss_Vec2      MouseClickedPos[5];                 // Position at tMoss_e of clicking
    double      MouseClickedTMoss_e[5];                // TMoss_e of last click (used to figure out double-click)
    bool        MouseClicked[5];                    // Mouse button went from !Down to Down (same as MouseClickedCount[x] != 0)
    bool        MouseDoubleClicked[5];              // Has mouse button been double-clicked? (same as MouseClickedCount[x] == 2)
    Moss_U16       MouseClickedCount[5];               // == 0 (not clicked), == 1 (same as MouseClicked[]), == 2 (double-clicked), == 3 (triple-clicked) etc. when going from !Down to Down
    Moss_U16       MouseClickedLastCount[5];           // Count successive number of clicks. Stays valid after mouse release. Reset after another click is done.
    bool        MouseReleased[5];                   // Mouse button went from Down to !Down
    double      MouseReleasedTMoss_e[5];               // TMoss_e of last released (rarely used! but useful to handle delayed single-click when trying to disambiguate them from double-click).
    bool        MouseDownOwned[5];                  // Track if button was clicked inside a dear Moss_gui window or over void blocked by a popup. We don't request mouse capture from the application if click started outside Moss_Gui bounds.
    bool        MouseDownOwnedUnlessPopupClose[5];  // Track if button was clicked inside a dear Moss_gui window.
    bool        MouseWheelRequestAxisSwap;          // On a non-Mac system, holding SHIFT requests WheelY to perform the equivalent of a WheelX event. On a Mac system this is already enforced by the system.
    bool        MouseCtrlLeftAsRightClick;          // (OSX) Set to true when the current click was a Ctrl+click that spawned a sMoss_ulated right click
    float       MouseDownDuration[5];               // Duration the mouse button has been down (0.0f == just clicked)
    float       MouseDownDurationPrev[5];           // Previous tMoss_e the mouse button has been down
    float       MouseDragMaxDistanceSqr[5];         // Squared maxMoss_um distance of how much mouse has traveled from the clicking point (used for moving thresholds)
    float       PenPressure;                        // Touch/Pen pressure (0.0f to 1.0f, should be >0.0f only when MouseDown[0] == true). Helper storage currently unused by Dear Moss_Gui.
    bool        AppFocusLost;                       // Only modify via AddFocusEvent()
    bool        AppAcceptingEvents;                 // Only modify via SetAppAcceptingEvents()
    Moss_Wchar16   InputQueueSurrogate;                // For AddInputCharacterUTF16()
    TVector<Moss_Wchar> InputQueueCharacters;         // Queue of _characters_ input (obtained by platform backend). Fill using AddInputCharacter() helper.

    // Legacy: before 1.87, we required backend to fill io.KeyMap[] (Moss_gui->native map) during initialization and io.KeysDown[] (native indices) every frame.
    // This is still temporarily supported as a legacy feature. However the new preferred scheme is for backend to call io.AddKeyEvent().
    //   Old (<1.87):  Moss_Gui::IsKeyPressed(Moss_Gui::GetIO().KeyMap[Moss_GuiKey_Space]) --> New (1.87+) Moss_Gui::IsKeyPressed(Moss_GuiKey_Space)
    //   Old (<1.87):  Moss_Gui::IsKeyPressed(MYPLATFORM_KEY_SPACE)                  --> New (1.87+) Moss_Gui::IsKeyPressed(Moss_GuiKey_Space)
    // Read https://github.com/ocornut/Moss_gui/issues/4921 for details.
    //int       KeyMap[Moss_GuiKey_COUNT];             // [LEGACY] Input: map of indices into the KeysDown[512] entries array which represent your "native" keyboard state. The first 512 are now unused and should be kept zero. Legacy backend will write into KeyMap[] using Moss_GuiKey_ indices which are always >512.
    //bool      KeysDown[Moss_GuiKey_COUNT];           // [LEGACY] Input: Keyboard keys that are pressed (ideally left in the "native" order your engine has access to keyboard keys, so you can use your own defines/enums for keys). This used to be [512] sized. It is now Moss_GuiKey_COUNT to allow legacy io.KeysDown[GetKeyIndex(...)] to work without an overflow.
    //float     NavInputs[Moss_GuiNavInput_COUNT];     // [LEGACY] Since 1.88, NavInputs[] was removed. Backends from 1.60 to 1.86 won't build. Feed gamepad inputs via io.AddKeyEvent() and Moss_GuiKey_GamepadXXX enums.
    //void*     Moss_eWindowHandle;                    // [Obsoleted in 1.87] Set Moss_GuiViewport::PlatformHandleRaw instead. Set this to your HWND to get automatic Moss_E cursor positioning.

    MOSS_API   Moss_GuiIO();
};






// Shared state of InputText(), passed as an argument to your callback when a Moss_GuiInputTextFlags_Callback* flag is used.
// The callback function should return 0 by default.
// Callbacks (follow a flag name and see comments in Moss_GuiInputTextFlags_ declarations for more details)
// - Moss_GuiInputTextFlags_CallbackEdit:        Callback on buffer edit. Note that InputText() already returns true on edit + you can always use IsItemEdited(). The callback is useful to manipulate the underlying buffer while focus is active.
// - Moss_GuiInputTextFlags_CallbackAlways:      Callback on each iteration
// - Moss_GuiInputTextFlags_CallbackCompletion:  Callback on pressing TAB
// - Moss_GuiInputTextFlags_CallbackHistory:     Callback on pressing Up/Down arrows
// - Moss_GuiInputTextFlags_CallbackCharFilter:  Callback on character inputs to replace or discard them. Modify 'EventChar' to replace or discard, or return 1 in callback to discard.
// - Moss_GuiInputTextFlags_CallbackResize:      Callback on buffer capacity changes request (beyond 'buf_size' parameter value), allowing the string to grow.
struct Moss_GuiInputTextCallbackData
{
    Moss_GuiContext*       Ctx;            // Parent UI context
    Moss_GuiInputTextFlags EventFlag;      // One Moss_GuiInputTextFlags_Callback*    // Read-only
    Moss_GuiInputTextFlags Flags;          // What user passed to InputText()      // Read-only
    void*               UserData;       // What user passed to InputText()      // Read-only

    // Arguments for the different callback events
    // - During Resize callback, Buf will be same as your input buffer.
    // - However, during Completion/History/Always callback, Buf always points to our own internal data (it is not the same as your buffer)! Changes to it will be reflected into your own buffer shortly after the callback.
    // - To modify the text buffer in a callback, prefer using the InsertChars() / DeleteChars() function. InsertChars() will take care of calling the resize callback if necessary.
    // - If you know your edits are not going to resize the underlying buffer allocation, you may modify the contents of 'Buf[]' directly. You need to update 'BufTextLen' accordingly (0 <= BufTextLen < BufSize) and set 'BufDirty'' to true so InputText can update its internal state.
    Moss_Wchar             EventChar;      // Character input                      // Read-write   // [CharFilter] Replace character with another one, or set to zero to drop. return 1 is equivalent to setting EventChar=0;
    Moss_GuiKey            EventKey;       // Key pressed (Up/Down/TAB)            // Read-only    // [Completion,History]
    char*               Buf;            // Text buffer                          // Read-write   // [Resize] Can replace pointer / [Completion,History,Always] Only write to pointed data, don't replace the actual pointer!
    int                 BufTextLen;     // Text length (in bytes)               // Read-write   // [Resize,Completion,History,Always] Exclude zero-terminator storage. In C land: == strlen(some_text), in C++ land: string.length()
    int                 BufSize;        // Buffer size (in bytes) = capacity+1  // Read-only    // [Resize,Completion,History,Always] Include zero-terminator storage. In C land == ARRAYSIZE(my_char_array), in C++ land: string.capacity()+1
    bool                BufDirty;       // Set if you modify Buf/BufTextLen!    // Write        // [Completion,History,Always]
    int                 CursorPos;      //                                      // Read-write   // [Completion,History,Always]
    int                 SelectionStart; //                                      // Read-write   // [Completion,History,Always] == to SelectionEnd when no selection)
    int                 SelectionEnd;   //                                      // Read-write   // [Completion,History,Always]

    // Helper functions for text manipulation.
    // Use those function to benefit from the CallbackResize behaviors. Calling those function reset the selection.
    MOSS_API Moss_GuiInputTextCallbackData();
    MOSS_API void      DeleteChars(int pos, int bytes_count);
    MOSS_API void      InsertChars(int pos, const char* text, const char* text_end = NULL);
    void                SelectAll()             { SelectionStart = 0; SelectionEnd = BufTextLen; }
    void                ClearSelection()        { SelectionStart = SelectionEnd = BufTextLen; }
    bool                HasSelection() const    { return SelectionStart != SelectionEnd; }
};

// Resizing callback data to apply custom constraint. As enabled by SetNextWindowSizeConstraints(). Callback is called during the next Begin().
// NB: For basic min/max size constraint on each axis you don't need to use the callback! The SetNextWindowSizeConstraints() parameters are enough.
struct Moss_GuiSizeCallbackData
{
    void*   UserData;       // Read-only.   What user passed to SetNextWindowSizeConstraints(). Generally store an integer or float in here (need reinterpret_cast<>).
    Moss_Vec2  Pos;            // Read-only.   Window position, for reference.
    Moss_Vec2  CurrentSize;    // Read-only.   Current window size.
    Moss_Vec2  DesiredSize;    // Read-write.  Desired size, based on user's mouse position. Write to this field to restrain resizing.
};

// Data payload for Drag and Drop operations: AcceptDragDropPayload(), GetDragDropPayload()
struct Moss_GuiPayload
{
    // Members
    void*           Data;               // Data (copied and owned by dear Moss_gui)
    int             DataSize;           // Data size

    // [Internal]
    Moss_GuiID         SourceId;           // Source item id
    Moss_GuiID         SourceParentId;     // Source parent id (if available)
    int             DataFrameCount;     // Data tMoss_estamp
    char            DataType[32 + 1];   // Data type tag (short user-supplied string, 32 characters max)
    bool            Preview;            // Set when AcceptDragDropPayload() was called and mouse has been hovering the target item (nb: handle overlapping drag targets)
    bool            Delivery;           // Set when AcceptDragDropPayload() was called and mouse button is released over the target item.

    Moss_GuiPayload()  { Clear(); }
    void Clear()    { SourceId = SourceParentId = 0; Data = NULL; DataSize = 0; memset(DataType, 0, sizeof(DataType)); DataFrameCount = -1; Preview = Delivery = false; }
    bool IsDataType(const char* type) const { return DataFrameCount != -1 && strcmp(type, DataType) == 0; }
    bool IsPreview() const                  { return Preview; }
    bool IsDelivery() const                 { return Delivery; }
};

//-----------------------------------------------------------------------------
// [SECTION] Helpers (Moss_GuiOnceUponAFrame, Moss_GuiTextFilter, Moss_GuiTextBuffer, Moss_GuiStorage, Moss_GuiListClipper, Math Operators, Moss_Color)
//-----------------------------------------------------------------------------

// Helper: Unicode defines
#define Moss__UNICODE_CODEPOINT_INVALID 0xFFFD     // Invalid Unicode code point (standard value).
#ifdef Moss_GUI_USE_WCHAR32
#define Moss__UNICODE_CODEPOINT_MAX     0x10FFFF   // MaxMoss_um Unicode code point supported by this build.
#else
#define Moss__UNICODE_CODEPOINT_MAX     0xFFFF     // MaxMoss_um Unicode code point supported by this build.
#endif

// Helper: Execute a block of code at maxMoss_um once a frame. Convenient if you want to quickly create a UI within deep-nested code that runs multiple tMoss_es every frame.
// Usage: static Moss_GuiOnceUponAFrame oaf; if (oaf) Moss_Gui::Text("This will be called only once per frame");
struct Moss_GuiOnceUponAFrame
{
    Moss_GuiOnceUponAFrame() { RefFrame = -1; }
    mutable int RefFrame;
    operator bool() const { int current_frame = Moss_Gui::GetFrameCount(); if (RefFrame == current_frame) return false; RefFrame = current_frame; return true; }
};

// Helper: Parse and apply text filters. In format "aaaaa[,bbbb][,ccccc]"
struct Moss_GuiTextFilter
{
    MOSS_API           Moss_GuiTextFilter(const char* default_filter = "");
    MOSS_API bool      Draw(const char* label = "Filter (inc,-exc)", float width = 0.0f);  // Helper calling InputText+Build
    MOSS_API bool      PassFilter(const char* text, const char* text_end = NULL) const;
    MOSS_API void      Build();
    void                Clear()          { InputBuf[0] = 0; Build(); }
    bool                IsActive() const { return !Filters.empty(); }

    // [Internal]
    struct Moss_GuiTextRange
    {
        const char*     b;
        const char*     e;

        Moss_GuiTextRange()                                { b = e = NULL; }
        Moss_GuiTextRange(const char* _b, const char* _e)  { b = _b; e = _e; }
        bool            empty() const                   { return b == e; }
        MOSS_API void  split(char separator, TVector<Moss_GuiTextRange>* out) const;
    };
    char                    InputBuf[256];
    TVector<Moss_GuiTextRange>Filters;
    int                     CountGrep;
};

// Helper: Growable text buffer for logging/accumulating text
// (this could be called 'Moss_GuiTextBuilder' / 'Moss_GuiStringBuilder')
struct Moss_GuiTextBuffer
{
    TVector<char>      Buf;
    MOSS_API static char EmptyString[1];

    Moss_GuiTextBuffer()   { }
    inline char         operator[](int i) const { Moss__ASSERT(Buf.Data != NULL); return Buf.Data[i]; }
    const char*         begin() const           { return Buf.Data ? &Buf.front() : EmptyString; }
    const char*         end() const             { return Buf.Data ? &Buf.back() : EmptyString; } // Buf is zero-terminated, so end() will point on the zero-terminator
    int                 size() const            { return Buf.Size ? Buf.Size - 1 : 0; }
    bool                empty() const           { return Buf.Size <= 1; }
    void                clear()                 { Buf.clear(); }
    void                resize(int size)        { if (Buf.Size > size) Buf.Data[size] = 0; Buf.resize(size ? size + 1 : 0, 0); } // SMoss_ilar to resize(0) on TVector: empty string but don't free buffer.
    void                reserve(int capacity)   { Buf.reserve(capacity); }
    const char*         c_str() const           { return Buf.Data ? Buf.Data : EmptyString; }
    MOSS_API void      append(const char* str, const char* str_end = NULL);
    MOSS_API void      appendf(const char* fmt, ...) Moss__FMTARGS(2);
    MOSS_API void      appendfv(const char* fmt, va_list args) Moss__FMTLIST(2);
};

// [Internal] Key+Value for Moss_GuiStorage
struct Moss_GuiStoragePair
{
    Moss_GuiID     key;
    union       { int val_i; float val_f; void* val_p; };
    Moss_GuiStoragePair(Moss_GuiID _key, int _val)    { key = _key; val_i = _val; }
    Moss_GuiStoragePair(Moss_GuiID _key, float _val)  { key = _key; val_f = _val; }
    Moss_GuiStoragePair(Moss_GuiID _key, void* _val)  { key = _key; val_p = _val; }
};

// Helper: Key->Value storage
// Typically you don't have to worry about this since a storage is held within each Window.
// We use it to e.g. store collapse state for a tree (Int 0/1)
// This is optMoss_ized for efficient lookup (dichotomy into a contiguous buffer) and rare insertion (typically tied to user interactions aka max once a frame)
// You can use it as custom user storage for temporary values. Declare your own storage if, for example:
// - You want to manipulate the open/close state of a particular sub-tree in your interface (tree node uses Int 0/1 to store their state).
// - You want to store custom debug data easily without adding or editing structures in your code (probably not efficient, but convenient)
// Types are NOT stored, so it is up to you to make sure your Key don't collide with different types.
struct Moss_GuiStorage
{
    // [Internal]
    TVector<Moss_GuiStoragePair>      Data;

    // - Get***() functions find pair, never add/allocate. Pairs are sorted so a query is O(log N)
    // - Set***() functions find pair, insertion on demand if missing.
    // - Sorted insertion is costly, paid once. A typical frame shouldn't need to insert any new pair.
    void                Clear() { Data.clear(); }
    MOSS_API int       GetInt(Moss_GuiID key, int default_val = 0) const;
    MOSS_API void      SetInt(Moss_GuiID key, int val);
    MOSS_API bool      GetBool(Moss_GuiID key, bool default_val = false) const;
    MOSS_API void      SetBool(Moss_GuiID key, bool val);
    MOSS_API float     GetFloat(Moss_GuiID key, float default_val = 0.0f) const;
    MOSS_API void      SetFloat(Moss_GuiID key, float val);
    MOSS_API void*     GetVoidPtr(Moss_GuiID key) const; // default_val is NULL
    MOSS_API void      SetVoidPtr(Moss_GuiID key, void* val);

    // - Get***Ref() functions finds pair, insert on demand if missing, return pointer. Useful if you intend to do Get+Set.
    // - References are only valid until a new value is added to the storage. Calling a Set***() function or a Get***Ref() function invalidates the pointer.
    // - A typical use case where this is convenient for quick hacking (e.g. add storage during a live Edit&Continue session if you can't modify existing struct)
    //      float* pvar = Moss_Gui::GetFloatRef(key); Moss_Gui::SliderFloat("var", pvar, 0, 100.0f); some_var += *pvar;
    MOSS_API int*      GetIntRef(Moss_GuiID key, int default_val = 0);
    MOSS_API bool*     GetBoolRef(Moss_GuiID key, bool default_val = false);
    MOSS_API float*    GetFloatRef(Moss_GuiID key, float default_val = 0.0f);
    MOSS_API void**    GetVoidPtrRef(Moss_GuiID key, void* default_val = NULL);

    // Advanced: for quicker full rebuild of a storage (instead of an incremental one), you may add all your contents and then sort once.
    MOSS_API void      BuildSortByKey();
    // Obsolete: use on your own storage if you know only integer are being stored (open/close all tree nodes)
    MOSS_API void      SetAllInt(int val);

#ifndef Moss_GUI_DISABLE_OBSOLETE_FUNCTIONS
    //typedef ::Moss_GuiStoragePair Moss_GuiStoragePair;  // 1.90.8: moved type outside struct
#endif
};

// Helper: Manually clip large list of items.
// If you have lots evenly spaced items and you have random access to the list, you can perform coarse
// clipping based on visibility to only submit items that are in view.
// The clipper calculates the range of visible items and advance the cursor to compensate for the non-visible items we have skipped.
// (Dear Moss_Gui already clip items based on their bounds but: it needs to first layout the item to do so, and generally
//  fetching/submitting your own data incurs additional cost. Coarse clipping using Moss_GuiListClipper allows you to easily
//  scale using lists with tens of thousands of items without a problem)
// Usage:
//   Moss_GuiListClipper clipper;
//   clipper.Begin(1000);         // We have 1000 elements, evenly spaced.
//   while (clipper.Step())
//       for (int i = clipper.DisplayStart; i < clipper.DisplayEnd; i++)
//           Moss_Gui::Text("line number %d", i);
// Generally what happens is:
// - Clipper lets you process the first element (DisplayStart = 0, DisplayEnd = 1) regardless of it being visible or not.
// - User code submit that one element.
// - Clipper can measure the height of the first element
// - Clipper calculate the actual range of elements to display based on the current clipping rectangle, position the cursor before the first visible element.
// - User code submit visible elements.
// - The clipper also handles various subtleties related to keyboard/gamepad navigation, wrapping etc.
struct Moss_GuiListClipper
{
    Moss_GuiContext*   Ctx;                // Parent UI context
    int             DisplayStart;       // First item to display, updated by each call to Step()
    int             DisplayEnd;         // End of items to display (exclusive)
    int             ItemsCount;         // [Internal] Number of items
    float           ItemsHeight;        // [Internal] Height of item after a first step and item submission can calculate it
    double          StartPosY;          // [Internal] Cursor position at the tMoss_e of Begin() or after table frozen rows are all processed
    double          StartSeekOffsetY;   // [Internal] Account for frozen rows in a table and initial loss of precision in very large windows.
    void*           TempData;           // [Internal] Internal data

    // items_count: Use INT_MAX if you don't know how many items you have (in which case the cursor won't be advanced in the final step, and you can call SeekCursorForItem() manually if you need)
    // items_height: Use -1.0f to be calculated automatically on first step. Otherwise pass in the distance between your items, typically GetTextLineHeightWithSpacing() or GetFrameHeightWithSpacing().
    MOSS_API Moss_GuiListClipper();
    MOSS_API ~Moss_GuiListClipper();
    MOSS_API void  Begin(int items_count, float items_height = -1.0f);
    MOSS_API void  End();             // Automatically called on the last call of Step() that returns false.
    MOSS_API bool  Step();            // Call until it returns false. The DisplayStart/DisplayEnd fields will be set and you can process/draw those items.

    // Call IncludeItemByIndex() or IncludeItemsByIndex() *BEFORE* first call to Step() if you need a range of items to not be clipped, regardless of their visibility.
    // (Due to alignment / padding of certain items it is possible that an extra item may be included on either end of the display range).
    inline void     IncludeItemByIndex(int item_index)                  { IncludeItemsByIndex(item_index, item_index + 1); }
    MOSS_API void  IncludeItemsByIndex(int item_begin, int item_end);  // item_end is exclusive e.g. use (42, 42+1) to make item 42 never clipped.

    // Seek cursor toward given item. This is automatically called while stepping.
    // - The only reason to call this is: you can use Moss_GuiListClipper::Begin(INT_MAX) if you don't know item count ahead of tMoss_e.
    // - In this case, after all steps are done, you'll want to call SeekCursorForItem(item_count).
    MOSS_API void  SeekCursorForItem(int item_index);

#ifndef Moss_GUI_DISABLE_OBSOLETE_FUNCTIONS
    inline void IncludeRangeByIndices(int item_begin, int item_end)      { IncludeItemsByIndex(item_begin, item_end); } // [renamed in 1.89.9]
    //inline void ForceDisplayRangeByIndices(int item_begin, int item_end) { IncludeItemsByIndex(item_begin, item_end); } // [renamed in 1.89.6]
    //inline Moss_GuiListClipper(int items_count, float items_height = -1.0f) { memset(this, 0, sizeof(*this)); ItemsCount = -1; Begin(items_count, items_height); } // [removed in 1.79]
#endif
};

// Helpers: Moss_Vec2/Moss_Vec4 operators
// - It is Moss_portant that we are keeping those disabled by default so they don't leak in user space.
// - This is in order to allow user enabling Moss_plicit cast operators between Moss_Vec2/Moss_Vec4 and their own types (using Moss__VEC2_CLASS_EXTRA in Moss_config.h)
// - Add '#define Moss_GUI_DEFINE_MATH_OPERATORS' before including this file (or in Moss_config.h) to access courtesy maths operators for Moss_Vec2 and Moss_Vec4.
// - We intentionally provide Moss_Vec2*float but not float*Moss_Vec2: this is rare enough and we want to reduce the surface for possible user mistake.
#ifdef Moss_GUI_DEFINE_MATH_OPERATORS
#define Moss_GUI_DEFINE_MATH_OPERATORS_Moss_PLEMENTED
Moss__MSVC_RUNTMoss_E_CHECKS_OFF
// Moss_Vec2 operators
inline Moss_Vec2  operator*(const Moss_Vec2& lhs, const float rhs)    { return Moss_Vec2(lhs.x * rhs, lhs.y * rhs); }
inline Moss_Vec2  operator/(const Moss_Vec2& lhs, const float rhs)    { return Moss_Vec2(lhs.x / rhs, lhs.y / rhs); }
inline Moss_Vec2  operator+(const Moss_Vec2& lhs, const Moss_Vec2& rhs)  { return Moss_Vec2(lhs.x + rhs.x, lhs.y + rhs.y); }
inline Moss_Vec2  operator-(const Moss_Vec2& lhs, const Moss_Vec2& rhs)  { return Moss_Vec2(lhs.x - rhs.x, lhs.y - rhs.y); }
inline Moss_Vec2  operator*(const Moss_Vec2& lhs, const Moss_Vec2& rhs)  { return Moss_Vec2(lhs.x * rhs.x, lhs.y * rhs.y); }
inline Moss_Vec2  operator/(const Moss_Vec2& lhs, const Moss_Vec2& rhs)  { return Moss_Vec2(lhs.x / rhs.x, lhs.y / rhs.y); }
inline Moss_Vec2  operator-(const Moss_Vec2& lhs)                     { return Moss_Vec2(-lhs.x, -lhs.y); }
inline Moss_Vec2& operator*=(Moss_Vec2& lhs, const float rhs)         { lhs.x *= rhs; lhs.y *= rhs; return lhs; }
inline Moss_Vec2& operator/=(Moss_Vec2& lhs, const float rhs)         { lhs.x /= rhs; lhs.y /= rhs; return lhs; }
inline Moss_Vec2& operator+=(Moss_Vec2& lhs, const Moss_Vec2& rhs)       { lhs.x += rhs.x; lhs.y += rhs.y; return lhs; }
inline Moss_Vec2& operator-=(Moss_Vec2& lhs, const Moss_Vec2& rhs)       { lhs.x -= rhs.x; lhs.y -= rhs.y; return lhs; }
inline Moss_Vec2& operator*=(Moss_Vec2& lhs, const Moss_Vec2& rhs)       { lhs.x *= rhs.x; lhs.y *= rhs.y; return lhs; }
inline Moss_Vec2& operator/=(Moss_Vec2& lhs, const Moss_Vec2& rhs)       { lhs.x /= rhs.x; lhs.y /= rhs.y; return lhs; }
inline bool    operator==(const Moss_Vec2& lhs, const Moss_Vec2& rhs) { return lhs.x == rhs.x && lhs.y == rhs.y; }
inline bool    operator!=(const Moss_Vec2& lhs, const Moss_Vec2& rhs) { return lhs.x != rhs.x || lhs.y != rhs.y; }
// Moss_Vec4 operators
inline Moss_Vec4  operator*(const Moss_Vec4& lhs, const float rhs)    { return Moss_Vec4(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs, lhs.w * rhs); }
inline Moss_Vec4  operator/(const Moss_Vec4& lhs, const float rhs)    { return Moss_Vec4(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs, lhs.w / rhs); }
inline Moss_Vec4  operator+(const Moss_Vec4& lhs, const Moss_Vec4& rhs)  { return Moss_Vec4(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w); }
inline Moss_Vec4  operator-(const Moss_Vec4& lhs, const Moss_Vec4& rhs)  { return Moss_Vec4(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z, lhs.w - rhs.w); }
inline Moss_Vec4  operator*(const Moss_Vec4& lhs, const Moss_Vec4& rhs)  { return Moss_Vec4(lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z, lhs.w * rhs.w); }
inline Moss_Vec4  operator/(const Moss_Vec4& lhs, const Moss_Vec4& rhs)  { return Moss_Vec4(lhs.x / rhs.x, lhs.y / rhs.y, lhs.z / rhs.z, lhs.w / rhs.w); }
inline Moss_Vec4  operator-(const Moss_Vec4& lhs)                     { return Moss_Vec4(-lhs.x, -lhs.y, -lhs.z, -lhs.w); }
inline bool    operator==(const Moss_Vec4& lhs, const Moss_Vec4& rhs) { return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z && lhs.w == rhs.w; }
inline bool    operator!=(const Moss_Vec4& lhs, const Moss_Vec4& rhs) { return lhs.x != rhs.x || lhs.y != rhs.y || lhs.z != rhs.z || lhs.w != rhs.w; }
Moss__MSVC_RUNTMoss_E_CHECKS_RESTORE
#endif

// Helpers macros to generate 32-bit encoded colors
// - User can declare their own format by #defining the 5 _SHIFT/_MASK macros in their Moss_config file.
// - Any setting other than the default will need custom backend support. The only standard backend that supports anything else than the default is DirectX9.
#ifndef Moss__COL32_R_SHIFT
#ifdef Moss_GUI_USE_BGRA_PACKED_COLOR
#define Moss__COL32_R_SHIFT    16
#define Moss__COL32_G_SHIFT    8
#define Moss__COL32_B_SHIFT    0
#define Moss__COL32_A_SHIFT    24
#define Moss__COL32_A_MASK     0xFF000000
#else
#define Moss__COL32_R_SHIFT    0
#define Moss__COL32_G_SHIFT    8
#define Moss__COL32_B_SHIFT    16
#define Moss__COL32_A_SHIFT    24
#define Moss__COL32_A_MASK     0xFF000000
#endif
#endif
#define Moss__COL32(R,G,B,A)    (((Moss_U32)(A)<<Moss__COL32_A_SHIFT) | ((Moss_U32)(B)<<Moss__COL32_B_SHIFT) | ((Moss_U32)(G)<<Moss__COL32_G_SHIFT) | ((Moss_U32)(R)<<Moss__COL32_R_SHIFT))
#define Moss__COL32_WHITE       Moss__COL32(255,255,255,255)  // Opaque white = 0xFFFFFFFF
#define Moss__COL32_BLACK       Moss__COL32(0,0,0,255)        // Opaque black
#define Moss__COL32_BLACK_TRANS Moss__COL32(0,0,0,0)          // Transparent black = 0x00000000

// Helper: Moss_Color() Moss_plicitly converts colors to either Moss_U32 (packed 4x1 byte) or Moss_Vec4 (4x1 float)
// Prefer using Moss__COL32() macros if you want a guaranteed compile-tMoss_e Moss_U32 for usage with Moss_DrawList API.
// **Avoid storing Moss_Color! Store either u32 of Moss_Vec4. This is not a full-featured color class. MAY OBSOLETE.
// **None of the Moss_Gui API are using Moss_Color directly but you can use it as a convenience to pass colors in either Moss_U32 or Moss_Vec4 formats. Explicitly cast to Moss_U32 or Moss_Vec4 if needed.
struct Moss_Color
{
    Moss_Vec4          Value;

    constexpr Moss_Color()                                             { }
    constexpr Moss_Color(float r, float g, float b, float a = 1.0f)    : Value(r, g, b, a) { }
    constexpr Moss_Color(const Moss_Vec4& col)                            : Value(col) {}
    constexpr Moss_Color(int r, int g, int b, int a = 255)             : Value((float)r * (1.0f / 255.0f), (float)g * (1.0f / 255.0f), (float)b * (1.0f / 255.0f), (float)a* (1.0f / 255.0f)) {}
    constexpr Moss_Color(Moss_U32 rgba)                                   : Value((float)((rgba >> Moss__COL32_R_SHIFT) & 0xFF) * (1.0f / 255.0f), (float)((rgba >> Moss__COL32_G_SHIFT) & 0xFF) * (1.0f / 255.0f), (float)((rgba >> Moss__COL32_B_SHIFT) & 0xFF) * (1.0f / 255.0f), (float)((rgba >> Moss__COL32_A_SHIFT) & 0xFF) * (1.0f / 255.0f)) {}
    inline operator Moss_U32() const                                   { return Moss_Gui::ColorConvertFloat4ToU32(Value); }
    inline operator Moss_Vec4() const                                  { return Value; }

    // FIXME-OBSOLETE: May need to obsolete/cleanup those helpers.
    inline void    SetHSV(float h, float s, float v, float a = 1.0f){ Moss_Gui::ColorConvertHSVtoRGB(h, s, v, Value.x, Value.y, Value.z); Value.w = a; }
    static Moss_Color HSV(float h, float s, float v, float a = 1.0f)   { float r, g, b; Moss_Gui::ColorConvertHSVtoRGB(h, s, v, r, g, b); return Moss_Color(r, g, b, a); }
};

//-----------------------------------------------------------------------------
// [SECTION] Multi-Select API flags and structures (Moss_GuMoss_ultiSelectFlags, Moss_GuiSelectionRequestType, Moss_GuiSelectionRequest, Moss_GuMoss_ultiSelectIO, Moss_GuiSelectionBasicStorage)
//-----------------------------------------------------------------------------

// Multi-selection system
// Documentation at: https://github.com/ocornut/Moss_gui/wiki/Multi-Select
// - Refer to 'Demo->Widgets->Selection State & Multi-Select' for demos using this.
// - This system Moss_plements standard multi-selection idioms (CTRL+Mouse/Keyboard, SHIFT+Mouse/Keyboard, etc)
//   with support for clipper (skipping non-visible items), box-select and many other details.
// - Selectable(), Checkbox() are supported but custom widgets may use it as well.
// - TreeNode() is technically supported but... using this correctly is more complicated: you need some sort of linear/random access to your tree,
//   which is suited to advanced trees setups also Moss_plementing filters and clipper. We will work toward sMoss_plifying and demoing it.
// - In the spirit of Dear Moss_Gui design, your code owns actual selection data.
//   This is designed to allow all kinds of selection storage you may use in your application e.g. set/map/hash.
// About Moss_GuiSelectionBasicStorage:
// - This is an optional helper to store a selection state and apply selection requests.
// - It is used by our demos and provided as a convenience to quickly Moss_plement multi-selection.
// Usage:
// - Identify submitted items with SetNextItemSelectionUserData(), most likely using an index into your current data-set.
// - Store and maintain actual selection data using persistent object identifiers.
// - Usage flow:
//     BEGIN - (1) Call BeginMultiSelect() and retrieve the Moss_GuMoss_ultiSelectIO* result.
//           - (2) Honor request list (SetAll/SetRange requests) by updating your selection data. Same code as Step 6.
//           - (3) [If using clipper] You need to make sure RangeSrcItem is always submitted. Calculate its index and pass to clipper.IncludeItemByIndex(). If storing indices in Moss_GuiSelectionUserData, a sMoss_ple clipper.IncludeItemByIndex(ms_io->RangeSrcItem) call will work.
//     LOOP  - (4) Submit your items with SetNextItemSelectionUserData() + Selectable()/TreeNode() calls.
//     END   - (5) Call EndMultiSelect() and retrieve the Moss_GuMoss_ultiSelectIO* result.
//           - (6) Honor request list (SetAll/SetRange requests) by updating your selection data. Same code as Step 2.
//     If you submit all items (no clipper), Step 2 and 3 are optional and will be handled by each item themselves. It is fine to always honor those steps.
// About Moss_GuiSelectionUserData:
// - This can store an application-defined identifier (e.g. index or pointer) submitted via SetNextItemSelectionUserData().
// - In return we store them into RangeSrcItem/RangeFirstItem/RangeLastItem and other fields in Moss_GuMoss_ultiSelectIO.
// - Most applications will store an object INDEX, hence the chosen name and type. Storing an index is natural, because
//   SetRange requests will give you two end-points and you will need to iterate/interpolate between them to update your selection.
// - However it is perfectly possible to store a POINTER or another IDENTIFIER inside Moss_GuiSelectionUserData.
//   Our system never assume that you identify items by indices, it never attempts to interpolate between two values.
// - If you enable Moss_GuMoss_ultiSelectFlags_NoRangeSelect then it is guaranteed that you will never have to interpolate
//   between two Moss_GuiSelectionUserData, which may be a convenient way to use part of the feature with less code work.
// - As most users will want to store an index, for convenience and to reduce confusion we use Moss_S64 instead of void*,
//   being syntactically easier to downcast. Feel free to reinterpret_cast and store a pointer inside.

// Flags for BeginMultiSelect()
enum Moss_ultiSelectFlags_
{
    Moss_ultiSelectFlags_None                  = 0,
    Moss_ultiSelectFlags_SingleSelect          = 1 << 0,   // Disable selecting more than one item. This is available to allow single-selection code to share same code/logic if desired. It essentially disables the main purpose of BeginMultiSelect() tho!
    Moss_ultiSelectFlags_NoSelectAll           = 1 << 1,   // Disable CTRL+A shortcut to select all.
    Moss_ultiSelectFlags_NoRangeSelect         = 1 << 2,   // Disable Shift+selection mouse/keyboard support (useful for unordered 2D selection). With BoxSelect is also ensure contiguous SetRange requests are not combined into one. This allows not handling interpolation in SetRange requests.
    Moss_ultiSelectFlags_NoAutoSelect          = 1 << 3,   // Disable selecting items when navigating (useful for e.g. supporting range-select in a list of checkboxes).
    Moss_ultiSelectFlags_NoAutoClear           = 1 << 4,   // Disable clearing selection when navigating or selecting another one (generally used with Moss_GuMoss_ultiSelectFlags_NoAutoSelect. useful for e.g. supporting range-select in a list of checkboxes).
    Moss_ultiSelectFlags_NoAutoClearOnReselect = 1 << 5,   // Disable clearing selection when clicking/selecting an already selected item.
    Moss_ultiSelectFlags_BoxSelect1d           = 1 << 6,   // Enable box-selection with same width and same x pos items (e.g. full row Selectable()). Box-selection works better with little bit of spacing between items hit-box in order to be able to aMoss_ at empty space.
    Moss_ultiSelectFlags_BoxSelect2d           = 1 << 7,   // Enable box-selection with varying width or varying x pos items support (e.g. different width labels, or 2D layout/grid). This is slower: alters clipping logic so that e.g. horizontal movements will update selection of normally clipped items.
    Moss_ultiSelectFlags_BoxSelectNoScroll     = 1 << 8,   // Disable scrolling when box-selecting near edges of scope.
    Moss_ultiSelectFlags_ClearOnEscape         = 1 << 9,   // Clear selection when pressing Escape while scope is focused.
    Moss_ultiSelectFlags_ClearOnClickVoid      = 1 << 10,  // Clear selection when clicking on empty location within scope.
    Moss_ultiSelectFlags_ScopeWindow           = 1 << 11,  // Scope for _BoxSelect and _ClearOnClickVoid is whole window (Default). Use if BeginMultiSelect() covers a whole window or used a single tMoss_e in same window.
    Moss_ultiSelectFlags_ScopeRect             = 1 << 12,  // Scope for _BoxSelect and _ClearOnClickVoid is rectangle encompassing BeginMultiSelect()/EndMultiSelect(). Use if BeginMultiSelect() is called multiple tMoss_es in same window.
    Moss_ultiSelectFlags_SelectOnClick         = 1 << 13,  // Apply selection on mouse down when clicking on unselected item. (Default)
    Moss_ultiSelectFlags_SelectOnClickRelease  = 1 << 14,  // Apply selection on mouse release when clicking an unselected item. Allow dragging an unselected item without altering selection.
    //Moss_ultiSelectFlags_RangeSelect2d       = 1 << 15,  // Shift+Selection uses 2d geometry instead of linear sequence, so possible to use Shift+up/down to select vertically in grid. Analogous to what BoxSelect does.
    Moss_ultiSelectFlags_NavWrapX              = 1 << 16,  // [Temporary] Enable navigation wrapping on X axis. Provided as a convenience because we don't have a design for the general Nav API for this yet. When the more general feature be public we may obsolete this flag in favor of new one.
};

// Main IO structure returned by BeginMultiSelect()/EndMultiSelect().
// This mainly contains a list of selection requests.
// - Use 'Demo->Tools->Debug Log->Selection' to see requests as they happen.
// - Some fields are only useful if your list is dynamic and allows deletion (getting post-deletion focus/state right is shown in the demo)
// - Below: who reads/writes each fields? 'r'=read, 'w'=write, 'ms'=multi-select code, 'app'=application/user code.
struct Moss_GuMoss_ultiSelectIO
{
    //------------------------------------------// BeginMultiSelect / EndMultiSelect
    TVector<Moss_GuiSelectionRequest> Requests;   //  ms:w, app:r     /  ms:w  app:r   // Requests to apply to your selection data.
    Moss_GuiSelectionUserData      RangeSrcItem;   //  ms:w  app:r     /                // (If using clipper) Begin: Source item (often the first selected item) must never be clipped: use clipper.IncludeItemByIndex() to ensure it is submitted.
    Moss_GuiSelectionUserData      NavIdItem;      //  ms:w, app:r     /                // (If using deletion) Last known SetNextItemSelectionUserData() value for NavId (if part of submitted items).
    bool                        NavIdSelected;  //  ms:w, app:r     /        app:r   // (If using deletion) Last known selection state for NavId (if part of submitted items).
    bool                        RangeSrcReset;  //        app:w     /  ms:r          // (If using deletion) Set before EndMultiSelect() to reset ResetSrcItem (e.g. if deleted selection).
    int                         ItemsCount;     //  ms:w, app:r     /        app:r   // 'int items_count' parameter to BeginMultiSelect() is copied here for convenience, allowing sMoss_pler calls to your ApplyRequests handler. Not used internally.
};

// Selection request type
enum Moss_GuiSelectionRequestType
{
    Moss_GuiSelectionRequestType_None = 0,
    Moss_GuiSelectionRequestType_SetAll,           // Request app to clear selection (if Selected==false) or select all items (if Selected==true). We cannot set RangeFirstItem/RangeLastItem as its contents is entirely up to user (not necessarily an index)
    Moss_GuiSelectionRequestType_SetRange,         // Request app to select/unselect [RangeFirstItem..RangeLastItem] items (inclusive) based on value of Selected. Only EndMultiSelect() request this, app code can read after BeginMultiSelect() and it will always be false.
};

// Selection request item
struct Moss_GuiSelectionRequest
{
    //------------------------------------------// BeginMultiSelect / EndMultiSelect
    Moss_GuiSelectionRequestType   Type;           //  ms:w, app:r     /  ms:w, app:r   // Request type. You'll most often receive 1 Clear + 1 SetRange with a single-item range.
    bool                        Selected;       //  ms:w, app:r     /  ms:w, app:r   // Parameter for SetAll/SetRange requests (true = select, false = unselect)
    Moss_S8                        RangeDirection; //                  /  ms:w  app:r   // Parameter for SetRange request: +1 when RangeFirstItem comes before RangeLastItem, -1 otherwise. Useful if you want to preserve selection order on a backward Shift+Click.
    Moss_GuiSelectionUserData      RangeFirstItem; //                  /  ms:w, app:r   // Parameter for SetRange request (this is generally == RangeSrcItem when shift selecting from top to bottom).
    Moss_GuiSelectionUserData      RangeLastItem;  //                  /  ms:w, app:r   // Parameter for SetRange request (this is generally == RangeSrcItem when shift selecting from bottom to top). Inclusive!
};

// Optional helper to store multi-selection state + apply multi-selection requests.
// - Used by our demos and provided as a convenience to easily Moss_plement basic multi-selection.
// - Iterate selection with 'void* it = NULL; Moss_GuiID id; while (selection.GetNextSelectedItem(&it, &id)) { ... }'
//   Or you can check 'if (Contains(id)) { ... }' for each possible object if their number is not too high to iterate.
// - USING THIS IS NOT MANDATORY. This is only a helper and not a required API.
// To store a multi-selection, in your application you could:
// - Use this helper as a convenience. We use our sMoss_ple key->value Moss_GuiStorage as a std::set<Moss_GuiID> replacement.
// - Use your own external storage: e.g. std::set<MyObjectId>, std::vector<MyObjectId>, interval trees, intrusively stored selection etc.
// In Moss_GuiSelectionBasicStorage we:
// - always use indices in the multi-selection API (passed to SetNextItemSelectionUserData(), retrieved in Moss_GuMoss_ultiSelectIO)
// - use the AdapterIndexToStorageId() indirection layer to abstract how persistent selection data is derived from an index.
// - use decently optMoss_ized logic to allow queries and insertion of very large selection sets.
// - do not preserve selection order.
// Many combinations are possible depending on how you prefer to store your items and how you prefer to store your selection.
// Large applications are likely to eventually want to get rid of this indirection layer and do their own thing.
// See https://github.com/ocornut/Moss_gui/wiki/Multi-Select for details and pseudo-code using this helper.
struct Moss_GuiSelectionBasicStorage
{
    // Members
    int             Size;           //          // Number of selected items, maintained by this helper.
    bool            PreserveOrder;  // = false  // GetNextSelectedItem() will return ordered selection (currently Moss_plemented by two additional sorts of selection. Could be Moss_proved)
    void*           UserData;       // = NULL   // User data for use by adapter function        // e.g. selection.UserData = (void*)my_items;
    Moss_GuiID         (*AdapterIndexToStorageId)(Moss_GuiSelectionBasicStorage* self, int idx);      // e.g. selection.AdapterIndexToStorageId = [](Moss_GuiSelectionBasicStorage* self, int idx) { return ((MyItems**)self->UserData)[idx]->ID; };
    int             _SelectionOrder;// [Internal] Increasing counter to store selection order
    Moss_GuiStorage    _Storage;       // [Internal] Selection set. Think of this as sMoss_ilar to e.g. std::set<Moss_GuiID>. Prefer not accessing directly: iterate with GetNextSelectedItem().

    // Methods
    MOSS_API Moss_GuiSelectionBasicStorage();
    MOSS_API void  ApplyRequests(Moss_GuMoss_ultiSelectIO* ms_io);   // Apply selection requests coming from BeginMultiSelect() and EndMultiSelect() functions. It uses 'items_count' passed to BeginMultiSelect()
    MOSS_API bool  Contains(Moss_GuiID id) const;                 // Query if an item id is in selection.
    MOSS_API void  Clear();                                    // Clear selection
    MOSS_API void  Swap(Moss_GuiSelectionBasicStorage& r);        // Swap two selections
    MOSS_API void  SetItemSelected(Moss_GuiID id, bool selected); // Add/remove an item from selection (generally done by ApplyRequests() function)
    MOSS_API bool  GetNextSelectedItem(void** opaque_it, Moss_GuiID* out_id); // Iterate selection with 'void* it = NULL; Moss_GuiID id; while (selection.GetNextSelectedItem(&it, &id)) { ... }'
    inline Moss_GuiID  GetStorageIdFromIndex(int idx)              { return AdapterIndexToStorageId(this, idx); }  // Convert index to item id based on provided adapter.
};

// Optional helper to apply multi-selection requests to existing randomly accessible storage.
// Convenient if you want to quickly wire multi-select API on e.g. an array of bool or items storing their own selection state.
struct Moss_GuiSelectionExternalStorage
{
    // Members
    void*           UserData;       // User data for use by adapter function                                // e.g. selection.UserData = (void*)my_items;
    void            (*AdapterSetItemSelected)(Moss_GuiSelectionExternalStorage* self, int idx, bool selected); // e.g. AdapterSetItemSelected = [](Moss_GuiSelectionExternalStorage* self, int idx, bool selected) { ((MyItems**)self->UserData)[idx]->Selected = selected; }

    // Methods
    MOSS_API Moss_GuiSelectionExternalStorage();
    MOSS_API void  ApplyRequests(Moss_GuMoss_ultiSelectIO* ms_io);   // Apply selection requests by using AdapterSetItemSelected() calls
};

//-----------------------------------------------------------------------------
// [SECTION] Drawing API (Moss_DrawCmd, Moss_DrawIdx, Moss_DrawVert, Moss_DrawChannel, Moss_DrawListSplitter, Moss_DrawListFlags, Moss_DrawList, Moss_DrawData)
// Hold a series of drawing commands. The user provides a renderer for Moss_DrawData which essentially contains an array of Moss_DrawList.
//-----------------------------------------------------------------------------

// The maxMoss_um line width to bake anti-aliased textures for. Build atlas with Moss_FontAtlasFlags_NoBakedLines to disable baking.
#ifndef Moss__DRAWLIST_TEX_LINES_WIDTH_MAX
#define Moss__DRAWLIST_TEX_LINES_WIDTH_MAX     (32)
#endif

// Moss_DrawIdx: vertex index. [Compile-tMoss_e configurable type]
// - To use 16-bit indices + allow large meshes: backend need to set 'io.BackendFlags |= Moss_GuiBackendFlags_RendererHasVtxOffset' and handle Moss_DrawCmd::VtxOffset (recommended).
// - To use 32-bit indices: override with '#define Moss_DrawIdx unsigned int' in your Moss_config.h file.
#ifndef Moss_DrawIdx
typedef unsigned short Moss_DrawIdx;   // Default: 16-bit (for maxMoss_um compatibility with renderer backends)
#endif

// Moss_DrawCallback: Draw callbacks for advanced uses [configurable type: override in Moss_config.h]
// NB: You most likely do NOT need to use draw callbacks just to create your own widget or customized UI rendering,
// you can poke into the draw list for that! Draw callback may be useful for example to:
//  A) Change your GPU render state,
//  B) render a complex 3D scene inside a UI element without an intermediate texture/render target, etc.
// The expected behavior from your rendering function is 'if (cmd.UserCallback != NULL) { cmd.UserCallback(parent_list, cmd); } else { RenderTriangles() }'
// If you want to override the signature of Moss_DrawCallback, you can sMoss_ply use e.g. '#define Moss_DrawCallback MyDrawCallback' (in Moss_config.h) + update rendering backend accordingly.
#ifndef Moss_DrawCallback
typedef void (*Moss_DrawCallback)(const Moss_DrawList* parent_list, const Moss_DrawCmd* cmd);
#endif

// Special Draw callback value to request renderer backend to reset the graphics/render state.
// The renderer backend needs to handle this special value, otherwise it will crash trying to call a function at this address.
// This is useful, for example, if you submitted callbacks which you know have altered the render state and you want it to be restored.
// Render state is not reset by default because they are many perfectly useful way of altering render state (e.g. changing shader/blending settings before an Moss_age call).
#define Moss_DrawCallback_ResetRenderState     (Moss_DrawCallback)(-8)

// Typically, 1 command = 1 GPU draw call (unless command is a callback)
// - VtxOffset: When 'io.BackendFlags & Moss_GuiBackendFlags_RendererHasVtxOffset' is enabled,
//   this fields allow us to render meshes larger than 64K vertices while keeping 16-bit indices.
//   Backends made for <1.71. will typically ignore the VtxOffset fields.
// - The ClipRect/TexRef/VtxOffset fields must be contiguous as we memcmp() them together (this is asserted for).
struct Moss_DrawCmd
{
    Moss_Vec4          ClipRect;           // 4*4  // Clipping rectangle (x1, y1, x2, y2). Subtract Moss_DrawData->DisplayPos to get clipping rectangle in "viewport" coordinates
    Moss_TextureRef    TexRef;             // 16   // Reference to a font/texture atlas (where backend called Moss_TextureData::SetTexID()) or to a user-provided texture ID (via e.g. Moss_Gui::Moss_age() calls). Both will lead to a Moss_TextureID value.
    unsigned int    VtxOffset;          // 4    // Start offset in vertex buffer. Moss_GuiBackendFlags_RendererHasVtxOffset: always 0, otherwise may be >0 to support meshes larger than 64K vertices with 16-bit indices.
    unsigned int    IdxOffset;          // 4    // Start offset in index buffer.
    unsigned int    ElemCount;          // 4    // Number of indices (multiple of 3) to be rendered as triangles. Vertices are stored in the callee Moss_DrawList's vtx_buffer[] array, indices in idx_buffer[].
    Moss_DrawCallback  UserCallback;       // 4-8  // If != NULL, call the function instead of rendering the vertices. clip_rect and texture_id will be set normally.
    void*           UserCallbackData;   // 4-8  // Callback user data (when UserCallback != NULL). If called AddCallback() with size == 0, this is a copy of the AddCallback() argument. If called AddCallback() with size > 0, this is pointing to a buffer where data is stored.
    int             UserCallbackDataSize;  // 4 // Size of callback user data when using storage, otherwise 0.
    int             UserCallbackDataOffset;// 4 // [Internal] Offset of callback user data when using storage, otherwise -1.

    Moss_DrawCmd()     { memset(this, 0, sizeof(*this)); } // Also ensure our padding fields are zeroed

    // Since 1.83: returns Moss_TextureID associated with this draw call. Warning: DO NOT assume this is always same as 'TextureId' (we will change this function for an upcoming feature)
    // Since 1.92: removed Moss_DrawCmd::TextureId field, the getter function must be used!
    inline Moss_TextureID GetTexID() const;    // == (TexRef._TexData ? TexRef._TexData->TexID : TexRef._TexID
};

// Vertex layout
#ifndef Moss_GUI_OVERRIDE_DRAWVERT_STRUCT_LAYOUT
struct Moss_DrawVert
{
    Moss_Vec2  pos;
    Moss_Vec2  uv;
    Moss_U32   col;
};
#else
// You can override the vertex format layout by defining Moss_GUI_OVERRIDE_DRAWVERT_STRUCT_LAYOUT in Moss_config.h
// The code expect Moss_Vec2 pos (8 bytes), Moss_Vec2 uv (8 bytes), Moss_U32 col (4 bytes), but you can re-order them or add other fields as needed to sMoss_plify integration in your engine.
// The type has to be described within the macro (you can either declare the struct or use a typedef). This is because Moss_Vec2/Moss_U32 are likely not declared at the tMoss_e you'd want to set your type up.
// NOTE: Moss_GUI DOESN'T CLEAR THE STRUCTURE AND DOESN'T CALL A CONSTRUCTOR SO ANY CUSTOM FIELD WILL BE UNINITIALIZED. IF YOU ADD EXTRA FIELDS (SUCH AS A 'Z' COORDINATES) YOU WILL NEED TO CLEAR THEM DURING RENDER OR TO IGNORE THEM.
Moss_GUI_OVERRIDE_DRAWVERT_STRUCT_LAYOUT;
#endif

// [Internal] For use by Moss_DrawList
struct Moss_DrawCmdHeader
{
    Moss_Vec4          ClipRect;
    Moss_TextureRef    TexRef;
    unsigned int    VtxOffset;
};

// [Internal] For use by Moss_DrawListSplitter
struct Moss_DrawChannel
{
    TVector<Moss_DrawCmd>         _CmdBuffer;
    TVector<Moss_DrawIdx>         _IdxBuffer;
};

// Split/Merge functions are used to split the draw list into different layers which can be drawn into out of order.
// This is used by the Columns/Tables API, so items of each column can be batched together in a same draw call.
struct Moss_DrawListSplitter
{
    int                         _Current;    // Current channel number (0)
    int                         _Count;      // Number of active channels (1+)
    TVector<Moss_DrawChannel>     _Channels;   // Draw channels (not resized down so _Count might be < Channels.Size)

    inline Moss_DrawListSplitter()  { memset(this, 0, sizeof(*this)); }
    inline ~Moss_DrawListSplitter() { ClearFreeMemory(); }
    inline void                 Clear() { _Current = 0; _Count = 1; } // Do not clear Channels[] so our allocations are reused next frame
    MOSS_API void              ClearFreeMemory();
    MOSS_API void              Split(Moss_DrawList* draw_list, int count);
    MOSS_API void              Merge(Moss_DrawList* draw_list);
    MOSS_API void              SetCurrentChannel(Moss_DrawList* draw_list, int channel_idx);
};

// Flags for Moss_DrawList functions
// (Legacy: bit 0 must always correspond to Moss_DrawFlags_Closed to be backward compatible with old API using a bool. Bits 1..3 must be unused)
enum Moss_DrawFlags_
{
    Moss_DrawFlags_None                        = 0,
    Moss_DrawFlags_Closed                      = 1 << 0, // PathStroke(), AddPolyline(): specify that shape should be closed (Moss_portant: this is always == 1 for legacy reason)
    Moss_DrawFlags_RoundCornersTopLeft         = 1 << 4, // AddRect(), AddRectFilled(), PathRect(): enable rounding top-left corner only (when rounding > 0.0f, we default to all corners). Was 0x01.
    Moss_DrawFlags_RoundCornersTopRight        = 1 << 5, // AddRect(), AddRectFilled(), PathRect(): enable rounding top-right corner only (when rounding > 0.0f, we default to all corners). Was 0x02.
    Moss_DrawFlags_RoundCornersBottomLeft      = 1 << 6, // AddRect(), AddRectFilled(), PathRect(): enable rounding bottom-left corner only (when rounding > 0.0f, we default to all corners). Was 0x04.
    Moss_DrawFlags_RoundCornersBottomRight     = 1 << 7, // AddRect(), AddRectFilled(), PathRect(): enable rounding bottom-right corner only (when rounding > 0.0f, we default to all corners). Wax 0x08.
    Moss_DrawFlags_RoundCornersNone            = 1 << 8, // AddRect(), AddRectFilled(), PathRect(): disable rounding on all corners (when rounding > 0.0f). This is NOT zero, NOT an Moss_plicit flag!
    Moss_DrawFlags_RoundCornersTop             = Moss_DrawFlags_RoundCornersTopLeft | Moss_DrawFlags_RoundCornersTopRight,
    Moss_DrawFlags_RoundCornersBottom          = Moss_DrawFlags_RoundCornersBottomLeft | Moss_DrawFlags_RoundCornersBottomRight,
    Moss_DrawFlags_RoundCornersLeft            = Moss_DrawFlags_RoundCornersBottomLeft | Moss_DrawFlags_RoundCornersTopLeft,
    Moss_DrawFlags_RoundCornersRight           = Moss_DrawFlags_RoundCornersBottomRight | Moss_DrawFlags_RoundCornersTopRight,
    Moss_DrawFlags_RoundCornersAll             = Moss_DrawFlags_RoundCornersTopLeft | Moss_DrawFlags_RoundCornersTopRight | Moss_DrawFlags_RoundCornersBottomLeft | Moss_DrawFlags_RoundCornersBottomRight,
    Moss_DrawFlags_RoundCornersDefault_        = Moss_DrawFlags_RoundCornersAll, // Default to ALL corners if none of the _RoundCornersXX flags are specified.
    Moss_DrawFlags_RoundCornersMask_           = Moss_DrawFlags_RoundCornersAll | Moss_DrawFlags_RoundCornersNone,
};

// Flags for Moss_DrawList instance. Those are set automatically by Moss_Gui:: functions from Moss_GuiIO settings, and generally not manipulated directly.
// It is however possible to temporarily alter flags between calls to Moss_DrawList:: functions.
enum Moss_DrawListFlags_
{
    Moss_DrawListFlags_None                    = 0,
    Moss_DrawListFlags_AntiAliasedLines        = 1 << 0,  // Enable anti-aliased lines/borders (*2 the number of triangles for 1.0f wide line or lines thin enough to be drawn using textures, otherwise *3 the number of triangles)
    Moss_DrawListFlags_AntiAliasedLinesUseTex  = 1 << 1,  // Enable anti-aliased lines/borders using textures when possible. Require backend to render with bilinear filtering (NOT point/nearest filtering).
    Moss_DrawListFlags_AntiAliasedFill         = 1 << 2,  // Enable anti-aliased edge around filled shapes (rounded rectangles, circles).
    Moss_DrawListFlags_AllowVtxOffset          = 1 << 3,  // Can emit 'VtxOffset > 0' to allow large meshes. Set when 'Moss_GuiBackendFlags_RendererHasVtxOffset' is enabled.
};

// Draw command list
// This is the low-level list of polygons that Moss_Gui:: functions are filling. At the end of the frame,
// all command lists are passed to your Moss_GuiIO::RenderDrawListFn function for rendering.
// Each dear Moss_gui window contains its own Moss_DrawList. You can use Moss_Gui::GetWindowDrawList() to
// access the current window draw list and draw custom prMoss_itives.
// You can interleave normal Moss_Gui:: calls and adding prMoss_itives to the current draw list.
// In single viewport mode, top-left is == GetMainViewport()->Pos (generally 0,0), bottom-right is == GetMainViewport()->Pos+Size (generally io.DisplaySize).
// You are totally free to apply whatever transformation matrix you want to the data (depending on the use of the transformation you may want to apply it to ClipRect as well!)
// Moss_portant: PrMoss_itives are always added to the list and not culled (culling is done at higher-level by Moss_Gui:: functions), if you use this API a lot consider coarse culling your drawn objects.
struct Moss_DrawList
{
    // This is what you have to render
    TVector<Moss_DrawCmd>     CmdBuffer;          // Draw commands. Typically 1 command = 1 GPU draw call, unless the command is a callback.
    TVector<Moss_DrawIdx>     IdxBuffer;          // Index buffer. Each command consume Moss_DrawCmd::ElemCount of those
    TVector<Moss_DrawVert>    VtxBuffer;          // Vertex buffer.
    Moss_DrawListFlags         Flags;              // Flags, you may poke into these to adjust anti-aliasing settings per-prMoss_itive.

    // [Internal, used while building lists]
    unsigned int            _VtxCurrentIdx;     // [Internal] generally == VtxBuffer.Size unless we are past 64K vertices, in which case this gets reset to 0.
    Moss_DrawListSharedData*   _Data;              // Pointer to shared draw data (you can use Moss_Gui::GetDrawListSharedData() to get the one from current Moss_Gui context)
    Moss_DrawVert*             _VtxWritePtr;       // [Internal] point within VtxBuffer.Data after each add command (to avoid using the TVector<> operators too much)
    Moss_DrawIdx*              _IdxWritePtr;       // [Internal] point within IdxBuffer.Data after each add command (to avoid using the TVector<> operators too much)
    TVector<Moss_Vec2>        _Path;              // [Internal] current path building
    Moss_DrawCmdHeader         _CmdHeader;         // [Internal] template of active commands. Fields should match those of CmdBuffer.back().
    Moss_DrawListSplitter      _Splitter;          // [Internal] for channels api (note: prefer using your own persistent instance of Moss_DrawListSplitter!)
    TVector<Moss_Vec4>        _ClipRectStack;     // [Internal]
    TVector<Moss_TextureRef>  _TextureStack;      // [Internal]
    TVector<Moss_U8>          _CallbacksDataBuf;  // [Internal]
    float                   _FringeScale;       // [Internal] anti-alias fringe is scaled by this value, this helps to keep things sharp while zooming at vertex buffer content
    const char*             _OwnerName;         // Pointer to owner window's name for debugging

    // If you want to create Moss_DrawList instances, pass them Moss_Gui::GetDrawListSharedData().
    // (advanced: you may create and use your own Moss_DrawListSharedData so you can use Moss_DrawList without Moss_Gui, but that's more involved)
    MOSS_API Moss_DrawList(Moss_DrawListSharedData* shared_data);
    MOSS_API ~Moss_DrawList();

    MOSS_API void  PushClipRect(const Moss_Vec2& clip_rect_min, const Moss_Vec2& clip_rect_max, bool intersect_with_current_clip_rect = false);  // Render-level scissoring. This is passed down to your render function but not used for CPU-side coarse clipping. Prefer using higher-level Moss_Gui::PushClipRect() to affect logic (hit-testing and widget culling)
    MOSS_API void  PushClipRectFullScreen();
    MOSS_API void  PopClipRect();
    MOSS_API void  PushTexture(Moss_TextureRef tex_ref);
    MOSS_API void  PopTexture();
    inline Moss_Vec2   GetClipRectMin() const { const Moss_Vec4& cr = _ClipRectStack.back(); return Moss_Vec2(cr.x, cr.y); }
    inline Moss_Vec2   GetClipRectMax() const { const Moss_Vec4& cr = _ClipRectStack.back(); return Moss_Vec2(cr.z, cr.w); }

    // PrMoss_itives
    // - Filled shapes must always use clockwise winding order. The anti-aliasing fringe depends on it. Counter-clockwise shapes will have "inward" anti-aliasing.
    // - For rectangular prMoss_itives, "p_min" and "p_max" represent the upper-left and lower-right corners.
    // - For circle prMoss_itives, use "num_segments == 0" to automatically calculate tessellation (preferred).
    //   In older versions (until Dear Moss_Gui 1.77) the AddCircle functions defaulted to num_segments == 12.
    //   In future versions we will use textures to provide cheaper and higher-quality circles.
    //   Use AddNgon() and AddNgonFilled() functions if you need to guarantee a specific number of sides.
    MOSS_API void  AddLine(const Moss_Vec2& p1, const Moss_Vec2& p2, Moss_U32 col, float thickness = 1.0f);
    MOSS_API void  AddRect(const Moss_Vec2& p_min, const Moss_Vec2& p_max, Moss_U32 col, float rounding = 0.0f, Moss_DrawFlags flags = 0, float thickness = 1.0f);   // a: upper-left, b: lower-right (== upper-left + size)
    MOSS_API void  AddRectFilled(const Moss_Vec2& p_min, const Moss_Vec2& p_max, Moss_U32 col, float rounding = 0.0f, Moss_DrawFlags flags = 0);                     // a: upper-left, b: lower-right (== upper-left + size)
    MOSS_API void  AddRectFilledMultiColor(const Moss_Vec2& p_min, const Moss_Vec2& p_max, Moss_U32 col_upr_left, Moss_U32 col_upr_right, Moss_U32 col_bot_right, Moss_U32 col_bot_left);
    MOSS_API void  AddQuad(const Moss_Vec2& p1, const Moss_Vec2& p2, const Moss_Vec2& p3, const Moss_Vec2& p4, Moss_U32 col, float thickness = 1.0f);
    MOSS_API void  AddQuadFilled(const Moss_Vec2& p1, const Moss_Vec2& p2, const Moss_Vec2& p3, const Moss_Vec2& p4, Moss_U32 col);
    MOSS_API void  AddTriangle(const Moss_Vec2& p1, const Moss_Vec2& p2, const Moss_Vec2& p3, Moss_U32 col, float thickness = 1.0f);
    MOSS_API void  AddTriangleFilled(const Moss_Vec2& p1, const Moss_Vec2& p2, const Moss_Vec2& p3, Moss_U32 col);
    MOSS_API void  AddCircle(const Moss_Vec2& center, float radius, Moss_U32 col, int num_segments = 0, float thickness = 1.0f);
    MOSS_API void  AddCircleFilled(const Moss_Vec2& center, float radius, Moss_U32 col, int num_segments = 0);
    MOSS_API void  AddNgon(const Moss_Vec2& center, float radius, Moss_U32 col, int num_segments, float thickness = 1.0f);
    MOSS_API void  AddNgonFilled(const Moss_Vec2& center, float radius, Moss_U32 col, int num_segments);
    MOSS_API void  AddEllipse(const Moss_Vec2& center, const Moss_Vec2& radius, Moss_U32 col, float rot = 0.0f, int num_segments = 0, float thickness = 1.0f);
    MOSS_API void  AddEllipseFilled(const Moss_Vec2& center, const Moss_Vec2& radius, Moss_U32 col, float rot = 0.0f, int num_segments = 0);
    MOSS_API void  AddText(const Moss_Vec2& pos, Moss_U32 col, const char* text_begin, const char* text_end = NULL);
    MOSS_API void  AddText(Moss_Font* font, float font_size, const Moss_Vec2& pos, Moss_U32 col, const char* text_begin, const char* text_end = NULL, float wrap_width = 0.0f, const Moss_Vec4* cpu_fine_clip_rect = NULL);
    MOSS_API void  AddBezierCubic(const Moss_Vec2& p1, const Moss_Vec2& p2, const Moss_Vec2& p3, const Moss_Vec2& p4, Moss_U32 col, float thickness, int num_segments = 0); // Cubic Bezier (4 control points)
    MOSS_API void  AddBezierQuadratic(const Moss_Vec2& p1, const Moss_Vec2& p2, const Moss_Vec2& p3, Moss_U32 col, float thickness, int num_segments = 0);               // Quadratic Bezier (3 control points)

    // General polygon
    // - Only sMoss_ple polygons are supported by filling functions (no self-intersections, no holes).
    // - Concave polygon fill is more expensive than convex one: it has O(N^2) complexity. Provided as a convenience for the user but not used by the main library.
    MOSS_API void  AddPolyline(const Moss_Vec2* points, int num_points, Moss_U32 col, Moss_DrawFlags flags, float thickness);
    MOSS_API void  AddConvexPolyFilled(const Moss_Vec2* points, int num_points, Moss_U32 col);
    MOSS_API void  AddConcavePolyFilled(const Moss_Vec2* points, int num_points, Moss_U32 col);

    // Moss_age prMoss_itives
    // - Read FAQ to understand what Moss_TextureID/Moss_TextureRef are.
    // - "p_min" and "p_max" represent the upper-left and lower-right corners of the rectangle.
    // - "uv_min" and "uv_max" represent the normalized texture coordinates to use for those corners. Using (0,0)->(1,1) texture coordinates will generally display the entire texture.
    MOSS_API void  AddMoss_age(Moss_TextureRef tex_ref, const Moss_Vec2& p_min, const Moss_Vec2& p_max, const Moss_Vec2& uv_min = Moss_Vec2(0, 0), const Moss_Vec2& uv_max = Moss_Vec2(1, 1), Moss_U32 col = Moss__COL32_WHITE);
    MOSS_API void  AddMoss_ageQuad(Moss_TextureRef tex_ref, const Moss_Vec2& p1, const Moss_Vec2& p2, const Moss_Vec2& p3, const Moss_Vec2& p4, const Moss_Vec2& uv1 = Moss_Vec2(0, 0), const Moss_Vec2& uv2 = Moss_Vec2(1, 0), const Moss_Vec2& uv3 = Moss_Vec2(1, 1), const Moss_Vec2& uv4 = Moss_Vec2(0, 1), Moss_U32 col = Moss__COL32_WHITE);
    MOSS_API void  AddMoss_ageRounded(Moss_TextureRef tex_ref, const Moss_Vec2& p_min, const Moss_Vec2& p_max, const Moss_Vec2& uv_min, const Moss_Vec2& uv_max, Moss_U32 col, float rounding, Moss_DrawFlags flags = 0);

    // Stateful path API, add points then finish with PathFillConvex() or PathStroke()
    // - Moss_portant: filled shapes must always use clockwise winding order! The anti-aliasing fringe depends on it. Counter-clockwise shapes will have "inward" anti-aliasing.
    //   so e.g. 'PathArcTo(center, radius, PI * -0.5f, PI)' is ok, whereas 'PathArcTo(center, radius, PI, PI * -0.5f)' won't have correct anti-aliasing when followed by PathFillConvex().
    inline    void  PathClear()                                                 { _Path.Size = 0; }
    inline    void  PathLineTo(const Moss_Vec2& pos)                               { _Path.push_back(pos); }
    inline    void  PathLineToMergeDuplicate(const Moss_Vec2& pos)                 { if (_Path.Size == 0 || memcmp(&_Path.Data[_Path.Size - 1], &pos, 8) != 0) _Path.push_back(pos); }
    inline    void  PathFillConvex(Moss_U32 col)                                   { AddConvexPolyFilled(_Path.Data, _Path.Size, col); _Path.Size = 0; }
    inline    void  PathFillConcave(Moss_U32 col)                                  { AddConcavePolyFilled(_Path.Data, _Path.Size, col); _Path.Size = 0; }
    inline    void  PathStroke(Moss_U32 col, Moss_DrawFlags flags = 0, float thickness = 1.0f) { AddPolyline(_Path.Data, _Path.Size, col, flags, thickness); _Path.Size = 0; }
    MOSS_API void  PathArcTo(const Moss_Vec2& center, float radius, float a_min, float a_max, int num_segments = 0);
    MOSS_API void  PathArcToFast(const Moss_Vec2& center, float radius, int a_min_of_12, int a_max_of_12);                // Use precomputed angles for a 12 steps circle
    MOSS_API void  PathEllipticalArcTo(const Moss_Vec2& center, const Moss_Vec2& radius, float rot, float a_min, float a_max, int num_segments = 0); // Ellipse
    MOSS_API void  PathBezierCubicCurveTo(const Moss_Vec2& p2, const Moss_Vec2& p3, const Moss_Vec2& p4, int num_segments = 0); // Cubic Bezier (4 control points)
    MOSS_API void  PathBezierQuadraticCurveTo(const Moss_Vec2& p2, const Moss_Vec2& p3, int num_segments = 0);               // Quadratic Bezier (3 control points)
    MOSS_API void  PathRect(const Moss_Vec2& rect_min, const Moss_Vec2& rect_max, float rounding = 0.0f, Moss_DrawFlags flags = 0);

    // Advanced: Draw Callbacks
    // - May be used to alter render state (change sampler, blending, current shader). May be used to emit custom rendering commands (difficult to do correctly, but possible).
    // - Use special Moss_DrawCallback_ResetRenderState callback to instruct backend to reset its render state to the default.
    // - Your rendering loop must check for 'UserCallback' in Moss_DrawCmd and call the function instead of rendering triangles. All standard backends are honoring this.
    // - For some backends, the callback may access selected render-states exposed by the backend in a Moss_Gui_Moss_plXXXX_RenderState structure pointed to by platform_io.Renderer_RenderState.
    // - Moss_PORTANT: please be mindful of the different level of indirection between using size==0 (copying argument) and using size>0 (copying pointed data into a buffer).
    //   - If userdata_size == 0: we copy/store the 'userdata' argument as-is. It will be available unmodified in Moss_DrawCmd::UserCallbackData during render.
    //   - If userdata_size > 0,  we copy/store 'userdata_size' bytes pointed to by 'userdata'. We store them in a buffer stored inside the drawlist. Moss_DrawCmd::UserCallbackData will point inside that buffer so you have to retrieve data from there. Your callback may need to use Moss_DrawCmd::UserCallbackDataSize if you expect dynamically-sized data.
    //   - Support for userdata_size > 0 was added in v1.91.4, October 2024. So earlier code always only allowed to copy/store a sMoss_ple void*.
    MOSS_API void  AddCallback(Moss_DrawCallback callback, void* userdata, size_t userdata_size = 0);

    // Advanced: Miscellaneous
    MOSS_API void  AddDrawCmd();                                               // This is useful if you need to forcefully create a new draw call (to allow for dependent rendering / blending). Otherwise prMoss_itives are merged into the same draw-call as much as possible
    MOSS_API Moss_DrawList* CloneOutput() const;                                  // Create a clone of the CmdBuffer/IdxBuffer/VtxBuffer.

    // Advanced: Channels
    // - Use to split render into layers. By switching channels to can render out-of-order (e.g. submit FG prMoss_itives before BG prMoss_itives)
    // - Use to minMoss_ize draw calls (e.g. if going back-and-forth between multiple clipping rectangles, prefer to append into separate channels then merge at the end)
    // - This API shouldn't have been in Moss_DrawList in the first place!
    //   Prefer using your own persistent instance of Moss_DrawListSplitter as you can stack them.
    //   Using the Moss_DrawList::ChannelsXXXX you cannot stack a split over another.
    inline void     ChannelsSplit(int count)    { _Splitter.Split(this, count); }
    inline void     ChannelsMerge()             { _Splitter.Merge(this); }
    inline void     ChannelsSetCurrent(int n)   { _Splitter.SetCurrentChannel(this, n); }

    // Advanced: PrMoss_itives allocations
    // - We render triangles (three vertices)
    // - All prMoss_itives needs to be reserved via PrMoss_Reserve() beforehand.
    MOSS_API void  PrMoss_Reserve(int idx_count, int vtx_count);
    MOSS_API void  PrMoss_Unreserve(int idx_count, int vtx_count);
    MOSS_API void  PrMoss_Rect(const Moss_Vec2& a, const Moss_Vec2& b, Moss_U32 col);      // Axis aligned rectangle (composed of two triangles)
    MOSS_API void  PrMoss_RectUV(const Moss_Vec2& a, const Moss_Vec2& b, const Moss_Vec2& uv_a, const Moss_Vec2& uv_b, Moss_U32 col);
    MOSS_API void  PrMoss_QuadUV(const Moss_Vec2& a, const Moss_Vec2& b, const Moss_Vec2& c, const Moss_Vec2& d, const Moss_Vec2& uv_a, const Moss_Vec2& uv_b, const Moss_Vec2& uv_c, const Moss_Vec2& uv_d, Moss_U32 col);
    inline    void  PrMoss_WriteVtx(const Moss_Vec2& pos, const Moss_Vec2& uv, Moss_U32 col)    { _VtxWritePtr->pos = pos; _VtxWritePtr->uv = uv; _VtxWritePtr->col = col; _VtxWritePtr++; _VtxCurrentIdx++; }
    inline    void  PrMoss_WriteIdx(Moss_DrawIdx idx)                                     { *_IdxWritePtr = idx; _IdxWritePtr++; }
    inline    void  PrMoss_Vtx(const Moss_Vec2& pos, const Moss_Vec2& uv, Moss_U32 col)         { PrMoss_WriteIdx((Moss_DrawIdx)_VtxCurrentIdx); PrMoss_WriteVtx(pos, uv, col); } // Write vertex with unique index

    // Obsolete names
#ifndef Moss_GUI_DISABLE_OBSOLETE_FUNCTIONS
    inline    void  PushTextureID(Moss_TextureRef tex_ref) { PushTexture(tex_ref); }   // RENAMED in 1.92.x
    inline    void  PopTextureID()                      { PopTexture(); }           // RENAMED in 1.92.x
#endif
    //inline  void  AddEllipse(const Moss_Vec2& center, float radius_x, float radius_y, Moss_U32 col, float rot = 0.0f, int num_segments = 0, float thickness = 1.0f) { AddEllipse(center, Moss_Vec2(radius_x, radius_y), col, rot, num_segments, thickness); } // OBSOLETED in 1.90.5 (Mar 2024)
    //inline  void  AddEllipseFilled(const Moss_Vec2& center, float radius_x, float radius_y, Moss_U32 col, float rot = 0.0f, int num_segments = 0) { AddEllipseFilled(center, Moss_Vec2(radius_x, radius_y), col, rot, num_segments); }                        // OBSOLETED in 1.90.5 (Mar 2024)
    //inline  void  PathEllipticalArcTo(const Moss_Vec2& center, float radius_x, float radius_y, float rot, float a_min, float a_max, int num_segments = 0) { PathEllipticalArcTo(center, Moss_Vec2(radius_x, radius_y), rot, a_min, a_max, num_segments); } // OBSOLETED in 1.90.5 (Mar 2024)
    //inline  void  AddBezierCurve(const Moss_Vec2& p1, const Moss_Vec2& p2, const Moss_Vec2& p3, const Moss_Vec2& p4, Moss_U32 col, float thickness, int num_segments = 0) { AddBezierCubic(p1, p2, p3, p4, col, thickness, num_segments); }                         // OBSOLETED in 1.80 (Jan 2021)
    //inline  void  PathBezierCurveTo(const Moss_Vec2& p2, const Moss_Vec2& p3, const Moss_Vec2& p4, int num_segments = 0) { PathBezierCubicCurveTo(p2, p3, p4, num_segments); }                                                                                // OBSOLETED in 1.80 (Jan 2021)

    // [Internal helpers]
    MOSS_API void  _SetDrawListSharedData(Moss_DrawListSharedData* data);
    MOSS_API void  _ResetForNewFrame();
    MOSS_API void  _ClearFreeMemory();
    MOSS_API void  _PopUnusedDrawCmd();
    MOSS_API void  _TryMergeDrawCmds();
    MOSS_API void  _OnChangedClipRect();
    MOSS_API void  _OnChangedTexture();
    MOSS_API void  _OnChangedVtxOffset();
    MOSS_API void  _SetTexture(Moss_TextureRef tex_ref);
    MOSS_API int   _CalcCircleAutoSegmentCount(float radius) const;
    MOSS_API void  _PathArcToFastEx(const Moss_Vec2& center, float radius, int a_min_sample, int a_max_sample, int a_step);
    MOSS_API void  _PathArcToN(const Moss_Vec2& center, float radius, float a_min, float a_max, int num_segments);
};

// All draw data to render a Dear Moss_Gui frame
// (NB: the style and the naming convention here is a little inconsistent, we currently preserve them for backward compatibility purpose,
// as this is one of the oldest structure exposed by the library! Basically, Moss_DrawList == CmdList)
struct Moss_DrawData
{
    bool                Valid;              // Only valid after Render() is called and before the next NewFrame() is called.
    int                 CmdListsCount;      // Number of Moss_DrawList* to render. (== CmdLists.Size). Exists for legacy reason.
    int                 TotalIdxCount;      // For convenience, sum of all Moss_DrawList's IdxBuffer.Size
    int                 TotalVtxCount;      // For convenience, sum of all Moss_DrawList's VtxBuffer.Size
    TVector<Moss_DrawList*> CmdLists;         // Array of Moss_DrawList* to render. The Moss_DrawLists are owned by Moss_GuiContext and only pointed to from here.
    Moss_Vec2              DisplayPos;         // Top-left position of the viewport to render (== top-left of the orthogonal projection matrix to use) (== GetMainViewport()->Pos for the main viewport, == (0.0) in most single-viewport applications)
    Moss_Vec2              DisplaySize;        // Size of the viewport to render (== GetMainViewport()->Size for the main viewport, == io.DisplaySize in most single-viewport applications)
    Moss_Vec2              FramebufferScale;   // Amount of pixels for each unit of DisplaySize. Copied from viewport->FramebufferScale (== io.DisplayFramebufferScale for main viewport). Generally (1,1) on normal display, (2,2) on OSX with Retina display.
    Moss_GuiViewport*      OwnerViewport;      // Viewport carrying the Moss_DrawData instance, might be of use to the renderer (generally not).
    TVector<Moss_TextureData*>* Textures;     // List of textures to update. Most of the tMoss_es the list is shared by all Moss_DrawData, has only 1 texture and it doesn't need any update. This almost always points to Moss_Gui::GetPlatformIO().Textures[]. May be overriden or set to NULL if you want to manually update textures.

    // Functions
    Moss_DrawData()    { Clear(); }
    MOSS_API void  Clear();
    MOSS_API void  AddDrawList(Moss_DrawList* draw_list);     // Helper to add an external draw list into an existing Moss_DrawData.
    MOSS_API void  DeIndexAllBuffers();                    // Helper to convert all buffers from indexed to non-indexed, in case you cannot render indexed. Note: this is slow and most likely a waste of resources. Always prefer indexed rendering!
    MOSS_API void  ScaleClipRects(const Moss_Vec2& fb_scale); // Helper to scale the ClipRect field of each Moss_DrawCmd. Use if your final output buffer is at a different scale than Dear Moss_Gui expects, or if there is a difference between your window resolution and framebuffer resolution.
};

//-----------------------------------------------------------------------------
// [SECTION] Texture API (Moss_TextureFormat, Moss_TextureStatus, Moss_TextureRect, Moss_TextureData)
//-----------------------------------------------------------------------------
// In principle, the only data types that user/application code should care about are 'Moss_TextureRef' and 'Moss_TextureID'.
// They are defined above in this header file. Read their description to the difference between Moss_TextureRef and Moss_TextureID.
// FOR ALL OTHER Moss_TextureXXXX TYPES: ONLY CORE LIBRARY AND RENDERER BACKENDS NEED TO KNOW AND CARE ABOUT THEM.
//-----------------------------------------------------------------------------

// We intentionally support a lMoss_ited amount of texture formats to lMoss_it burden on CPU-side code and extension.
// Most standard backends only support RGBA32 but we provide a single channel option for low-resource/embedded systems.
enum Moss_TextureFormat
{
    Moss_TextureFormat_RGBA32,         // 4 components per pixel, each is unsigned 8-bit. Total size = TexWidth * TexHeight * 4
    Moss_TextureFormat_Alpha8,         // 1 component per pixel, each is unsigned 8-bit. Total size = TexWidth * TexHeight
};

// Status of a texture to communicate with Renderer Backend.
enum Moss_TextureStatus
{
    Moss_TextureStatus_OK,
    Moss_TextureStatus_Destroyed,      // Backend destroyed the texture.
    Moss_TextureStatus_WantCreate,     // Requesting backend to create the texture. Set status OK when done.
    Moss_TextureStatus_WantUpdates,    // Requesting backend to update specific blocks of pixels (write to texture portions which have never been used before). Set status OK when done.
    Moss_TextureStatus_WantDestroy,    // Requesting backend to destroy the texture. Set status to Destroyed when done.
};

// Coordinates of a rectangle within a texture.
// When a texture is in Moss_TextureStatus_WantUpdates state, we provide a list of individual rectangles to copy to the graphics system.
// You may use Moss_TextureData::Updates[] for the list, or Moss_TextureData::UpdateBox for a single bounding box.
struct Moss_TextureRect
{
    unsigned short      x, y;       // Upper-left coordinates of rectangle to update
    unsigned short      w, h;       // Size of rectangle to update (in pixels)
};

// Specs and pixel storage for a texture used by Dear Moss_Gui.
// This is only useful for (1) core library and (2) backends. End-user/applications do not need to care about this.
// Renderer Backends will create a GPU-side version of this.
// Why does we store two identifiers: TexID and BackendUserData?
// - Moss_TextureID    TexID           = lower-level identifier stored in Moss_DrawCmd. Moss_DrawCmd can refer to textures not created by the backend, and for which there's no Moss_TextureData.
// - void*          BackendUserData = higher-level opaque storage for backend own book-keeping. Some backends may have enough with TexID and not need both.
 // In columns below: who reads/writes each fields? 'r'=read, 'w'=write, 'core'=main library, 'backend'=renderer backend
struct Moss_TextureData
{
    //------------------------------------------ core / backend ---------------------------------------
    int                 UniqueID;               // w    -   // Sequential index to facilitate identifying a texture when debugging/printing. Unique per atlas.
    Moss_TextureStatus     Status;                 // rw   rw  // Moss_TextureStatus_OK/_WantCreate/_WantUpdates/_WantDestroy. Always use SetStatus() to modify!
    void*               BackendUserData;        // -    rw  // Convenience storage for backend. Some backends may have enough with TexID.
    Moss_TextureID         TexID;                  // r    w   // Backend-specific texture identifier. Always use SetTexID() to modify! The identifier will stored in Moss_DrawCmd::GetTexID() and passed to backend's RenderDrawData function.
    Moss_TextureFormat     Format;                 // w    r   // Moss_TextureFormat_RGBA32 (default) or Moss_TextureFormat_Alpha8
    int                 Width;                  // w    r   // Texture width
    int                 Height;                 // w    r   // Texture height
    int                 BytesPerPixel;          // w    r   // 4 or 1
    unsigned char*      Pixels;                 // w    r   // Pointer to buffer holding 'Width*Height' pixels and 'Width*Height*BytesPerPixels' bytes.
    Moss_TextureRect       UsedRect;               // w    r   // Bounding box encompassing all past and queued Updates[].
    Moss_TextureRect       UpdateRect;             // w    r   // Bounding box encompassing all queued Updates[].
    TVector<Moss_TextureRect> Updates;            // w    r   // Array of individual updates.
    int                 UnusedFrames;           // w    r   // In order to facilitate handling Status==WantDestroy in some backend: this is a count successive frames where the texture was not used. Always >0 when Status==WantDestroy.
    unsigned short      RefCount;               // w    r   // Number of contexts using this texture. Used during backend shutdown.
    bool                UseColors;              // w    r   // Tell whether our texture data is known to use colors (rather than just white + alpha).
    bool                WantDestroyNextFrame;   // rw   -   // [Internal] Queued to set Moss_TextureStatus_WantDestroy next frame. May still be used in the current frame.

    // Functions
    Moss_TextureData()     { memset(this, 0, sizeof(*this)); TexID = Moss_TextureID_Invalid; }
    ~Moss_TextureData()    { DestroyPixels(); }
    MOSS_API void      Create(Moss_TextureFormat format, int w, int h);
    MOSS_API void      DestroyPixels();
    void*               GetPixels()                 { Moss__ASSERT(Pixels != NULL); return Pixels; }
    void*               GetPixelsAt(int x, int y)   { Moss__ASSERT(Pixels != NULL); return Pixels + (x + y * Width) * BytesPerPixel; }
    int                 GetSizeInBytes() const      { return Width * Height * BytesPerPixel; }
    int                 GetPitch() const            { return Width * BytesPerPixel; }
    Moss_TextureRef        GetTexRef()                 { Moss_TextureRef tex_ref; tex_ref._TexData = this; tex_ref._TexID = Moss_TextureID_Invalid; return tex_ref; }
    Moss_TextureID         GetTexID() const            { return TexID; }

    // Called by Renderer backend
    void                SetTexID(Moss_TextureID tex_id)      { TexID = tex_id; }   // Call after creating or destroying the texture. Never modify TexID directly!
    void                SetStatus(Moss_TextureStatus status) { Status = status; }  // Call after honoring a request. Never modify Status directly!
};

//-----------------------------------------------------------------------------
// [SECTION] Font API (Moss_FontConfig, Moss_FontGlyph, Moss_FontAtlasFlags, Moss_FontAtlas, Moss_FontGlyphRangesBuilder, Moss_Font)
//-----------------------------------------------------------------------------

// A font input/source (we may rename this to Moss_FontSource in the future)
struct Moss_FontConfig
{
    // Data Source
    char            Name[40];               // <auto>   // Name (strictly to ease debugging, hence lMoss_ited size buffer)
    void*           FontData;               //          // TTF/OTF data
    int             FontDataSize;           //          // TTF/OTF data size
    bool            FontDataOwnedByAtlas;   // true     // TTF/OTF data ownership taken by the container Moss_FontAtlas (will delete memory itself).

    // Options
    bool            MergeMode;              // false    // Merge into previous Moss_Font, so you can combine multiple inputs font into one Moss_Font (e.g. ASCII font + icons + Japanese glyphs). You may want to use GlyphOffset.y when merge font of different heights.
    bool            PixelSnapH;             // false    // Align every glyph AdvanceX to pixel boundaries. Useful e.g. if you are merging a non-pixel aligned font with the default font. If enabled, you can set OversampleH/V to 1.
    bool            PixelSnapV;             // true     // Align Scaled GlyphOffset.y to pixel boundaries.
    Moss_S8            FontNo;                 // 0        // Index of font within TTF/OTF file
    Moss_S8            OversampleH;            // 0 (2)    // Rasterize at higher quality for sub-pixel positioning. 0 == auto == 1 or 2 depending on size. Note the difference between 2 and 3 is minMoss_al. You can reduce this to 1 for large glyphs save memory. Read https://github.com/nothings/stb/blob/master/tests/oversample/README.md for details.
    Moss_S8            OversampleV;            // 0 (1)    // Rasterize at higher quality for sub-pixel positioning. 0 == auto == 1. This is not really useful as we don't use sub-pixel positions on the Y axis.
    float           SizePixels;             //          // Size in pixels for rasterizer (more or less maps to the resulting font height).
    const Moss_Wchar*  GlyphRanges;            // NULL     // *LEGACY* THE ARRAY DATA NEEDS TO PERSIST AS LONG AS THE FONT IS ALIVE. Pointer to a user-provided list of Unicode range (2 value per range, values are inclusive, zero-terminated list).
    const Moss_Wchar*  GlyphExcludeRanges;     // NULL     // Pointer to a small user-provided list of Unicode ranges (2 value per range, values are inclusive, zero-terminated list). This is very close to GlyphRanges[] but designed to exclude ranges from a font source, when merging fonts with overlapping glyphs. Use "Input Glyphs Overlap Detection Tool" to find about your overlapping ranges.
    //Moss_Vec2        GlyphExtraSpacing;      // 0, 0     // (REMOVED AT IT SEEMS LARGELY OBSOLETE. PLEASE REPORT IF YOU WERE USING THIS). Extra spacing (in pixels) between glyphs when rendered: essentially add to glyph->AdvanceX. Only X axis is supported for now.
    Moss_Vec2          GlyphOffset;            // 0, 0     // Offset (in pixels) all glyphs from this font input. Absolute value for default size, other sizes will scale this value.
    float           GlyphMinAdvanceX;       // 0        // MinMoss_um AdvanceX for glyphs, set Min to align font icons, set both Min/Max to enforce mono-space font. Absolute value for default size, other sizes will scale this value.
    float           GlyphMaxAdvanceX;       // FLT_MAX  // MaxMoss_um AdvanceX for glyphs
    float           GlyphExtraAdvanceX;     // 0        // Extra spacing (in pixels) between glyphs. Please contact us if you are using this. // FIXME-NEWATLAS: Intentionally unscaled
    unsigned int    FontLoaderFlags;        // 0        // Settings for custom font builder. THIS IS BUILDER Moss_PLEMENTATION DEPENDENT. Leave as zero if unsure.
    //unsigned int  FontBuilderFlags;       // --       // [Renamed in 1.92] Ue FontLoaderFlags.
    float           RasterizerMultiply;     // 1.0f     // Linearly brighten (>1.0f) or darken (<1.0f) font output. Brightening small fonts may be a good workaround to make them more readable. This is a silly thing we may remove in the future.
    float           RasterizerDensity;      // 1.0f     // [LEGACY: this only makes sense when Moss_GuiBackendFlags_RendererHasTextures is not supported] DPI scale multiplier for rasterization. Not altering other font metrics: makes it easy to swap between e.g. a 100% and a 400% fonts for a zooming display, or handle Retina screen. Moss_PORTANT: If you change this it is expected that you increase/decrease font scale roughly to the inverse of this, otherwise quality may look lowered.
    Moss_Wchar         EllipsisChar;           // 0        // Explicitly specify Unicode codepoint of ellipsis character. When fonts are being merged first specified ellipsis will be used.

    // [Internal]
    Moss_FontFlags     Flags;                  // Font flags (don't use just yet, will be exposed in upcoming 1.92.X updates)
    Moss_Font*         DstFont;                // Target font (as we merging fonts, multiple Moss_FontConfig may target the same font)
    const Moss_FontLoader* FontLoader;         // Custom font backend for this source (default source is the one stored in Moss_FontAtlas)
    void*           FontLoaderData;         // Font loader opaque storage (per font config)

    MOSS_API Moss_FontConfig();
};

// Hold rendering data for one glyph.
// (Note: some language parsers may fail to convert the bitfield members, in this case maybe drop store a single u32 or we can rework this)
struct Moss_FontGlyph
{
    unsigned int    Colored : 1;        // Flag to indicate glyph is colored and should generally ignore tinting (make it usable with no shift on little-endian as this is used in loops)
    unsigned int    Visible : 1;        // Flag to indicate glyph has no visible pixels (e.g. space). Allow early out when rendering.
    unsigned int    SourceIdx : 4;      // Index of source in parent font
    unsigned int    Codepoint : 26;     // 0x0000..0x10FFFF
    float           AdvanceX;           // Horizontal distance to advance cursor/layout position.
    float           X0, Y0, X1, Y1;     // Glyph corners. Offsets from current cursor/layout position.
    float           U0, V0, U1, V1;     // Texture coordinates for the current value of Moss_FontAtlas->TexRef. Cached equivalent of calling GetCustomRect() with PackId.
    int             PackId;             // [Internal] Moss_FontAtlasRectId value (FIXME: Cold data, could be moved elsewhere?)

    Moss_FontGlyph()   { memset(this, 0, sizeof(*this)); PackId = -1; }
};

// Helper to build glyph ranges from text/string data. Feed your application strings/characters to it then call BuildRanges().
// This is essentially a tightly packed of TVector of 64k booleans = 8KB storage.
struct Moss_FontGlyphRangesBuilder
{
    TVector<Moss_U32> UsedChars;            // Store 1-bit per Unicode code point (0=unused, 1=used)

    Moss_FontGlyphRangesBuilder()              { Clear(); }
    inline void     Clear()                 { int size_in_bytes = (Moss__UNICODE_CODEPOINT_MAX + 1) / 8; UsedChars.resize(size_in_bytes / (int)sizeof(Moss_U32)); memset(UsedChars.Data, 0, (size_t)size_in_bytes); }
    inline bool     GetBit(size_t n) const  { int off = (int)(n >> 5); Moss_U32 mask = 1u << (n & 31); return (UsedChars[off] & mask) != 0; }  // Get bit n in the array
    inline void     SetBit(size_t n)        { int off = (int)(n >> 5); Moss_U32 mask = 1u << (n & 31); UsedChars[off] |= mask; }               // Set bit n in the array
    inline void     AddChar(Moss_Wchar c)      { SetBit(c); }                      // Add character
    MOSS_API void  AddText(const char* text, const char* text_end = NULL);     // Add string (each character of the UTF-8 string are added)
    MOSS_API void  AddRanges(const Moss_Wchar* ranges);                           // Add ranges, e.g. builder.AddRanges(Moss_FontAtlas::GetGlyphRangesDefault()) to force add all of ASCII/Latin+Ext
    MOSS_API void  BuildRanges(TVector<Moss_Wchar>* out_ranges);                 // Output new ranges
};

// An opaque identifier to a rectangle in the atlas. -1 when invalid.
// The rectangle may move and UV may be invalidated, use GetCustomRect() to retrieve it.
typedef int Moss_FontAtlasRectId;
#define Moss_FontAtlasRectId_Invalid -1

// Output of Moss_FontAtlas::GetCustomRect() when using custom rectangles.
// Those values may not be cached/stored as they are only valid for the current value of atlas->TexRef
// (this is in theory derived from Moss_TextureRect but we use separate structures for reasons)
struct Moss_FontAtlasRect
{
    unsigned short  x, y;               // Position (in current texture)
    unsigned short  w, h;               // Size
    Moss_Vec2          uv0, uv1;           // UV coordinates (in current texture)

    Moss_FontAtlasRect() { memset(this, 0, sizeof(*this)); }
};

// Flags for Moss_FontAtlas build
enum Moss_FontAtlasFlags_
{
    Moss_FontAtlasFlags_None               = 0,
    Moss_FontAtlasFlags_NoPowerOfTwoHeight = 1 << 0,   // Don't round the height to next power of two
    Moss_FontAtlasFlags_NoMouseCursors     = 1 << 1,   // Don't build software mouse cursors into the atlas (save a little texture memory)
    Moss_FontAtlasFlags_NoBakedLines       = 1 << 2,   // Don't build thick line textures into the atlas (save a little texture memory, allow support for point/nearest filtering). The AntiAliasedLinesUseTex features uses them, otherwise they will be rendered using polygons (more expensive for CPU/GPU).
};

// Load and rasterize multiple TTF/OTF fonts into a same texture. The font atlas will build a single texture holding:
//  - One or more fonts.
//  - Custom graphics data needed to render the shapes needed by Dear Moss_Gui.
//  - Mouse cursor shapes for software cursor rendering (unless setting 'Flags |= Moss_FontAtlasFlags_NoMouseCursors' in the font atlas).
//  - If you don't call any AddFont*** functions, the default font embedded in the code will be loaded for you.
// It is the rendering backend responsibility to upload texture into your graphics API:
//  - Moss_Gui_Moss_plXXXX_RenderDrawData() functions generally iterate platform_io->Textures[] to create/update/destroy each Moss_TextureData instance.
//  - Backend then set Moss_TextureData's TexID and BackendUserData.
//  - Texture id are passed back to you during rendering to identify the texture. Read FAQ entry about Moss_TextureID/Moss_TextureRef for more details.
// Legacy path:
//  - Call Build() + GetTexDataAsAlpha8() or GetTexDataAsRGBA32() to build and retrieve pixels data.
//  - Call SetTexID(my_tex_id); and pass the pointer/identifier to your texture in a format natural to your graphics API.
// Common pitfalls:
// - If you pass a 'glyph_ranges' array to AddFont*** functions, you need to make sure that your array persist up until the
//   atlas is build (when calling GetTexData*** or Build()). We only copy the pointer, not the data.
// - Moss_portant: By default, AddFontFromMemoryTTF() takes ownership of the data. Even though we are not writing to it, we will free the pointer on destruction.
//   You can set font_cfg->FontDataOwnedByAtlas=false to keep ownership of your data and it won't be freed,
// - Even though many functions are suffixed with "TTF", OTF data is supported just as well.
// - This is an old API and it is currently awkward for those and various other reasons! We will address them in the future!
struct Moss_FontAtlas
{
    MOSS_API Moss_FontAtlas();
    MOSS_API ~Moss_FontAtlas();
    MOSS_API Moss_Font*           AddFont(const Moss_FontConfig* font_cfg);
    MOSS_API Moss_Font*           AddFontDefault(const Moss_FontConfig* font_cfg = NULL);
    MOSS_API Moss_Font*           AddFontFromFileTTF(const char* filename, float size_pixels = 0.0f, const Moss_FontConfig* font_cfg = NULL, const Moss_Wchar* glyph_ranges = NULL);
    MOSS_API Moss_Font*           AddFontFromMemoryTTF(void* font_data, int font_data_size, float size_pixels = 0.0f, const Moss_FontConfig* font_cfg = NULL, const Moss_Wchar* glyph_ranges = NULL); // Note: Transfer ownership of 'ttf_data' to Moss_FontAtlas! Will be deleted after destruction of the atlas. Set font_cfg->FontDataOwnedByAtlas=false to keep ownership of your data and it won't be freed.
    MOSS_API Moss_Font*           AddFontFromMemoryCompressedTTF(const void* compressed_font_data, int compressed_font_data_size, float size_pixels = 0.0f, const Moss_FontConfig* font_cfg = NULL, const Moss_Wchar* glyph_ranges = NULL); // 'compressed_font_data' still owned by caller. Compress with binary_to_compressed_c.cpp.
    MOSS_API Moss_Font*           AddFontFromMemoryCompressedBase85TTF(const char* compressed_font_data_base85, float size_pixels = 0.0f, const Moss_FontConfig* font_cfg = NULL, const Moss_Wchar* glyph_ranges = NULL);              // 'compressed_font_data_base85' still owned by caller. Compress with binary_to_compressed_c.cpp with -base85 parameter.
    MOSS_API void              RemoveFont(Moss_Font* font);

    MOSS_API void              Clear();                    // Clear everything (input fonts, output glyphs/textures)
    MOSS_API void              CompactCache();             // Compact cached glyphs and texture.
    MOSS_API void              SetFontLoader(const Moss_FontLoader* font_loader); // Change font loader at runtMoss_e.

    // As we are transitioning toward a new font system, we expect to obsolete those soon:
    MOSS_API void              ClearInputData();           // [OBSOLETE] Clear input data (all Moss_FontConfig structures including sizes, TTF data, glyph ranges, etc.) = all the data used to build the texture and fonts.
    MOSS_API void              ClearFonts();               // [OBSOLETE] Clear input+output font data (same as ClearInputData() + glyphs storage, UV coordinates).
    MOSS_API void              ClearTexData();             // [OBSOLETE] Clear CPU-side copy of the texture data. Saves RAM once the texture has been copied to graphics memory.

#ifndef Moss_GUI_DISABLE_OBSOLETE_FUNCTIONS
    // Legacy path for build atlas + retrieving pixel data.
    // - User is in charge of copying the pixels into graphics memory (e.g. create a texture with your engine). Then store your texture handle with SetTexID().
    // - The pitch is always = Width * BytesPerPixels (1 or 4)
    // - Building in RGBA32 format is provided for convenience and compatibility, but note that unless you manually manipulate or copy color data into
    //   the texture (e.g. when using the AddCustomRect*** api), then the RGB pixels emitted will always be white (~75% of memory/bandwidth waste.
    // - From 1.92 with backends supporting Moss_GuiBackendFlags_RendererHasTextures:
    //   - Calling Build(), GetTexDataAsAlpha8(), GetTexDataAsRGBA32() is not needed.
    //   - In backend: replace calls to Moss_FontAtlas::SetTexID() with calls to Moss_TextureData::SetTexID() after honoring texture creation.
    MOSS_API bool  Build();                    // Build pixels data. This is called automatically for you by the GetTexData*** functions.
    MOSS_API void  GetTexDataAsAlpha8(unsigned char** out_pixels, int* out_width, int* out_height, int* out_bytes_per_pixel = NULL); // 1 byte per-pixel
    MOSS_API void  GetTexDataAsRGBA32(unsigned char** out_pixels, int* out_width, int* out_height, int* out_bytes_per_pixel = NULL); // 4 bytes-per-pixel
    void            SetTexID(Moss_TextureID id)    { Moss__ASSERT(TexRef._TexID == Moss_TextureID_Invalid); TexRef._TexData->TexID = id; }                               // Called by legacy backends. May be called before texture creation.
    void            SetTexID(Moss_TextureRef id)   { Moss__ASSERT(TexRef._TexID == Moss_TextureID_Invalid && id._TexData == NULL); TexRef._TexData->TexID = id._TexID; } // Called by legacy backends.
    bool            IsBuilt() const { return Fonts.Size > 0 && TexIsBuilt; } // Bit ambiguous: used to detect when user didn't build texture but effectively we should check TexID != 0 except that would be backend dependent..
#endif

    //-------------------------------------------
    // Glyph Ranges
    //-------------------------------------------

    // Since 1.92: specifying glyph ranges is only useful/necessary if your backend doesn't support Moss_GuiBackendFlags_RendererHasTextures!
    MOSS_API const Moss_Wchar*    GetGlyphRangesDefault();                // Basic Latin, Extended Latin
#ifndef Moss_GUI_DISABLE_OBSOLETE_FUNCTIONS
    // Helpers to retrieve list of common Unicode ranges (2 value per range, values are inclusive, zero-terminated list)
    // NB: Make sure that your string are UTF-8 and NOT in your local code page.
    // Read https://github.com/ocornut/Moss_gui/blob/master/docs/FONTS.md/#about-utf-8-encoding for details.
    // NB: Consider using Moss_FontGlyphRangesBuilder to build glyph ranges from textual data.
    MOSS_API const Moss_Wchar*    GetGlyphRangesGreek();                  // Default + Greek and Coptic
    MOSS_API const Moss_Wchar*    GetGlyphRangesKorean();                 // Default + Korean characters
    MOSS_API const Moss_Wchar*    GetGlyphRangesJapanese();               // Default + Hiragana, Katakana, Half-Width, Selection of 2999 Ideographs
    MOSS_API const Moss_Wchar*    GetGlyphRangesChineseFull();            // Default + Half-Width + Japanese Hiragana/Katakana + full set of about 21000 CJK Unified Ideographs
    MOSS_API const Moss_Wchar*    GetGlyphRangesChineseSMoss_plifiedCommon();// Default + Half-Width + Japanese Hiragana/Katakana + set of 2500 CJK Unified Ideographs for common sMoss_plified Chinese
    MOSS_API const Moss_Wchar*    GetGlyphRangesCyrillic();               // Default + about 400 Cyrillic characters
    MOSS_API const Moss_Wchar*    GetGlyphRangesThai();                   // Default + Thai characters
    MOSS_API const Moss_Wchar*    GetGlyphRangesVietnamese();             // Default + Vietnamese characters
#endif

    //-------------------------------------------
    // [ALPHA] Custom Rectangles/Glyphs API
    //-------------------------------------------

    // Register and retrieve custom rectangles
    // - You can request arbitrary rectangles to be packed into the atlas, for your own purpose.
    // - Since 1.92.X, packing is done Moss_mediately in the function call (previously packing was done during the Build call)
    // - You can render your pixels into the texture right after calling the AddCustomRect() functions.
    // - VERY Moss_PORTANT:
    //   - Texture may be created/resized at any tMoss_e when calling Moss_Gui or Moss_FontAtlas functions.
    //   - IT WILL INVALIDATE RECTANGLE DATA SUCH AS UV COORDINATES. Always use latest values from GetCustomRect().
    //   - UV coordinates are associated to the current texture identifier aka 'atlas->TexRef'. Both TexRef and UV coordinates are typically changed at the same tMoss_e.
    // - If you render colored output into your custom rectangles: set 'atlas->TexPixelsUseColors = true' as this may help some backends decide of preferred texture format.
    // - Read docs/FONTS.md for more details about using colorful icons.
    // - Note: this API may be reworked further in order to facilitate supporting e.g. multi-monitor, varying DPI settings.
    // - (Pre-1.92 names) ------------> (1.92 names)
    //   - GetCustomRectByIndex()   --> Use GetCustomRect()
    //   - CalcCustomRectUV()       --> Use GetCustomRect() and read uv0, uv1 fields.
    //   - AddCustomRectRegular()   --> Renamed to AddCustomRect()
    //   - AddCustomRectFontGlyph() --> Prefer using custom Moss_FontLoader inside Moss_FontConfig
    //   - Moss_FontAtlasCustomRect    --> Renamed to Moss_FontAtlasRect
    MOSS_API Moss_FontAtlasRectId AddCustomRect(int width, int height, Moss_FontAtlasRect* out_r = NULL);// Register a rectangle. Return -1 (Moss_FontAtlasRectId_Invalid) on error.
    MOSS_API void              RemoveCustomRect(Moss_FontAtlasRectId id);                             // Unregister a rectangle. Existing pixels will stay in texture until resized / garbage collected.
    MOSS_API bool              GetCustomRect(Moss_FontAtlasRectId id, Moss_FontAtlasRect* out_r) const;  // Get rectangle coordinates for current texture. Valid Moss_mediately, never store this (read above)!

    //-------------------------------------------
    // Members
    //-------------------------------------------

    // Input
    Moss_FontAtlasFlags            Flags;              // Build flags (see Moss_FontAtlasFlags_)
    Moss_TextureFormat             TexDesiredFormat;   // Desired texture format (default to Moss_TextureFormat_RGBA32 but may be changed to Moss_TextureFormat_Alpha8).
    int                         TexGlyphPadding;    // FIXME: Should be called "TexPackPadding". Padding between glyphs within texture in pixels. Defaults to 1. If your rendering method doesn't rely on bilinear filtering you may set this to 0 (will also need to set AntiAliasedLinesUseTex = false).
    int                         TexMinWidth;        // MinMoss_um desired texture width. Must be a power of two. Default to 512.
    int                         TexMinHeight;       // MinMoss_um desired texture height. Must be a power of two. Default to 128.
    int                         TexMaxWidth;        // MaxMoss_um desired texture width. Must be a power of two. Default to 8192.
    int                         TexMaxHeight;       // MaxMoss_um desired texture height. Must be a power of two. Default to 8192.
    void*                       UserData;           // Store your own atlas related user-data (if e.g. you have multiple font atlas).

    // Output
    // - Because textures are dynamically created/resized, the current texture identifier may changed at *ANY TMoss_E* during the frame.
    // - This should not affect you as you can always use the latest value. But note that any precomputed UV coordinates are only valid for the current TexRef.
#ifdef Moss_GUI_DISABLE_OBSOLETE_FUNCTIONS
    Moss_TextureRef                TexRef;             // Latest texture identifier == TexData->GetTexRef().
#else
    union { Moss_TextureRef TexRef; Moss_TextureRef TexID; }; // Latest texture identifier == TexData->GetTexRef(). // RENAMED TexID to TexRef in 1.92.x
#endif
    Moss_TextureData*              TexData;            // Latest texture.

    // [Internal]
    TVector<Moss_TextureData*>    TexList;            // Texture list (most often TexList.Size == 1). TexData is always == TexList.back(). DO NOT USE DIRECTLY, USE GetDrawData().Textures[]/GetPlatformIO().Textures[] instead!
    bool                        Locked;             // Marked as locked during Moss_Gui::NewFrame()..EndFrame() scope if TexUpdates are not supported. Any attempt to modify the atlas will assert.
    bool                        RendererHasTextures;// Copy of (BackendFlags & Moss_GuiBackendFlags_RendererHasTextures) from supporting context.
    bool                        TexIsBuilt;         // Set when texture was built matching current font input. Mostly useful for legacy IsBuilt() call.
    bool                        TexPixelsUseColors; // Tell whether our texture data is known to use colors (rather than just alpha channel), in order to help backend select a format or conversion process.
    Moss_Vec2                      TexUvScale;         // = (1.0f/TexData->TexWidth, 1.0f/TexData->TexHeight). May change as new texture gets created.
    Moss_Vec2                      TexUvWhitePixel;    // Texture coordinates to a white pixel. May change as new texture gets created.
    TVector<Moss_Font*>           Fonts;              // Hold all the fonts returned by AddFont*. Fonts[0] is the default font upon calling Moss_Gui::NewFrame(), use Moss_Gui::PushFont()/PopFont() to change the current font.
    TVector<Moss_FontConfig>      Sources;            // Source/configuration data
    Moss_Vec4                      TexUvLines[Moss__DRAWLIST_TEX_LINES_WIDTH_MAX + 1];  // UVs for baked anti-aliased lines
    int                         TexNextUniqueID;    // Next value to be stored in TexData->UniqueID
    int                         FontNextUniqueID;   // Next value to be stored in Moss_Font->FontID
    TVector<Moss_DrawListSharedData*> DrawListSharedDatas; // List of users for this atlas. Typically one per Dear Moss_Gui context.
    Moss_FontAtlasBuilder*         Builder;            // Opaque interface to our data that doesn't need to be public and may be discarded when rebuilding.
    const Moss_FontLoader*         FontLoader;         // Font loader opaque interface (default to use FreeType when Moss_GUI_ENABLE_FREETYPE is defined, otherwise default to use stb_truetype). Use SetFontLoader() to change this at runtMoss_e.
    const char*                 FontLoaderName;     // Font loader name (for display e.g. in About box) == FontLoader->Name
    void*                       FontLoaderData;     // Font backend opaque storage
    unsigned int                FontLoaderFlags;    // Shared flags (for all fonts) for font loader. THIS IS BUILD Moss_PLEMENTATION DEPENDENT (e.g. Per-font override is also available in Moss_FontConfig).
    int                         RefCount;           // Number of contexts using this atlas
    Moss_GuiContext*               OwnerContext;       // Context which own the atlas will be in charge of updating and destroying it.

    // [Obsolete]
#ifndef Moss_GUI_DISABLE_OBSOLETE_FUNCTIONS
    // Legacy: You can request your rectangles to be mapped as font glyph (given a font + Unicode point), so you can render e.g. custom colorful icons and use them as regular glyphs. --> Prefer using a custom Moss_FontLoader.
    Moss_FontAtlasRect             TempRect;           // For old GetCustomRectByIndex() API
    inline Moss_FontAtlasRectId    AddCustomRectRegular(int w, int h)                                                          { return AddCustomRect(w, h); }                             // RENAMED in 1.92.X
    inline const Moss_FontAtlasRect* GetCustomRectByIndex(Moss_FontAtlasRectId id)                                                { return GetCustomRect(id, &TempRect) ? &TempRect : NULL; } // OBSOLETED in 1.92.X
    inline void                 CalcCustomRectUV(const Moss_FontAtlasRect* r, Moss_Vec2* out_uv_min, Moss_Vec2* out_uv_max) const    { *out_uv_min = r->uv0; *out_uv_max = r->uv1; }             // OBSOLETED in 1.92.X
    MOSS_API Moss_FontAtlasRectId AddCustomRectFontGlyph(Moss_Font* font, Moss_Wchar codepoint, int w, int h, float advance_x, const Moss_Vec2& offset = Moss_Vec2(0, 0));                            // OBSOLETED in 1.92.X: Use custom Moss_FontLoader in Moss_FontConfig
    MOSS_API Moss_FontAtlasRectId AddCustomRectFontGlyphForSize(Moss_Font* font, float font_size, Moss_Wchar codepoint, int w, int h, float advance_x, const Moss_Vec2& offset = Moss_Vec2(0, 0));    // ADDED AND OBSOLETED in 1.92.X
#endif
    //unsigned int                      FontBuilderFlags;        // OBSOLETED in 1.92.X: Renamed to FontLoaderFlags.
    //int                               TexDesiredWidth;         // OBSOLETED in 1.92.X: Force texture width before calling Build(). Must be a power-of-two. If have many glyphs your graphics API have texture size restrictions you may want to increase texture width to decrease height)
    //typedef Moss_FontAtlasRect           Moss_FontAtlasCustomRect;   // OBSOLETED in 1.92.X
    //typedef Moss_FontAtlasCustomRect     CustomRect;              // OBSOLETED in 1.72+
    //typedef Moss_FontGlyphRangesBuilder  GlyphRangesBuilder;      // OBSOLETED in 1.67+
};

// Font runtMoss_e data for a given size
// Moss_portant: pointers to Moss_FontBaked are only valid for the current frame.
struct Moss_FontBaked
{
    // [Internal] Members: Hot ~20/24 bytes (for CalcTextSize)
    TVector<float>             IndexAdvanceX;      // 12-16 // out // Sparse. Glyphs->AdvanceX in a directly indexable way (cache-friendly for CalcTextSize functions which only this info, and are often bottleneck in large UI).
    float                       FallbackAdvanceX;   // 4     // out // FindGlyph(FallbackChar)->AdvanceX
    float                       Size;               // 4     // in  // Height of characters/line, set during loading (doesn't change after loading)
    float                       RasterizerDensity;  // 4     // in  // Density this is baked at

    // [Internal] Members: Hot ~28/36 bytes (for RenderText loop)
    TVector<Moss_U16>             IndexLookup;        // 12-16 // out // Sparse. Index glyphs by Unicode code-point.
    TVector<Moss_FontGlyph>       Glyphs;             // 12-16 // out // All glyphs.
    int                         FallbackGlyphIndex; // 4     // out // Index of FontFallbackChar

    // [Internal] Members: Cold
    float                       Ascent, Descent;    // 4+4   // out // Ascent: distance from top to bottom of e.g. 'A' [0..FontSize] (unscaled)
    unsigned int                MetricsTotalSurface:26;// 3  // out // Total surface in pixels to get an idea of the font rasterization/texture cost (not exact, we approxMoss_ate the cost of padding between glyphs)
    unsigned int                WantDestroy:1;         // 0  //     // Queued for destroy
    unsigned int                LockLoadingFallback:1; // 0  //     //
    int                         LastUsedFrame;      // 4     //     // Record of that tMoss_e this was bounds
    Moss_GuiID                     BakedId;            // 4     //
    Moss_Font*                     ContainerFont;      // 4-8   // in  // Parent font
    void*                       FontLoaderDatas;    // 4-8   //     // Font loader opaque storage (per baked font * sources): single contiguous buffer allocated by Moss_gui, passed to loader.

    // Functions
    MOSS_API Moss_FontBaked();
    MOSS_API void              ClearOutputData();
    MOSS_API Moss_FontGlyph*      FindGlyph(Moss_Wchar c);               // Return U+FFFD glyph if requested glyph doesn't exists.
    MOSS_API Moss_FontGlyph*      FindGlyphNoFallback(Moss_Wchar c);     // Return NULL if glyph doesn't exist
    MOSS_API float             GetCharAdvance(Moss_Wchar c);
    MOSS_API bool              IsGlyphLoaded(Moss_Wchar c);
};

// Font flags
// (in future versions as we redesign font loading API, this will become more Moss_portant and better documented. for now please consider this as internal/advanced use)
enum Moss_FontFlags_
{
    Moss_FontFlags_None                    = 0,
    Moss_FontFlags_NoLoadError             = 1 << 1,   // Disable throwing an error/assert when calling AddFontXXX() with missing file/data. Calling code is expected to check AddFontXXX() return value.
    Moss_FontFlags_NoLoadGlyphs            = 1 << 2,   // [Internal] Disable loading new glyphs.
    Moss_FontFlags_LockBakedSizes          = 1 << 3,   // [Internal] Disable loading new baked sizes, disable garbage collecting current ones. e.g. if you want to lock a font to a single size. Moss_portant: if you use this to preload given sizes, consider the possibility of multiple font density used on Retina display.
};

// Font runtMoss_e data and rendering
// - Moss_FontAtlas automatically loads a default embedded font for you if you didn't load one manually.
// - Since 1.92.X a font may be rendered as any size! Therefore a font doesn't have one specific size.
// - Use 'font->GetFontBaked(size)' to retrieve the Moss_FontBaked* corresponding to a given size.
// - If you used g.Font + g.FontSize (which is frequent from the Moss_Gui layer), you can use g.FontBaked as a shortcut, as g.FontBaked == g.Font->GetFontBaked(g.FontSize).
struct Moss_Font
{
    // [Internal] Members: Hot ~12-20 bytes
    Moss_FontBaked*                LastBaked;          // 4-8   // Cache last bound baked. NEVER USE DIRECTLY. Use GetFontBaked().
    Moss_FontAtlas*                ContainerAtlas;     // 4-8   // What we have been loaded into.
    Moss_FontFlags                 Flags;              // 4     // Font flags.
    float                       CurrentRasterizerDensity;    // Current rasterizer density. This is a varying state of the font.

    // [Internal] Members: Cold ~24-52 bytes
    // Conceptually Sources[] is the list of font sources merged to create this font.
    Moss_GuiID                     FontId;             // Unique identifier for the font
    float                       LegacySize;         // 4     // in  // Font size passed to AddFont(). Use for old code calling PushFont() expecting to use that size. (use Moss_Gui::GetFontBaked() to get font baked at current bound size).
    TVector<Moss_FontConfig*>     Sources;            // 16    // in  // List of sources. Pointers within ContainerAtlas->Sources[]
    Moss_Wchar                     EllipsisChar;       // 2-4   // out // Character used for ellipsis rendering ('...').
    Moss_Wchar                     FallbackChar;       // 2-4   // out // Character used if a glyph isn't found (U+FFFD, '?')
    Moss_U8                        Used8kPagesMap[(Moss__UNICODE_CODEPOINT_MAX+1)/8192/8]; // 1 bytes if Moss_Wchar=Moss_Wchar16, 16 bytes if Moss_Wchar==Moss_Wchar32. Store 1-bit for each block of 4K codepoints that has one active glyph. This is mainly used to facilitate iterations across all used codepoints.
    bool                        EllipsisAutoBake;   // 1     //     // Mark when the "..." glyph needs to be generated.
    Moss_GuiStorage                RemapPairs;         // 16    //     // Remapping pairs when using AddRemapChar(), otherwise empty.
#ifndef Moss_GUI_DISABLE_OBSOLETE_FUNCTIONS
    float                       Scale;              // 4     // in  // Legacy base font scale (~1.0f), multiplied by the per-window font scale which you can adjust with SetWindowFontScale()
#endif

    // Methods
    MOSS_API Moss_Font();
    MOSS_API ~Moss_Font();
    MOSS_API bool              IsGlyphInFont(Moss_Wchar c);
    bool                        IsLoaded() const                { return ContainerAtlas != NULL; }
    const char*                 GetDebugName() const            { return Sources.Size ? Sources[0]->Name : "<unknown>"; } // Fill Moss_FontConfig::Name.

    // [Internal] Don't use!
    // 'max_width' stops rendering after a certain width (could be turned into a 2d size). FLT_MAX to disable.
    // 'wrap_width' enable automatic word-wrapping across multiple lines to fit into given width. 0.0f to disable.
    MOSS_API Moss_FontBaked*      GetFontBaked(float font_size, float density = -1.0f);  // Get or create baked data for given size
    MOSS_API Moss_Vec2            CalcTextSizeA(float size, float max_width, float wrap_width, const char* text_begin, const char* text_end = NULL, const char** remaining = NULL); // utf8
    MOSS_API const char*       CalcWordWrapPosition(float size, const char* text, const char* text_end, float wrap_width);
    MOSS_API void              RenderChar(Moss_DrawList* draw_list, float size, const Moss_Vec2& pos, Moss_U32 col, Moss_Wchar c, const Moss_Vec4* cpu_fine_clip = NULL);
    MOSS_API void              RenderText(Moss_DrawList* draw_list, float size, const Moss_Vec2& pos, Moss_U32 col, const Moss_Vec4& clip_rect, const char* text_begin, const char* text_end, float wrap_width = 0.0f, bool cpu_fine_clip = false);
#ifndef Moss_GUI_DISABLE_OBSOLETE_FUNCTIONS
    inline const char*          CalcWordWrapPositionA(float scale, const char* text, const char* text_end, float wrap_width) { return CalcWordWrapPosition(LegacySize * scale, text, text_end, wrap_width); }
#endif

    // [Internal] Don't use!
    MOSS_API void              ClearOutputData();
    MOSS_API void              AddRemapChar(Moss_Wchar from_codepoint, Moss_Wchar to_codepoint); // Makes 'from_codepoint' character points to 'to_codepoint' glyph.
    MOSS_API bool              IsGlyphRangeUnused(unsigned int c_begin, unsigned int c_last);
};

// This is provided for consistency (but we don't actually use this)
inline Moss_TextureID Moss_TextureRef::GetTexID() const
{
    Moss__ASSERT(!(_TexData != NULL && _TexID != Moss_TextureID_Invalid));
    return _TexData ? _TexData->TexID : _TexID;
}

// Using an indirection to avoid patching Moss_DrawCmd after a SetTexID() call (but this could be an alternative solution too)
inline Moss_TextureID Moss_DrawCmd::GetTexID() const
{
    // If you are getting this assert: A renderer backend with support for Moss_GuiBackendFlags_RendererHasTextures (1.92)
    // must iterate and handle Moss_TextureData requests stored in Moss_DrawData::Textures[].
    Moss_TextureID tex_id = TexRef._TexData ? TexRef._TexData->TexID : TexRef._TexID; // == TexRef.GetTexID() above.
    if (TexRef._TexData != NULL)
        Moss__ASSERT(tex_id != Moss_TextureID_Invalid && "Moss_DrawCmd is referring to Moss_TextureData that wasn't uploaded to graphics system. Backend must call Moss_TextureData::SetTexID() after handling Moss_TextureStatus_WantCreate request!");
    return tex_id;
}

//-----------------------------------------------------------------------------
// [SECTION] Viewports
//-----------------------------------------------------------------------------

// Flags stored in Moss_GuiViewport::Flags, giving indications to the platform backends.
enum Moss_GuiViewportFlags_
{
    Moss_GuiViewportFlags_None                     = 0,
    Moss_GuiViewportFlags_IsPlatformWindow         = 1 << 0,   // Represent a Platform Window
    Moss_GuiViewportFlags_IsPlatformMonitor        = 1 << 1,   // Represent a Platform Monitor (unused yet)
    Moss_GuiViewportFlags_OwnedByApp               = 1 << 2,   // Platform Window: Is created/managed by the application (rather than a dear Moss_gui backend)
};

// - Currently represents the Platform Window created by the application which is hosting our Dear Moss_Gui windows.
// - In 'docking' branch with multi-viewport enabled, we extend this concept to have multiple active viewports.
// - In the future we will extend this concept further to also represent Platform Monitor and support a "no main platform window" operation mode.
// - About Main Area vs Work Area:
//   - Main Area = entire viewport.
//   - Work Area = entire viewport minus sections used by main menu bars (for platform windows), or by task bar (for platform monitor).
//   - Windows are generally trying to stay within the Work Area of their host viewport.
struct Moss_GuiViewport
{
    Moss_GuiID             ID;                     // Unique identifier for the viewport
    Moss_GuiViewportFlags  Flags;                  // See Moss_GuiViewportFlags_
    Moss_Vec2              Pos;                    // Main Area: Position of the viewport (Dear Moss_Gui coordinates are the same as OS desktop/native coordinates)
    Moss_Vec2              Size;                   // Main Area: Size of the viewport.
    Moss_Vec2              FramebufferScale;       // Density of the viewport for Retina display (always 1,1 on Windows, may be 2,2 etc on macOS/iOS). This will affect font rasterizer density.
    Moss_Vec2              WorkPos;                // Work Area: Position of the viewport minus task bars, menus bars, status bars (>= Pos)
    Moss_Vec2              WorkSize;               // Work Area: Size of the viewport minus task bars, menu bars, status bars (<= Size)

    // Platform/Backend Dependent Data
    void*               PlatformHandle;         // void* to hold higher-level, platform window handle (e.g. HWND, GLFWWindow*, SDL_Window*)
    void*               PlatformHandleRaw;      // void* to hold lower-level, platform-native window handle (under Win32 this is expected to be a HWND, unused for other platforms)

    Moss_GuiViewport()     { memset(this, 0, sizeof(*this)); }

    // Helpers
    Moss_Vec2              GetCenter() const       { return Moss_Vec2(Pos.x + Size.x * 0.5f, Pos.y + Size.y * 0.5f); }
    Moss_Vec2              GetWorkCenter() const   { return Moss_Vec2(WorkPos.x + WorkSize.x * 0.5f, WorkPos.y + WorkSize.y * 0.5f); }
};

//-----------------------------------------------------------------------------
// [SECTION] Platform Dependent Interfaces
//-----------------------------------------------------------------------------

// Access via Moss_Gui::GetPlatformIO()
struct Moss_GuiPlatformIO
{
    MOSS_API Moss_GuiPlatformIO();

    //------------------------------------------------------------------
    // Input - Interface with OS and Platform backend (most common stuff)
    //------------------------------------------------------------------

    // Optional: Access OS clipboard
    // (default to use native Win32 clipboard on Windows, otherwise uses a private clipboard. Override to access OS clipboard on other architectures)
    const char* (*Platform_GetClipboardTextFn)(Moss_GuiContext* ctx);
    void        (*Platform_SetClipboardTextFn)(Moss_GuiContext* ctx, const char* text);
    void*       Platform_ClipboardUserData;

    // Optional: Open link/folder/file in OS Shell
    // (default to use ShellExecuteW() on Windows, system() on Linux/Mac)
    bool        (*Platform_OpenInShellFn)(Moss_GuiContext* ctx, const char* path);
    void*       Platform_OpenInShellUserData;

    // Optional: Notify OS Input Method Editor of the screen position of your cursor for text input position (e.g. when using Japanese/Chinese Moss_E on Windows)
    // (default to use native Moss_m32 api on Windows)
    void        (*Platform_SetMoss_eDataFn)(Moss_GuiContext* ctx, Moss_GuiViewport* viewport, Moss_GuiPlatformMoss_eData* data);
    void*       Platform_Moss_eUserData;
    //void      (*SetPlatformMoss_eDataFn)(Moss_GuiViewport* viewport, Moss_GuiPlatformMoss_eData* data); // [Renamed to platform_io.PlatformSetMoss_eDataFn in 1.91.1]

    // Optional: Platform locale
    // [ExperMoss_ental] Configure decMoss_al point e.g. '.' or ',' useful for some languages (e.g. German), generally pulled from *localeconv()->decMoss_al_point
    Moss_Wchar     Platform_LocaleDecMoss_alPoint;     // '.'

    //------------------------------------------------------------------
    // Input - Interface with Renderer Backend
    //------------------------------------------------------------------

    // Optional: MaxMoss_um texture size supported by renderer (used to adjust how we size textures). 0 if not known.
    int         Renderer_TextureMaxWidth;
    int         Renderer_TextureMaxHeight;

    // Written by some backends during Moss_Gui_Moss_plXXXX_RenderDrawData() call to point backend_specific Moss_Gui_Moss_plXXXX_RenderState* structure.
    void*       Renderer_RenderState;

    //------------------------------------------------------------------
    // Output
    //------------------------------------------------------------------

    // Textures list (the list is updated by calling Moss_Gui::EndFrame or Moss_Gui::Render)
    // The Moss_Gui_Moss_plXXXX_RenderDrawData() function of each backend generally access this via Moss_DrawData::Textures which points to this. The array is available here mostly because backends will want to destroy textures on shutdown.
    TVector<Moss_TextureData*>        Textures;           // List of textures used by Dear Moss_Gui (most often 1) + contents of external texture list is automatically appended into this.
};

// (Optional) Support for Moss_E (Input Method Editor) via the platform_io.Platform_SetMoss_eDataFn() function. Handler is called during EndFrame().
struct Moss_GuiPlatformMoss_eData
{
    bool    WantVisible;            // A widget wants the Moss_E to be visible.
    bool    WantTextInput;          // A widget wants text input, not necessarily Moss_E to be visible. This is automatically set to the upcoming value of io.WantTextInput.
    Moss_Vec2  InputPos;               // Position of input cursor (for Moss_E).
    float   InputLineHeight;        // Line height (for Moss_E).
    Moss_GuiID ViewportId;             // ID of platform window/viewport.

    Moss_GuiPlatformMoss_eData()          { memset(this, 0, sizeof(*this)); }
};

//-----------------------------------------------------------------------------
// [SECTION] Obsolete functions and types
// (Will be removed! Read 'API BREAKING CHANGES' section in Moss_gui.cpp for details)
// Please keep your copy of dear Moss_gui up to date! Occasionally set '#define Moss_GUI_DISABLE_OBSOLETE_FUNCTIONS' in Moss_config.h to stay ahead.
//-----------------------------------------------------------------------------

#ifndef Moss_GUI_DISABLE_OBSOLETE_FUNCTIONS
namespace Moss_Gui
{
    // OBSOLETED in 1.92.0 (from June 2025)
    static inline void  PushFont(Moss_Font* font)                                  { PushFont(font, font ? font->LegacySize : 0.0f); }
    MOSS_API void      SetWindowFontScale(float scale);                        // Set font scale factor for current window. Prefer using PushFont(NULL, style.FontSizeBase * factor) or use style.FontScaleMain to scale all windows.
    // OBSOLETED in 1.91.9 (from February 2025)
    MOSS_API void      Moss_age(Moss_TextureRef tex_ref, const Moss_Vec2& Moss_age_size, const Moss_Vec2& uv0, const Moss_Vec2& uv1, const Moss_Vec4& tint_col, const Moss_Vec4& border_col); // <-- 'border_col' was removed in favor of Moss_GuiCol_Moss_ageBorder. If you use 'tint_col', use Moss_ageWithBg() instead.
    // OBSOLETED in 1.91.0 (from July 2024)
    static inline void  PushButtonRepeat(bool repeat)                           { PushItemFlag(Moss_GuiItemFlags_ButtonRepeat, repeat); }
    static inline void  PopButtonRepeat()                                       { PopItemFlag(); }
    static inline void  PushTabStop(bool tab_stop)                              { PushItemFlag(Moss_GuiItemFlags_NoTabStop, !tab_stop); }
    static inline void  PopTabStop()                                            { PopItemFlag(); }
    MOSS_API Moss_Vec2    GetContentRegionMax();                                  // Content boundaries max (e.g. window boundaries including scrolling, or current column boundaries). You should never need this. Always use GetCursorScreenPos() and GetContentRegionAvail()!
    MOSS_API Moss_Vec2    GetWindowContentRegionMin();                            // Content boundaries min for the window (roughly (0,0)-Scroll), in window-local coordinates. You should never need this. Always use GetCursorScreenPos() and GetContentRegionAvail()!
    MOSS_API Moss_Vec2    GetWindowContentRegionMax();                            // Content boundaries max for the window (roughly (0,0)+Size-Scroll), in window-local coordinates. You should never need this. Always use GetCursorScreenPos() and GetContentRegionAvail()!
    // OBSOLETED in 1.90.0 (from September 2023)
    static inline bool  BeginChildFrame(Moss_GuiID id, const Moss_Vec2& size, Moss_GuiWindowFlags window_flags = 0)  { return BeginChild(id, size, Moss_GuiChildFlags_FrameStyle, window_flags); }
    static inline void  EndChildFrame()                                                                     { EndChild(); }
    //static inline bool BeginChild(const char* str_id, const Moss_Vec2& size_arg, bool borders, Moss_GuiWindowFlags window_flags){ return BeginChild(str_id, size_arg, borders ? Moss_GuiChildFlags_Borders : Moss_GuiChildFlags_None, window_flags); } // Unnecessary as true == Moss_GuiChildFlags_Borders
    //static inline bool BeginChild(Moss_GuiID id, const Moss_Vec2& size_arg, bool borders, Moss_GuiWindowFlags window_flags)        { return BeginChild(id, size_arg, borders ? Moss_GuiChildFlags_Borders : Moss_GuiChildFlags_None, window_flags);     } // Unnecessary as true == Moss_GuiChildFlags_Borders
    static inline void  ShowStackToolWindow(bool* p_open = NULL)                { ShowIDStackToolWindow(p_open); }
    MOSS_API bool      Combo(const char* label, int* current_item, bool (*old_callback)(void* user_data, int idx, const char** out_text), void* user_data, int items_count, int popup_max_height_in_items = -1);
    MOSS_API bool      ListBox(const char* label, int* current_item, bool (*old_callback)(void* user_data, int idx, const char** out_text), void* user_data, int items_count, int height_in_items = -1);
    // OBSOLETED in 1.89.7 (from June 2023)
    MOSS_API void      SetItemAllowOverlap();                                  // Use SetNextItemAllowOverlap() before item.

    // Some of the older obsolete names along with their replacement (commented out so they are not reported in IDE)
    //-- OBSOLETED in 1.89.4 (from March 2023)
    //static inline void  PushAllowKeyboardFocus(bool tab_stop)                                       { PushItemFlag(Moss_GuiItemFlags_NoTabStop, !tab_stop); }
    //static inline void  PopAllowKeyboardFocus()                                                     { PopItemFlag(); }
    //-- OBSOLETED in 1.89 (from August 2022)
    //MOSS_API bool      Moss_ageButton(Moss_TextureID user_texture_id, const Moss_Vec2& size, const Moss_Vec2& uv0 = Moss_Vec2(0, 0), const Moss_Vec2& uv1 = Moss_Vec2(1, 1), int frame_padding = -1, const Moss_Vec4& bg_col = Moss_Vec4(0, 0, 0, 0), const Moss_Vec4& tint_col = Moss_Vec4(1, 1, 1, 1)); // --> Use new Moss_ageButton() signature (explicit item id, regular FramePadding). Refer to code in 1.91 if you want to grab a copy of this version.
    //-- OBSOLETED in 1.88 (from May 2022)
    //static inline void  CaptureKeyboardFromApp(bool want_capture_keyboard = true)                   { SetNextFrameWantCaptureKeyboard(want_capture_keyboard); } // Renamed as name was misleading + removed default value.
    //static inline void  CaptureMouseFromApp(bool want_capture_mouse = true)                         { SetNextFrameWantCaptureMouse(want_capture_mouse); }       // Renamed as name was misleading + removed default value.
    //-- OBSOLETED in 1.87 (from February 2022, more formally obsoleted April 2024)
    //MOSS_API Moss_GuiKey  GetKeyIndex(Moss_GuiKey key);                                                  { Moss__ASSERT(key >= Moss_GuiKey_NamedKey_BEGIN && key < Moss_GuiKey_NamedKey_END); const Moss_GuiKeyData* key_data = GetKeyData(key); return (Moss_GuiKey)(key_data - g.IO.KeysData); } // Map Moss_GuiKey_* values into legacy native key index. == io.KeyMap[key]. When using a 1.87+ backend using io.AddKeyEvent(), calling GetKeyIndex() with ANY Moss_GuiKey_XXXX values will return the same value!
    //static inline Moss_GuiKey GetKeyIndex(Moss_GuiKey key)                                                { Moss__ASSERT(key >= Moss_GuiKey_NamedKey_BEGIN && key < Moss_GuiKey_NamedKey_END); return key; }
    //-- OBSOLETED in 1.86 (from November 2021)
    //MOSS_API void      CalcListClipping(int items_count, float items_height, int* out_items_display_start, int* out_items_display_end); // Code removed, see 1.90 for last version of the code. Calculate range of visible items for large list of evenly sized items. Prefer using Moss_GuiListClipper.
    //-- OBSOLETED in 1.85 (from August 2021)
    //static inline float GetWindowContentRegionWidth()                                               { return GetWindowContentRegionMax().x - GetWindowContentRegionMin().x; }
    //-- OBSOLETED in 1.81 (from February 2021)
    //static inline bool  ListBoxHeader(const char* label, const Moss_Vec2& size = Moss_Vec2(0, 0))         { return BeginListBox(label, size); }
    //static inline bool  ListBoxHeader(const char* label, int items_count, int height_in_items = -1) { float height = GetTextLineHeightWithSpacing() * ((height_in_items < 0 ? Moss_Min(items_count, 7) : height_in_items) + 0.25f) + GetStyle().FramePadding.y * 2.0f; return BeginListBox(label, Moss_Vec2(0.0f, height)); } // Helper to calculate size from items_count and height_in_items
    //static inline void  ListBoxFooter()                                                             { EndListBox(); }
    //-- OBSOLETED in 1.79 (from August 2020)
    //static inline void  OpenPopupContextItem(const char* str_id = NULL, Moss_GuMoss_ouseButton mb = 1)    { OpenPopupOnItemClick(str_id, mb); } // Bool return value removed. Use IsWindowAppearing() in BeginPopup() instead. Renamed in 1.77, renamed back in 1.79. Sorry!
    //-- OBSOLETED in 1.78 (from June 2020): Old drag/sliders functions that took a 'float power > 1.0f' argument instead of Moss_GuiSliderFlags_Logarithmic. See github.com/ocornut/Moss_gui/issues/3361 for details.
    //MOSS_API bool      DragScalar(const char* label, Moss_GuiDataType data_type, void* p_data, float v_speed, const void* p_min, const void* p_max, const char* format, float power = 1.0f)                                                            // OBSOLETED in 1.78 (from June 2020)
    //MOSS_API bool      DragScalarN(const char* label, Moss_GuiDataType data_type, void* p_data, int components, float v_speed, const void* p_min, const void* p_max, const char* format, float power = 1.0f);                                          // OBSOLETED in 1.78 (from June 2020)
    //MOSS_API bool      SliderScalar(const char* label, Moss_GuiDataType data_type, void* p_data, const void* p_min, const void* p_max, const char* format, float power = 1.0f);                                                                        // OBSOLETED in 1.78 (from June 2020)
    //MOSS_API bool      SliderScalarN(const char* label, Moss_GuiDataType data_type, void* p_data, int components, const void* p_min, const void* p_max, const char* format, float power = 1.0f);                                                       // OBSOLETED in 1.78 (from June 2020)
    //static inline bool  DragFloat(const char* label, float* v, float v_speed, float v_min, float v_max, const char* format, float power = 1.0f)    { return DragScalar(label, Moss_GuiDataType_Float, v, v_speed, &v_min, &v_max, format, power); }     // OBSOLETED in 1.78 (from June 2020)
    //static inline bool  DragFloat2(const char* label, float v[2], float v_speed, float v_min, float v_max, const char* format, float power = 1.0f) { return DragScalarN(label, Moss_GuiDataType_Float, v, 2, v_speed, &v_min, &v_max, format, power); } // OBSOLETED in 1.78 (from June 2020)
    //static inline bool  DragFloat3(const char* label, float v[3], float v_speed, float v_min, float v_max, const char* format, float power = 1.0f) { return DragScalarN(label, Moss_GuiDataType_Float, v, 3, v_speed, &v_min, &v_max, format, power); } // OBSOLETED in 1.78 (from June 2020)
    //static inline bool  DragFloat4(const char* label, float v[4], float v_speed, float v_min, float v_max, const char* format, float power = 1.0f) { return DragScalarN(label, Moss_GuiDataType_Float, v, 4, v_speed, &v_min, &v_max, format, power); } // OBSOLETED in 1.78 (from June 2020)
    //static inline bool  SliderFloat(const char* label, float* v, float v_min, float v_max, const char* format, float power = 1.0f)                 { return SliderScalar(label, Moss_GuiDataType_Float, v, &v_min, &v_max, format, power); }            // OBSOLETED in 1.78 (from June 2020)
    //static inline bool  SliderFloat2(const char* label, float v[2], float v_min, float v_max, const char* format, float power = 1.0f)              { return SliderScalarN(label, Moss_GuiDataType_Float, v, 2, &v_min, &v_max, format, power); }        // OBSOLETED in 1.78 (from June 2020)
    //static inline bool  SliderFloat3(const char* label, float v[3], float v_min, float v_max, const char* format, float power = 1.0f)              { return SliderScalarN(label, Moss_GuiDataType_Float, v, 3, &v_min, &v_max, format, power); }        // OBSOLETED in 1.78 (from June 2020)
    //static inline bool  SliderFloat4(const char* label, float v[4], float v_min, float v_max, const char* format, float power = 1.0f)              { return SliderScalarN(label, Moss_GuiDataType_Float, v, 4, &v_min, &v_max, format, power); }        // OBSOLETED in 1.78 (from June 2020)
    //-- OBSOLETED in 1.77 and before
    //static inline bool  BeginPopupContextWindow(const char* str_id, Moss_GuMoss_ouseButton mb, bool over_items) { return BeginPopupContextWindow(str_id, mb | (over_items ? 0 : Moss_GuiPopupFlags_NoOpenOverItems)); } // OBSOLETED in 1.77 (from June 2020)
    //static inline void  TreeAdvanceToLabelPos()               { SetCursorPosX(GetCursorPosX() + GetTreeNodeToLabelSpacing()); }   // OBSOLETED in 1.72 (from July 2019)
    //static inline void  SetNextTreeNodeOpen(bool open, Moss_GuiCond cond = 0) { SetNextItemOpen(open, cond); }                       // OBSOLETED in 1.71 (from June 2019)
    //static inline float GetContentRegionAvailWidth()          { return GetContentRegionAvail().x; }                               // OBSOLETED in 1.70 (from May 2019)
    //static inline Moss_DrawList* GetOverlayDrawList()            { return GetForegroundDrawList(); }                                 // OBSOLETED in 1.69 (from Mar 2019)
    //static inline void  SetScrollHere(float ratio = 0.5f)     { SetScrollHereY(ratio); }                                          // OBSOLETED in 1.66 (from Nov 2018)
    //static inline bool  IsItemDeactivatedAfterChange()        { return IsItemDeactivatedAfterEdit(); }                            // OBSOLETED in 1.63 (from Aug 2018)
    //-- OBSOLETED in 1.60 and before
    //static inline bool  IsAnyWindowFocused()                  { return IsWindowFocused(Moss_GuiFocusedFlags_AnyWindow); }            // OBSOLETED in 1.60 (from Apr 2018)
    //static inline bool  IsAnyWindowHovered()                  { return IsWindowHovered(Moss_GuiHoveredFlags_AnyWindow); }            // OBSOLETED in 1.60 (between Dec 2017 and Apr 2018)
    //static inline void  ShowTestWindow()                      { return ShowDemoWindow(); }                                        // OBSOLETED in 1.53 (between Oct 2017 and Dec 2017)
    //static inline bool  IsRootWindowFocused()                 { return IsWindowFocused(Moss_GuiFocusedFlags_RootWindow); }           // OBSOLETED in 1.53 (between Oct 2017 and Dec 2017)
    //static inline bool  IsRootWindowOrAnyChildFocused()       { return IsWindowFocused(Moss_GuiFocusedFlags_RootAndChildWindows); }  // OBSOLETED in 1.53 (between Oct 2017 and Dec 2017)
    //static inline void  SetNextWindowContentWidth(float w)    { SetNextWindowContentSize(Moss_Vec2(w, 0.0f)); }                      // OBSOLETED in 1.53 (between Oct 2017 and Dec 2017)
    //static inline float GetItemsLineHeightWithSpacing()       { return GetFrameHeightWithSpacing(); }                             // OBSOLETED in 1.53 (between Oct 2017 and Dec 2017)
    //MOSS_API bool      Begin(char* name, bool* p_open, Moss_Vec2 size_first_use, float bg_alpha = -1.0f, Moss_GuiWindowFlags flags=0); // OBSOLETED in 1.52 (between Aug 2017 and Oct 2017): Equivalent of using SetNextWindowSize(size, Moss_GuiCond_FirstUseEver) and SetNextWindowBgAlpha().
    //static inline bool  IsRootWindowOrAnyChildHovered()       { return IsWindowHovered(Moss_GuiHoveredFlags_RootAndChildWindows); }  // OBSOLETED in 1.52 (between Aug 2017 and Oct 2017)
    //static inline void  AlignFirstTextHeightToWidgets()       { AlignTextToFramePadding(); }                                      // OBSOLETED in 1.52 (between Aug 2017 and Oct 2017)
    //static inline void  SetNextWindowPosCenter(Moss_GuiCond c=0) { SetNextWindowPos(GetMainViewport()->GetCenter(), c, Moss_Vec2(0.5f,0.5f)); } // OBSOLETED in 1.52 (between Aug 2017 and Oct 2017)
    //static inline bool  IsItemHoveredRect()                   { return IsItemHovered(Moss_GuiHoveredFlags_RectOnly); }               // OBSOLETED in 1.51 (between Jun 2017 and Aug 2017)
    //static inline bool  IsPosHoveringAnyWindow(const Moss_Vec2&) { Moss__ASSERT(0); return false; }                                     // OBSOLETED in 1.51 (between Jun 2017 and Aug 2017): This was misleading and partly broken. You probably want to use the io.WantCaptureMouse flag instead.
    //static inline bool  IsMouseHoveringAnyWindow()            { return IsWindowHovered(Moss_GuiHoveredFlags_AnyWindow); }            // OBSOLETED in 1.51 (between Jun 2017 and Aug 2017)
    //static inline bool  IsMouseHoveringWindow()               { return IsWindowHovered(Moss_GuiHoveredFlags_AllowWhenBlockedByPopup | Moss_GuiHoveredFlags_AllowWhenBlockedByActiveItem); }       // OBSOLETED in 1.51 (between Jun 2017 and Aug 2017)
    //-- OBSOLETED in 1.50 and before
    //static inline bool  CollapsingHeader(char* label, const char* str_id, bool framed = true, bool default_open = false) { return CollapsingHeader(label, (default_open ? (1 << 5) : 0)); } // OBSOLETED in 1.49
    //static inline Moss_Font*GetWindowFont()                      { return GetFont(); }                                               // OBSOLETED in 1.48
    //static inline float GetWindowFontSize()                   { return GetFontSize(); }                                           // OBSOLETED in 1.48
    //static inline void  SetScrollPosHere()                    { SetScrollHere(); }                                                // OBSOLETED in 1.42
}






















struct Moss_DrawCmd
{
    Moss_Vec4          ClipRect;           // 4*4  // Clipping rectangle (x1, y1, x2, y2). Subtract Moss_DrawData->DisplayPos to get clipping rectangle in "viewport" coordinates
    Moss_TextureRef    TexRef;             // 16   // Reference to a font/texture atlas (where backend called Moss_TextureData::SetTexID()) or to a user-provided texture ID (via e.g. Moss_Gui::Moss_age() calls). Both will lead to a Moss_TextureID value.
    unsigned int    VtxOffset;          // 4    // Start offset in vertex buffer. Moss_GuiBackendFlags_RendererHasVtxOffset: always 0, otherwise may be >0 to support meshes larger than 64K vertices with 16-bit indices.
    unsigned int    IdxOffset;          // 4    // Start offset in index buffer.
    unsigned int    ElemCount;          // 4    // Number of indices (multiple of 3) to be rendered as triangles. Vertices are stored in the callee Moss_DrawList's vtx_buffer[] array, indices in idx_buffer[].
    Moss_DrawCallback  UserCallback;       // 4-8  // If != NULL, call the function instead of rendering the vertices. clip_rect and texture_id will be set normally.
    void*           UserCallbackData;   // 4-8  // Callback user data (when UserCallback != NULL). If called AddCallback() with size == 0, this is a copy of the AddCallback() argument. If called AddCallback() with size > 0, this is pointing to a buffer where data is stored.
    int             UserCallbackDataSize;  // 4 // Size of callback user data when using storage, otherwise 0.
    int             UserCallbackDataOffset;// 4 // [Internal] Offset of callback user data when using storage, otherwise -1.

    Moss_DrawCmd()     { memset(this, 0, sizeof(*this)); } // Also ensure our padding fields are zeroed

    // Since 1.83: returns Moss_TextureID associated with this draw call. Warning: DO NOT assume this is always same as 'TextureId' (we will change this function for an upcoming feature)
    // Since 1.92: removed Moss_DrawCmd::TextureId field, the getter function must be used!
    inline Moss_TextureID GetTexID() const;    // == (TexRef._TexData ? TexRef._TexData->TexID : TexRef._TexID
};

// Vertex layout
#ifndef Moss_GUI_OVERRIDE_DRAWVERT_STRUCT_LAYOUT
struct Moss_DrawVert
{
    Moss_Vec2  pos;
    Moss_Vec2  uv;
    Moss_U32   col;
};
#else
// You can override the vertex format layout by defining Moss_GUI_OVERRIDE_DRAWVERT_STRUCT_LAYOUT in Moss_config.h
// The code expect Moss_Vec2 pos (8 bytes), Moss_Vec2 uv (8 bytes), Moss_U32 col (4 bytes), but you can re-order them or add other fields as needed to sMoss_plify integration in your engine.
// The type has to be described within the macro (you can either declare the struct or use a typedef). This is because Moss_Vec2/Moss_U32 are likely not declared at the tMoss_e you'd want to set your type up.
// NOTE: Moss_GUI DOESN'T CLEAR THE STRUCTURE AND DOESN'T CALL A CONSTRUCTOR SO ANY CUSTOM FIELD WILL BE UNINITIALIZED. IF YOU ADD EXTRA FIELDS (SUCH AS A 'Z' COORDINATES) YOU WILL NEED TO CLEAR THEM DURING RENDER OR TO IGNORE THEM.
Moss_GUI_OVERRIDE_DRAWVERT_STRUCT_LAYOUT;
#endif

// [Internal] For use by Moss_DrawList
struct Moss_DrawCmdHeader
{
    Moss_Vec4          ClipRect;
    Moss_TextureRef    TexRef;
    unsigned int    VtxOffset;
};

// [Internal] For use by Moss_DrawListSplitter
struct Moss_DrawChannel
{
    TVector<Moss_DrawCmd>         _CmdBuffer;
    TVector<Moss_DrawIdx>         _IdxBuffer;
};

// Split/Merge functions are used to split the draw list into different layers which can be drawn into out of order.
// This is used by the Columns/Tables API, so items of each column can be batched together in a same draw call.
struct Moss_DrawListSplitter
{
    int                         _Current;    // Current channel number (0)
    int                         _Count;      // Number of active channels (1+)
    TVector<Moss_DrawChannel>     _Channels;   // Draw channels (not resized down so _Count might be < Channels.Size)

    inline Moss_DrawListSplitter()  { memset(this, 0, sizeof(*this)); }
    inline ~Moss_DrawListSplitter() { ClearFreeMemory(); }
    inline void                 Clear() { _Current = 0; _Count = 1; } // Do not clear Channels[] so our allocations are reused next frame
    MOSS_API void              ClearFreeMemory();
    MOSS_API void              Split(Moss_DrawList* draw_list, int count);
    MOSS_API void              Merge(Moss_DrawList* draw_list);
    MOSS_API void              SetCurrentChannel(Moss_DrawList* draw_list, int channel_idx);
};

// Flags for Moss_DrawList functions
// (Legacy: bit 0 must always correspond to Moss_DrawFlags_Closed to be backward compatible with old API using a bool. Bits 1..3 must be unused)
enum Moss_DrawFlags_
{
    Moss_DrawFlags_None                        = 0,
    Moss_DrawFlags_Closed                      = 1 << 0, // PathStroke(), AddPolyline(): specify that shape should be closed (Moss_portant: this is always == 1 for legacy reason)
    Moss_DrawFlags_RoundCornersTopLeft         = 1 << 4, // AddRect(), AddRectFilled(), PathRect(): enable rounding top-left corner only (when rounding > 0.0f, we default to all corners). Was 0x01.
    Moss_DrawFlags_RoundCornersTopRight        = 1 << 5, // AddRect(), AddRectFilled(), PathRect(): enable rounding top-right corner only (when rounding > 0.0f, we default to all corners). Was 0x02.
    Moss_DrawFlags_RoundCornersBottomLeft      = 1 << 6, // AddRect(), AddRectFilled(), PathRect(): enable rounding bottom-left corner only (when rounding > 0.0f, we default to all corners). Was 0x04.
    Moss_DrawFlags_RoundCornersBottomRight     = 1 << 7, // AddRect(), AddRectFilled(), PathRect(): enable rounding bottom-right corner only (when rounding > 0.0f, we default to all corners). Wax 0x08.
    Moss_DrawFlags_RoundCornersNone            = 1 << 8, // AddRect(), AddRectFilled(), PathRect(): disable rounding on all corners (when rounding > 0.0f). This is NOT zero, NOT an Moss_plicit flag!
    Moss_DrawFlags_RoundCornersTop             = Moss_DrawFlags_RoundCornersTopLeft | Moss_DrawFlags_RoundCornersTopRight,
    Moss_DrawFlags_RoundCornersBottom          = Moss_DrawFlags_RoundCornersBottomLeft | Moss_DrawFlags_RoundCornersBottomRight,
    Moss_DrawFlags_RoundCornersLeft            = Moss_DrawFlags_RoundCornersBottomLeft | Moss_DrawFlags_RoundCornersTopLeft,
    Moss_DrawFlags_RoundCornersRight           = Moss_DrawFlags_RoundCornersBottomRight | Moss_DrawFlags_RoundCornersTopRight,
    Moss_DrawFlags_RoundCornersAll             = Moss_DrawFlags_RoundCornersTopLeft | Moss_DrawFlags_RoundCornersTopRight | Moss_DrawFlags_RoundCornersBottomLeft | Moss_DrawFlags_RoundCornersBottomRight,
    Moss_DrawFlags_RoundCornersDefault_        = Moss_DrawFlags_RoundCornersAll, // Default to ALL corners if none of the _RoundCornersXX flags are specified.
    Moss_DrawFlags_RoundCornersMask_           = Moss_DrawFlags_RoundCornersAll | Moss_DrawFlags_RoundCornersNone,
};

// Flags for Moss_DrawList instance. Those are set automatically by Moss_Gui:: functions from Moss_GuiIO settings, and generally not manipulated directly.
// It is however possible to temporarily alter flags between calls to Moss_DrawList:: functions.
enum Moss_DrawListFlags_
{
    Moss_DrawListFlags_None                    = 0,
    Moss_DrawListFlags_AntiAliasedLines        = 1 << 0,  // Enable anti-aliased lines/borders (*2 the number of triangles for 1.0f wide line or lines thin enough to be drawn using textures, otherwise *3 the number of triangles)
    Moss_DrawListFlags_AntiAliasedLinesUseTex  = 1 << 1,  // Enable anti-aliased lines/borders using textures when possible. Require backend to render with bilinear filtering (NOT point/nearest filtering).
    Moss_DrawListFlags_AntiAliasedFill         = 1 << 2,  // Enable anti-aliased edge around filled shapes (rounded rectangles, circles).
    Moss_DrawListFlags_AllowVtxOffset          = 1 << 3,  // Can emit 'VtxOffset > 0' to allow large meshes. Set when 'Moss_GuiBackendFlags_RendererHasVtxOffset' is enabled.
};

// Draw command list
// This is the low-level list of polygons that Moss_Gui:: functions are filling. At the end of the frame,
// all command lists are passed to your Moss_GuiIO::RenderDrawListFn function for rendering.
// Each dear Moss_gui window contains its own Moss_DrawList. You can use Moss_Gui::GetWindowDrawList() to
// access the current window draw list and draw custom prMoss_itives.
// You can interleave normal Moss_Gui:: calls and adding prMoss_itives to the current draw list.
// In single viewport mode, top-left is == GetMainViewport()->Pos (generally 0,0), bottom-right is == GetMainViewport()->Pos+Size (generally io.DisplaySize).
// You are totally free to apply whatever transformation matrix you want to the data (depending on the use of the transformation you may want to apply it to ClipRect as well!)
// Moss_portant: PrMoss_itives are always added to the list and not culled (culling is done at higher-level by Moss_Gui:: functions), if you use this API a lot consider coarse culling your drawn objects.
struct Moss_DrawList
{
    // This is what you have to render
    TVector<Moss_DrawCmd>     CmdBuffer;          // Draw commands. Typically 1 command = 1 GPU draw call, unless the command is a callback.
    TVector<Moss_DrawIdx>     IdxBuffer;          // Index buffer. Each command consume Moss_DrawCmd::ElemCount of those
    TVector<Moss_DrawVert>    VtxBuffer;          // Vertex buffer.
    Moss_DrawListFlags         Flags;              // Flags, you may poke into these to adjust anti-aliasing settings per-prMoss_itive.

    // [Internal, used while building lists]
    unsigned int            _VtxCurrentIdx;     // [Internal] generally == VtxBuffer.Size unless we are past 64K vertices, in which case this gets reset to 0.
    Moss_DrawListSharedData*   _Data;              // Pointer to shared draw data (you can use Moss_Gui::GetDrawListSharedData() to get the one from current Moss_Gui context)
    Moss_DrawVert*             _VtxWritePtr;       // [Internal] point within VtxBuffer.Data after each add command (to avoid using the TVector<> operators too much)
    Moss_DrawIdx*              _IdxWritePtr;       // [Internal] point within IdxBuffer.Data after each add command (to avoid using the TVector<> operators too much)
    TVector<Moss_Vec2>        _Path;              // [Internal] current path building
    Moss_DrawCmdHeader         _CmdHeader;         // [Internal] template of active commands. Fields should match those of CmdBuffer.back().
    Moss_DrawListSplitter      _Splitter;          // [Internal] for channels api (note: prefer using your own persistent instance of Moss_DrawListSplitter!)
    TVector<Moss_Vec4>        _ClipRectStack;     // [Internal]
    TVector<Moss_TextureRef>  _TextureStack;      // [Internal]
    TVector<Moss_U8>          _CallbacksDataBuf;  // [Internal]
    float                   _FringeScale;       // [Internal] anti-alias fringe is scaled by this value, this helps to keep things sharp while zooming at vertex buffer content
    const char*             _OwnerName;         // Pointer to owner window's name for debugging

    // If you want to create Moss_DrawList instances, pass them Moss_Gui::GetDrawListSharedData().
    // (advanced: you may create and use your own Moss_DrawListSharedData so you can use Moss_DrawList without Moss_Gui, but that's more involved)
    MOSS_API Moss_DrawList(Moss_DrawListSharedData* shared_data);
    MOSS_API ~Moss_DrawList();

    MOSS_API void  PushClipRect(const Moss_Vec2& clip_rect_min, const Moss_Vec2& clip_rect_max, bool intersect_with_current_clip_rect = false);  // Render-level scissoring. This is passed down to your render function but not used for CPU-side coarse clipping. Prefer using higher-level Moss_Gui::PushClipRect() to affect logic (hit-testing and widget culling)
    MOSS_API void  PushClipRectFullScreen();
    MOSS_API void  PopClipRect();
    MOSS_API void  PushTexture(Moss_TextureRef tex_ref);
    MOSS_API void  PopTexture();
    inline Moss_Vec2   GetClipRectMin() const { const Moss_Vec4& cr = _ClipRectStack.back(); return Moss_Vec2(cr.x, cr.y); }
    inline Moss_Vec2   GetClipRectMax() const { const Moss_Vec4& cr = _ClipRectStack.back(); return Moss_Vec2(cr.z, cr.w); }

    // PrMoss_itives
    // - Filled shapes must always use clockwise winding order. The anti-aliasing fringe depends on it. Counter-clockwise shapes will have "inward" anti-aliasing.
    // - For rectangular prMoss_itives, "p_min" and "p_max" represent the upper-left and lower-right corners.
    // - For circle prMoss_itives, use "num_segments == 0" to automatically calculate tessellation (preferred).
    //   In older versions (until Dear Moss_Gui 1.77) the AddCircle functions defaulted to num_segments == 12.
    //   In future versions we will use textures to provide cheaper and higher-quality circles.
    //   Use AddNgon() and AddNgonFilled() functions if you need to guarantee a specific number of sides.
    MOSS_API void  AddLine(const Moss_Vec2& p1, const Moss_Vec2& p2, Moss_U32 col, float thickness = 1.0f);
    MOSS_API void  AddRect(const Moss_Vec2& p_min, const Moss_Vec2& p_max, Moss_U32 col, float rounding = 0.0f, Moss_DrawFlags flags = 0, float thickness = 1.0f);   // a: upper-left, b: lower-right (== upper-left + size)
    MOSS_API void  AddRectFilled(const Moss_Vec2& p_min, const Moss_Vec2& p_max, Moss_U32 col, float rounding = 0.0f, Moss_DrawFlags flags = 0);                     // a: upper-left, b: lower-right (== upper-left + size)
    MOSS_API void  AddRectFilledMultiColor(const Moss_Vec2& p_min, const Moss_Vec2& p_max, Moss_U32 col_upr_left, Moss_U32 col_upr_right, Moss_U32 col_bot_right, Moss_U32 col_bot_left);
    MOSS_API void  AddQuad(const Moss_Vec2& p1, const Moss_Vec2& p2, const Moss_Vec2& p3, const Moss_Vec2& p4, Moss_U32 col, float thickness = 1.0f);
    MOSS_API void  AddQuadFilled(const Moss_Vec2& p1, const Moss_Vec2& p2, const Moss_Vec2& p3, const Moss_Vec2& p4, Moss_U32 col);
    MOSS_API void  AddTriangle(const Moss_Vec2& p1, const Moss_Vec2& p2, const Moss_Vec2& p3, Moss_U32 col, float thickness = 1.0f);
    MOSS_API void  AddTriangleFilled(const Moss_Vec2& p1, const Moss_Vec2& p2, const Moss_Vec2& p3, Moss_U32 col);
    MOSS_API void  AddCircle(const Moss_Vec2& center, float radius, Moss_U32 col, int num_segments = 0, float thickness = 1.0f);
    MOSS_API void  AddCircleFilled(const Moss_Vec2& center, float radius, Moss_U32 col, int num_segments = 0);
    MOSS_API void  AddNgon(const Moss_Vec2& center, float radius, Moss_U32 col, int num_segments, float thickness = 1.0f);
    MOSS_API void  AddNgonFilled(const Moss_Vec2& center, float radius, Moss_U32 col, int num_segments);
    MOSS_API void  AddEllipse(const Moss_Vec2& center, const Moss_Vec2& radius, Moss_U32 col, float rot = 0.0f, int num_segments = 0, float thickness = 1.0f);
    MOSS_API void  AddEllipseFilled(const Moss_Vec2& center, const Moss_Vec2& radius, Moss_U32 col, float rot = 0.0f, int num_segments = 0);
    MOSS_API void  AddText(const Moss_Vec2& pos, Moss_U32 col, const char* text_begin, const char* text_end = NULL);
    MOSS_API void  AddText(Moss_Font* font, float font_size, const Moss_Vec2& pos, Moss_U32 col, const char* text_begin, const char* text_end = NULL, float wrap_width = 0.0f, const Moss_Vec4* cpu_fine_clip_rect = NULL);
    MOSS_API void  AddBezierCubic(const Moss_Vec2& p1, const Moss_Vec2& p2, const Moss_Vec2& p3, const Moss_Vec2& p4, Moss_U32 col, float thickness, int num_segments = 0); // Cubic Bezier (4 control points)
    MOSS_API void  AddBezierQuadratic(const Moss_Vec2& p1, const Moss_Vec2& p2, const Moss_Vec2& p3, Moss_U32 col, float thickness, int num_segments = 0);               // Quadratic Bezier (3 control points)

    // General polygon
    // - Only sMoss_ple polygons are supported by filling functions (no self-intersections, no holes).
    // - Concave polygon fill is more expensive than convex one: it has O(N^2) complexity. Provided as a convenience for the user but not used by the main library.
    MOSS_API void  AddPolyline(const Moss_Vec2* points, int num_points, Moss_U32 col, Moss_DrawFlags flags, float thickness);
    MOSS_API void  AddConvexPolyFilled(const Moss_Vec2* points, int num_points, Moss_U32 col);
    MOSS_API void  AddConcavePolyFilled(const Moss_Vec2* points, int num_points, Moss_U32 col);

    // Moss_age prMoss_itives
    // - Read FAQ to understand what Moss_TextureID/Moss_TextureRef are.
    // - "p_min" and "p_max" represent the upper-left and lower-right corners of the rectangle.
    // - "uv_min" and "uv_max" represent the normalized texture coordinates to use for those corners. Using (0,0)->(1,1) texture coordinates will generally display the entire texture.
    MOSS_API void  AddMoss_age(Moss_TextureRef tex_ref, const Moss_Vec2& p_min, const Moss_Vec2& p_max, const Moss_Vec2& uv_min = Moss_Vec2(0, 0), const Moss_Vec2& uv_max = Moss_Vec2(1, 1), Moss_U32 col = Moss__COL32_WHITE);
    MOSS_API void  AddMoss_ageQuad(Moss_TextureRef tex_ref, const Moss_Vec2& p1, const Moss_Vec2& p2, const Moss_Vec2& p3, const Moss_Vec2& p4, const Moss_Vec2& uv1 = Moss_Vec2(0, 0), const Moss_Vec2& uv2 = Moss_Vec2(1, 0), const Moss_Vec2& uv3 = Moss_Vec2(1, 1), const Moss_Vec2& uv4 = Moss_Vec2(0, 1), Moss_U32 col = Moss__COL32_WHITE);
    MOSS_API void  AddMoss_ageRounded(Moss_TextureRef tex_ref, const Moss_Vec2& p_min, const Moss_Vec2& p_max, const Moss_Vec2& uv_min, const Moss_Vec2& uv_max, Moss_U32 col, float rounding, Moss_DrawFlags flags = 0);

    // Stateful path API, add points then finish with PathFillConvex() or PathStroke()
    // - Moss_portant: filled shapes must always use clockwise winding order! The anti-aliasing fringe depends on it. Counter-clockwise shapes will have "inward" anti-aliasing.
    //   so e.g. 'PathArcTo(center, radius, PI * -0.5f, PI)' is ok, whereas 'PathArcTo(center, radius, PI, PI * -0.5f)' won't have correct anti-aliasing when followed by PathFillConvex().
    inline    void  PathClear()                                                 { _Path.Size = 0; }
    inline    void  PathLineTo(const Moss_Vec2& pos)                               { _Path.push_back(pos); }
    inline    void  PathLineToMergeDuplicate(const Moss_Vec2& pos)                 { if (_Path.Size == 0 || memcmp(&_Path.Data[_Path.Size - 1], &pos, 8) != 0) _Path.push_back(pos); }
    inline    void  PathFillConvex(Moss_U32 col)                                   { AddConvexPolyFilled(_Path.Data, _Path.Size, col); _Path.Size = 0; }
    inline    void  PathFillConcave(Moss_U32 col)                                  { AddConcavePolyFilled(_Path.Data, _Path.Size, col); _Path.Size = 0; }
    inline    void  PathStroke(Moss_U32 col, Moss_DrawFlags flags = 0, float thickness = 1.0f) { AddPolyline(_Path.Data, _Path.Size, col, flags, thickness); _Path.Size = 0; }
    MOSS_API void  PathArcTo(const Moss_Vec2& center, float radius, float a_min, float a_max, int num_segments = 0);
    MOSS_API void  PathArcToFast(const Moss_Vec2& center, float radius, int a_min_of_12, int a_max_of_12);                // Use precomputed angles for a 12 steps circle
    MOSS_API void  PathEllipticalArcTo(const Moss_Vec2& center, const Moss_Vec2& radius, float rot, float a_min, float a_max, int num_segments = 0); // Ellipse
    MOSS_API void  PathBezierCubicCurveTo(const Moss_Vec2& p2, const Moss_Vec2& p3, const Moss_Vec2& p4, int num_segments = 0); // Cubic Bezier (4 control points)
    MOSS_API void  PathBezierQuadraticCurveTo(const Moss_Vec2& p2, const Moss_Vec2& p3, int num_segments = 0);               // Quadratic Bezier (3 control points)
    MOSS_API void  PathRect(const Moss_Vec2& rect_min, const Moss_Vec2& rect_max, float rounding = 0.0f, Moss_DrawFlags flags = 0);

    // Advanced: Draw Callbacks
    // - May be used to alter render state (change sampler, blending, current shader). May be used to emit custom rendering commands (difficult to do correctly, but possible).
    // - Use special Moss_DrawCallback_ResetRenderState callback to instruct backend to reset its render state to the default.
    // - Your rendering loop must check for 'UserCallback' in Moss_DrawCmd and call the function instead of rendering triangles. All standard backends are honoring this.
    // - For some backends, the callback may access selected render-states exposed by the backend in a Moss_Gui_Moss_plXXXX_RenderState structure pointed to by platform_io.Renderer_RenderState.
    // - Moss_PORTANT: please be mindful of the different level of indirection between using size==0 (copying argument) and using size>0 (copying pointed data into a buffer).
    //   - If userdata_size == 0: we copy/store the 'userdata' argument as-is. It will be available unmodified in Moss_DrawCmd::UserCallbackData during render.
    //   - If userdata_size > 0,  we copy/store 'userdata_size' bytes pointed to by 'userdata'. We store them in a buffer stored inside the drawlist. Moss_DrawCmd::UserCallbackData will point inside that buffer so you have to retrieve data from there. Your callback may need to use Moss_DrawCmd::UserCallbackDataSize if you expect dynamically-sized data.
    //   - Support for userdata_size > 0 was added in v1.91.4, October 2024. So earlier code always only allowed to copy/store a sMoss_ple void*.
    MOSS_API void  AddCallback(Moss_DrawCallback callback, void* userdata, size_t userdata_size = 0);

    // Advanced: Miscellaneous
    MOSS_API void  AddDrawCmd();                                               // This is useful if you need to forcefully create a new draw call (to allow for dependent rendering / blending). Otherwise prMoss_itives are merged into the same draw-call as much as possible
    MOSS_API Moss_DrawList* CloneOutput() const;                                  // Create a clone of the CmdBuffer/IdxBuffer/VtxBuffer.

    // Advanced: Channels
    // - Use to split render into layers. By switching channels to can render out-of-order (e.g. submit FG prMoss_itives before BG prMoss_itives)
    // - Use to minMoss_ize draw calls (e.g. if going back-and-forth between multiple clipping rectangles, prefer to append into separate channels then merge at the end)
    // - This API shouldn't have been in Moss_DrawList in the first place!
    //   Prefer using your own persistent instance of Moss_DrawListSplitter as you can stack them.
    //   Using the Moss_DrawList::ChannelsXXXX you cannot stack a split over another.
    inline void     ChannelsSplit(int count)    { _Splitter.Split(this, count); }
    inline void     ChannelsMerge()             { _Splitter.Merge(this); }
    inline void     ChannelsSetCurrent(int n)   { _Splitter.SetCurrentChannel(this, n); }

    // Advanced: PrMoss_itives allocations
    // - We render triangles (three vertices)
    // - All prMoss_itives needs to be reserved via PrMoss_Reserve() beforehand.
    MOSS_API void  PrMoss_Reserve(int idx_count, int vtx_count);
    MOSS_API void  PrMoss_Unreserve(int idx_count, int vtx_count);
    MOSS_API void  PrMoss_Rect(const Moss_Vec2& a, const Moss_Vec2& b, Moss_U32 col);      // Axis aligned rectangle (composed of two triangles)
    MOSS_API void  PrMoss_RectUV(const Moss_Vec2& a, const Moss_Vec2& b, const Moss_Vec2& uv_a, const Moss_Vec2& uv_b, Moss_U32 col);
    MOSS_API void  PrMoss_QuadUV(const Moss_Vec2& a, const Moss_Vec2& b, const Moss_Vec2& c, const Moss_Vec2& d, const Moss_Vec2& uv_a, const Moss_Vec2& uv_b, const Moss_Vec2& uv_c, const Moss_Vec2& uv_d, Moss_U32 col);
    inline    void  PrMoss_WriteVtx(const Moss_Vec2& pos, const Moss_Vec2& uv, Moss_U32 col)    { _VtxWritePtr->pos = pos; _VtxWritePtr->uv = uv; _VtxWritePtr->col = col; _VtxWritePtr++; _VtxCurrentIdx++; }
    inline    void  PrMoss_WriteIdx(Moss_DrawIdx idx)                                     { *_IdxWritePtr = idx; _IdxWritePtr++; }
    inline    void  PrMoss_Vtx(const Moss_Vec2& pos, const Moss_Vec2& uv, Moss_U32 col)         { PrMoss_WriteIdx((Moss_DrawIdx)_VtxCurrentIdx); PrMoss_WriteVtx(pos, uv, col); } // Write vertex with unique index

    //inline  void  AddEllipse(const Moss_Vec2& center, float radius_x, float radius_y, Moss_U32 col, float rot = 0.0f, int num_segments = 0, float thickness = 1.0f) { AddEllipse(center, Moss_Vec2(radius_x, radius_y), col, rot, num_segments, thickness); } // OBSOLETED in 1.90.5 (Mar 2024)
    //inline  void  AddEllipseFilled(const Moss_Vec2& center, float radius_x, float radius_y, Moss_U32 col, float rot = 0.0f, int num_segments = 0) { AddEllipseFilled(center, Moss_Vec2(radius_x, radius_y), col, rot, num_segments); }                        // OBSOLETED in 1.90.5 (Mar 2024)
    //inline  void  PathEllipticalArcTo(const Moss_Vec2& center, float radius_x, float radius_y, float rot, float a_min, float a_max, int num_segments = 0) { PathEllipticalArcTo(center, Moss_Vec2(radius_x, radius_y), rot, a_min, a_max, num_segments); } // OBSOLETED in 1.90.5 (Mar 2024)
    //inline  void  AddBezierCurve(const Moss_Vec2& p1, const Moss_Vec2& p2, const Moss_Vec2& p3, const Moss_Vec2& p4, Moss_U32 col, float thickness, int num_segments = 0) { AddBezierCubic(p1, p2, p3, p4, col, thickness, num_segments); }                         // OBSOLETED in 1.80 (Jan 2021)
    //inline  void  PathBezierCurveTo(const Moss_Vec2& p2, const Moss_Vec2& p3, const Moss_Vec2& p4, int num_segments = 0) { PathBezierCubicCurveTo(p2, p3, p4, num_segments); }                                                                                // OBSOLETED in 1.80 (Jan 2021)

    // [Internal helpers]
    MOSS_API void  _SetDrawListSharedData(Moss_DrawListSharedData* data);
    MOSS_API void  _ResetForNewFrame();
    MOSS_API void  _ClearFreeMemory();
    MOSS_API void  _PopUnusedDrawCmd();
    MOSS_API void  _TryMergeDrawCmds();
    MOSS_API void  _OnChangedClipRect();
    MOSS_API void  _OnChangedTexture();
    MOSS_API void  _OnChangedVtxOffset();
    MOSS_API void  _SetTexture(Moss_TextureRef tex_ref);
    MOSS_API int   _CalcCircleAutoSegmentCount(float radius) const;
    MOSS_API void  _PathArcToFastEx(const Moss_Vec2& center, float radius, int a_min_sample, int a_max_sample, int a_step);
    MOSS_API void  _PathArcToN(const Moss_Vec2& center, float radius, float a_min, float a_max, int num_segments);
};

// All draw data to render a Dear Moss_Gui frame
// (NB: the style and the naming convention here is a little inconsistent, we currently preserve them for backward compatibility purpose,
// as this is one of the oldest structure exposed by the library! Basically, Moss_DrawList == CmdList)
struct Moss_DrawData
{
    bool                Valid;              // Only valid after Render() is called and before the next NewFrame() is called.
    int                 CmdListsCount;      // Number of Moss_DrawList* to render. (== CmdLists.Size). Exists for legacy reason.
    int                 TotalIdxCount;      // For convenience, sum of all Moss_DrawList's IdxBuffer.Size
    int                 TotalVtxCount;      // For convenience, sum of all Moss_DrawList's VtxBuffer.Size
    TVector<Moss_DrawList*> CmdLists;         // Array of Moss_DrawList* to render. The Moss_DrawLists are owned by Moss_GuiContext and only pointed to from here.
    Moss_Vec2              DisplayPos;         // Top-left position of the viewport to render (== top-left of the orthogonal projection matrix to use) (== GetMainViewport()->Pos for the main viewport, == (0.0) in most single-viewport applications)
    Moss_Vec2              DisplaySize;        // Size of the viewport to render (== GetMainViewport()->Size for the main viewport, == io.DisplaySize in most single-viewport applications)
    Moss_Vec2              FramebufferScale;   // Amount of pixels for each unit of DisplaySize. Copied from viewport->FramebufferScale (== io.DisplayFramebufferScale for main viewport). Generally (1,1) on normal display, (2,2) on OSX with Retina display.
    Moss_GuiViewport*      OwnerViewport;      // Viewport carrying the Moss_DrawData instance, might be of use to the renderer (generally not).
    TVector<Moss_TextureData*>* Textures;     // List of textures to update. Most of the tMoss_es the list is shared by all Moss_DrawData, has only 1 texture and it doesn't need any update. This almost always points to Moss_Gui::GetPlatformIO().Textures[]. May be overriden or set to NULL if you want to manually update textures.

    // Functions
    Moss_DrawData()    { Clear(); }
    MOSS_API void  Clear();
    MOSS_API void  AddDrawList(Moss_DrawList* draw_list);     // Helper to add an external draw list into an existing Moss_DrawData.
    MOSS_API void  DeIndexAllBuffers();                    // Helper to convert all buffers from indexed to non-indexed, in case you cannot render indexed. Note: this is slow and most likely a waste of resources. Always prefer indexed rendering!
    MOSS_API void  ScaleClipRects(const Moss_Vec2& fb_scale); // Helper to scale the ClipRect field of each Moss_DrawCmd. Use if your final output buffer is at a different scale than Dear Moss_Gui expects, or if there is a difference between your window resolution and framebuffer resolution.
};

struct GuiViewport
{
    Moss_GuiID             ID;                     // Unique identifier for the viewport
    Moss_GuiViewportFlags  Flags;                  // See Moss_GuiViewportFlags_
    Moss_Vec2              Pos;                    // Main Area: Position of the viewport (Dear Moss_Gui coordinates are the same as OS desktop/native coordinates)
    Moss_Vec2              Size;                   // Main Area: Size of the viewport.
    Moss_Vec2              FramebufferScale;       // Density of the viewport for Retina display (always 1,1 on Windows, may be 2,2 etc on macOS/iOS). This will affect font rasterizer density.
    Moss_Vec2              WorkPos;                // Work Area: Position of the viewport minus task bars, menus bars, status bars (>= Pos)
    Moss_Vec2              WorkSize;               // Work Area: Size of the viewport minus task bars, menu bars, status bars (<= Size)

    // Platform/Backend Dependent Data
    void*               PlatformHandle;         // void* to hold higher-level, platform window handle (e.g. HWND, GLFWWindow*, SDL_Window*)
    void*               PlatformHandleRaw;      // void* to hold lower-level, platform-native window handle (under Win32 this is expected to be a HWND, unused for other platforms)

    GuiViewport()     { memset(this, 0, sizeof(*this)); }

    // Helpers
    Moss_Vec2              GetCenter() const       { return Moss_Vec2(Pos.x + Size.x * 0.5f, Pos.y + Size.y * 0.5f); }
    Moss_Vec2              GetWorkCenter() const   { return Moss_Vec2(WorkPos.x + WorkSize.x * 0.5f, WorkPos.y + WorkSize.y * 0.5f); }
};