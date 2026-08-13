#include "win32_platform.h"

#include <Xinput.h>
#include <hidsdi.h>
#include <setupapi.h>
#include <cmath>
#include <cstring>
#include <cstdlib>

#pragma comment(lib, "xinput9_1_0.lib")
#pragma comment(lib, "hid.lib")
#pragma comment(lib, "setupapi.lib")

// Helpers:
static float Moss_ApplyDeadzone(float value, float deadzone) {
    if (std::fabs(value) <= deadzone) return 0.0f;
    const float sign = value < 0.0f ? -1.0f : 1.0f;
    return sign * ((std::fabs(value) - deadzone) / (1.0f - deadzone));
}
static float Moss_NormalizeStick(SHORT value) {
    return value < 0 ? (float)value / 32768.0f : (float)value / 32767.0f;
}
static bool TriggerPressed(float value) { return value >= g_trigger_button_threshold; }
void Moss_SetTriggerButtonThreshold(float threshold) { g_trigger_button_threshold = threshold; }
static bool Moss_TriggerPressed(float value) { return value >= g_trigger_button_threshold; }
/////////////////////////////////////

_frame g_frame {};
INPUT_STATE io;

KeyState g_keyboardState[static_cast<size_t>(Keyboard::COUNT)];
KeyState* keyboardState = g_keyboardState;        // optional pointer

static int VirtualMouseButtonMap[static_cast<int>(Mouse::COUNT)] = {
    VK_LBUTTON, VK_RBUTTON, VK_MBUTTON, VK_XBUTTON1, VK_XBUTTON2, 0, 0, 0
};

/*--------------------------  key maps  --------------------------*/
const uint16_t VirtualKeyMap[static_cast<int>(Keyboard::COUNT)] = {
    /* 0–9 */
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',

    /* A–Z */
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',
    'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T',
    'U', 'V', 'W', 'X', 'Y', 'Z',

    /* Punctuation / Symbols */
    VK_OEM_7,      // Apostrophe
    VK_OEM_5,      // Backslash
    VK_OEM_COMMA,  // Comma
    VK_OEM_PLUS,   // Equal
    VK_OEM_3,      // Grave accent
    VK_OEM_4,      // Left bracket
    VK_OEM_MINUS,  // Minus
    VK_OEM_PERIOD, // Period
    VK_OEM_6,      // Right bracket
    VK_OEM_1,      // Semicolon
    VK_OEM_2,      // Slash
    0,             // World 2 (no standard VK code)

    /* Control keys */
    VK_BACK, VK_DELETE, VK_END, VK_RETURN, VK_ESCAPE, VK_HOME,
    VK_INSERT, VK_APPS, VK_NEXT, VK_PRIOR, VK_PAUSE,
    VK_SPACE, VK_TAB, VK_CAPITAL, VK_NUMLOCK, VK_SCROLL,

    /* Function keys */
    VK_F1, VK_F2, VK_F3, VK_F4, VK_F5, VK_F6,
    VK_F7, VK_F8, VK_F9, VK_F10, VK_F11, VK_F12,
    VK_F13, VK_F14, VK_F15, VK_F16, VK_F17, VK_F18,
    VK_F19, VK_F20, VK_F21, VK_F22, VK_F23, VK_F24,

    /* Modifiers */
    VK_LMENU, VK_LCONTROL, VK_LSHIFT, VK_LWIN,
    VK_SNAPSHOT, VK_RMENU, VK_RCONTROL, VK_RSHIFT, VK_RWIN,

    /* Arrows */
    VK_DOWN, VK_LEFT, VK_RIGHT, VK_UP,

    /* Numpad */
    VK_NUMPAD0, VK_NUMPAD1, VK_NUMPAD2, VK_NUMPAD3, VK_NUMPAD4,
    VK_NUMPAD5, VK_NUMPAD6, VK_NUMPAD7, VK_NUMPAD8, VK_NUMPAD9,
    VK_ADD, VK_DECIMAL, VK_DIVIDE, VK_RETURN, VK_OEM_PLUS, VK_MULTIPLY, VK_SUBTRACT
};
static const uint8_t g_vk_mouse[Mouse::COUNT] = { VK_LBUTTON, VK_RBUTTON, VK_MBUTTON, VK_XBUTTON1, VK_XBUTTON2, 0,0,0 };


static const Gamepad g_moss_to_gamepad_button[Moss_GamepadButton::COUNT] =
{
    /* INVALID */        Gamepad::GAMEPAD_BUTTON_LAST,

    /* SOUTH */          Gamepad::GAMEPAD_BUTTON_A,
    /* EAST */           Gamepad::GAMEPAD_BUTTON_B,
    /* WEST */           Gamepad::GAMEPAD_BUTTON_X,
    /* NORTH */          Gamepad::GAMEPAD_BUTTON_Y,

    /* BACK */           Gamepad::GAMEPAD_BUTTON_BACK,
    /* GUIDE */          Gamepad::GAMEPAD_BUTTON_GUIDE,
    /* START */          Gamepad::GAMEPAD_BUTTON_START,

    /* LEFT_STICK */     Gamepad::GAMEPAD_BUTTON_LEFT_THUMB,
    /* RIGHT_STICK */    Gamepad::GAMEPAD_BUTTON_RIGHT_THUMB,

    /* LEFT_SHOULDER */  Gamepad::GAMEPAD_BUTTON_LEFT_BUMPER,
    /* RIGHT_SHOULDER */ Gamepad::GAMEPAD_BUTTON_RIGHT_BUMPER,

    /* DPAD_UP */        Gamepad::GAMEPAD_BUTTON_DPAD_UP,
    /* DPAD_DOWN */      Gamepad::GAMEPAD_BUTTON_DPAD_DOWN,
    /* DPAD_LEFT */      Gamepad::GAMEPAD_BUTTON_DPAD_LEFT,
    /* DPAD_RIGHT */     Gamepad::GAMEPAD_BUTTON_DPAD_RIGHT,

    /* MISC1 */          Gamepad::GAMEPAD_BUTTON_LAST, // Share (not in XInput)
    /* RIGHT_PADDLE1 */  Gamepad::GAMEPAD_BUTTON_LAST,
    /* LEFT_PADDLE1 */   Gamepad::GAMEPAD_BUTTON_LAST,
    /* RIGHT_PADDLE2 */  Gamepad::GAMEPAD_BUTTON_LAST,
    /* LEFT_PADDLE2 */   Gamepad::GAMEPAD_BUTTON_LAST,
    /* TOUCHPAD */       Gamepad::GAMEPAD_BUTTON_LAST,
    /* MISC2 */          Gamepad::GAMEPAD_BUTTON_LAST,
    /* MISC3 */          Gamepad::GAMEPAD_BUTTON_LAST,
    /* MISC4 */          Gamepad::GAMEPAD_BUTTON_LAST,
    /* MISC5 */          Gamepad::GAMEPAD_BUTTON_LAST,
    /* MISC6 */          Gamepad::GAMEPAD_BUTTON_LAST,
};

static const Joystick g_moss_to_gamepad_axis[Moss_GamepadAxis::COUNT] =
{
    /* INVALID */        Joystick::GAMEPAD_AXIS_LEFT_X,

    /* LEFT_X */         Joystick::GAMEPAD_AXIS_LEFT_X,
    /* LEFT_Y */         Joystick::GAMEPAD_AXIS_LEFT_Y,
    /* RIGHT_X */        Joystick::GAMEPAD_AXIS_RIGHT_X,
    /* RIGHT_Y */        Joystick::GAMEPAD_AXIS_RIGHT_Y,
    /* LEFT_TRIGGER */   Joystick::GAMEPAD_AXIS_LEFT_TRIGGER,
    /* RIGHT_TRIGGER */  Joystick::GAMEPAD_AXIS_RIGHT_TRIGGER,
};

struct Moss_GamepadAxisConfig {
    float deadzone;
    bool invert;
};

static Moss_GamepadAxisConfig g_axis_config[static_cast<size_t>(Moss_GamepadAxis::COUNT)] = {
    /* LEFT_X */        { 0.15f, false },
    /* LEFT_Y */        { 0.15f, true  }, // Y inverted by default (common)
    /* RIGHT_X */       { 0.15f, false },
    /* RIGHT_Y */       { 0.15f, true  },
    /* LEFT_TRIGGER */  { 0.05f, false },
    /* RIGHT_TRIGGER */ { 0.05f, false },
};

/*-------------------  RawInput registration  --------------------*/
static BOOL g_raw_mouse_supported = FALSE;

/*-------------------  Global scratch  ---------------------------*/

/*-----------------------  Low level poll  -----------------------*/
void PollGamepads(INPUT_STATE* io) {
    for (DWORD i = 0; i < 4; ++i) {
        GamepadState& pad = io->pads[i];
        XINPUT_STATE s; 
        ZeroMemory(&s, sizeof s);

        if (XInputGetState(i, &s) == ERROR_SUCCESS) {
            pad.connected = true;
            const WORD b = s.Gamepad.wButtons;

            #define BTN(id, flag) pad.buttons[static_cast<size_t>(id)] = ((b & (flag)) ? 1 : 0)

            BTN(Gamepad::GAMEPAD_BUTTON_A,            XINPUT_GAMEPAD_A);
            BTN(Gamepad::GAMEPAD_BUTTON_B,            XINPUT_GAMEPAD_B);
            BTN(Gamepad::GAMEPAD_BUTTON_X,            XINPUT_GAMEPAD_X);
            BTN(Gamepad::GAMEPAD_BUTTON_Y,            XINPUT_GAMEPAD_Y);
            BTN(Gamepad::GAMEPAD_BUTTON_LEFT_BUMPER,  XINPUT_GAMEPAD_LEFT_SHOULDER);
            BTN(Gamepad::GAMEPAD_BUTTON_RIGHT_BUMPER, XINPUT_GAMEPAD_RIGHT_SHOULDER);
            BTN(Gamepad::GAMEPAD_BUTTON_BACK,         XINPUT_GAMEPAD_BACK);
            BTN(Gamepad::GAMEPAD_BUTTON_START,        XINPUT_GAMEPAD_START);
            BTN(Gamepad::GAMEPAD_BUTTON_LEFT_THUMB,   XINPUT_GAMEPAD_LEFT_THUMB);
            BTN(Gamepad::GAMEPAD_BUTTON_RIGHT_THUMB,  XINPUT_GAMEPAD_RIGHT_THUMB);
            BTN(Gamepad::GAMEPAD_BUTTON_DPAD_UP,      XINPUT_GAMEPAD_DPAD_UP);
            BTN(Gamepad::GAMEPAD_BUTTON_DPAD_RIGHT,   XINPUT_GAMEPAD_DPAD_RIGHT);
            BTN(Gamepad::GAMEPAD_BUTTON_DPAD_DOWN,    XINPUT_GAMEPAD_DPAD_DOWN);
            BTN(Gamepad::GAMEPAD_BUTTON_DPAD_LEFT,    XINPUT_GAMEPAD_DPAD_LEFT);

            #undef BTN

            // Normalize sticks [-1, 1]
            pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_LEFT_X)]  = s.Gamepad.sThumbLX / 32767.0f;
            pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_LEFT_Y)]  = s.Gamepad.sThumbLY / 32767.0f;
            pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_RIGHT_X)] = s.Gamepad.sThumbRX / 32767.0f;
            pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_RIGHT_Y)] = s.Gamepad.sThumbRY / 32767.0f;

            // Normalize triggers [0, 1]
            pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_LEFT_TRIGGER)]  = s.Gamepad.bLeftTrigger  / 255.0f;
            pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_RIGHT_TRIGGER)] = s.Gamepad.bRightTrigger / 255.0f;
        } 
        else {
            pad.connected = false;
            memset(pad.buttons, 0, sizeof(pad.buttons));
            memset(pad.axes, 0, sizeof(pad.axes));
        }
    }
}

/*-----------------------  Public facade  ------------------------*/
void Input_Poll(INPUT_STATE* state) {
    if (!state) return;

    for (int i = 0; i < static_cast<int>(Keyboard::COUNT); ++i) {
        state->keys_prev[i] = state->keys[i];
        const bool down = (VirtualKeyMap[i] != 0) && ((GetAsyncKeyState(VirtualKeyMap[i]) & 0x8000) != 0);
        state->keys[i] = down ? 1 : 0;
        g_keyboardState[i].justPressed = down && !g_keyboardState[i].pressed;
        g_keyboardState[i].justReleased = !down && g_keyboardState[i].pressed;
        g_keyboardState[i].pressed = down;
    }

    POINT point = {};
    if (GetCursorPos(&point)) {
        if (handle) ScreenToClient(handle, &point);
        state->mouse_dx = point.x - state->mouse_x;
        state->mouse_dy = point.y - state->mouse_y;
        state->mouse_x = point.x;
        state->mouse_y = point.y;
    }

    for (int i = 0; i < static_cast<int>(Mouse::COUNT); ++i) {
        state->mouse_buttons_prev[i] = state->mouse_buttons[i];
        state->mouse_buttons[i] = VirtualMouseButtonMap[i] && (GetAsyncKeyState(VirtualMouseButtonMap[i]) & 0x8000) ? 1 : 0;
    }

    Moss_UpdateGamepads();
}

void Input_Poll(INPUT_STATE* io)
{
    for (int i = 0; i < static_cast<int>(Keyboard::COUNT); i++) {
        io->keys_prev[i] = io->keys[i];
    }
    PollGamepads(io);
}


////////////////////////////////////////////////////////////////
inline bool IsPressed(size_t padIndex, Gamepad b) { return io.pads[padIndex].buttons[static_cast<size_t>(b)] != 0; }
inline bool IsReleased(size_t padIndex, Gamepad b) { return io.pads[padIndex].buttons[static_cast<size_t>(b)] == 0; }
inline bool IsJustPressed(size_t padIndex, Gamepad b) { return false; }
inline float GetAxis(size_t padIndex, Joystick a) { return io.pads[padIndex].axes[static_cast<size_t>(a)]; }
////////////////////////////////////////////////////////////////


bool Moss_IsKeyPressed(Moss_Keyboard key) { return io.keys[static_cast<size_t>(k)] != 0; }
bool Moss_IsReleased(Moss_Keyboard k) { return io.keys[static_cast<size_t>(k)] == 0; }
bool Moss_IsKeyJustPressed(Moss_Keyboard key) { return io.keys[static_cast<size_t>(k)] && !io.keys_prev[static_cast<size_t>(k)]; }
bool Moss_IsKeyJustReleased(Moss_Keyboard key)  { size_t i = static_cast<size_t>(k); return !io.keys[i] && io.keys_prev[i]; }
Moss_Keyboard Moss_InputGetKey() { size_t i = static_cast<size_t>(k); return !io.keys[i]; }


//bool Moss_IsKeyJustPressed(Moss_Keyboard key) { return io.keys[static_cast<size_t>(key)] && !io.keys_prev[static_cast<size_t>(key)]; }
//bool Moss_IsKeyJustReleased(Moss_Keyboard key) { return !io.keys[static_cast<size_t>(key)] && io.keys_prev[static_cast<size_t>(key)]; }
Moss_Keyboard Moss_InputGetKey() { for (int i = 0; i < static_cast<int>(Keyboard::COUNT); ++i) if (io.keys[i]) return static_cast<Moss_Keyboard>(i); return Moss_Keyboard::COUNT; }

bool Moss_IsMousePressed(Moss_MouseButton button) { return io.mouse_buttons[static_cast<size_t>(button)] != 0; }
bool Moss_IsMouseReleased(Moss_MouseButton button) { return io.mouse_buttons[static_cast<size_t>(button)] == 0; }
bool Moss_IsMouseJustPressed(Moss_MouseButton button) { size_t i = static_cast<size_t>(button); return io.mouse_buttons[i] && !io.mouse_buttons_prev[i]; }
bool Moss_IsMouseJustReleased(Moss_MouseButton button) { size_t i = static_cast<size_t>(button); return !io.mouse_buttons[i] && io.mouse_buttons_prev[i]; }
Moss_MouseButton Moss_InputGetMouseButton() { for (int i = 0; i < static_cast<int>(Mouse::COUNT); ++i) if (io.mouse_buttons[i]) return static_cast<Moss_MouseButton>(i); return Moss_MouseButton::COUNT; }

void Moss_GetMousePosition(int* x, int* y) { if (x) *x = io.mouse_x; if (y) *y = io.mouse_y; }
void Moss_SetMousePosition(int x, int y) { POINT p = { x, y }; if (handle) ClientToScreen(handle, &p); SetCursorPos(p.x, p.y); }
void Moss_SetMouseVisible(bool visible) { ShowCursor(visible ? TRUE : FALSE); }




static void Moss_UpdateSingleGamepad(DWORD index) {
    GamepadState& pad = io.pads[index];
    std::memcpy(pad.buttons_prev, pad.buttons, sizeof(pad.buttons));

    if (g_gamepads[index].backend == Moss_GamepadBackend::HID && g_gamepads[index].backend_handle) {
        Moss_UpdateHIDGamepad(&g_gamepads[index]);
        return;
    }

    XINPUT_STATE state;
    ZeroMemory(&state, sizeof(state));

    if (XInputGetState(index, &state) != ERROR_SUCCESS) {
        pad.connected = false;
        std::memset(pad.buttons, 0, sizeof(pad.buttons));
        std::memset(pad.axes, 0, sizeof(pad.axes));
        g_gamepads[index].connected = false;
        return;
    }

    pad.connected = true;
    g_gamepads[index].index = index;
    g_gamepads[index].backend = Moss_GamepadBackend::XINPUT;
    g_gamepads[index].connected = true;
    g_gamepads[index].name = "XInput Gamepad";
    g_gamepads[index].type = Moss_GamepadType::STANDARD;

    const WORD b = state.Gamepad.wButtons;
    pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_A)] = (b & XINPUT_GAMEPAD_A) != 0;
    pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_B)] = (b & XINPUT_GAMEPAD_B) != 0;
    pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_X)] = (b & XINPUT_GAMEPAD_X) != 0;
    pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_Y)] = (b & XINPUT_GAMEPAD_Y) != 0;
    pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_LEFT_BUMPER)] = (b & XINPUT_GAMEPAD_LEFT_SHOULDER) != 0;
    pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_RIGHT_BUMPER)] = (b & XINPUT_GAMEPAD_RIGHT_SHOULDER) != 0;
    pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_BACK)] = (b & XINPUT_GAMEPAD_BACK) != 0;
    pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_START)] = (b & XINPUT_GAMEPAD_START) != 0;
    pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_LEFT_THUMB)] = (b & XINPUT_GAMEPAD_LEFT_THUMB) != 0;
    pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_RIGHT_THUMB)] = (b & XINPUT_GAMEPAD_RIGHT_THUMB) != 0;
    pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_DPAD_UP)] = (b & XINPUT_GAMEPAD_DPAD_UP) != 0;
    pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_DPAD_RIGHT)] = (b & XINPUT_GAMEPAD_DPAD_RIGHT) != 0;
    pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_DPAD_DOWN)] = (b & XINPUT_GAMEPAD_DPAD_DOWN) != 0;
    pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_DPAD_LEFT)] = (b & XINPUT_GAMEPAD_DPAD_LEFT) != 0;

    pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_LEFT_X)] = Moss_NormalizeStick(state.Gamepad.sThumbLX);
    pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_LEFT_Y)] = Moss_NormalizeStick(state.Gamepad.sThumbLY);
    pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_RIGHT_X)] = Moss_NormalizeStick(state.Gamepad.sThumbRX);
    pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_RIGHT_Y)] = Moss_NormalizeStick(state.Gamepad.sThumbRY);
    pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_LEFT_TRIGGER)] = state.Gamepad.bLeftTrigger / 255.0f;
    pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_RIGHT_TRIGGER)] = state.Gamepad.bRightTrigger / 255.0f;
}

// Gamepad management
static void Moss_OpenSonyHIDGamepads(void) {
    GUID hid_guid;
    HidD_GetHidGuid(&hid_guid);

    HDEVINFO dev_info = SetupDiGetClassDevsA(&hid_guid, nullptr, nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (dev_info == INVALID_HANDLE_VALUE) return;

    for (DWORD device_index = 0;; ++device_index) {
        SP_DEVICE_INTERFACE_DATA iface = {};
        iface.cbSize = sizeof(iface);
        if (!SetupDiEnumDeviceInterfaces(dev_info, nullptr, &hid_guid, device_index, &iface)) break;

        DWORD required_size = 0;
        SetupDiGetDeviceInterfaceDetailA(dev_info, &iface, nullptr, 0, &required_size, nullptr);
        if (required_size == 0) continue;

        PSP_DEVICE_INTERFACE_DETAIL_DATA_A detail = (PSP_DEVICE_INTERFACE_DETAIL_DATA_A)std::malloc(required_size);
        if (!detail) continue;
        detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);

        if (!SetupDiGetDeviceInterfaceDetailA(dev_info, &iface, detail, required_size, nullptr, nullptr)) {
            std::free(detail);
            continue;
        }

        if (Moss_HIDPathAlreadyOpen(detail->DevicePath)) {
            std::free(detail);
            continue;
        }

        HANDLE device = CreateFileA(detail->DevicePath, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
        if (device == INVALID_HANDLE_VALUE) {
            device = CreateFileA(detail->DevicePath, GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
        }
        if (device == INVALID_HANDLE_VALUE) {
            std::free(detail);
            continue;
        }

        HIDD_ATTRIBUTES attributes = {};
        attributes.Size = sizeof(attributes);
        if (!HidD_GetAttributes(device, &attributes)) {
            CloseHandle(device);
            std::free(detail);
            continue;
        }

        Moss_GamepadType type = Moss_SonyGamepadType(attributes.VendorID, attributes.ProductID);
        if (type == Moss_GamepadType::UNKNOWN) {
            CloseHandle(device);
            std::free(detail);
            continue;
        }

        const int slot = Moss_FindFreeHIDSlot();
        if (slot < 0) {
            CloseHandle(device);
            std::free(detail);
            break;
        }

        Moss_WinHIDGamepadHandle* hid = (Moss_WinHIDGamepadHandle*)std::calloc(1, sizeof(Moss_WinHIDGamepadHandle));
        if (!hid) { CloseHandle(device); std::free(detail); continue; }
        hid->device = device;
        hid->read_overlapped.hEvent = CreateEventA(nullptr, TRUE, FALSE, nullptr);
        if (!hid->read_overlapped.hEvent) { CloseHandle(device); std::free(hid); std::free(detail); continue; }

        Moss_Gamepad& gp = g_gamepads[slot];
        gp.index = (uint32_t)slot;
        gp.backend = Moss_GamepadBackend::HID;
        gp.connected = true;
        gp.backend_handle = hid;
        gp.name = type == Moss_GamepadType::PS5 ? "DualSense Wireless Controller" : "DualShock 4 Wireless Controller";
        gp.type = type;
        io.pads[slot].connected = true;
        strncpy(g_hid_paths[slot], detail->DevicePath, sizeof(g_hid_paths[slot]) - 1);
        g_hid_paths[slot][sizeof(g_hid_paths[slot]) - 1] = '\0';

        std::free(detail);
    }

    SetupDiDestroyDeviceInfoList(dev_info);
}

void Moss_UpdateGamepads(void) { for (DWORD i = 0; i < XUSER_MAX_COUNT; ++i) Moss_UpdateSingleGamepad(i); Moss_OpenSonyHIDGamepads(); }
int Moss_GetNumGamepads(void) { Moss_UpdateGamepads(); int count = 0; for (int i = 0; i < XUSER_MAX_COUNT; ++i) if (io.pads[i].connected) ++count; return count; }
Moss_Gamepad* Moss_OpenGamepad(Moss_GamepadID id) { if (id >= XUSER_MAX_COUNT) return nullptr; Moss_UpdateSingleGamepad(id); return io.pads[id].connected ? &g_gamepads[id] : nullptr; }
void Moss_CloseGamepad(Moss_Gamepad* gp) { if (!gp) return; if (gp->backend == Moss_GamepadBackend::HID && gp->backend_handle) { Moss_CloseWinHIDHandle(gp); g_hid_paths[gp->index][0] = '\0'; } gp->connected = false; io.pads[gp->index].connected = false; }
bool Moss_GamepadConnected(Moss_Gamepad* gp) { if (!gp || gp->index >= XUSER_MAX_COUNT) return false; Moss_UpdateSingleGamepad(gp->index); return io.pads[gp->index].connected; }

// Button & axis
bool Moss_IsGamepadButtonPressed(Moss_Gamepad* gp, Moss_GamepadButton button) {
    if (!gp || !gp->connected || static_cast<int>(button) < 0) return false;
    if (button == Moss_GamepadButton::LEFT_TRIGGER) return Moss_TriggerPressed(Moss_GetGamepadAxis(gp, Moss_GamepadAxis::LEFT_TRIGGER));
    if (button == Moss_GamepadButton::RIGHT_TRIGGER) return Moss_TriggerPressed(Moss_GetGamepadAxis(gp, Moss_GamepadAxis::RIGHT_TRIGGER));
    const Gamepad raw = g_moss_to_raw_button[static_cast<size_t>(button)];
    if (raw == Gamepad::GAMEPAD_BUTTON_LAST) return false;
    return io.pads[gp->index].buttons[static_cast<size_t>(raw)] != 0;
}

bool Moss_IsGamepadButtonJustPressed(Moss_Gamepad* gp, Moss_GamepadButton button) {
    if (!gp || !gp->connected || static_cast<int>(button) < 0) return false;
    const Gamepad raw = g_moss_to_raw_button[static_cast<size_t>(button)];
    if (raw == Gamepad::GAMEPAD_BUTTON_LAST) return false;
    const size_t i = static_cast<size_t>(raw);
    return io.pads[gp->index].buttons[i] && !io.pads[gp->index].buttons_prev[i];
}

bool Moss_IsGamepadButtonJustReleased(Moss_Gamepad* gp, Moss_GamepadButton button) {
    if (!gp || !gp->connected || static_cast<int>(button) < 0) return false;
    const Gamepad raw = g_moss_to_raw_button[static_cast<size_t>(button)];
    if (raw == Gamepad::GAMEPAD_BUTTON_LAST) return false;
    const size_t i = static_cast<size_t>(raw);
    return !io.pads[gp->index].buttons[i] && io.pads[gp->index].buttons_prev[i];
}

float Moss_GetGamepadAxis(Moss_Gamepad* gp, Moss_GamepadAxis axis) {
    if (!gp || !gp->connected || static_cast<int>(axis) < 0) return 0.0f;
    const size_t i = static_cast<size_t>(axis);
    float value = io.pads[gp->index].axes[static_cast<size_t>(g_moss_to_raw_axis[i])];
    value = Moss_ApplyDeadzone(value, g_axis_config[i].deadzone);
    return g_axis_config[i].invert ? -value : value;
}

void Moss_SetGamepadAxisDeadzone(Moss_GamepadAxis axis, float dz) { g_axis_config[(size_t)axis].deadzone = dz; }
void Moss_SetGamepadAxisInverted(Moss_GamepadAxis axis, bool inverted) { g_axis_config[(size_t)axis].invert = inverted; }


// Rumble / LED
bool Moss_RumbleGamepad(Moss_Gamepad* gp, uint16_t low, uint16_t high, uint32_t duration_ms) {
    (void)duration_ms;
    if (!gp) return false;
    if (gp->backend == Moss_GamepadBackend::HID && gp->backend_handle) {
        uint8_t report[64];
        if (gp->type == Moss_GamepadType::PS4) { size_t report_size = Moss_BuildDS4OutputReport(report, sizeof(report), (uint8_t)(high >> 8), (uint8_t)(low >> 8), 0, 0, 255); return Moss_SendHIDReport(gp, report, report_size); }
        if (gp->type == Moss_GamepadType::PS5) { size_t report_size = Moss_BuildDS5OutputReport(report, sizeof(report), (uint8_t)(high >> 8), (uint8_t)(low >> 8), 0, 0, 255, 0, 0); return Moss_SendHIDReport(gp, report, report_size); }
    }
    if (gp->backend != Moss_GamepadBackend::XINPUT) return false;
    XINPUT_VIBRATION vib = {};
    vib.wLeftMotorSpeed = low;
    vib.wRightMotorSpeed = high;
    return XInputSetState(gp->index, &vib) == ERROR_SUCCESS;
}

bool Moss_GamepadRumble(Moss_Gamepad* gp, float low_frequency, float high_frequency, uint32_t duration_ms) {
    const float low = low_frequency < 0.0f ? 0.0f : (low_frequency > 1.0f ? 1.0f : low_frequency);
    const float high = high_frequency < 0.0f ? 0.0f : (high_frequency > 1.0f ? 1.0f : high_frequency);
    return Moss_RumbleGamepad(gp, (uint16_t)(low * 65535.0f), (uint16_t)(high * 65535.0f), duration_ms);
}

bool Moss_RumbleGamepadTriggers(Moss_Gamepad* gp, uint16_t left, uint16_t right, uint32_t duration_ms) { (void)duration_ms; if (!gp || gp->backend != Moss_GamepadBackend::HID || !gp->backend_handle || gp->type != Moss_GamepadType::PS5) return false; uint8_t report[64]; size_t report_size = Moss_BuildDS5OutputReport(report, sizeof(report), 0, 0, 0, 0, 255, (uint8_t)(left >> 8), (uint8_t)(right >> 8)); return Moss_SendHIDReport(gp, report, report_size); }
bool Moss_SetGamepadLED(Moss_Gamepad* gp, uint8_t r, uint8_t g, uint8_t b) { if (!gp || gp->backend != Moss_GamepadBackend::HID || !gp->backend_handle) return false; uint8_t report[64]; if (gp->type == Moss_GamepadType::PS4) { size_t report_size = Moss_BuildDS4OutputReport(report, sizeof(report), 0, 0, r, g, b); return Moss_SendHIDReport(gp, report, report_size); } if (gp->type == Moss_GamepadType::PS5) { size_t report_size = Moss_BuildDS5OutputReport(report, sizeof(report), 0, 0, r, g, b, 0, 0); return Moss_SendHIDReport(gp, report, report_size); } return false; }

void Moss_SetGamepadAxisDeadzone(Moss_GamepadAxis axis, float dz) { if (static_cast<int>(axis) >= 0) g_axis_config[static_cast<size_t>(axis)].deadzone = dz; }
void Moss_SetGamepadAxisInverted(Moss_GamepadAxis axis, bool inverted) { if (static_cast<int>(axis) >= 0) g_axis_config[static_cast<size_t>(axis)].invert = inverted; }

// Metadata
const char* Moss_GetGamepadName(Moss_Gamepad* gp) { return gp && gp->name ? gp->name : "Unknown Gamepad"; }
Moss_GamepadID Moss_GetGamepadID(Moss_Gamepad* gp) { return gp ? gp->index : UINT32_MAX; }
int Moss_GetGamepadPlayerIndex(Moss_Gamepad* gp) { return gp ? (int)gp->index : -1; }
Moss_PowerState Moss_GetGamepadPowerInfo(Moss_Gamepad* gp, int* percent) {
    if (percent) *percent = -1;

    if (!gp || gp->backend != MOSS_GAMEPAD_BACKEND_XINPUT)
        return MOSS_POWERSTATE_UNKNOWN;

    XINPUT_BATTERY_INFORMATION bat;
    if (XInputGetBatteryInformation(gp->index, XINPUT_BATTERY_DEVTYPE_GAMEPAD, &bat) != ERROR_SUCCESS)
        return MOSS_POWERSTATE_UNKNOWN;

    if (percent) {
        static const int map[] = { 0, 25, 50, 75, 100 };
        *percent = map[bat.BatteryLevel <= BATTERY_LEVEL_FULL ? bat.BatteryLevel : 0];
    }
    return bat.BatteryType == BATTERY_TYPE_WIRED ? Moss_PowerState::CHARGED : Moss_PowerState::ON_BATTERY;
}



int Moss_GetNumGamepadTouchpads(Moss_Gamepad* gp) { (void)gp; return 0; }
int Moss_GetNumGamepadTouchpadFingers(Moss_Gamepad* gp) { (void)gp; return 0; }
bool Moss_GetGamepadTouchpadFinger(Moss_Gamepad* gp, int pad, int finger, bool* down, float* x, float* y, float* pressure) { (void)gp; (void)pad; (void)finger; if (down) *down = false; if (x) *x = 0.0f; if (y) *y = 0.0f; if (pressure) *pressure = 0.0f; return false; }
// Mapping & type
const char* Moss_GetGamepadMapping(Moss_Gamepad* gp) { (void)gp; return "xinput"; }
bool Moss_SetGamepadMapping(Moss_Gamepad* gp, const char* mapping) { (void)gp; (void)mapping; return false; }
void Moss_ReloadGamepadMappings(void) {}
Moss_GamepadButton Moss_InputGetGamepadButton() { return Moss_GamepadButton::INVALID; }
Moss_GamepadAxis Moss_InputGetGamepadAxis() { return Moss_GamepadAxis::INVALID; }

// Pen and Fingers
Moss_PenDeviceType Moss_GetPenDeviceType(Moss_PenID instance_id) { (void)instance_id; return Moss_PenDeviceType::UNKNOWN; }
const char* Moss_GetTouchDeviceName(Moss_TouchID touchID) { (void)touchID; return "Windows Touch Device"; }
Moss_TouchID* Moss_GetTouchDevices(int* count) { static Moss_TouchID ids[1] = { 0 }; if (count) *count = 1; return ids; }
Moss_TouchDeviceType Moss_GetTouchDeviceType(Moss_TouchID touchID) { (void)touchID; return Moss_TouchDeviceType::UNKNOWN; }
Moss_Finger** Moss_GetTouchFingers(Moss_TouchID touchID, int* count) { (void)touchID; if (count) *count = 0; return nullptr; }


/*
INPUT_STATE io;
KeyState g_keyboardState[static_cast<size_t>(Keyboard::COUNT)] = {};
KeyState* keyboardState = g_keyboardState;

static Moss_Gamepad g_gamepads[XUSER_MAX_COUNT] = {};
static char g_hid_paths[XUSER_MAX_COUNT][512] = {};

struct Moss_WinHIDGamepadHandle {
    HANDLE device;
    OVERLAPPED read_overlapped;
    bool read_pending;
    uint8_t input_report[128];
};
static float g_trigger_button_threshold = 0.5f;

static const Gamepad g_moss_to_raw_button[static_cast<size_t>(Moss_GamepadButton::COUNT)] = {
    Gamepad::GAMEPAD_BUTTON_A,
    Gamepad::GAMEPAD_BUTTON_B,
    Gamepad::GAMEPAD_BUTTON_X,
    Gamepad::GAMEPAD_BUTTON_Y,
    Gamepad::GAMEPAD_BUTTON_BACK,
    Gamepad::GAMEPAD_BUTTON_GUIDE,
    Gamepad::GAMEPAD_BUTTON_START,
    Gamepad::GAMEPAD_BUTTON_LEFT_THUMB,
    Gamepad::GAMEPAD_BUTTON_RIGHT_THUMB,
    Gamepad::GAMEPAD_BUTTON_LEFT_BUMPER,
    Gamepad::GAMEPAD_BUTTON_RIGHT_BUMPER,
    Gamepad::GAMEPAD_BUTTON_DPAD_UP,
    Gamepad::GAMEPAD_BUTTON_DPAD_DOWN,
    Gamepad::GAMEPAD_BUTTON_DPAD_LEFT,
    Gamepad::GAMEPAD_BUTTON_DPAD_RIGHT,
    Gamepad::GAMEPAD_BUTTON_LAST,
    Gamepad::GAMEPAD_BUTTON_LAST,
    Gamepad::GAMEPAD_BUTTON_LAST,
    Gamepad::GAMEPAD_BUTTON_LAST,
    Gamepad::GAMEPAD_BUTTON_LAST,
    Gamepad::GAMEPAD_BUTTON_LAST,
    Gamepad::GAMEPAD_BUTTON_LAST,
    Gamepad::GAMEPAD_BUTTON_LAST,
    Gamepad::GAMEPAD_BUTTON_LAST,
    Gamepad::GAMEPAD_BUTTON_LAST,
    Gamepad::GAMEPAD_BUTTON_LAST,
    Gamepad::GAMEPAD_BUTTON_LAST,
    Gamepad::GAMEPAD_BUTTON_LAST,
    Gamepad::GAMEPAD_BUTTON_LAST
};

static const Joystick g_moss_to_raw_axis[static_cast<size_t>(Moss_GamepadAxis::COUNT)] = {
    Joystick::GAMEPAD_AXIS_LEFT_X,
    Joystick::GAMEPAD_AXIS_LEFT_Y,
    Joystick::GAMEPAD_AXIS_RIGHT_X,
    Joystick::GAMEPAD_AXIS_RIGHT_Y,
    Joystick::GAMEPAD_AXIS_LEFT_TRIGGER,
    Joystick::GAMEPAD_AXIS_RIGHT_TRIGGER
};

static Moss_GamepadType Moss_SonyGamepadType(USHORT vendor_id, USHORT product_id) {
    if (vendor_id != 0x054C) return Moss_GamepadType::UNKNOWN;
    switch (product_id) {
    case 0x05C4:
    case 0x09CC:
        return Moss_GamepadType::PS4;
    case 0x0CE6:
        return Moss_GamepadType::PS5;
    default:
        return Moss_GamepadType::UNKNOWN;
    }
}

static int Moss_FindFreeHIDSlot(void) {
    for (DWORD i = 0; i < XUSER_MAX_COUNT; ++i) {
        if (!g_gamepads[i].connected && !io.pads[i].connected) return (int)i;
    }
    return -1;
}

static Moss_WinHIDGamepadHandle* Moss_WinHIDHandle(Moss_Gamepad* gp) {
    return gp ? (Moss_WinHIDGamepadHandle*)gp->backend_handle : nullptr;
}

static void Moss_CloseWinHIDHandle(Moss_Gamepad* gp) {
    Moss_WinHIDGamepadHandle* hid = Moss_WinHIDHandle(gp);
    if (!hid) return;
    if (hid->read_pending) CancelIo(hid->device);
    if (hid->read_overlapped.hEvent) CloseHandle(hid->read_overlapped.hEvent);
    if (hid->device && hid->device != INVALID_HANDLE_VALUE) CloseHandle(hid->device);
    std::free(hid);
    gp->backend_handle = nullptr;
}

static float Moss_NormalizeByteAxis(uint8_t value) {
    return ((float)value - 128.0f) / 127.0f;
}

static void Moss_DecodeSonyHat(GamepadState& pad, uint8_t hat) {
    pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_DPAD_UP)] = hat == 0 || hat == 1 || hat == 7;
    pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_DPAD_RIGHT)] = hat == 1 || hat == 2 || hat == 3;
    pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_DPAD_DOWN)] = hat == 3 || hat == 4 || hat == 5;
    pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_DPAD_LEFT)] = hat == 5 || hat == 6 || hat == 7;
}

static void Moss_DecodeSonyHIDInput(Moss_Gamepad* gp, const uint8_t* report, DWORD report_size) {
    if (!gp || !report || report_size < 10) return;
    GamepadState& pad = io.pads[gp->index];
    std::memcpy(pad.buttons_prev, pad.buttons, sizeof(pad.buttons));
    pad.connected = true;

    if (gp->type == Moss_GamepadType::PS4 && report[0] == 0x01 && report_size >= 10) {
        pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_LEFT_X)] = Moss_NormalizeByteAxis(report[1]);
        pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_LEFT_Y)] = Moss_NormalizeByteAxis(report[2]);
        pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_RIGHT_X)] = Moss_NormalizeByteAxis(report[3]);
        pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_RIGHT_Y)] = Moss_NormalizeByteAxis(report[4]);
        const uint8_t b0 = report[5];
        const uint8_t b1 = report[6];
        const uint8_t b2 = report[7];
        Moss_DecodeSonyHat(pad, b0 & 0x0F);
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_X)] = (b0 & 0x10) != 0;
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_A)] = (b0 & 0x20) != 0;
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_B)] = (b0 & 0x40) != 0;
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_Y)] = (b0 & 0x80) != 0;
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_LEFT_BUMPER)] = (b1 & 0x01) != 0;
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_RIGHT_BUMPER)] = (b1 & 0x02) != 0;
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_BACK)] = (b1 & 0x10) != 0;
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_START)] = (b1 & 0x20) != 0;
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_LEFT_THUMB)] = (b1 & 0x40) != 0;
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_RIGHT_THUMB)] = (b1 & 0x80) != 0;
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_GUIDE)] = (b2 & 0x01) != 0;
        pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_LEFT_TRIGGER)] = report[8] / 255.0f;
        pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_RIGHT_TRIGGER)] = report[9] / 255.0f;
        return;
    }

    if (gp->type == Moss_GamepadType::PS5 && (report[0] == 0x01 || report[0] == 0x31)) {
        const int offset = report[0] == 0x31 ? 1 : 0;
        if (report_size < (DWORD)(11 + offset)) return;
        pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_LEFT_X)] = Moss_NormalizeByteAxis(report[1 + offset]);
        pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_LEFT_Y)] = Moss_NormalizeByteAxis(report[2 + offset]);
        pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_RIGHT_X)] = Moss_NormalizeByteAxis(report[3 + offset]);
        pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_RIGHT_Y)] = Moss_NormalizeByteAxis(report[4 + offset]);
        pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_LEFT_TRIGGER)] = report[5 + offset] / 255.0f;
        pad.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_RIGHT_TRIGGER)] = report[6 + offset] / 255.0f;
        const uint8_t b0 = report[8 + offset];
        const uint8_t b1 = report[9 + offset];
        const uint8_t b2 = report[10 + offset];
        Moss_DecodeSonyHat(pad, b0 & 0x0F);
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_X)] = (b0 & 0x10) != 0;
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_A)] = (b0 & 0x20) != 0;
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_B)] = (b0 & 0x40) != 0;
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_Y)] = (b0 & 0x80) != 0;
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_LEFT_BUMPER)] = (b1 & 0x01) != 0;
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_RIGHT_BUMPER)] = (b1 & 0x02) != 0;
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_BACK)] = (b1 & 0x10) != 0;
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_START)] = (b1 & 0x20) != 0;
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_LEFT_THUMB)] = (b1 & 0x40) != 0;
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_RIGHT_THUMB)] = (b1 & 0x80) != 0;
        pad.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_GUIDE)] = (b2 & 0x01) != 0;
    }
}

static void Moss_DisconnectHIDGamepad(Moss_Gamepad* gp) {
    if (!gp) return;
    Moss_CloseWinHIDHandle(gp);
    io.pads[gp->index].connected = false;
    std::memset(io.pads[gp->index].buttons, 0, sizeof(io.pads[gp->index].buttons));
    std::memset(io.pads[gp->index].axes, 0, sizeof(io.pads[gp->index].axes));
    g_hid_paths[gp->index][0] = '\0';
    gp->connected = false;
}

static void Moss_UpdateHIDGamepad(Moss_Gamepad* gp) {
    Moss_WinHIDGamepadHandle* hid = Moss_WinHIDHandle(gp);
    if (!gp || !hid || !hid->device || hid->device == INVALID_HANDLE_VALUE) return;

    if (hid->read_pending) {
        DWORD bytes = 0;
        if (GetOverlappedResult(hid->device, &hid->read_overlapped, &bytes, FALSE)) {
            hid->read_pending = false;
            Moss_DecodeSonyHIDInput(gp, hid->input_report, bytes);
        } else if (GetLastError() != ERROR_IO_INCOMPLETE) {
            Moss_DisconnectHIDGamepad(gp);
            return;
        }
    }

    if (!hid->read_pending) {
        DWORD bytes = 0;
        std::memset(hid->input_report, 0, sizeof(hid->input_report));
        ResetEvent(hid->read_overlapped.hEvent);
        if (ReadFile(hid->device, hid->input_report, sizeof(hid->input_report), &bytes, &hid->read_overlapped)) {
            Moss_DecodeSonyHIDInput(gp, hid->input_report, bytes);
        } else if (GetLastError() == ERROR_IO_PENDING) {
            hid->read_pending = true;
        } else {
            Moss_DisconnectHIDGamepad(gp);
        }
    }
}
static bool Moss_HIDPathAlreadyOpen(const char* path) {
    if (!path) return false;
    for (DWORD i = 0; i < XUSER_MAX_COUNT; ++i) {
        if (g_hid_paths[i][0] && strcmp(g_hid_paths[i], path) == 0) return true;
    }
    return false;
}
*/