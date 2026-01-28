#include <Moss/Platform/Windows/win32_platform.h>
#include <Xinput.h>

#pragma comment(lib,"xinput9_1_0.lib")

// Helpers:
static float ApplyDeadzone(float v, float dz)
{
    if (fabsf(v) <= dz)
        return 0.0f;

    float sign = (v < 0.0f) ? -1.0f : 1.0f;
    return sign * ((fabsf(v) - dz) / (1.0f - dz));
}
static bool TriggerPressed(float value) { return value >= g_trigger_button_threshold; }
void Moss_SetTriggerButtonThreshold(float t) { g_trigger_button_threshold = t; }

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

static Moss_GamepadAxisConfig g_axis_config[Moss_GamepadAxis::COUNT] = {
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
void PollKeyboard(INPUT_STATE* io) {
    for (int i = 0; i < static_cast<int>(Keyboard::COUNT); ++i) {
        bool isDown = (GetAsyncKeyState(VirtualKeyMap[i]) & 0x8000) != 0;

        //if (isDown) {
            //printf("Key %d is down\n", i);
        //}

        KeyState& state = g_keyboardState[i];
        state.justPressed  = !state.pressed && isDown;
        state.justReleased =  state.pressed && !isDown;
        state.pressed      = isDown;

        io->keys_prev[i] = io->keys[i];
        io->keys[i] = state.pressed ? 1 : 0;
    }
}

void PollMouse(INPUT_STATE* io)
{
    POINT p;                        // position
    if (GetCursorPos(&p)) {         // screen coordinates
        ScreenToClient(handle, &p); // convert to client‑area coords
        io->mouse_dx = p.x - io->mouse_x;
        io->mouse_dy = p.y - io->mouse_y;
        io->mouse_x = p.x;
        io->mouse_y = p.y;
    }

    // Wheel (from the last message pump)
    io->mouse_wheel = g_frame.wheel;
    g_frame.wheel   = 0;

    // Buttons
    for (int i = 0; i < static_cast<int>(Mouse::COUNT); ++i) {
        io->mouse_buttons_prev[i] = io->mouse_buttons[i];
        io->mouse_buttons[i]      = (GetAsyncKeyState(VirtualMouseButtonMap[i]) & 0x8000) ? 1 : 0;
    }
}

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
void Input_Poll(INPUT_STATE* io)
{
    for (int i = 0; i < static_cast<int>(Keyboard::COUNT); i++) {
        io->keys_prev[i] = io->keys[i];
    }
    PollKeyboard(io);
    PollMouse(io);
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
bool Moss_IsKeyJustReleased(Moss_Keyboard key)  { size_t i = static_cast<size_t>(k); return !io.keys[i] &&  io.keys_prev[i]; }
Moss_Keyboard Moss_InputGetKey() { size_t i = static_cast<size_t>(k); return !io.keys[i]; }

bool Moss_IsMousePressed(Mouse button) { return io.mouse_buttons[static_cast<size_t>(b)] != 0; }
bool Moss_IsMouseReleased(Mouse b) { return io.mouse_buttons[static_cast<size_t>(b)] == 0; }
bool Moss_IsMouseJustPressed(Mouse button) { return io.mouse_buttons[static_cast<size_t>(b)] && !io.mouse_buttons_prev[static_cast<size_t>(b)]; }
bool Moss_IsMouseJustReleased(Mouse button) { return !io.mouse_buttons[static_cast<size_t>(b)] &&  io.mouse_buttons_prev[static_cast<size_t>(b)]; }
Moss_Keyboard Moss_InputGetMouseButton() { return !io.mouse_buttons[static_cast<size_t>(b)] }

void Moss_GetMousePosition(int* x, int* y) { if (x) *x = io.mouse_x; if (y) *y = io.mouse_y; }
void Moss_SetMousePosition(int x, int y) { POINT p = { x, y }; ClientToScreen(handle, &p); SetCursorPos(p.x, p.y); }
void Moss_SetMouseVisible(bool visible) { ShowCursor(visible); }


// Gamepad management
int Moss_GetNumGamepads(void) { }
Moss_Gamepad* Moss_OpenGamepad(Moss_GamepadID id) { }
void Moss_CloseGamepad(Moss_Gamepad* gp) { }
bool Moss_GamepadConnected(Moss_Gamepad* gp) { }
void Moss_UpdateGamepads(void) { }

// Button & axis
bool Moss_IsGamepadButtonPressed(Moss_Gamepad* gp, Moss_GamepadButton button) {
    if (!gp) return false;

    // Trigger-as-button support
    if (button == Moss_GamepadButton::LEFT_TRIGGER)
        return TriggerPressed(Moss_GetGamepadAxis(gp, Moss_GamepadAxis::LEFT_TRIGGER));

    if (button == Moss_GamepadButton::RIGHT_TRIGGER)
        return TriggerPressed(Moss_GetGamepadAxis(gp, Moss_GamepadAxis::RIGHT_TRIGGER));

    Gamepad g = g_moss_to_gamepad_button[(size_t)button];
    if (g == Gamepad::GAMEPAD_BUTTON_LAST)
        return false;

    return io.pads[gp->index].buttons[(size_t)g] != 0;
}

bool Moss_IsGamepadButtonJustPressed(Moss_Gamepad* gp, Moss_GamepadButton button) {
    if (!gp || button <= Moss_GamepadButton::INVALID)
        return false;

    Gamepad g = g_moss_to_gamepad_button[(size_t)button];
    if (g == Gamepad::GAMEPAD_BUTTON_LAST)
        return false;

    GamepadState& p = io.pads[gp->index];
    size_t i = (size_t)g;
    return p.buttons[i] && !p.buttons_prev[i];
}

bool Moss_IsGamepadButtonJustReleased(Moss_Gamepad* gp, Moss_GamepadButton button) {
    if (!gp || button <= Moss_GamepadButton::INVALID)
        return false;

    Gamepad g = g_moss_to_gamepad_button[(size_t)button];
    if (g == Gamepad::GAMEPAD_BUTTON_LAST)
        return false;

    GamepadState& p = io.pads[gp->index];
    size_t i = (size_t)g;
    return !p.buttons[i] && p.buttons_prev[i];
}
float Moss_GetGamepadAxis(Moss_Gamepad* gp, Moss_GamepadAxis axis) {
    if (!gp || axis <= Moss_GamepadAxis::INVALID)
        return 0.0f;

    Joystick src = g_moss_to_gamepad_axis[(size_t)axis];
    float v = io.pads[gp->index].axes[(size_t)src];

    Moss_GamepadAxisConfig cfg = g_axis_config[(size_t)axis];

    // Deadzone
    v = ApplyDeadzone(v, cfg.deadzone);

    // Inversion
    if (cfg.invert)
        v = -v;

    return v;
}

void Moss_SetGamepadAxisDeadzone(Moss_GamepadAxis axis, float dz) { g_axis_config[(size_t)axis].deadzone = dz; }
void Moss_SetGamepadAxisInverted(Moss_GamepadAxis axis, bool inverted) { g_axis_config[(size_t)axis].invert = inverted; }


// Rumble / LED
bool Moss_RumbleGamepad(Moss_Gamepad* gp, uint16_t low, uint16_t high, uint32_t duration_ms) {
    if (!gp || gp->backend != MOSS_GAMEPAD_BACKEND_XINPUT)
        return false;

    XINPUT_VIBRATION vib = {};
    vib.wLeftMotorSpeed  = low;
    vib.wRightMotorSpeed = high;

    return XInputSetState(gp->index, &vib) == ERROR_SUCCESS;

    // Playstation 4 / 5
    if (!gp) return false;

    if (gp->backend == MOSS_GAMEPAD_BACKEND_XINPUT) {
        XINPUT_VIBRATION vib = {};
        vib.wLeftMotorSpeed  = low;
        vib.wRightMotorSpeed = high;
        return XInputSetState(gp->index, &vib) == ERROR_SUCCESS;
    }

    // DS4 / DS5 require HID output reports (not implemented)
    return false;
}
bool Moss_RumbleGamepadTriggers(Moss_Gamepad* gp, uint16_t left, uint16_t right, uint32_t duration_ms) {
    // Trigger rumble is NOT exposed by XInput
    return false;
}
bool Moss_SetGamepadLED(Moss_Gamepad* gp, uint8_t r, uint8_t g, uint8_t b) {
    // Not supported without vendor HID output
    return false;
}

// Metadata
const char* Moss_GetGamepadName(Moss_Gamepad* gp) { }
Moss_GamepadID Moss_GetGamepadID(Moss_Gamepad* gp) { }
int Moss_GetGamepadPlayerIndex(Moss_Gamepad* gp) { }
Moss_PowerState Moss_GetGamepadPowerInfo(Moss_Gamepad* gp, int* percent) {
    if (percent) *percent = -1;

    if (!gp || gp->backend != MOSS_GAMEPAD_BACKEND_XINPUT)
        return MOSS_POWERSTATE_UNKNOWN;

    XINPUT_BATTERY_INFORMATION bat;
    if (XInputGetBatteryInformation(gp->index, XINPUT_BATTERY_DEVTYPE_GAMEPAD, &bat) != ERROR_SUCCESS)
        return MOSS_POWERSTATE_UNKNOWN;

    if (percent) {
        static const int map[] = { 0, 25, 50, 75, 100 };
        *percent = map[bat.BatteryLevel];
    }

    return (bat.BatteryType == XINPUT_BATTERY_TYPE_WIRED) ? MOSS_POWERSTATE_CHARGING : MOSS_POWERSTATE_ON_BATTERY;
}
int Moss_GetNumGamepadTouchpads(Moss_Gamepad* gp) { }
int Moss_GetNumGamepadTouchpadFingers(Moss_Gamepad* gp) { }
bool Moss_GetGamepadTouchpadFinger(Moss_Gamepad* gp, int pad, int finger, bool* down, float* x, float* y, float* pressure) { }

// Mapping & type
const char* Moss_GetGamepadMapping(Moss_Gamepad* gp) { }
bool Moss_SetGamepadMapping(Moss_Gamepad* gp, const char* mapping) { }
void Moss_ReloadGamepadMappings(void) { }



// Pen and Fingers
Moss_PenDeviceType Moss_GetPenDeviceType(Moss_PenID instance_id){ return MOSS_PEN_DEVICE_UNKNOWN; }
const char* Moss_GetTouchDeviceName(Moss_TouchID touchID) { return "Windows Touch Device"; }
Moss_TouchID* Moss_GetTouchDevices(int *count) { 
    static Moss_TouchID ids[1] = { 0 };
    if (count) *count = 1;
    return ids; 
}
Moss_TouchDeviceType Moss_GetTouchDeviceType(Moss_TouchID touchID) { return MOSS_TOUCH_DEVICE_UNKNOWN; }
Moss_Finger** Moss_GetTouchFingers(Moss_TouchID touchID, int *count) {
    static Moss_Finger* fingers[16];
    int n = 0;

    for (int i = 0; i < g_pointer_count; ++i) {
        Moss_Pointer* p = &g_pointers[i];
        if (p->type == PT_TOUCH && p->down) {
            static Moss_Finger f;
            f.id = p->pointerId;
            f.x = p->x;
            f.y = p->y;
            f.pressure = 1.0f;

            fingers[n++] = &f;
        }
    }

    if (count) *count = n;
    return fingers;
}
