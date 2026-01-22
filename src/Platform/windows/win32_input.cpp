#include <Moss/Platform/Windows/win32_platform.h>

#include <Xinput.h>

#pragma comment(lib,"xinput9_1_0.lib")

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

inline bool IsPressed(Keyboard k) { return io.keys[static_cast<size_t>(k)] != 0; }
inline bool IsReleased(Keyboard k) { return io.keys[static_cast<size_t>(k)] == 0; }
inline bool IsJustPressed(Keyboard k) { return io.keys[static_cast<size_t>(k)] && !io.keys_prev[static_cast<size_t>(k)]; }
inline bool IsJustReleased(Keyboard k) { size_t i = static_cast<size_t>(k); return !io.keys[i] &&  io.keys_prev[i]; }
inline bool IsPressed(Mouse b) { return io.mouse_buttons[static_cast<size_t>(b)] != 0; }
inline bool IsReleased(Mouse b) { return io.mouse_buttons[static_cast<size_t>(b)] == 0; }
inline bool IsJustPressed(Mouse b) { return io.mouse_buttons[static_cast<size_t>(b)] && !io.mouse_buttons_prev[static_cast<size_t>(b)]; }
inline bool IsJustReleased(Mouse b) { return !io.mouse_buttons[static_cast<size_t>(b)] &&  io.mouse_buttons_prev[static_cast<size_t>(b)]; }

inline bool IsPressed(size_t padIndex, Gamepad b) { return io.pads[padIndex].buttons[static_cast<size_t>(b)] != 0; }
inline bool IsReleased(size_t padIndex, Gamepad b) { return io.pads[padIndex].buttons[static_cast<size_t>(b)] == 0; }
inline bool IsJustPressed(size_t padIndex, Gamepad b) { return false; }
inline float GetAxis(size_t padIndex, Joystick a) { return io.pads[padIndex].axes[static_cast<size_t>(a)]; }



bool Moss_IsKeyPressed(Moss_Key key) { }
bool Moss_IsKeyJustPressed(Moss_Key key) { }
bool Moss_IsKeyJustReleased(Moss_Key key) { }

bool Moss_IsMousePressed(Moss_MouseButton button) { }
bool Moss_IsMouseJustPressed(Moss_MouseButton button) { }
bool Moss_IsMouseJustReleased(Moss_MouseButton button) { }
void Moss_GetMousePosition(int* x, int* y) { }
void Moss_SetMousePosition(int x, int y) { }
void Moss_SetMouseVisible(bool visible) { }


// Gamepad management
int Moss_GetNumGamepads(void) { }
Moss_Gamepad* Moss_OpenGamepad(Moss_GamepadID id) { }
void Moss_CloseGamepad(Moss_Gamepad* gp) { }
bool Moss_GamepadConnected(Moss_Gamepad* gp) { }
void Moss_UpdateGamepads(void) { }

// Button & axis
bool Moss_IsGamepadButtonPressed(Moss_Gamepad* gp, Moss_GamepadButton button) { }
bool Moss_IsGamepadButtonJustPressed(Moss_Gamepad* gp, Moss_GamepadButton button) { }
bool Moss_IsGamepadButtonJustReleased(Moss_Gamepad* gp, Moss_GamepadButton button) { }
float Moss_GetGamepadAxis(Moss_Gamepad* gp, Moss_GamepadAxis axis) { }

// Rumble / LED
bool Moss_RumbleGamepad(Moss_Gamepad* gp, uint16_t low, uint16_t high, uint32_t duration_ms) { }
bool Moss_RumbleGamepadTriggers(Moss_Gamepad* gp, uint16_t left, uint16_t right, uint32_t duration_ms) { }
bool Moss_SetGamepadLED(Moss_Gamepad* gp, uint8_t r, uint8_t g, uint8_t b) { }

// Metadata
const char* Moss_GetGamepadName(Moss_Gamepad* gp) { }
Moss_GamepadID Moss_GetGamepadID(Moss_Gamepad* gp) { }
int Moss_GetGamepadPlayerIndex(Moss_Gamepad* gp) { }
Moss_PowerState Moss_GetGamepadPowerInfo(Moss_Gamepad* gp, int* percent) { }
int Moss_GetNumGamepadTouchpads(Moss_Gamepad* gp) { }
int Moss_GetNumGamepadTouchpadFingers(Moss_Gamepad* gp) { }
bool Moss_GetGamepadTouchpadFinger(Moss_Gamepad* gp, int pad, int finger, bool* down, float* x, float* y, float* pressure) { }

// Mapping & type
const char* Moss_GetGamepadMapping(Moss_Gamepad* gp) { }
bool Moss_SetGamepadMapping(Moss_Gamepad* gp, const char* mapping) { }
void Moss_ReloadGamepadMappings(void) { }


Moss_PenDeviceType Moss_GetPenDeviceType(Moss_PenID instance_id);
const char* Moss_GetTouchDeviceName(Moss_TouchID touchID);
Moss_TouchID* Moss_GetTouchDevices(int *count);
Moss_TouchDeviceType Moss_GetTouchDeviceType(Moss_TouchID touchID);
Moss_Finger** Moss_GetTouchFingers(Moss_TouchID touchID, int *count);