
#include <Moss/Platform/Mac/cocoa_platform.h>


Keyboard MapMacKeyCode(uint16_t macKeyCode) {
    switch (macKeyCode) {
        case 0x00: return Keyboard::KEY_A;
        case 0x0B: return Keyboard::KEY_B;
        case 0x08: return Keyboard::KEY_C;
        case 0x02: return Keyboard::KEY_D;
        case 0x0E: return Keyboard::KEY_E;
        case 0x03: return Keyboard::KEY_F;
        case 0x05: return Keyboard::KEY_G;
        case 0x04: return Keyboard::KEY_H;
        case 0x22: return Keyboard::KEY_I;
        case 0x26: return Keyboard::KEY_J;
        case 0x28: return Keyboard::KEY_K;
        case 0x25: return Keyboard::KEY_L;
        case 0x2E: return Keyboard::KEY_M;
        case 0x2D: return Keyboard::KEY_N;
        case 0x1F: return Keyboard::KEY_O;
        case 0x23: return Keyboard::KEY_P;
        case 0x0C: return Keyboard::KEY_Q;
        case 0x0F: return Keyboard::KEY_R;
        case 0x01: return Keyboard::KEY_S;
        case 0x11: return Keyboard::KEY_T;
        case 0x20: return Keyboard::KEY_U;
        case 0x09: return Keyboard::KEY_V;
        case 0x0D: return Keyboard::KEY_W;
        case 0x07: return Keyboard::KEY_X;
        case 0x10: return Keyboard::KEY_Y;
        case 0x06: return Keyboard::KEY_Z;
        case 0x31: return Keyboard::KEY_SPACE;
        case 0x24: return Keyboard::KEY_ENTER;
        case 0x35: return Keyboard::KEY_ESCAPE;
        case 0x30: return Keyboard::KEY_TAB;
        case 0x33: return Keyboard::KEY_BACKSPACE;
        // Add more keys as needed
        default: return Keyboard::COUNT;
    }
}


inline bool IsPressed(Keyboard k) {
    return currentKeys[static_cast<size_t>(k)];
}
inline bool IsReleased(Keyboard k) {
    return !currentKeys[static_cast<size_t>(k)];
}
inline bool IsJustPressed(Keyboard k) {
    size_t i = static_cast<size_t>(k);
    return currentKeys[i] && !previousKeys[i];
}
inline bool IsJustReleased(Keyboard k) {
    size_t i = static_cast<size_t>(k);
    return !currentKeys[i] && previousKeys[i];
}

inline bool IsPressed(Mouse b) {
    return currentMouse[static_cast<size_t>(b)];
}
inline bool IsReleased(Mouse b) {
    return !currentMouse[static_cast<size_t>(b)];
}
inline bool IsJustPressed(Mouse b) {
    size_t i = static_cast<size_t>(b);
    return currentMouse[i] && !previousMouse[i];
}
inline bool IsJustReleased(Mouse b) {
    size_t i = static_cast<size_t>(b);
    return !currentMouse[i] && previousMouse[i];
}

inline bool IsPressed(Gamepad b) {
    return currentGamepad[static_cast<size_t>(b)];
}
inline bool IsReleased(Gamepad b) {
    return !currentGamepad[static_cast<size_t>(b)];
}
inline bool IsJustPressed(Gamepad b) {
    size_t i = static_cast<size_t>(b);
    return currentGamepad[i] && !previousGamepad[i];
}
inline bool IsJustReleased(Gamepad b) {
    size_t i = static_cast<size_t>(b);
    return !currentGamepad[i] && previousGamepad[i];
}

// used for pollevents
void Moss_UpdateInputStates() {
    previousKeys = currentKeys;
    previousMouse = currentMouse;
    previousGamepad = currentGamepad;
}



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
