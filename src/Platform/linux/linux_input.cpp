#include <Moss/Platform/Linux/linux_platform.h>

extern Moss_InputState io;


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
