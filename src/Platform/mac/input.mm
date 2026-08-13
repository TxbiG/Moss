
#include <Moss/Platform/Mac/cocoa_platform.h>
#include "cocoa_platform.h"
#import <GameController/GameController.h>
#import <CoreHaptics/CoreHaptics.h>

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
        case 0x38: return Keyboard::KEY_LEFT_SHIFT;
        case 0x3C: return Keyboard::KEY_RIGHT_SHIFT;
        case 0x3B: return Keyboard::KEY_LEFT_CONTROL;
        case 0x3E: return Keyboard::KEY_RIGHT_CONTROL;
        case 0x3A: return Keyboard::KEY_LEFT_ALT;
        case 0x3D: return Keyboard::KEY_RIGHT_ALT;
        case 0x37: return Keyboard::KEY_LEFT_SUPER;
        case 0x36: return Keyboard::KEY_RIGHT_SUPER;
        case 0x39: return Keyboard::KEY_CAPS_LOCK;
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

bool Moss_IsKeyPressed(Moss_Keyboard key) { (void)key; return false; }
bool Moss_IsReleased(Moss_Keyboard key) { (void)key; return true; }
bool Moss_IsKeyJustPressed(Moss_Keyboard key) { (void)key; return false; }
bool Moss_IsKeyJustReleased(Moss_Keyboard key) { (void)key; return false; }
Moss_Keyboard Moss_InputGetKey() { return Moss_Keyboard::COUNT; }

bool Moss_IsMousePressed(Moss_MouseButton button) { (void)button; return false; }
bool Moss_IsMouseReleased(Moss_MouseButton button) { (void)button; return true; }
bool Moss_IsMouseJustPressed(Moss_MouseButton button) { (void)button; return false; }
bool Moss_IsMouseJustReleased(Moss_MouseButton button) { (void)button; return false; }
Moss_MouseButton Moss_InputGetMouseButton() { return Moss_MouseButton::COUNT; }
void Moss_GetMousePosition(int* x, int* y) { if (x) *x = 0; if (y) *y = 0; }
void Moss_SetMousePosition(int x, int y) { (void)x; (void)y; }
void Moss_SetMouseVisible(bool visible) { (void)visible; }

// Gamepad management

int Moss_GetNumGamepads(void) { return (int)[[GCController controllers] count]; }

static void Moss_UpdateMacGamepad(Moss_Gamepad* gp) {
    if (!gp || !gp->backend_handle || !gp->connected) return;
    GCController* controller = (GCController*)gp->backend_handle;
    GCExtendedGamepad* gamepad = controller.extendedGamepad;
    if (!gamepad) {
        gp->connected = false;
        io.pads[gp->index].connected = false;
        return;
    }

    GAMEPAD_STATE& state = io.pads[gp->index];
    memcpy(state.buttons_prev, state.buttons, sizeof(state.buttons));
    state.connected = true;

    state.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_A)] = gamepad.buttonA.isPressed;
    state.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_B)] = gamepad.buttonB.isPressed;
    state.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_X)] = gamepad.buttonX.isPressed;
    state.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_Y)] = gamepad.buttonY.isPressed;
    state.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_LEFT_BUMPER)] = gamepad.leftShoulder.isPressed;
    state.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_RIGHT_BUMPER)] = gamepad.rightShoulder.isPressed;
    state.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_LEFT_THUMB)] = gamepad.leftThumbstickButton ? gamepad.leftThumbstickButton.isPressed : false;
    state.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_RIGHT_THUMB)] = gamepad.rightThumbstickButton ? gamepad.rightThumbstickButton.isPressed : false;
    state.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_DPAD_UP)] = gamepad.dpad.up.isPressed;
    state.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_DPAD_DOWN)] = gamepad.dpad.down.isPressed;
    state.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_DPAD_LEFT)] = gamepad.dpad.left.isPressed;
    state.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_DPAD_RIGHT)] = gamepad.dpad.right.isPressed;
    state.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_BACK)] = gamepad.buttonOptions ? gamepad.buttonOptions.isPressed : false;
    state.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_START)] = gamepad.buttonMenu ? gamepad.buttonMenu.isPressed : false;
    state.buttons[static_cast<size_t>(Gamepad::GAMEPAD_BUTTON_GUIDE)] = false;

    state.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_LEFT_X)] = gamepad.leftThumbstick.xAxis.value;
    state.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_LEFT_Y)] = gamepad.leftThumbstick.yAxis.value;
    state.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_RIGHT_X)] = gamepad.rightThumbstick.xAxis.value;
    state.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_RIGHT_Y)] = gamepad.rightThumbstick.yAxis.value;
    state.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_LEFT_TRIGGER)] = gamepad.leftTrigger.value;
    state.axes[static_cast<size_t>(Joystick::GAMEPAD_AXIS_RIGHT_TRIGGER)] = gamepad.rightTrigger.value;
}

Moss_Gamepad* Moss_OpenGamepad(Moss_GamepadID id) {
    NSArray<GCController*>* controllers = [GCController controllers];
    if (id >= (Moss_GamepadID)[controllers count] || id >= 4) return nullptr;
    GCController* controller = [controllers objectAtIndex:id];
    if (!controller.extendedGamepad) return nullptr;

    Moss_Gamepad* gp = &g_macGamepads[id];
    memset(gp, 0, sizeof(*gp));
    gp->index = id;
    gp->backend = Moss_GamepadBackend::APPLE_GAME_CONTROLLER;
    gp->backend_handle = [controller retain];
    gp->connected = true;
    gp->name = "macOS Game Controller";
    gp->type = Moss_GamepadType::STANDARD;
    Moss_UpdateMacGamepad(gp);
    return gp;
}
void Moss_CloseGamepad(Moss_Gamepad* gp) {
    if (!gp) return;
    if (gp->backend_handle) [(GCController*)gp->backend_handle release];
    io.pads[gp->index].connected = false;
    gp->backend_handle = nullptr;
    gp->connected = false;
}

bool Moss_GamepadConnected(Moss_Gamepad* gp) {
    if (!gp || !gp->backend_handle) return false;
    return [[GCController controllers] containsObject:(GCController*)gp->backend_handle];
}

void Moss_UpdateGamepads(void) { for (uint32_t i = 0; i < 4; ++i) Moss_UpdateMacGamepad(&g_macGamepads[i]); }

// Button & axis
bool Moss_IsGamepadButtonPressed(Moss_Gamepad* gp, Moss_GamepadButton button) {
    Gamepad internal = Moss_MacInternalButton(button);
    return gp && gp->connected && internal != Gamepad::COUNT && io.pads[gp->index].buttons[static_cast<size_t>(internal)];
}

bool Moss_IsGamepadButtonJustPressed(Moss_Gamepad* gp, Moss_GamepadButton button) {
    Gamepad internal = Moss_MacInternalButton(button);
    if (!gp || !gp->connected || internal == Gamepad::COUNT) return false;
    const size_t i = static_cast<size_t>(internal);
    return io.pads[gp->index].buttons[i] && !io.pads[gp->index].buttons_prev[i];
}

bool Moss_IsGamepadButtonJustReleased(Moss_Gamepad* gp, Moss_GamepadButton button) {
    Gamepad internal = Moss_MacInternalButton(button);
    if (!gp || !gp->connected || internal == Gamepad::COUNT) return false;
    const size_t i = static_cast<size_t>(internal);
    return !io.pads[gp->index].buttons[i] && io.pads[gp->index].buttons_prev[i];
}

float Moss_GetGamepadAxis(Moss_Gamepad* gp, Moss_GamepadAxis axis) {
    Joystick internal = Moss_MacInternalAxis(axis);
    if (!gp || !gp->connected || internal == Joystick::COUNT) return 0.0f;
    return io.pads[gp->index].axes[static_cast<size_t>(internal)];
}

// Rumble / LED
static bool Moss_MacPlayHaptics(Moss_Gamepad* gp, float low, float high, uint32_t duration_ms) {
    if (!gp || !gp->backend_handle) return false;
    if (@available(macOS 11.0, *)) {
        GCController* controller = (GCController*)gp->backend_handle;
        if (!controller.haptics) return false;

        NSError* error = nil;
        CHHapticEngine* engine = [controller.haptics createEngineWithLocality:GCHapticsLocalityDefault];
        if (!engine) return false;
        if (![engine startAndReturnError:&error] || error) return false;

        float intensity_value = low > high ? low : high;
        if (intensity_value < 0.0f) intensity_value = 0.0f;
        if (intensity_value > 1.0f) intensity_value = 1.0f;
        float sharpness_value = high;
        if (sharpness_value < 0.0f) sharpness_value = 0.0f;
        if (sharpness_value > 1.0f) sharpness_value = 1.0f;

        CHHapticEventParameter* intensity = [[CHHapticEventParameter alloc] initWithParameterID:CHHapticEventParameterIDHapticIntensity value:intensity_value];
        CHHapticEventParameter* sharpness = [[CHHapticEventParameter alloc] initWithParameterID:CHHapticEventParameterIDHapticSharpness value:sharpness_value];
        NSTimeInterval duration = duration_ms ? ((NSTimeInterval)duration_ms / 1000.0) : 0.05;
        CHHapticEvent* event = [[CHHapticEvent alloc] initWithEventType:CHHapticEventTypeHapticContinuous parameters:@[ intensity, sharpness ] relativeTime:0 duration:duration];
        CHHapticPattern* pattern = [[CHHapticPattern alloc] initWithEvents:@[ event ] parameters:@[] error:&error];
        if (!pattern || error) return false;
        id<CHHapticPatternPlayer> player = [engine createPlayerWithPattern:pattern error:&error];
        if (!player || error) return false;
        return [player startAtTime:0 error:&error] && !error;
    }
    return false;
}

bool Moss_RumbleGamepad(Moss_Gamepad* gp, uint16_t low, uint16_t high, uint32_t duration_ms) {
    return Moss_MacPlayHaptics(gp, low / 65535.0f, high / 65535.0f, duration_ms);
}

bool Moss_RumbleGamepadTriggers(Moss_Gamepad* gp, uint16_t left, uint16_t right, uint32_t duration_ms) {
    return Moss_MacPlayHaptics(gp, left / 65535.0f, right / 65535.0f, duration_ms);
}
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
bool Moss_SetGamepadMapping(Moss_Gamepad* gp, const char* mapping) { (void)gp; (void)mapping; return false; }
void Moss_ReloadGamepadMappings(void) { }
Moss_GamepadButton Moss_InputGetGamepadButton() {
    for (uint32_t pad_index = 0; pad_index < 4; ++pad_index) {
        for (int button = 0; button < static_cast<int>(Gamepad::COUNT); ++button) {
            const size_t i = (size_t)button;
            if (io.pads[pad_index].buttons[i] && !io.pads[pad_index].buttons_prev[i]) return Moss_MacPublicButton((Gamepad)button);
        }
    }
    return Moss_GamepadButton::INVALID;
}
















// Gamepad management
static Moss_Gamepad g_macGamepads[4] = {};

static Gamepad Moss_MacInternalButton(Moss_GamepadButton button) {
    switch (button) {
    case Moss_GamepadButton::SOUTH: return Gamepad::GAMEPAD_BUTTON_A;
    case Moss_GamepadButton::EAST: return Gamepad::GAMEPAD_BUTTON_B;
    case Moss_GamepadButton::WEST: return Gamepad::GAMEPAD_BUTTON_X;
    case Moss_GamepadButton::NORTH: return Gamepad::GAMEPAD_BUTTON_Y;
    case Moss_GamepadButton::BACK: return Gamepad::GAMEPAD_BUTTON_BACK;
    case Moss_GamepadButton::GUIDE: return Gamepad::GAMEPAD_BUTTON_GUIDE;
    case Moss_GamepadButton::START: return Gamepad::GAMEPAD_BUTTON_START;
    case Moss_GamepadButton::LEFT_STICK: return Gamepad::GAMEPAD_BUTTON_LEFT_THUMB;
    case Moss_GamepadButton::RIGHT_STICK: return Gamepad::GAMEPAD_BUTTON_RIGHT_THUMB;
    case Moss_GamepadButton::LEFT_SHOULDER: return Gamepad::GAMEPAD_BUTTON_LEFT_BUMPER;
    case Moss_GamepadButton::RIGHT_SHOULDER: return Gamepad::GAMEPAD_BUTTON_RIGHT_BUMPER;
    case Moss_GamepadButton::DPAD_UP: return Gamepad::GAMEPAD_BUTTON_DPAD_UP;
    case Moss_GamepadButton::DPAD_DOWN: return Gamepad::GAMEPAD_BUTTON_DPAD_DOWN;
    case Moss_GamepadButton::DPAD_LEFT: return Gamepad::GAMEPAD_BUTTON_DPAD_LEFT;
    case Moss_GamepadButton::DPAD_RIGHT: return Gamepad::GAMEPAD_BUTTON_DPAD_RIGHT;
    default: return Gamepad::COUNT;
    }
}

static Moss_GamepadButton Moss_MacPublicButton(Gamepad button) {
    switch (button) {
    case Gamepad::GAMEPAD_BUTTON_A: return Moss_GamepadButton::SOUTH;
    case Gamepad::GAMEPAD_BUTTON_B: return Moss_GamepadButton::EAST;
    case Gamepad::GAMEPAD_BUTTON_X: return Moss_GamepadButton::WEST;
    case Gamepad::GAMEPAD_BUTTON_Y: return Moss_GamepadButton::NORTH;
    case Gamepad::GAMEPAD_BUTTON_BACK: return Moss_GamepadButton::BACK;
    case Gamepad::GAMEPAD_BUTTON_GUIDE: return Moss_GamepadButton::GUIDE;
    case Gamepad::GAMEPAD_BUTTON_START: return Moss_GamepadButton::START;
    case Gamepad::GAMEPAD_BUTTON_LEFT_THUMB: return Moss_GamepadButton::LEFT_STICK;
    case Gamepad::GAMEPAD_BUTTON_RIGHT_THUMB: return Moss_GamepadButton::RIGHT_STICK;
    case Gamepad::GAMEPAD_BUTTON_LEFT_BUMPER: return Moss_GamepadButton::LEFT_SHOULDER;
    case Gamepad::GAMEPAD_BUTTON_RIGHT_BUMPER: return Moss_GamepadButton::RIGHT_SHOULDER;
    case Gamepad::GAMEPAD_BUTTON_DPAD_UP: return Moss_GamepadButton::DPAD_UP;
    case Gamepad::GAMEPAD_BUTTON_DPAD_DOWN: return Moss_GamepadButton::DPAD_DOWN;
    case Gamepad::GAMEPAD_BUTTON_DPAD_LEFT: return Moss_GamepadButton::DPAD_LEFT;
    case Gamepad::GAMEPAD_BUTTON_DPAD_RIGHT: return Moss_GamepadButton::DPAD_RIGHT;
    default: return Moss_GamepadButton::INVALID;
    }
}

static Joystick Moss_MacInternalAxis(Moss_GamepadAxis axis) {
    switch (axis) {
    case Moss_GamepadAxis::LEFT_X: return Joystick::GAMEPAD_AXIS_LEFT_X;
    case Moss_GamepadAxis::LEFT_Y: return Joystick::GAMEPAD_AXIS_LEFT_Y;
    case Moss_GamepadAxis::RIGHT_X: return Joystick::GAMEPAD_AXIS_RIGHT_X;
    case Moss_GamepadAxis::RIGHT_Y: return Joystick::GAMEPAD_AXIS_RIGHT_Y;
    case Moss_GamepadAxis::LEFT_TRIGGER: return Joystick::GAMEPAD_AXIS_LEFT_TRIGGER;
    case Moss_GamepadAxis::RIGHT_TRIGGER: return Joystick::GAMEPAD_AXIS_RIGHT_TRIGGER;
    default: return Joystick::COUNT;
    }
}


void Moss_SetGamepadAxisDeadzone(Moss_GamepadAxis axis, float dz) { (void)axis; (void)dz; }
void Moss_SetGamepadAxisInverted(Moss_GamepadAxis axis, bool inverted) { (void)axis; (void)inverted; }

bool Moss_SetGamepadLED(Moss_Gamepad* gp, uint8_t r, uint8_t g, uint8_t b) { (void)gp; (void)r; (void)g; (void)b; return false; }

// Metadata
const char* Moss_GetGamepadName(Moss_Gamepad* gp) { return gp && gp->name ? gp->name : "macOS Game Controller"; }
Moss_GamepadID Moss_GetGamepadID(Moss_Gamepad* gp) { return gp ? gp->index : UINT32_MAX; }
int Moss_GetGamepadPlayerIndex(Moss_Gamepad* gp) { return gp ? (int)gp->index : -1; }
Moss_PowerState Moss_GetGamepadPowerInfo(Moss_Gamepad* gp, int* percent) { (void)gp; if (percent) *percent = -1; return Moss_PowerState::UNKNOWN; }
int Moss_GetNumGamepadTouchpads(Moss_Gamepad* gp) { (void)gp; return 0; }
int Moss_GetNumGamepadTouchpadFingers(Moss_Gamepad* gp) { (void)gp; return 0; }
bool Moss_GetGamepadTouchpadFinger(Moss_Gamepad* gp, int pad, int finger, bool* down, float* x, float* y, float* pressure) { (void)gp; (void)pad; (void)finger; if (down) *down = false; if (x) *x = 0.0f; if (y) *y = 0.0f; if (pressure) *pressure = 0.0f; return false; }



Moss_GamepadAxis Moss_InputGetGamepadAxis() { return Moss_GamepadAxis::INVALID; }