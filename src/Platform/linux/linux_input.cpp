
#include <Moss/Platform/Linux/linux_platform.h>
#include "linux_platform.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <linux/input.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <string.h>
#include <unistd.h>


extern Moss_InputState io;
//extern INPUT_STATE io;

static Moss_Gamepad g_linuxGamepads[4] = {};

struct Moss_LinuxGamepadHandle {
    int js_fd;
    int evdev_fd;
    int ff_effect_id;
};

static const Gamepad g_button_map[] = {
    Gamepad::GAMEPAD_BUTTON_A,
    Gamepad::GAMEPAD_BUTTON_B,
    Gamepad::GAMEPAD_BUTTON_X,
    Gamepad::GAMEPAD_BUTTON_Y,
    Gamepad::GAMEPAD_BUTTON_LEFT_BUMPER,
    Gamepad::GAMEPAD_BUTTON_RIGHT_BUMPER,
    Gamepad::GAMEPAD_BUTTON_BACK,
    Gamepad::GAMEPAD_BUTTON_START,
    Gamepad::GAMEPAD_BUTTON_GUIDE,
    Gamepad::GAMEPAD_BUTTON_LEFT_THUMB,
    Gamepad::GAMEPAD_BUTTON_RIGHT_THUMB,
    Gamepad::GAMEPAD_BUTTON_DPAD_UP,
    Gamepad::GAMEPAD_BUTTON_DPAD_DOWN,
    Gamepad::GAMEPAD_BUTTON_DPAD_LEFT,
    Gamepad::GAMEPAD_BUTTON_DPAD_RIGHT
};

static const Joystick g_axis_map[] = {
    Joystick::GAMEPAD_AXIS_LEFT_X,
    Joystick::GAMEPAD_AXIS_LEFT_Y,
    Joystick::GAMEPAD_AXIS_RIGHT_X,
    Joystick::GAMEPAD_AXIS_RIGHT_Y,
    Joystick::GAMEPAD_AXIS_LEFT_TRIGGER,
    Joystick::GAMEPAD_AXIS_RIGHT_TRIGGER
};

static Moss_LinuxGamepadHandle* Moss_LinuxGamepadHandleFromGamepad(Moss_Gamepad* gp) {
    return gp ? (Moss_LinuxGamepadHandle*)gp->backend_handle : nullptr;
}

static int Moss_LinuxGamepadFD(Moss_Gamepad* gp) {
    Moss_LinuxGamepadHandle* handle = Moss_LinuxGamepadHandleFromGamepad(gp);
    return handle ? handle->js_fd : -1;
}

static int Moss_OpenLinuxEventDeviceByName(const char* joystick_name) {
    if (!joystick_name || !joystick_name[0]) return -1;

    for (int i = 0; i < 64; ++i) {
        char path[64];
        snprintf(path, sizeof(path), "/dev/input/event%d", i);
        int fd = open(path, O_RDWR | O_NONBLOCK);
        if (fd < 0) continue;

        char event_name[256] = {0};
        if (ioctl(fd, EVIOCGNAME(sizeof(event_name)), event_name) >= 0 && strcmp(event_name, joystick_name) == 0) {
            return fd;
        }

        close(fd);
    }

    return -1;
}

static bool Moss_LinuxPlayRumble(Moss_LinuxGamepadHandle* handle, uint16_t low, uint16_t high, uint32_t duration_ms) {
    if (!handle || handle->evdev_fd < 0) return false;

    ff_effect effect = {};
    effect.type = FF_RUMBLE;
    effect.id = handle->ff_effect_id;
    effect.u.rumble.strong_magnitude = low;
    effect.u.rumble.weak_magnitude = high;
    effect.replay.length = duration_ms ? (uint16_t)duration_ms : 1;
    effect.replay.delay = 0;

    if (ioctl(handle->evdev_fd, EVIOCSFF, &effect) < 0) return false;
    handle->ff_effect_id = effect.id;

    input_event play = {};
    play.type = EV_FF;
    play.code = effect.id;
    play.value = 1;
    return write(handle->evdev_fd, &play, sizeof(play)) == sizeof(play);
}

static bool Moss_LinuxGamepadPath(uint32_t index, char* out_path, size_t out_path_size) {
    if (!out_path || out_path_size == 0) return false;
    snprintf(out_path, out_path_size, "/dev/input/js%u", index);
    return true;
}

static bool Moss_LinuxGamepadPathExists(uint32_t index) {
    char path[64];
    Moss_LinuxGamepadPath(index, path, sizeof(path));
    return access(path, R_OK | W_OK) == 0 || access(path, R_OK) == 0;
}

static float Moss_NormalizeLinuxAxis(int16_t value) {
    if (value < 0) return (float)value / 32768.0f;
    return (float)value / 32767.0f;
}

static Moss_GamepadButton Moss_PublicButton(Gamepad button) {
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

static Gamepad Moss_InternalButton(Moss_GamepadButton button) {
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

static Joystick Moss_InternalAxis(Moss_GamepadAxis axis) {
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

static void Moss_ReadLinuxGamepad(Moss_Gamepad* gp) {
    const int fd = Moss_LinuxGamepadFD(gp);
    if (fd < 0 || !gp->connected) return;

    GAMEPAD_STATE& pad = io.pads[gp->index];
    memcpy(pad.buttons_prev, pad.buttons, sizeof(pad.buttons));

    js_event event;
    errno = 0;
    while (read(fd, &event, sizeof(event)) == sizeof(event)) {
        const uint8_t type = event.type & ~JS_EVENT_INIT;
        if (type == JS_EVENT_BUTTON && event.number < sizeof(g_button_map) / sizeof(g_button_map[0])) {
            pad.buttons[static_cast<size_t>(g_button_map[event.number])] = event.value != 0;
        } else if (type == JS_EVENT_AXIS && event.number < sizeof(g_axis_map) / sizeof(g_axis_map[0])) {
            float value = Moss_NormalizeLinuxAxis((int16_t)event.value);
            if (g_axis_map[event.number] == Joystick::GAMEPAD_AXIS_LEFT_TRIGGER || g_axis_map[event.number] == Joystick::GAMEPAD_AXIS_RIGHT_TRIGGER) {
                value = (value + 1.0f) * 0.5f;
            }
            pad.axes[static_cast<size_t>(g_axis_map[event.number])] = value;
        }
    }

    if (errno != EAGAIN && errno != EWOULDBLOCK && errno != 0) {
        gp->connected = false;
        pad.connected = false;
    }
}

bool Moss_IsKeyPressed(Moss_Keyboard key) { return io.keys[static_cast<size_t>(key)] != 0; }
bool Moss_IsReleased(Moss_Keyboard key) { return io.keys[static_cast<size_t>(key)] == 0; }
bool Moss_IsKeyJustPressed(Moss_Keyboard key) { size_t i = static_cast<size_t>(key); return io.keys[i] && !io.keys_prev[i]; }
bool Moss_IsKeyJustReleased(Moss_Keyboard key) { size_t i = static_cast<size_t>(key); return !io.keys[i] && io.keys_prev[i]; }
Moss_Keyboard Moss_InputGetKey() { return Moss_Keyboard::COUNT; }

inline bool IsPressed(size_t padIndex, Gamepad b) { return io.pads[padIndex].buttons[static_cast<size_t>(b)] != 0; }
inline bool IsReleased(size_t padIndex, Gamepad b) { return io.pads[padIndex].buttons[static_cast<size_t>(b)] == 0; }
inline bool IsJustPressed(size_t padIndex, Gamepad b) { return false; }
inline float GetAxis(size_t padIndex, Joystick a) { return io.pads[padIndex].axes[static_cast<size_t>(a)]; }

bool Moss_IsMousePressed(Moss_MouseButton button) { return io.mouse_buttons[static_cast<size_t>(button)] != 0; }
bool Moss_IsMouseReleased(Moss_MouseButton button) { return io.mouse_buttons[static_cast<size_t>(button)] == 0; }
bool Moss_IsMouseJustPressed(Moss_MouseButton button) { size_t i = static_cast<size_t>(button); return io.mouse_buttons[i] && !io.mouse_buttons_prev[i]; }
bool Moss_IsMouseJustReleased(Moss_MouseButton button) { size_t i = static_cast<size_t>(button); return !io.mouse_buttons[i] && io.mouse_buttons_prev[i]; }
Moss_MouseButton Moss_InputGetMouseButton() { return Moss_MouseButton::COUNT; }
void Moss_GetMousePosition(int* x, int* y) { if (x) *x = io.mouse_x; if (y) *y = io.mouse_y; }
void Moss_SetMousePosition(int x, int y) { (void)x; (void)y; }
void Moss_SetMouseVisible(bool visible) { (void)visible; }



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
int Moss_GetNumGamepads(void) { int count = 0; for (uint32_t i = 0; i < 4; ++i) if (Moss_LinuxGamepadPathExists(i)) ++count; return count; }
Moss_Gamepad* Moss_OpenGamepad(Moss_GamepadID id) {
    if (id >= 4 || !Moss_LinuxGamepadPathExists(id)) return nullptr;

    char path[64];
    Moss_LinuxGamepadPath(id, path, sizeof(path));
    int fd = open(path, O_RDONLY | O_NONBLOCK);
    if (fd < 0) return nullptr;

    char joystick_name[256] = {0};
    ioctl(fd, JSIOCGNAME(sizeof(joystick_name)), joystick_name);

    Moss_LinuxGamepadHandle* handle = (Moss_LinuxGamepadHandle*)calloc(1, sizeof(Moss_LinuxGamepadHandle));
    if (!handle) { close(fd); return nullptr; }
    handle->js_fd = fd;
    handle->evdev_fd = Moss_OpenLinuxEventDeviceByName(joystick_name);
    handle->ff_effect_id = -1;

    Moss_Gamepad* gp = &g_linuxGamepads[id];
    memset(gp, 0, sizeof(*gp));
    gp->index = id;
    gp->backend = Moss_GamepadBackend::HID;
    gp->backend_handle = handle;
    gp->connected = true;
    gp->name = joystick_name[0] ? "Linux joystick" : "Linux joystick";
    gp->type = Moss_GamepadType::STANDARD;

    GAMEPAD_STATE& pad = io.pads[id];
    memset(&pad, 0, sizeof(pad));
    pad.connected = true;
    Moss_ReadLinuxGamepad(gp);
    return gp;
}
void Moss_CloseGamepad(Moss_Gamepad* gp) {
    if (!gp) return;
    Moss_LinuxGamepadHandle* handle = Moss_LinuxGamepadHandleFromGamepad(gp);
    if (handle) {
        if (handle->evdev_fd >= 0 && handle->ff_effect_id >= 0) ioctl(handle->evdev_fd, EVIOCRMFF, handle->ff_effect_id);
        if (handle->evdev_fd >= 0) close(handle->evdev_fd);
        if (handle->js_fd >= 0) close(handle->js_fd);
        free(handle);
    }
    io.pads[gp->index].connected = false;
    gp->backend_handle = nullptr;
    gp->connected = false;
}
bool Moss_GamepadConnected(Moss_Gamepad* gp) { return gp && gp->connected && Moss_LinuxGamepadPathExists(gp->index); }
void Moss_UpdateGamepads(void) { for (uint32_t i = 0; i < 4; ++i) Moss_ReadLinuxGamepad(&g_linuxGamepads[i]); }

// Button & axis
bool Moss_IsGamepadButtonPressed(Moss_Gamepad* gp, Moss_GamepadButton button) {
    Gamepad internal = Moss_InternalButton(button);
    return gp && gp->connected && internal != Gamepad::COUNT && io.pads[gp->index].buttons[static_cast<size_t>(internal)];
}
bool Moss_IsGamepadButtonJustPressed(Moss_Gamepad* gp, Moss_GamepadButton button) {
    Gamepad internal = Moss_InternalButton(button);
    if (!gp || !gp->connected || internal == Gamepad::COUNT) return false;
    const size_t i = static_cast<size_t>(internal);
    return io.pads[gp->index].buttons[i] && !io.pads[gp->index].buttons_prev[i];
}
float Moss_GetGamepadAxis(Moss_Gamepad* gp, Moss_GamepadAxis axis) {
    Joystick internal = Moss_InternalAxis(axis);
    if (!gp || !gp->connected || internal == Joystick::COUNT) return 0.0f;
    return io.pads[gp->index].axes[static_cast<size_t>(internal)];
}
float Moss_GetGamepadAxis(Moss_Gamepad* gp, Moss_GamepadAxis axis) {
    Joystick internal = Moss_InternalAxis(axis);
    if (!gp || !gp->connected || internal == Joystick::COUNT) return 0.0f;
    return io.pads[gp->index].axes[static_cast<size_t>(internal)];
}

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
const char* Moss_GetGamepadMapping(Moss_Gamepad* gp) { (void)gp; return "linux-js"; }
bool Moss_SetGamepadMapping(Moss_Gamepad* gp, const char* mapping) { (void)gp; (void)mapping; return false; }
void Moss_ReloadGamepadMappings(void) {}
Moss_GamepadButton Moss_InputGetGamepadButton() {
    for (uint32_t pad_index = 0; pad_index < 4; ++pad_index) {
        for (size_t i = 0; i < sizeof(g_button_map) / sizeof(g_button_map[0]); ++i) {
            const size_t b = static_cast<size_t>(g_button_map[i]);
            if (io.pads[pad_index].buttons[b] && !io.pads[pad_index].buttons_prev[b]) return Moss_PublicButton(g_button_map[i]);
        }
    }
    return Moss_GamepadButton::INVALID;
}
Moss_GamepadAxis Moss_InputGetGamepadAxis() { return Moss_GamepadAxis::INVALID; }


void Moss_SetGamepadAxisDeadzone(Moss_GamepadAxis axis, float dz) { (void)axis; (void)dz; }
void Moss_SetGamepadAxisInverted(Moss_GamepadAxis axis, bool inverted) { (void)axis; (void)inverted; }
bool Moss_RumbleGamepad(Moss_Gamepad* gp, uint16_t low, uint16_t high, uint32_t duration_ms) { return Moss_LinuxPlayRumble(Moss_LinuxGamepadHandleFromGamepad(gp), low, high, duration_ms); }
bool Moss_RumbleGamepadTriggers(Moss_Gamepad* gp, uint16_t left, uint16_t right, uint32_t duration_ms) { (void)gp; (void)left; (void)right; (void)duration_ms; return false; }
bool Moss_SetGamepadLED(Moss_Gamepad* gp, uint8_t r, uint8_t g, uint8_t b) { (void)gp; (void)r; (void)g; (void)b; return false; }
const char* Moss_GetGamepadName(Moss_Gamepad* gp) { return gp && gp->name ? gp->name : "Linux joystick"; }
Moss_GamepadID Moss_GetGamepadID(Moss_Gamepad* gp) { return gp ? gp->index : UINT32_MAX; }
int Moss_GetGamepadPlayerIndex(Moss_Gamepad* gp) { return gp ? (int)gp->index : -1; }
Moss_PowerState Moss_GetGamepadPowerInfo(Moss_Gamepad* gp, int* percent) { (void)gp; if (percent) *percent = -1; return Moss_PowerState::UNKNOWN; }
int Moss_GetNumGamepadTouchpads(Moss_Gamepad* gp) { (void)gp; return 0; }
int Moss_GetNumGamepadTouchpadFingers(Moss_Gamepad* gp) { (void)gp; return 0; }
bool Moss_GetGamepadTouchpadFinger(Moss_Gamepad* gp, int pad, int finger, bool* down, float* x, float* y, float* pressure) { (void)gp; (void)pad; (void)finger; if (down) *down = false; if (x) *x = 0.0f; if (y) *y = 0.0f; if (pressure) *pressure = 0.0f; return false; }
const char* Moss_GetGamepadMapping(Moss_Gamepad* gp) { (void)gp; return "linux-js"; }
bool Moss_SetGamepadMapping(Moss_Gamepad* gp, const char* mapping) { (void)gp; (void)mapping; return false; }
void Moss_ReloadGamepadMappings(void) {}