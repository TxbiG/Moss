#ifndef MOSS_PLATFORM_INTERNAL_H
#define MOSS_PLATFORM_INTERNAL_H

#include <atomic>
#include <cstdint>

#include <Moss/Moss_stdinc.h>
#include <Moss/Moss_Platform.h>

#ifndef _guarded
#define _guarded
#endif

struct SurfaceList {
    Moss_Surface* surface = nullptr;
    SurfaceList* next = nullptr;
};

struct GamepadMapping_t;

enum class Moss_GamepadBindingType {
    NONE = 0,
    BUTTON,
    AXIS,
    HAT
};


#if defined(MOSS_PLATFORM_WINDOWS)
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <mfapi.h>     // If using Media Foundation types
#include <mfidl.h>     // If using Media Foundation interfaces
#include <shlwapi.h>
#endif // MOSS_PLATFORM_WINDOWS

enum class Moss_GamepadBackend {
    UNKNOWN = 0,
    XINPUT,
    HID,
    APPLE_GAME_CONTROLLER,
    ANDROID_INPUT
};

#define MOSS_GAMEPAD_BACKEND_UNKNOWN Moss_GamepadBackend::UNKNOWN
#define MOSS_GAMEPAD_BACKEND_XINPUT Moss_GamepadBackend::XINPUT
#define MOSS_GAMEPAD_BACKEND_HID Moss_GamepadBackend::HID

enum class Moss_GamepadType {
    UNKNOWN = 0,
    STANDARD,
    XBOX360,
    XBOXONE,
    PS3,
    PS4,
    PS5,
    NINTENDO_SWITCH_PRO,
    NINTENDO_SWITCH_JOYCON_LEFT,
    NINTENDO_SWITCH_JOYCON_RIGHT,
    NINTENDO_SWITCH_JOYCON_PAIR,
    GAMECUBE,
    COUNT
};

struct GAMEPAD_STATE {
    bool connected = false;
    bool buttons[static_cast<size_t>(Gamepad::COUNT)] = {};
    float axes[static_cast<int>(GamepadAxis::COUNT)] = {};

    bool is_dualshock = false;
    bool is_dualsense = false;
};

using GamepadState = GAMEPAD_STATE;

struct KeyState {
    bool pressed;
    bool justPressed;
    bool justReleased;
};

struct INPUT_STATE {
    // keyboard
    uint8_t keys[static_cast<size_t>(Keyboard::COUNT)];
    uint8_t keys_prev[static_cast<size_t>(Keyboard::COUNT)];
    KeyState keyboardState[static_cast<size_t>(Keyboard::COUNT)];

    // mouse
    uint8_t mouse_buttons[static_cast<size_t>(Mouse::COUNT)] = {};
    uint8_t mouse_buttons_prev[static_cast<size_t>(Mouse::COUNT)] = {};
    int32_t mouse_x, mouse_y;
    int32_t mouse_dx, mouse_dy;
    float   mouse_wheel; // +1 per notch

    // gamepads (XInput)
    GAMEPAD_STATE pads[4]{};
};

extern INPUT_STATE io;
extern KeyState* keyboardState;

using AcquireFrameFunc = Moss_CaptureFrameResult(*)(Moss_Capture *device, Moss_Surface *frame, uint64_t *timestampNS, float *rotation);

struct Moss_Storage {
    /* The version of this interface */
    uint32_t version;

    /* Called when the storage is closed */
    bool (MOSS_CALL *close)(void *userdata);

    /* Optional, returns whether the storage is currently ready for access */
    bool (MOSS_CALL *ready)(void *userdata);

    /* Enumerate a directory, optional for write-only storage */
    bool (MOSS_CALL *enumerate)(void *userdata, const char *path, Moss_EnumerateDirectoryCallback callback, void *callback_userdata);

    /* Get path information, optional for write-only storage */
    bool (MOSS_CALL *info)(void *userdata, const char *path, Moss_PathInfo *info);

    /* Read a file from storage, optional for write-only storage */
    bool (MOSS_CALL *read_file)(void *userdata, const char *path, void *destination, uint64_t length);

    /* Write a file to storage, optional for read-only storage */
    bool (MOSS_CALL *write_file)(void *userdata, const char *path, const void *source, uint64_t length);

    /* Create a directory, optional for read-only storage */
    bool (MOSS_CALL *mkdir)(void *userdata, const char *path);

    /* Remove a file or empty directory, optional for read-only storage */
    bool (MOSS_CALL *remove)(void *userdata, const char *path);

    /* Rename a path, optional for read-only storage */
    bool (MOSS_CALL *rename)(void *userdata, const char *oldpath, const char *newpath);

    /* Copy a file, optional for read-only storage */
    bool (MOSS_CALL *copy)(void *userdata, const char *oldpath, const char *newpath);

    /* Get the space remaining, optional for read-only storage */
    uint64_t (MOSS_CALL *space_remaining)(void *userdata);


    void *userdata;
    char root[4096];
};

struct Moss_Gamepad {
    uint32_t index = 0;
    Moss_GamepadBackend backend = Moss_GamepadBackend::UNKNOWN;
    bool connected = false;
    void* backend_handle = nullptr;

    int ref_count = 0;

    const char* name = nullptr;
    Moss_GamepadType type = Moss_GamepadType::UNKNOWN;

    GamepadMapping_t* mapping = nullptr;
    int num_bindings = 0;
    Moss_GamepadBinding* bindings = nullptr;
    Moss_GamepadBinding** last_match_axis = nullptr;
    uint8_t* last_hat_mask = nullptr;
    uint64_t guide_button_down = 0;

    Moss_Gamepad* next = nullptr;
};

struct Moss_GamepadBinding {
    Moss_GamepadBindingType input_type;
    union {
        int button;

        struct {
            int axis;
            int axis_min;
            int axis_max;
        } axis;

        struct {
            int hat;
            int hat_mask;
        } hat;

    } input;

    Moss_GamepadBindingType output_type;
    union {
        Moss_GamepadButton button;

        struct {
            GamepadAxis axis;
            int axis_min;
            int axis_max;
        } axis;

    } output;
};


#endif // MOSS_PLATFORM_INTERNAL_H