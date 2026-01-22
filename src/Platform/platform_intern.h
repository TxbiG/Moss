#ifndef MOSS_PLATFORM_INTERNAL_H
#define MOSS_PLATFORM_INTERNAL_H

#include <Moss/Moss_Platform.h>


#if defined(MOSS_PLATFORM_WINDOWS)
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#endif // MOSS_PLATFORM_WINDOWS


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

typedef struct GAMEPAD_STATE {
    bool connected = false;
    bool buttons[static_cast<size_t>(Gamepad::COUNT)] = {};
    float axes[static_cast<int>(Joystick::COUNT)] = {};
} GAMEPAD_STATE;

struct KeyState {
    bool pressed;
    bool justPressed;
    bool justReleased;
};


typedef struct INPUT_STATE {
    // keyboard
    uint8_t keys[Keyboard::COUNT];
    uint8_t keys_prev[Keyboard::COUNT];
    KeyState keyboardState[Keyboard::COUNT];

    // mouse
    uint8_t mouse_buttons[Mouse::COUNT] = {};
    uint8_t mouse_buttons_prev[Mouse::COUNT] = {};
    int32_t mouse_x, mouse_y;
    int32_t mouse_dx, mouse_dy;
    float   mouse_wheel; // +1 per notch

    // gamepads (XInput)
    GAMEPAD_STATE pads[4]{};
} INPUT_STATE;

extern INPUT_STATE io;
extern KeyState* keyboardState;



struct Moss_Camera
{
    // A mutex for locking
    Moss_Mutex *lock;

    // Human-readable device name.
    char *name;

    // Position of camera (front-facing, back-facing, etc).
    Moss_CameraPosition position;

    // When refcount hits zero, we destroy the device object.
    Moss_AtomicInt refcount;

    // These are, initially, set from camera_driver, but we might swap them out with Zombie versions on disconnect/failure.
    bool (*WaitDevice)(Moss_Camera *device);
    Moss_CameraFrameResult (*AcquireFrame)(Moss_Camera *device, Moss_Surface *frame, Uint64 *timestampNS, float *rotation);
    void (*ReleaseFrame)(Moss_Camera *device, Moss_Surface *frame);

    // All supported formats/dimensions for this device.
    Moss_CameraSpec *all_specs;

    // Elements in all_specs.
    int num_specs;

    // The device's actual specification that the camera is outputting, before conversion.
    Moss_CameraSpec actual_spec;

    // The device's current camera specification, after conversions.
    Moss_CameraSpec spec;

    // Unique value assigned at creation time.
    Moss_CameraID instance_id;

    // Driver-specific hardware data on how to open device (`hidden` is driver-specific data _when opened_).
    void *handle;

    // Dropping the first frame(s) after open seems to help timing on some platforms.
    int drop_frames;

    // Backend timestamp of first acquired frame, so we can keep these meaningful regardless of epoch.
    Uint64 base_timestamp;

    // Moss timestamp of first acquired frame, so we can roughly convert to Moss ticks.
    Uint64 adjust_timestamp;

    // Pixel data flows from the driver into these, then gets converted for the app if necessary.
    Moss_Surface *acquire_surface;

    // acquire_surface converts or scales to this surface before landing in output_surfaces, if necessary.
    Moss_Surface *conversion_surface;

    // A queue of surfaces that buffer converted/scaled frames of video until the app claims them.
    SurfaceList output_surfaces[8];
    SurfaceList filled_output_surfaces;        // this is FIFO
    SurfaceList empty_output_surfaces;         // this is LIFO
    SurfaceList app_held_output_surfaces;

    // A fake video frame we allocate if the camera fails/disconnects.
    Uint8 *zombie_pixels;

    // non-zero if acquire_surface needs to be scaled for final output.
    int needs_scaling;  // -1: downscale, 0: no scaling, 1: upscale

    // true if acquire_surface needs to be converted for final output.
    bool needs_conversion;

    // Current state flags
    Moss_AtomicInt shutdown;
    Moss_AtomicInt zombie;

    // A thread to feed the camera device
    Moss_Thread *thread;

    // Optional properties.
    Moss_PropertiesID props;

    // Current state of user permission check.
    Moss_CameraPermissionState permission;

    // Data private to this driver, used when device is opened and running.
    struct Moss_PrivateCameraData *hidden;
};


typedef struct Moss_Storage
{
    /* The version of this interface */
    Uint32 version;

    /* Called when the storage is closed */
    bool (MossCALL *close)(void *userdata);

    /* Optional, returns whether the storage is currently ready for access */
    bool (MossCALL *ready)(void *userdata);

    /* Enumerate a directory, optional for write-only storage */
    bool (MossCALL *enumerate)(void *userdata, const char *path, Moss_EnumerateDirectoryCallback callback, void *callback_userdata);

    /* Get path information, optional for write-only storage */
    bool (MossCALL *info)(void *userdata, const char *path, Moss_PathInfo *info);

    /* Read a file from storage, optional for write-only storage */
    bool (MossCALL *read_file)(void *userdata, const char *path, void *destination, Uint64 length);

    /* Write a file to storage, optional for read-only storage */
    bool (MossCALL *write_file)(void *userdata, const char *path, const void *source, Uint64 length);

    /* Create a directory, optional for read-only storage */
    bool (MossCALL *mkdir)(void *userdata, const char *path);

    /* Remove a file or empty directory, optional for read-only storage */
    bool (MossCALL *remove)(void *userdata, const char *path);

    /* Rename a path, optional for read-only storage */
    bool (MossCALL *rename)(void *userdata, const char *oldpath, const char *newpath);

    /* Copy a file, optional for read-only storage */
    bool (MossCALL *copy)(void *userdata, const char *oldpath, const char *newpath);

    /* Get the space remaining, optional for read-only storage */
    Uint64 (MossCALL *space_remaining)(void *userdata);


    void *userdata;
};


struct Moss_Gamepad
{
    Moss_Joystick *joystick _guarded; // underlying joystick device
    int ref_count _guarded;

    const char *name _guarded;
    Moss_GamepadType type _guarded;
    GamepadMapping_t *mapping _guarded;
    int num_bindings _guarded;
    Moss_GamepadBinding *bindings _guarded;
    Moss_GamepadBinding **last_match_axis _guarded;
    uint8 *last_hat_mask _guarded;
    uint64 guide_button_down _guarded;

    struct Moss_Gamepad *next _guarded; // pointer to next gamepad we have allocated
};

typedef struct Moss_GamepadBinding
{
    Moss_GamepadBindingType input_type;
    union
    {
        int button;

        struct
        {
            int axis;
            int axis_min;
            int axis_max;
        } axis;

        struct
        {
            int hat;
            int hat_mask;
        } hat;

    } input;

    Moss_GamepadBindingType output_type;
    union
    {
        Moss_GamepadButton button;

        struct
        {
            Moss_GamepadAxis axis;
            int axis_min;
            int axis_max;
        } axis;

    } output;
} Moss_GamepadBinding;


#endif // MOSS_PLATFORM_INTERNAL_H