//                        MIT License
//
//                  Copyright (c) 2026 Toby
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/*!
 * @file Moss_Platform.h
 * @brief Core platform abstraction layer for the Moss Framework.
 *
 * The platform module provides a unified API for interacting with
 * the operating system, hardware devices, and runtime environment. It ensures 
 * cross-platform consistency across Windows, Linux, BSD, macOS, iOS, and Android.
 *
 * ### Primary Responsibilities:
 * - **Window Management** — Creation and control of application and popup windows.
 * - **Display & Monitor Handling** — Query and manage monitor configurations, resolutions, and refresh rates.
 * - **Input System** — Unified handling of Moss_keyboard, mouse, controller, and touch inputs.
 * - **Camera Support** — Management of active viewports, XR cameras, and display orientation.
 * - **Haptic Feedback** — Device vibration and tactile feedback support.
 * - **Timing Utilities** — High-precision timers, delta-time tracking, and performance profiling.
 * - **System Integration** — OS-level utilities such as:
 *    - Dynamic library loading/get/unloading (DLL/SO/DYLIB)
 *    - Opening URLs or external resources
 *    - Querying CPU, and memory information
 *    - Environment and locale access
 */


#ifndef MOSS_PLATFORM_H
#define MOSS_PLATFORM_H

#include <Moss/Moss_stdinc.h>

#ifdef MOSS_GRAPHICS_VULKAN
#include <vulkan/vulkan.h>
#endif // MOSS_GRAPHICS_VULKAN


using Moss_FingerID = uint32_t;
using Moss_TouchID = uint32_t;
using Moss_HapticID = uint32_t;
using Moss_CameraID = uint32_t;
using Moss_GamepadID = uint32_t;
using Moss_PropertiesID = uint32_t;
using Moss_PenID = uint32_t;
using Moss_HapticEffectID = uint32_t;

struct Moss_Window;
struct Moss_Monitor;
struct Moss_Curser;
struct Moss_GamepadBinding;
struct Moss_Capture; // Camera Device Dont use as the rendering camera
struct Moss_Storage;
struct Moss_Surface;

struct Moss_HapticDirection;
struct Moss_HapticConstant;
struct Moss_HapticPeriodic;
struct Moss_HapticCondition;
struct Moss_HapticRamp;
struct Moss_HapticLeftRight;
struct Moss_HapticCustom; 
union Moss_HapticEffect;
struct Moss_Haptic;
struct Moss_Gamepad;
struct Moss_GamepadAxis;

enum class Keyboard {
    KEY_0, 
    KEY_1, 
    KEY_2, 
    KEY_3, 
    KEY_4, 
    KEY_5, 
    KEY_6, 
    KEY_7, 
    KEY_8, 
    KEY_9, 
    KEY_A, 
    KEY_B, 
    KEY_C, 
    KEY_D, 
    KEY_E, 
    KEY_F, 
    KEY_G, 
    KEY_H, 
    KEY_I, 
    KEY_J, 
    KEY_K, 
    KEY_L, 
    KEY_M,
    KEY_N, 
    KEY_O, 
    KEY_P, 
    KEY_Q, 
    KEY_R, 
    KEY_S, 
    KEY_T, 
    KEY_U, 
    KEY_V, 
    KEY_W, 
    KEY_X, 
    KEY_Y, 
    KEY_Z,
    KEY_APOSTROPHE, 
    KEY_BACKSLASH, 
    KEY_COMMA, 
    KEY_EQUAL,
    KEY_GRAVE_ACCENT, 
    KEY_LEFT_BRACKET, 
    KEY_MINUS, 
    KEY_PERIOD, 
    KEY_RIGHT_BRACKET, 
    KEY_SEMICOLON, 
    KEY_SLASH, 
    KEY_WORLD_1, 
    KEY_WORLD_2,     
    KEY_BACKSPACE,
    KEY_DELETE, 
    KEY_END, 
    KEY_ENTER, 
    KEY_ESCAPE, 
    KEY_HOME, 
    KEY_INSERT, 
    KEY_MENU, 
    KEY_PAGE_DOWN, 
    KEY_PAGE_UP, 
    KEY_PAUSE, 
    KEY_SPACE, 
    KEY_TAB, 
    KEY_CAPS_LOCK, 
    KEY_NUM_LOCK, 
    KEY_SCROLL_LOCK,
    KEY_F1, 
    KEY_F2, 
    KEY_F3,  
    KEY_F4,  
    KEY_F5,  
    KEY_F6,  
    KEY_F7,  
    KEY_F8,  
    KEY_F9, 
    KEY_F10, 
    KEY_F11, 
    KEY_F12, 
    KEY_F13, 
    KEY_F14, 
    KEY_F15, 
    KEY_F16, 
    KEY_F17, 
    KEY_F18, 
    KEY_F19, 
    KEY_F20, 
    KEY_F21, 
    KEY_F22, 
    KEY_F23, 
    KEY_F24,
    KEY_LEFT_ALT, 
    KEY_LEFT_CONTROL, 
    KEY_LEFT_SHIFT, 
    KEY_LEFT_SUPER,
    KEY_PRINT_SCREEN, 
    KEY_RIGHT_ALT, 
    KEY_RIGHT_CONTROL, 
    KEY_RIGHT_SHIFT, 
    KEY_RIGHT_SUPER, 
    KEY_DOWN, 
    KEY_LEFT, 
    KEY_RIGHT, 
    KEY_UP,
    KEY_KP_0, 
    KEY_KP_1, 
    KEY_KP_2, 
    KEY_KP_3, 
    KEY_KP_4, 
    KEY_KP_5, 
    KEY_KP_6, 
    KEY_KP_7, 
    KEY_KP_8, 
    KEY_KP_9, 
    KEY_KP_ADD, 
    KEY_KP_DECIMAL, 
    KEY_KP_DIVIDE, 
    KEY_KP_ENTER, 
    KEY_KP_EQUAL, 
    KEY_KP_MULTIPLY, 
    KEY_KP_SUBTRACT,
    LAST_KEY = KEY_KP_SUBTRACT,
    COUNT = KEY_KP_SUBTRACT
};

// Mouse
enum class Mouse { 
    LEFT, 
    RIGHT, 
    MIDDLE, 
    BUTTON_4, 
    BUTTON_5, 
    BUTTON_6, 
    BUTTON_7, 
    BUTTON_8, 
    COUNT = 8 
};

enum class Gamepad {
    GAMEPAD_BUTTON_A, 
    GAMEPAD_BUTTON_B, 
    GAMEPAD_BUTTON_X, 
    GAMEPAD_BUTTON_Y,
    GAMEPAD_BUTTON_LEFT_BUMPER, 
    GAMEPAD_BUTTON_RIGHT_BUMPER,
    GAMEPAD_BUTTON_BACK, 
    GAMEPAD_BUTTON_START, 
    GAMEPAD_BUTTON_GUIDE,
    GAMEPAD_BUTTON_LEFT_THUMB, 
    GAMEPAD_BUTTON_RIGHT_THUMB,
    GAMEPAD_BUTTON_DPAD_UP, 
    GAMEPAD_BUTTON_DPAD_RIGHT, 
    GAMEPAD_BUTTON_DPAD_DOWN, 
    GAMEPAD_BUTTON_DPAD_LEFT,

    GAMEPAD_BUTTON_LAST = GAMEPAD_BUTTON_DPAD_LEFT,

    GAMEPAD_BUTTON_CROSS  =  GAMEPAD_BUTTON_A,
    GAMEPAD_BUTTON_CIRCLE =  GAMEPAD_BUTTON_B,
    GAMEPAD_BUTTON_SQUARE =  GAMEPAD_BUTTON_X,
    GAMEPAD_BUTTON_TRIANGLE = GAMEPAD_BUTTON_Y,

    COUNT = GAMEPAD_BUTTON_LAST
};

enum class GamepadAxis : uint8_t {
    LEFT_X = 0,        // Left Stick X Axis
    LEFT_Y,            // Left Stick Y Axis
    RIGHT_X,           // Right Stick X Axis
    RIGHT_Y,           // Right Stick Y Axis
    LEFT_TRIGGER,      // Left Trigger
    RIGHT_TRIGGER,     // Right Trigger
    TOUCHPAD_X,        // (PS4/PS5)
    TOUCHPAD_Y,
    GYRO_X,
    GYRO_Y,
    GYRO_Z,
    COUNT
};

// Cursor
enum class Moss_CursorMode {
    VISIBLE = 0,             //
    HIDDEN = 1,              //
    CAPTURED = 2,            //
    CONFINED = 3,            //
    CONFINED_HIDDEN = 4,     //
};

enum class Moss_CursorShape {
    ARROW = 0,           //
    IBEAM = 1,           //
    POINTING_HAND = 2,   //
    CROSS = 3,           //
    WAIT = 4,            //
    BUSY = 5,            //
    DRAG = 6,            //
    CAN_DROP = 7,        //
    FORBIDDEN = 8,       //
    VSIZE = 9,           //
    HSIZE = 10,          //
    BDIAGSIZE = 11,      //
    FDIAGSIZE = 12,      //
    MOVE = 13,           //
    VSPLIT = 14,         //
    HSPLIT = 15,         //
    HELP = 16            //
};

////////////////////////////////////////////////////////////////
// GamePad
////////////////////////////////////////////////////////////////

enum class Moss_GamepadButton {
    INVALID = -1,
    SOUTH,           // Bottom face button (e.g. Xbox A button) */
    EAST,            // Right face button (e.g. Xbox B button) */
    WEST,            // Left face button (e.g. Xbox X button) */
    NORTH,           // Top face button (e.g. Xbox Y button) */
    BACK,
    GUIDE,
    START,
    LEFT_STICK,
    RIGHT_STICK,
    LEFT_SHOULDER,
    RIGHT_SHOULDER,
    DPAD_UP,
    DPAD_DOWN,
    DPAD_LEFT,
    DPAD_RIGHT,
    MISC1,           // Additional button (e.g. Xbox Series X share button, PS5 microphone button, Nintendo Switch Pro capture button, Amazon Luna microphone button, Google Stadia capture button) */
    RIGHT_PADDLE1,   // Upper or primary paddle, under your right hand (e.g. Xbox Elite paddle P1, DualSense Edge RB button, Right Joy-Con SR button) */
    LEFT_PADDLE1,    // Upper or primary paddle, under your left hand (e.g. Xbox Elite paddle P3, DualSense Edge LB button, Left Joy-Con SL button) */
    RIGHT_PADDLE2,   // Lower or secondary paddle, under your right hand (e.g. Xbox Elite paddle P2, DualSense Edge right Fn button, Right Joy-Con SL button) */
    LEFT_PADDLE2,    // Lower or secondary paddle, under your left hand (e.g. Xbox Elite paddle P4, DualSense Edge left Fn button, Left Joy-Con SR button) */
    TOUCHPAD,        // PS4/PS5 touchpad button */
    MISC2,           // Additional button */
    MISC3,           // Additional button (e.g. Nintendo GameCube left trigger click) */
    MISC4,           // Additional button (e.g. Nintendo GameCube right trigger click) */
    MISC5,           // Additional button */
    MISC6,           // Additional button */
    COUNT
};

////////////////////////////////////////////////////////////////
//          Haptic          
////////////////////////////////////////////////////////////////
enum class Moss_HapticFeedbackType {
    AUTOCENTER = 0,
    CARTESIAN,
    CONSTANT,
    CUSTOM,
    DAMPER,
    FRICTION,
    GAIN,
    INERTIA,
    INFINITY_,
    LEFTRIGHT,
    PAUSE,
    POLAR,
    RAMP,
    RESERVED1,
    RESERVED2,
    RESERVED3,
    SAWTOOTHDOWN,
    SAWTOOTHUP,
    SINE,
    SPHERICAL,
    SPRING,
    SQUARE,
    STATUS,
    STEERING_AXIS,
    TRIANGLE
};


enum class Moss_HapticEffectType {
    CONSTANT,
    PERIODIC,
    CONDITION,
    RAMP,
    LEFTRIGHT,
    CUSTOM 
};


enum class Moss_GamepadSensorType : uint8_t {
    ACCELEROMETER,
    GYROSCOPE
};

////////////////////////////////////////////////////////////////
// Window
////////////////////////////////////////////////////////////////
enum class Moss_WindowFlags {
    NOTITLEBAR = 0x00000001,        // no window decoration
    RESIZE_DISABLED = 0,
    ALWAYS_ON_TOP = 20,
    WINDOW_TRANSPARENT = 3,
    NO_FOCUS = 4,
    POPUP = 5,
    EXTEND_TO_TITLE = 6,
    MOUSE_PASSTHROUGH = 7,
    SHARP_CORNERS = 8,
    EXCLUDE_FROM_CAPTURE = 9,
    MAX = 10,
    SHOWN = 0x00000004,              // window is visible
    HIDDEN = 0x00000008,             // window is not visible
    BORDERLESS = 0x00000010,         // no window decoration
    RESIZABLE = 0x00000020,          // window can be resized 
    MINIMIZED = 0x00000040,          // window is minimized
    MAXIMIZED = 0x00000080,          // window is maximized 
    MOUSE_GRABBED = 0x00000100,      // window has grabbed mouse input
    INPUT_FOCUS = 0x00000200,        // window has input focus
    MOUSE_FOCUS = 0x00000400,        // window has mouse focus
    FOREIGN = 0x00000800,            // window not created by Moss
    ALLOW_HIGHDPI = 0x00002000,      // window should be created in high-DPI mode if supported. On macOS NSHighResolutionCapable must be set true in the application's Info.plist for this to have any effect.
    
    KEYBOARD_GRABBED = 0x00100000,   // window has grabbed Moss_input

    INPUT_GRABBED = MOUSE_GRABBED // equivalent to MOUSE_GRABBED for compatibility
};

enum class Moss_MessageBoxFlags {
    ABORT_ENTRY_IGNORE = 0,  // The message box contains three push buttons: Abort, Retry, and Ignore.
    CANCEL_TRY_CONTINUE = 1, // The message box contains three push buttons: Cancel, Try Again, Continue.
    HELP = 3,              // Adds a Help button to the message box.
    OK = 4,                // The message box contains one push button: OK. This is the default.
    OK_CANCEL = 5,          // The message box contains two push buttons: OK and Cancel.
    RETRY_CANCEL = 6,       // The message box contains two push buttons: Retry and Cancel.
    YES_NO = 7,             // The message box contains two push buttons: Yes and No.
    YES_NO_CANCEL = 8,       // The message box contains three push buttons: Yes, No, and Cancel.
};

enum class Moss_WindowMode {
    WINDOWED = 0,              //
    MINIMIZED = 1,             //
    MAXIMIZED = 2,             //
    FULLSCREEN = 3,            //
    EXCLUSIVE_FULLSCREEN = 4,  //
};

// Gesture used for ios and android
enum class Moss_Gesture {
    NONE        = 0<<0,     // No gesture
    TAP         = 1<<0,     // Tap gesture
    DOUBLETAP   = 2<<0,     // Double tap gesture
    HOLD        = 3<<0,     // Hold gesture
    DRAG        = 4<<0,     // Drag gesture
    SWIPE_RIGHT = 5<<0,     // Swipe right gesture
    SWIPE_LEFT  = 6<<0,     // Swipe left gesture
    SWIPE_UP    = 7<<0,     // Swipe up gesture
    SWIPE_DOWN  = 8<<0,     // Swipe down gesture
    PINCH_IN    = 9<<0,     // Pinch in gesture
    PINCH_OUT   = 10<<0     // Pinch out gesture
};

enum class Moss_PowerState : int8_t {
    ERROR,   // error determining power status */
    UNKNOWN,      // cannot determine power status */
    ON_BATTERY,   // Not plugged in, running on the battery */
    NO_BATTERY,   // Plugged in, no battery available */
    CHARGING,     // Plugged in, charging battery */
    CHARGED       // Plugged in, battery charged */
};

enum class Moss_FileDialogType {
    OPENFILE,
    SAVEFILE,
    OPENFOLDER
};

enum class Moss_PenAxis {
    PRESSURE,  // Pen pressure.  Unidirectional: 0 to 1.0 */
    XTILT,     // Pen horizontal tilt angle.  Bidirectional: -90.0 to 90.0 (left-to-right). */
    YTILT,     // Pen vertical tilt angle.  Bidirectional: -90.0 to 90.0 (top-to-down). */
    DISTANCE,  // Pen distance to drawing surface.  Unidirectional: 0.0 to 1.0 */
    ROTATION,  // Pen barrel rotation.  Bidirectional: -180 to 179.9 (clockwise, 0 is facing up, -180.0 is facing down). */
    SLIDER,    // Pen finger wheel or slider (e.g., Airbrush Pen).  Unidirectional: 0 to 1.0 */
    TANGENTIAL_PRESSURE,    // Pressure from squeezing the pen ("barrel pressure"). */
    COUNT       // Total known pen axis types in this version of Moss. This number may grow in future releases! */
};

enum class Moss_PenDeviceType {
    INVALID = -1, // Not a valid pen device. */
    UNKNOWN,      // Don't know specifics of this pen. */
    DIRECT,       // Pen touches display. */
    INDIRECT      // Pen touches something that isn't the display. */
};
enum class Moss_TouchDeviceType {
    INVALID = -1,
    DIRECT,            // touch screen with window-relative coordinates */
    INDIRECT_ABSOLUTE, // trackpad with absolute device coordinates */
    INDIRECT_RELATIVE  // trackpad with screen cursor-relative coordinates */
};

enum class Moss_JoystickConnectionState {
    INVALID = -1,
    UNKNOWN,
    WIRED,
    WIRELESS
};


enum class Moss_UserFolder {
    HOME,
    DESKTOP,
    DOCUMENTS,
    DOWNLOADS,
    PICTURES,
    MUSIC,
    VIDEOS,
    APPDATA,
    CACHE
};

enum class Moss_PathType {
    UNKNOWN = 0,
    FILE,
    DIRECTORY,
    SYMLINK
};

enum class Moss_CameraPermissionState { 
    DENIED = -1, 
    PENDING, 
    APPROVED 
};

enum class Moss_CameraPosition { 
    UNKNOWN, 
    FRONT_FACING, 
    BACK_FACING 
};

struct Moss_CameraSpec {
    PixelFormat format;         // Frame format
    Colorspace colorspace;      // Frame colorspace
    int width;                  // Frame width
    int height;                 // Frame height
    int framerate_numerator;    // Frame rate numerator ((num / denom) == FPS, (denom / num) == duration in seconds)
    int framerate_denominator;  // Frame rate demoninator ((num / denom) == FPS, (denom / num) == duration in seconds)
};

struct Moss_GammaRamp {
    uint32_t size;
    uint8_t* red; 
    uint8_t* green; 
    uint8_t* blue; 
};

struct Moss_VideoMode { 
    int width, height, redBits, greenBits, blueBits, refreshRate; 
};

struct Moss_Image { 
    int width, height; 
    unsigned char* pixels; 
};


struct Moss_Locale {
    char* country;
    char* language;
};

struct Moss_GamepadSensorData
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    // Monotonic timestamp in nanoseconds.
    // Use the same clock source for all samples.
    uint64_t timestamp_ns = 0;
};

enum class Moss_PenInputFlags {
    UNKNOWN = 0
};

struct Moss_PenAxisEvent {
    //Moss_EventType type;     // Moss_EVENT_PEN_AXIS */
    uint32_t reserved;
    uint64_t timestamp;       // In nanoseconds, populated using Moss_GetTicksNS() */
    uint32_t id;        // The pen instance id */
    Moss_PenInputFlags pen_state;   // Complete pen input state at time of event */
    float x;                // X coordinate, relative to window */
    float y;                // Y coordinate, relative to window */
    Moss_PenAxis axis;       // Axis that has changed */
    float value;            // New value of axis */
};

struct Moss_PenButtonEvent {
    //Moss_EventType type; // Moss_EVENT_PEN_BUTTON_DOWN or Moss_EVENT_PEN_BUTTON_UP */
    uint32_t reserved;
    uint64_t timestamp;   // In nanoseconds, populated using Moss_GetTicksNS() */
    Moss_PenID id;        // The pen instance id */
    Moss_PenInputFlags pen_state;   // Complete pen input state at time of event */
    float x;                // X coordinate, relative to window */
    float y;                // Y coordinate, relative to window */
    uint8_t button;       // The pen button index (first button is 1). */
    bool down;      // true if the button is pressed */
};

struct Moss_PenMotionEvent {
    //Moss_EventType type; // Moss_EVENT_PEN_MOTION */
    uint32_t reserved;
    uint64_t timestamp;   // In nanoseconds, populated using Moss_GetTicksNS() */
    Moss_PenID id;        // The pen instance id */
    Moss_PenInputFlags pen_state;   // Complete pen input state at time of event */
    float x;                // X coordinate, relative to window */
    float y;                // Y coordinate, relative to window */
};

struct Moss_PenProximityEvent {
    //Moss_EventType type; // Moss_EVENT_PEN_PROXIMITY_IN or Moss_EVENT_PEN_PROXIMITY_OUT */
    uint32_t reserved;
    uint64 timestamp;   // In nanoseconds, populated using Moss_GetTicksNS() */
    Moss_PenID id;        // The pen instance id */
};

struct Moss_PenTouchEvent {
    //Moss_EventType type;     // Moss_EVENT_PEN_DOWN or Moss_EVENT_PEN_UP */
    uint32_t reserved;
    uint64 timestamp;       // In nanoseconds, populated using Moss_GetTicksNS() */
    Moss_PenID id;        // The pen instance id */
    Moss_PenInputFlags pen_state;   // Complete pen input state at time of event */
    float x;                // X coordinate, relative to window */
    float y;                // Y coordinate, relative to window */
    bool eraser;        // true if eraser end is used (not all pens support this). */
    bool down;          // true if the pen is touching or false if the pen is lifted off */
};

struct Moss_Finger {
    Moss_FingerID id;  // the finger ID */
    float x;  // the x-axis location of the touch event, normalized (0...1) */
    float y;  // the y-axis location of the touch event, normalized (0...1) */
    float pressure; // the quantity of pressure applied, normalized (0...1) */
};

/*! @param type The type of encoding. @param dir The encoded direction. */
struct Moss_HapticDirection { 
    uint8_t type; 
    int32_t dir[3]; 
};

/*! @param type Header (HAPTIC_LEFTRIGHT). @param length Duration of the effect in milliseconds. (Replay) @param large_magnitude Control of the large controller motor. (Rumble) @param small_magnitude Control of the small controller motor. (Rumble) */
struct Moss_HapticLeftRight { 
    Moss_HapticEffectType type; 
    uint32_t length; 
    uint16_t large_magnitude; 
    uint16_t small_magnitude; 
};

/*! @param type e.g. HAPTIC_CUSTOM. (Header) @param direction Direction of the effect. (Header) @param length Duration of the effect. (Replay) @param delay Delay before starting the effect. (Replay) 
    @param button Button that triggers the effect. (Trigger)  @param interval How soon it can be triggered again after button. (Trigger) @param channels Axes to use, minimum of one. (Custom) 
    @param period Sample periods. (Custom) @param samples Amount of samples. (Custom) @param data Should contain channels*samples items. (Custom) 
    @param attack_length Duration of the attack. (Envelope) @param attack_level Level at the start of the attack. (Envelope) @param fade_length Duration of the fade. (Envelope) 
    @param fade_level Level at the end of the fade. (Envelope)*/
struct Moss_HapticCustom { 
    Moss_HapticEffectType type; 
    Moss_HapticDirection direction; 
    uint32_t length; 
    uint16_t delay; 
    uint16_t button; 
    uint16_t interval;  
    uint8_t channels; 
    uint16_t period; 
    uint16_t samples; 
    uint16_t *data;
    uint16_t attack_length; 
    uint16_t attack_level; 
    uint16_t fade_length; 
    uint16_t fade_level;
};

struct Moss_HapticRamp {
    // Header
    Moss_HapticEffectType type;      // HAPTIC_RAMP
    Moss_HapticDirection direction;  // Direction of the effect.

    // Replay
    uint32_t length;          // Duration of the effect.
    uint16_t delay;           // Delay before starting the effect.

    // Trigger
    uint16_t button;          //Button that triggers the effect.
    uint16_t interval;        // How soon it can be triggered again after button.

    // Ramp
    int16_t start;           // Beginning strength level.
    int16_t end;             // Ending strength level.

    // Envelope
    uint16_t attack_length;   // Duration of the attack.
    uint16_t attack_level;    // Level at the start of the attack.
    uint16_t fade_length;     // Duration of the fade.
    uint16_t fade_level;      // Level at the end of the fade.
};

struct Moss_HapticConstant {
    /* Header */
    Moss_HapticEffectType type;      // HAPTIC_CONSTANT
    Moss_HapticDirection direction;  // Direction of the effect.

    /* Replay */
    uint32_t length;          // Duration of the effect.
    uint16_t delay;           // Delay before starting the effect.

    /* Trigger */
    uint16_t button;          // Button that triggers the effect.
    uint16_t interval;        // How soon it can be triggered again after button.

    /* Constant */
    int16_t level;           // Strength of the constant effect.

    /* Envelope */
    uint16_t attack_length;   // Duration of the attack.
    uint16_t attack_level;    // Level at the start of the attack.
    uint16_t fade_length;     // Duration of the fade.
    uint16_t fade_level;      // Level at the end of the fade.
};

struct Moss_HapticCondition {
    // Header
    Moss_HapticEffectType type;      // HAPTIC_SPRING, HAPTIC_DAMPER, HAPTIC_INERTIA or HAPTIC_FRICTION
    Moss_HapticDirection direction;  // Direction of the effect.

    // Replay
    uint32_t length;          // Duration of the effect.
    uint16_t delay;           // Delay before starting the effect.

    // Trigger
    uint16_t button;          // Button that triggers the effect.
    uint16_t interval;        // How soon it can be triggered again after button.

    // Condition
    uint16_t right_sat[3];    // Level when joystick is to the positive side; maxF.
    uint16_t left_sat[3];     // Level when joystick is to the negative side; maxF.
    int16_t right_coeff[3];  // How fast to increase the force towards the positive side.
    int16_t left_coeff[3];   // How fast to increase the force towards the negative side.
    uint16_t deadband[3];     // Size of the dead zone; maxF: whole axis-range when 0-centered.
    int16_t center[3];       // Position of the dead zone.
};

struct Moss_HapticPeriodic {
    /* Header */
    Moss_HapticEffectType type;      // HAPTIC_SINE, HAPTIC_SQUARE HAPTIC_TRIANGLE, HAPTIC_SAWTOOTHUP or HAPTIC_SAWTOOTHDOWN
    Moss_HapticDirection direction;  // Direction of the effect.

    /* Replay */
    uint32_t length;      // Duration of the effect.
    uint16_t delay;       // Delay before starting the effect.

    /* Trigger */
    uint16_t button;      // Button that triggers the effect.
    uint16_t interval;    // How soon it can be triggered again after button.

    /* Periodic */
    uint16_t period;      // Period of the wave.
    int16_t magnitude;   // Peak value; if negative, equivalent to 180 degrees extra phase shift.
    int16_t offset;      // Mean value of the wave.
    uint16_t phase;       // Positive phase shift given by hundredth of a degree.

    /* Envelope */
    uint16_t attack_length;   // Duration of the attack.
    uint16_t attack_level;    // Level at the start of the attack.
    uint16_t fade_length; // Duration of the fade.
    uint16_t fade_level;  // Level at the end of the fade.
};

union Moss_HapticEffect {
    uint16_t type;                        // Effect type.
    Moss_HapticConstant constant;       // Constant effect.
    Moss_HapticPeriodic periodic;       // Periodic effect.
    Moss_HapticCondition condition;     // Condition effect.
    Moss_HapticRamp ramp;               // Ramp effect.
    Moss_HapticLeftRight leftright;     // Left/Right effect.
    Moss_HapticCustom custom;           // Custom effect.
};


struct Moss_Haptic {
    // Replay - All effects have this
    uint32_t duration;        // Duration of effect (ms).
    uint16_t delay;           // Delay before starting effect.

    // Trigger - All effects have this
    uint16_t button;          // Button that triggers effect.
    uint16_t interval;        // How soon before effect can be triggered again.

    // Envelope - All effects except condition effects have this
    uint16_t attack_length;   // Duration of the attack (ms).
    uint16_t attack_level;    // Level at the start of the attack.
    uint16_t fade_length;     // Duration of the fade out (ms).
    uint16_t fade_level;      // Level at the end of the fade.
};

struct Moss_PathInfo {
    Moss_PathType type;      // the path type */
    uint64_t size;           // the file size in bytes */
    Moss_Time create_time;   // the time when the path was created */
    Moss_Time modify_time;   // the last time the path was modified */
    Moss_Time access_time;   // the last time the path was read */

    bool readable, writable, executable;
};

// =================================================
//                 Callback Type Definitions
// =================================================
//! @brief Callback for framebuffer resize events. @param width  New framebuffer width, in pixels. @param height New framebuffer height, in pixels.
typedef void (*Moss_FramebufferResizeCallback)(int width, int height);
//! @brief Callback for logical window size changes. @param width  New window width, in screen coordinates. @param height New window height, in screen coordinates.
typedef void (*Moss_WindowSizeCallback)(int width, int height);
//! @brief Callback for window position changes on screen. @param xpos New X coordinate of the window's top-left corner. @param ypos New Y coordinate of the window's top-left corner.
typedef void (*Moss_WindowPositionCallback)(int xpos, int ypos);
//! @brief Callback for window focus events. @param focused True if the window gained focus; false if it lost focus.
typedef void (*Moss_WindowFocusCallback)(bool focused);
//! @brief Callback for content scale changes (e.g., HiDPI scaling). @param xscale X-axis content scale factor. @param yscale Y-axis content scale factor.
typedef void (*Moss_WindowContentScaleCallback)(float xscale, float yscale);
//! @brief Callback for general window resize notifications (platform-driven). @param width  New window width in pixels. @param height New window height in pixels.
typedef void (*Moss_WindowResizeCallback)(int width, int height);
//! @brief Callback for monitor configuration changes (e.g. hotplug events). @param monitorName Name or ID of the monitor that changed. @param connected True if the monitor was connected; false if disconnected.
typedef void (*Moss_MonitorCallback)(const char* monitorName, bool connected);
//! @brief Callback invoked with the user's selection from a file dialog. @param userdata User data pointer passed through from the dialog call. @param filelist Null-terminated array of selected file paths. @param filter Index of the file filter that was active when the dialog closed.
typedef void (MOSS_CALL* Moss_DialogFileCallback)(void* userdata, const char* const* filelist, int filter);
//! @brief Callback invoked once per entry while iterating a directory. @param info Metadata for the current path entry. @param path Full path of the current entry. @param user_data User data pointer passed through from the iterate call.
typedef bool (*Moss_DirectoryIterateFn)(const Moss_PathInfo* info, const char* path, void* user_data);
//! @brief Callback invoked once per entry while enumerating a directory. @param info Metadata for the current path entry. @param path Full path of the current entry. @param user_data User data pointer passed through from the enumerate call.
typedef bool (*Moss_EnumerateDirectoryCallback)(const Moss_PathInfo* info, const char* path, void* user_data);

/*! @brief Initialization of Moss. Must be called before anything else. @param X X. @ingroup Moss */
//MOSS_API bool Moss_Init();
/*! @brief Terminates Moss. Must be called at the end.  @param X X. @ingroup Moss */
//MOSS_API void Moss_Terminate();


//void Moss_WindowHint(int hint, int value);
/**
 * @brief Creates a window with the requested title, dimensions, monitor, and optional shared context.
 * @param title Window or dialog title.
 * @param width Width in pixels or screen coordinates, as applicable.
 * @param height Height in pixels or screen coordinates, as applicable.
 * @param monitor Monitor associated with the operation.
 * @param share Value for `share`.
 * @return A handle to the created or opened resource, or `nullptr` on failure.
 */
MOSS_API Moss_Window* Moss_CreateWindow(const char* title, int width, int height, Moss_Monitor* monitor, Moss_Window* share);
/**
 * @brief Destroys a window and releases its associated platform resources.
 * @param window Window handle affected by this operation.
 */
MOSS_API void Moss_TerminateWindow(Moss_Window* window);
/**
 * @brief Displays a message box associated with a window.
 * @param title Window or dialog title.
 * @param message Value for `message`.
 * @param flags Options controlling the operation.
 * @param window Window handle affected by this operation.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_CreateMessageBox(const char* title, const char* message, Moss_MessageBoxFlags flags, Moss_Window* window);
/**
 * @brief Checks whether the window has been requested to close.
 * @param window Window handle affected by this operation.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_ShouldWindowClose(Moss_Window* window);
/**
 * @brief Processes pending platform events and updates input and window state.
 */
MOSS_API void Moss_PollEvents(void);
/**
 * @brief Returns the current window width.
 * @return The requested value.
 */
MOSS_API int Moss_GetWindowWidth();
/**
 * @brief Returns the current window height.
 * @return The requested value.
 */
MOSS_API int Moss_GetWindowHeight();
/**
 * @brief sets the window title.
 * @param window Window handle affected by this operation.
 * @param title Window or dialog title.
 */
MOSS_API void Moss_SetWindowTitle(Moss_Window* window, const char* title);
/**
 * @brief sets the window icon.
 * @param window Window handle affected by this operation.
 * @param image Value for `image`.
 */
MOSS_API void Moss_SetWindowIcon(Moss_Window* window, Moss_Image image);
/**
 * @brief Requests that the specified window close.
 * @param window Window handle affected by this operation.
 */
MOSS_API void Moss_CloseWindow(Moss_Window* window);

//MOSS_API void Moss_WindowShow(Moss_Window* window, bool show); 
//MOSS_API void Moss_WindowClose(Moss_Window* window); 
//MOSS_API void Moss_WindowSetOpacity(Moss_Window* window, float opacity);


/**
 * @brief Returns the primary monitor, if available.
 * @return A pointer to the requested resource, or `nullptr` if unavailable.
 */
MOSS_API Moss_Monitor* Moss_MonitorGetPrimary();
/**
 * @brief Returns the secondary monitor, if available.
 * @return A pointer to the requested resource, or `nullptr` if unavailable.
 * @note Returns `nullptr` when a secondary monitor is not available.
 */
MOSS_API Moss_Monitor* Moss_MonitorGetSecondary();
/**
 * @brief Retrieves or configures the monitor get physical size.
 * @param monitor Monitor associated with the operation.
 * @param width_mm Destination for monitor width in millimetres.
 * @param height_mm Destination for monitor height in millimetres.
 */
MOSS_API void Moss_MonitorGetPhysicalSize(Moss_Monitor* monitor, int* width_mm, int* height_mm);
/**
 * @brief Retrieves or configures the monitor get content scale.
 * @param monitor Monitor associated with the operation.
 * @param xscale Horizontal content scale factor.
 * @param yscale Vertical content scale factor.
 */
MOSS_API void Moss_MonitorGetContentScale(Moss_Monitor* monitor, float* xscale, float* yscale);
/**
 * @brief Retrieves or configures the monitor get position.
 * @param monitor Monitor associated with the operation.
 * @param x X coordinate or output value.
 * @param y Y coordinate or output value.
 */
MOSS_API void Moss_MonitorGetPosition(Moss_Monitor* monitor, int* x, int* y);
/**
 * @brief Retrieves or configures the monitor get name.
 * @param monitor Monitor associated with the operation.
 * @return A pointer to the requested string, or `nullptr` if unavailable.
 */
MOSS_API const char* Moss_MonitorGetName(Moss_Monitor* monitor);
/**
 * @brief Retrieves or configures the monitor set gamma ramp.
 * @param monitor Monitor associated with the operation.
 * @param gammaRamp Value for `gammaRamp`.
 */
MOSS_API void Moss_MonitorSetGammaRamp(Moss_Monitor* monitor, const Moss_GammaRamp* gammaRamp);
/**
 * @brief Retrieves or configures the monitor get gamma ramp.
 * @param monitor Monitor associated with the operation.
 * @return A pointer to the requested resource, or `nullptr` if unavailable.
 */
MOSS_API Moss_GammaRamp* Moss_MonitorGetGammaRamp(Moss_Monitor* monitor);
/**
 * @brief Retrieves or configures the monitor set gamma.
 * @param monitor Monitor associated with the operation.
 * @param gamma Gamma correction value.
 */
MOSS_API void Moss_MonitorSetGamma(Moss_Monitor* monitor, float gamma);



/**
 * @brief Checks whether a keyboard key is currently pressed.
 * @param key Keyboard key to query.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_IsKeyPressed(Keyboard  key);
/**
 * @brief Checks whether a keyboard key is currently released.
 * @param key Keyboard key to query.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_IsReleased(Keyboard  key);
/**
 * @brief Checks whether a keyboard key transitioned to the pressed state.
 * @param key Keyboard key to query.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_IsKeyJustPressed(Keyboard  key);
/**
 * @brief Checks whether a keyboard key transitioned to the released state.
 * @param key Keyboard key to query.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_IsKeyJustReleased(Keyboard  key);
/**
 * @brief Returns the keyboard key reported by the current input state.
 * @return The value or handle requested by the function.
 */
MOSS_API Keyboard Moss_InputGetKey();

/**
 * @brief Checks whether a mouse button is currently pressed.
 * @param button Input button to query.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_IsMousePressed(Mouse button);
/**
 * @brief Checks whether a mouse button is currently released.
 * @param button Input button to query.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_IsMouseReleased(Mouse button);
/**
 * @brief Checks whether a mouse button transitioned to the pressed state.
 * @param button Input button to query.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_IsMouseJustPressed(Mouse button);
/**
 * @brief Checks whether a mouse button transitioned to the released state.
 * @param button Input button to query.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_IsMouseJustReleased(Mouse button);
/**
 * @brief Returns the mouse button reported by the current input state.
 * @return The value or handle requested by the function.
 */
MOSS_API Mouse Moss_InputGetMouseButton();
/**
 * @brief Retrieves the mouse position.
 * @param x X coordinate or output value.
 * @param y Y coordinate or output value.
 */
MOSS_API void Moss_GetMousePosition(int* x, int* y);
/**
 * @brief sets the mouse position.
 * @param x X coordinate or output value.
 * @param y Y coordinate or output value.
 */
MOSS_API void Moss_SetMousePosition(int x, int y);
/**
 * @brief sets the mouse visible.
 * @param visible Whether the mouse cursor should be visible.
 */
MOSS_API void Moss_SetMouseVisible(bool visible);


// Gamepad management
/**
 * @brief Returns the number of gamepads currently known to the platform layer.
 * @return The requested value.
 */
MOSS_API int Moss_GetNumGamepads(void);
/**
 * @brief Opens a gamepad by its identifier.
 * @param id Identifier of the requested device or resource.
 * @return A handle to the created or opened resource, or `nullptr` on failure.
 */
MOSS_API Moss_Gamepad* Moss_OpenGamepad(Moss_GamepadID id);
/**
 * @brief Closes a previously opened gamepad handle.
 * @param gp Gamepad handle used by this operation.
 */
MOSS_API void Moss_CloseGamepad(Moss_Gamepad* gp);
/**
 * @brief Checks whether the specified gamepad is connected.
 * @param gp Gamepad handle used by this operation.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_GamepadConnected(Moss_Gamepad* gp);
/**
 * @brief Refreshes the platform's gamepad state.
 */
MOSS_API void Moss_UpdateGamepads(void); // poll / refresh all gamepads

// Button & axis
/**
 * @brief Checks whether gamepad button pressed.
 * @param gp Gamepad handle used by this operation.
 * @param button Input button to query.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_IsGamepadButtonPressed(Moss_Gamepad* gp, Moss_GamepadButton button);
/**
 * @brief Checks whether gamepad button just pressed.
 * @param gp Gamepad handle used by this operation.
 * @param button Input button to query.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_IsGamepadButtonJustPressed(Moss_Gamepad* gp, Moss_GamepadButton button);
/**
 * @brief Checks whether gamepad button just released.
 * @param gp Gamepad handle used by this operation.
 * @param button Input button to query.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_IsGamepadButtonJustReleased(Moss_Gamepad* gp, Moss_GamepadButton button);
/**
 * @brief Retrieves the gamepad axis.
 * @param gp Gamepad handle used by this operation.
 * @param axis Gamepad axis to query or configure.
 * @return The value or handle requested by the function.
 */
MOSS_API float Moss_GetGamepadAxis(Moss_Gamepad* gp, GamepadAxis axis);
/**
 * @brief Retrieves the gamepad orientation.
 * @param gp Gamepad handle used by this operation.
 * @param w Output or destination for `w`.
 * @param x X coordinate or output value.
 * @param y Y coordinate or output value.
 * @param z Output or destination for `z`.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_GetGamepadOrientation(Moss_Gamepad* gp, float* w, float* x, float* y, float* z);
/**
 * @brief Retrieves the gamepad player space gyro.
 * @param gp Gamepad handle used by this operation.
 * @param outX Destination for the X-axis result.
 * @param outY Destination for the Y-axis result.
 * @param yawRelaxFactor Output or destination for `yawRelaxFactor`.
 */
MOSS_API void Moss_GetGamepadPlayerSpaceGyro(Moss_Gamepad* gp, float* outX, float* outY, const float yawRelaxFactor = 1.41f);
/**
 * @brief Retrieves the gamepad world space gyro.
 * @param gp Gamepad handle used by this operation.
 * @param outX Destination for the X-axis result.
 * @param outY Destination for the Y-axis result.
 * @param sideReductionThreshold Value for `sideReductionThreshold`.
 */
MOSS_API void Moss_GetGamepadWorldSpaceGyro(Moss_Gamepad* gp, float* outX, float* outY, const float sideReductionThreshold = 0.125f);
/**
 * @brief Resets the reset gamepad calibration.
 * @param gp Gamepad handle used by this operation.
 */
MOSS_API void Moss_ResetGamepadCalibration(Moss_Gamepad* gp);

/**
 * @brief sets the gamepad axis deadzone.
 * @param axis Gamepad axis to query or configure.
 * @param dz Value for `dz`.
 */
void Moss_SetGamepadAxisDeadzone(GamepadAxis axis, float dz);

/**
 * @brief sets the gamepad axis inverted.
 * @param axis Gamepad axis to query or configure.
 * @param inverted Value for `inverted`.
 */
void Moss_SetGamepadAxisInverted(GamepadAxis axis, bool inverted);
// Rumble / LED
/**
 * @brief Triggers rumble on the rumble gamepad.
 * @param gp Gamepad handle used by this operation.
 * @param low Low-frequency motor intensity.
 * @param high High-frequency motor intensity.
 * @param duration_ms Rumble duration in milliseconds.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_RumbleGamepad(Moss_Gamepad* gp, uint16_t low, uint16_t high, uint32_t duration_ms);
/**
 * @brief Triggers rumble on the rumble gamepad triggers.
 * @param gp Gamepad handle used by this operation.
 * @param left Left-side intensity or value.
 * @param right Right-side intensity or value.
 * @param duration_ms Rumble duration in milliseconds.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_RumbleGamepadTriggers(Moss_Gamepad* gp, uint16_t left, uint16_t right, uint32_t duration_ms);
/**
 * @brief sets the gamepad led.
 * @param gp Gamepad handle used by this operation.
 * @param r Red color component.
 * @param g Green color component.
 * @param b Blue color component.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_SetGamepadLED(Moss_Gamepad* gp, uint8_t r, uint8_t g, uint8_t b);

// Metadata
/**
 * @brief Retrieves the gamepad name.
 * @param gp Gamepad handle used by this operation.
 * @return A pointer to the requested string, or `nullptr` if unavailable.
 */
MOSS_API const char* Moss_GetGamepadName(Moss_Gamepad* gp);
/**
 * @brief Retrieves the gamepad id.
 * @param gp Gamepad handle used by this operation.
 * @return The value or handle requested by the function.
 */
MOSS_API Moss_GamepadID Moss_GetGamepadID(Moss_Gamepad* gp);
/**
 * @brief Retrieves the gamepad player index.
 * @param gp Gamepad handle used by this operation.
 * @return The value or handle requested by the function.
 */
MOSS_API int Moss_GetGamepadPlayerIndex(Moss_Gamepad* gp);
/**
 * @brief Retrieves the gamepad power info.
 * @param gp Gamepad handle used by this operation.
 * @param percent Destination for the battery percentage.
 * @return The value or handle requested by the function.
 */
MOSS_API Moss_PowerState Moss_GetGamepadPowerInfo(Moss_Gamepad* gp, int* percent);
/**
 * @brief Retrieves the num gamepad touchpads.
 * @param gp Gamepad handle used by this operation.
 * @return The value or handle requested by the function.
 */
MOSS_API int Moss_GetNumGamepadTouchpads(Moss_Gamepad* gp);
/**
 * @brief Retrieves the num gamepad touchpad fingers.
 * @param gp Gamepad handle used by this operation.
 * @return The value or handle requested by the function.
 */
MOSS_API int Moss_GetNumGamepadTouchpadFingers(Moss_Gamepad* gp);
/**
 * @brief Retrieves the gamepad touchpad finger.
 * @param gp Gamepad handle used by this operation.
 * @param pad Output or destination for `pad`.
 * @param finger Value for `finger`.
 * @param down Value for `down`.
 * @param x X coordinate or output value.
 * @param y Y coordinate or output value.
 * @param pressure Output or destination for `pressure`.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_GetGamepadTouchpadFinger(Moss_Gamepad* gp, int pad, int finger, bool* down, float* x, float* y, float* pressure);

// Mapping & type
/**
 * @brief Retrieves the gamepad mapping.
 * @param gp Gamepad handle used by this operation.
 * @return A pointer to the requested string, or `nullptr` if unavailable.
 */
MOSS_API const char* Moss_GetGamepadMapping(Moss_Gamepad* gp);
/**
 * @brief sets the gamepad mapping.
 * @param gp Gamepad handle used by this operation.
 * @param mapping Value for `mapping`.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_SetGamepadMapping(Moss_Gamepad* gp, const char* mapping);
/**
 * @brief Reloads the reload gamepad mappings.
 */
MOSS_API void Moss_ReloadGamepadMappings(void);

/**
 * @brief Retrieves the current the input get gamepad button.
 * @return The value or handle requested by the function.
 */
MOSS_API Moss_GamepadButton Moss_InputGetGamepadButton();
/**
 * @brief Retrieves the current the input get gamepad axis.
 * @return The value or handle requested by the function.
 */
MOSS_API GamepadAxis Moss_InputGetGamepadAxis();

/**
 * @brief Retrieves the pen device type.
 * @param instance_id Value for `instance_id`.
 * @return The value or handle requested by the function.
 */
MOSS_API Moss_PenDeviceType Moss_GetPenDeviceType(Moss_PenID instance_id);
/**
 * @brief Retrieves the touch device name.
 * @param touchID Value for `touchID`.
 * @return A pointer to the requested string, or `nullptr` if unavailable.
 */
MOSS_API const char* Moss_GetTouchDeviceName(Moss_TouchID touchID);
/**
 * @brief Retrieves the touch devices.
 * @param count Destination for the number of returned items.
 * @return A pointer to the requested resource, or `nullptr` if unavailable.
 * @note The lifetime and ownership of returned data depend on the platform implementation; consult the implementation before retaining the pointer.
 */
MOSS_API Moss_TouchID* Moss_GetTouchDevices(int* count);
/**
 * @brief Retrieves the touch device type.
 * @param touchID Value for `touchID`.
 * @return The value or handle requested by the function.
 */
MOSS_API Moss_TouchDeviceType Moss_GetTouchDeviceType(Moss_TouchID touchID);
/**
 * @brief Retrieves the touch fingers.
 * @param touchID Value for `touchID`.
 * @param count Destination for the number of returned items.
 * @return A pointer to the requested resource, or `nullptr` if unavailable.
 * @note The lifetime and ownership of returned data depend on the platform implementation; consult the implementation before retaining the pointer.
 */
MOSS_API Moss_Finger** Moss_GetTouchFingers(Moss_TouchID touchID, int* count);

/**
 * @brief Determines whether a gamepad supports the specified motion sensor.
 * @param gp Gamepad handle used by this operation.
 * @param sensor Motion sensor to query or configure.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_GamepadHasSensor(Moss_Gamepad* gp, Moss_GamepadSensorType sensor);

/**
 * @brief Enables or disables a supported gamepad motion sensor.
 * @param gp Gamepad handle used by this operation.
 * @param sensor Motion sensor to query or configure.
 * @param enabled Whether the feature should be enabled.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_SetGamepadSensorEnabled(Moss_Gamepad* gp, Moss_GamepadSensorType sensor, bool enabled);

/**
 * @brief Retrieves the latest available data from a gamepad motion sensor.
 * @param gp Gamepad handle used by this operation.
 * @param sensor Motion sensor to query or configure.
 * @param data Data structure used to receive or provide values.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 * @note Returns `false` when the sensor is unavailable or no valid sample has been received.
 */
MOSS_API bool Moss_GetGamepadSensorData(Moss_Gamepad* gp, Moss_GamepadSensorType sensor, Moss_GamepadSensorData* data);

/*            Haptic Feedback          */
/**
 * @brief Opens the haptic.
 * @param id Identifier of the requested device or resource.
 * @return A handle to the created or opened resource, or `nullptr` on failure.
 */
MOSS_API Moss_Haptic* Moss_OpenHaptic(Moss_HapticID id);
/**
 * @brief Closes the haptic.
 * @param haptic Haptic device handle used by this operation.
 */
MOSS_API void Moss_CloseHaptic(Moss_Haptic* haptic);
/**
 * @brief Creates the haptic effect.
 * @param haptic Haptic device handle used by this operation.
 * @return The value or handle requested by the function.
 */
MOSS_API Moss_HapticID Moss_CreateHapticEffect(Moss_Haptic* haptic);
/**
 * @brief Destroys the haptic effect.
 * @param haptic Haptic device handle used by this operation.
 */
MOSS_API void Moss_DestroyHapticEffect(Moss_Haptic* haptic);
/**
 * @brief Retrieves the haptic effect status.
 * @param haptic Haptic device handle used by this operation.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_GetHapticEffectStatus(Moss_Haptic* haptic);
/**
 * @brief Retrieves the haptic features.
 * @param haptic Haptic device handle used by this operation.
 * @return The value or handle requested by the function.
 */
MOSS_API uint32_t Moss_GetHapticFeatures(Moss_Haptic* haptic);
/**
 * @brief Retrieves the haptic from id.
 * @param haptic Haptic device handle used by this operation.
 * @return A pointer to the requested resource, or `nullptr` if unavailable.
 */
MOSS_API Moss_Haptic* Moss_GetHapticFromID(Moss_Haptic* haptic);
/**
 * @brief Retrieves the haptic id.
 * @param haptic Haptic device handle used by this operation.
 * @return A pointer to the requested resource, or `nullptr` if unavailable.
 */
MOSS_API Moss_HapticID* Moss_GetHapticID(Moss_Haptic* haptic);
/**
 * @brief Retrieves the haptic name.
 * @param haptic Haptic device handle used by this operation.
 * @return A pointer to the requested string, or `nullptr` if unavailable.
 */
MOSS_API const char* Moss_GetHapticName(Moss_Haptic* haptic);
/**
 * @brief Retrieves the haptic name for id.
 * @param haptic Haptic device handle used by this operation.
 * @return A pointer to the requested string, or `nullptr` if unavailable.
 */
MOSS_API const char* Moss_GetHapticNameForID(Moss_Haptic* haptic);
/**
 * @brief Retrieves the haptics.
 * @param haptic Haptic device handle used by this operation.
 * @return A pointer to the requested resource, or `nullptr` if unavailable.
 */
MOSS_API Moss_HapticID* Moss_GetHaptics(Moss_Haptic* haptic);
/**
 * @brief Retrieves the max haptic effects.
 * @param haptic Haptic device handle used by this operation.
 * @return The value or handle requested by the function.
 */
MOSS_API int Moss_GetMaxHapticEffects(Moss_Haptic* haptic);
/**
 * @brief Retrieves the max haptic effects playing.
 * @param haptic Haptic device handle used by this operation.
 * @return The value or handle requested by the function.
 */
MOSS_API int Moss_GetMaxHapticEffectsPlaying(Moss_Haptic* haptic);
/**
 * @brief Retrieves the num haptic axes.
 * @param haptic Haptic device handle used by this operation.
 * @return The value or handle requested by the function.
 */
MOSS_API int Moss_GetNumHapticAxes(Moss_Haptic* haptic);
/**
 * @brief Performs the haptic effect supported.
 * @param haptic Haptic device handle used by this operation.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_HapticEffectSupported(Moss_Haptic* haptic);
/**
 * @brief Performs the haptic rumble supported.
 * @param haptic Haptic device handle used by this operation.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_HapticRumbleSupported(Moss_Haptic* haptic);
/**
 * @brief Performs the init haptic rumble.
 * @param joystick Joystick or gamepad handle used by this operation.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_InitHapticRumble(Moss_Haptic* joystick);
/**
 * @brief Checks whether joystick haptic.
 * @param joystick Joystick or gamepad handle used by this operation.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_IsJoystickHaptic(Moss_Gamepad* joystick);
/**
 * @brief Checks whether mouse haptic.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_IsMouseHaptic(void);
/**
 * @brief Opens the haptic from joystick.
 * @param joystick Joystick or gamepad handle used by this operation.
 * @return A pointer to the requested resource, or `nullptr` if unavailable.
 */
MOSS_API Moss_Haptic* Moss_OpenHapticFromJoystick(Moss_Gamepad* joystick);
/**
 * @brief Opens the haptic from mouse.
 * @return A pointer to the requested resource, or `nullptr` if unavailable.
 */
MOSS_API Moss_Haptic* Moss_OpenHapticFromMouse(void);
/**
 * @brief Pauses the pause haptic.
 * @param haptic Haptic device handle used by this operation.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_PauseHaptic(Moss_Haptic* haptic);
/**
 * @brief Starts playback of the play haptic rumble.
 * @param haptic Haptic device handle used by this operation.
 * @param strength Rumble strength.
 * @param length Number of bytes or duration, as indicated by the parameter type.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_PlayHapticRumble(Moss_Haptic* haptic, float strength, uint32_t length);
/**
 * @brief Resumes the resume haptic.
 * @param haptic Haptic device handle used by this operation.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_ResumeHaptic(Moss_Haptic* haptic);
/**
 * @brief Runs the run haptic effect.
 * @param haptic Haptic device handle used by this operation.
 * @param iterations Value for `iterations`.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_RunHapticEffect(Moss_Haptic* haptic, uint32_t iterations);
/**
 * @brief sets the haptic autocenter.
 * @param haptic Haptic device handle used by this operation.
 * @param center Value for `center`.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_SetHapticAutocenter(Moss_Haptic* haptic, int center);
/**
 * @brief sets the haptic gain.
 * @param haptic Haptic device handle used by this operation.
 * @param gain Value for `gain`.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_SetHapticGain(Moss_Haptic* haptic, int gain);
/**
 * @brief Stops the stop haptic effect.
 * @param haptic Haptic device handle used by this operation.
 * @param effect Value for `effect`.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_StopHapticEffect(Moss_Haptic* haptic, Moss_HapticEffectID effect);
/**
 * @brief Stops the stop haptic effects.
 * @param haptic Haptic device handle used by this operation.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_StopHapticEffects(Moss_Haptic* haptic);
/**
 * @brief Stops the stop haptic rumble.
 * @param haptic Haptic device handle used by this operation.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_StopHapticRumble(Moss_Haptic* haptic);
/**
 * @brief Updates the update haptic effect.
 * @param haptic Haptic device handle used by this operation.
 * @param effect Value for `effect`.
 * @param data Data structure used to receive or provide values.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_UpdateHapticEffect(Moss_Haptic* haptic, Moss_HapticEffectID effect, const Moss_HapticEffect* data);


/*              CPU Info          */
/**
 * @brief Returns the number of logical CPU cores available.
 * @return The requested value.
 */
MOSS_API int Moss_GetAvailableCPUCores(void);
/**
 * @brief Returns the CPU cache-line size in bytes.
 * @return The requested value.
 */
MOSS_API int Moss_GetCPUCacheLineSize(void);
/**
 * @brief Returns the amount of system RAM, in the unit defined by the implementation.
 * @return The requested value.
 */
MOSS_API int Moss_GetSystemRAM(void);


/*             OS Spesific        */
/**
 * @brief Opens a URL using the platform's default handler.
 * @param url URL to open.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_OpenURL(const char* url);
/**
 * @brief Retrieves the operating system locale information.
 * @return Pointer to locale information, or `nullptr` if unavailable.
 */
MOSS_API Moss_Locale* Moss_GetLocale();
/**
 * @brief Retrieves the system power state and, when available, battery information.
 * @param seconds Destination for the estimated remaining battery time in seconds.
 * @param percent Destination for the battery percentage.
 * @return The value or handle requested by the function.
 */
MOSS_API Moss_PowerState Moss_GetPowerInfo(int *seconds, int *percent);
/**
 * @brief Checks whether a process with the specified executable name is running.
 * @param executable_path Value for `executable_path`.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_IsProcessRunningByName(const char* executable_path);
/**
 * @brief Loads a dynamic library from the specified path.
 * @param lib_path Path to the dynamic library.
 * @return The requested handle or symbol address, or `nullptr` on failure.
 */
MOSS_API void* Moss_LoadDynamicLibrary(const char* lib_path);
/**
 * @brief Looks up a symbol in a loaded dynamic library.
 * @param handle Handle to the loaded library or platform resource.
 * @param symbol_name Name of the symbol to look up.
 * @return The requested handle or symbol address, or `nullptr` on failure.
 */
MOSS_API void* Moss_GetLibrarySymbol(void* handle, const char* symbol_name);
/**
 * @brief Unloads a dynamic library previously loaded by Moss.
 * @param handle Handle to the loaded library or platform resource.
 */
MOSS_API void Moss_UnloadDynamicLibrary(void* handle);

// =================================================
//              Callback Registration API
// =================================================

/**
 * @brief sets the framebuffer resize callback.
 * @param callback Callback function to register or invoke.
 */
MOSS_API void Moss_SetFramebufferResizeCallback(Moss_FramebufferResizeCallback callback);
/**
 * @brief sets the window size callback.
 * @param callback Callback function to register or invoke.
 */
MOSS_API void Moss_SetWindowSizeCallback(Moss_WindowSizeCallback callback);
/**
 * @brief sets the window resize callback.
 * @param callback Callback function to register or invoke.
 */
MOSS_API void Moss_SetWindowResizeCallback(Moss_WindowResizeCallback callback);
/**
 * @brief sets the window position callback.
 * @param callback Callback function to register or invoke.
 */
MOSS_API void Moss_SetWindowPositionCallback(Moss_WindowPositionCallback callback);
/**
 * @brief sets the window focus callback.
 * @param callback Callback function to register or invoke.
 */
MOSS_API void Moss_SetWindowFocusCallback(Moss_WindowFocusCallback callback);
/**
 * @brief sets the window content scale callback.
 * @param callback Callback function to register or invoke.
 */
MOSS_API void Moss_SetWindowContentScaleCallback(Moss_WindowContentScaleCallback callback);
/**
 * @brief sets the monitor callback.
 * @param callback Callback function to register or invoke.
 */
MOSS_API void Moss_SetMonitorCallback(Moss_MonitorCallback callback);


/**
 * @brief Returns the available camera identifiers.
 * @param count Destination for the number of returned items.
 * @return A pointer to the requested resource, or `nullptr` if unavailable.
 * @note The lifetime and ownership of returned data depend on the platform implementation; consult the implementation before retaining the pointer.
 */
MOSS_API Moss_CameraID* Moss_GetCameras(int* count);
/* Returned array valid until next call or shutdown */
/**
 * @brief Retrieves the camera name.
 * @param camera_id Identifier of the camera.
 * @return A pointer to the requested string, or `nullptr` if unavailable.
 */
MOSS_API const char* Moss_GetCameraName(Moss_CameraID camera_id);
/**
 * @brief Retrieves the camera position.
 * @param camera_id Identifier of the camera.
 * @return The value or handle requested by the function.
 */
MOSS_API Moss_CameraPosition Moss_GetCameraPosition(Moss_CameraID camera_id);
/**
 * @brief Retrieves the current camera driver.
 * @return A pointer to the requested string, or `nullptr` if unavailable.
 */
MOSS_API const char* Moss_GetCurrentCameraDriver(void);
/**
 * @brief Retrieves the num camera drivers.
 * @return The value or handle requested by the function.
 */
MOSS_API int Moss_GetNumCameraDrivers(void);
/**
 * @brief Returns the supported capture formats for a camera.
 * @param camera_id Identifier of the camera.
 * @param count Destination for the number of returned items.
 * @return A pointer to the requested resource, or `nullptr` if unavailable.
 * @note The lifetime and ownership of returned data depend on the platform implementation; consult the implementation before retaining the pointer.
 */
MOSS_API const Moss_CameraSpec* Moss_GetCameraSupportedFormats(Moss_CameraID camera_id, int* count);
/**
 * @brief Acquires a frame from an opened camera capture.
 * @param camera Camera capture handle.
 * @param timestamp_ns Destination for the frame timestamp in nanoseconds.
 * @return A pointer to the requested resource, or `nullptr` if unavailable.
 * @note Release a successfully acquired frame with `Moss_ReleaseCameraFrame()`.
 */
MOSS_API Moss_Surface* Moss_AcquireCameraFrame(Moss_Capture* camera, uint64_t* timestamp_ns);
/**
 * @brief Releases a previously acquired camera frame.
 * @param camera Camera capture handle.
 * @param frame Frame handle to release.
 */
MOSS_API void Moss_ReleaseCameraFrame(Moss_Capture* camera, Moss_Surface* frame);
/**
 * @brief Retrieves the camera format.
 * @param camera Camera capture handle.
 * @param out_spec Destination for the camera format specification.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_GetCameraFormat(Moss_Capture* camera, Moss_CameraSpec* out_spec);
/**
 * @brief Retrieves the camera permission state.
 * @param camera Camera capture handle.
 * @return The value or handle requested by the function.
 */
MOSS_API Moss_CameraPermissionState Moss_GetCameraPermissionState(Moss_Capture* camera);
/**
 * @brief Retrieves the camera properties.
 * @param camera Camera capture handle.
 * @return The value or handle requested by the function.
 */
MOSS_API Moss_PropertiesID Moss_GetCameraProperties(Moss_Capture* camera);
/**
 * @brief Closes the camera.
 * @param camera Camera capture handle.
 */
MOSS_API void Moss_CloseCamera(Moss_Capture* camera);
/**
 * @brief Retrieves the camera id.
 * @param camera Camera capture handle.
 * @return The value or handle requested by the function.
 */
MOSS_API Moss_CameraID Moss_GetCameraID(Moss_Capture* camera);
/**
 * @brief Retrieves the camera properties.
 * @param camera Camera capture handle.
 * @return The value or handle requested by the function.
 */
MOSS_API Moss_PropertiesID Moss_GetCameraProperties(Moss_Capture* camera);
/**
 * @brief Opens a camera capture using the requested format.
 * @param id Identifier of the requested device or resource.
 * @param spec Requested camera capture format.
 * @return A handle to the created or opened resource, or `nullptr` on failure.
 */
MOSS_API Moss_Capture* Moss_OpenCapture(Moss_CameraID id, const Moss_CameraSpec* spec);


/**
 * @brief Copies a file from a source path to a destination path.
 * @param src_path Source file path.
 * @param dst_path Destination file path.
 * @param overwrite Whether an existing destination may be overwritten.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_CopyFile(const char* src_path, const char* dst_path, bool overwrite);
/**
 * @brief Creates a directory at the specified path.
 * @param path Path to the file-system entry.
 * @param recursive Whether to include or process nested entries recursively.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_CreateDirectory(const char* path, bool recursive);
/**
 * @brief Removes a file or directory at the specified path.
 * @param path Path to the file-system entry.
 * @param recursive Whether to include or process nested entries recursively.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_RemovePath(const char* path, bool recursive);
/**
 * @brief Renames or moves a path to a new destination.
 * @param old_path Existing path.
 * @param new_path Destination path.
 * @param overwrite Whether an existing destination may be overwritten.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_RenamePath(const char* old_path, const char* new_path, bool overwrite);


/**
 * @brief Retrieves metadata for a file-system path.
 * @param path Path to the file-system entry.
 * @param out_info Destination for the path metadata.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_GetPathInfo(const char* path, Moss_PathInfo* out_info);
/**
 * @brief Writes the current working directory to the supplied buffer.
 * @param out_path Destination buffer for the resulting path.
 * @param max_len Capacity of the destination buffer.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_GetCurrentDirectory(char* out_path, int max_len);
/**
 * @brief Writes the application base path to the supplied buffer.
 * @param out_path Destination buffer for the resulting path.
 * @param max_len Capacity of the destination buffer.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_GetBasePath(char* out_path, int max_len);

/**
 * @brief Retrieves the path to a standard user folder.
 * @param folder Value for `folder`.
 * @param out_path Destination buffer for the resulting path.
 * @param max_len Capacity of the destination buffer.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_GetUserFolder(Moss_UserFolder folder, char* out_path, int max_len);
/**
 * @brief Retrieves the application-specific preference or configuration path.
 * @param org_name Organization name used to form the preference path.
 * @param app_name Application name used to form the preference path.
 * @param out_path Destination buffer for the resulting path.
 * @param max_len Capacity of the destination buffer.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_GetPrefPath(const char* org_name, const char* app_name, char* out_path, int max_len);
/**
 * @brief Enumerates entries in a directory and invokes a callback for each entry.
 * @param path Path to the file-system entry.
 * @param recursive Whether to include or process nested entries recursively.
 * @param callback Callback function to register or invoke.
 * @param user_data User-defined data passed to the callback.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 * @note The callback return value and traversal behavior should be checked against the implementation.
 */
MOSS_API bool Moss_EnumerateDirectory( const char* path, bool recursive, Moss_DirectoryIterateFn callback, void* user_data);
/**
 * @brief Finds directory entries matching a glob pattern.
 * @param pattern Pattern used to match entries.
 * @param callback Callback function to register or invoke.
 * @param user_data User-defined data passed to the callback.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 * @note The callback return value and traversal behavior should be checked against the implementation.
 */
MOSS_API bool Moss_GlobDirectory(const char* pattern, Moss_DirectoryIterateFn callback, void* user_data);



struct Moss_DialogFileFilter {
    const char* name;
    const char* pattern;
};

/**
 * @brief Displays a file dialog configured with the supplied properties and filters.
 * @param callback Callback function to register or invoke.
 * @param userdata User data passed through to the callback.
 * @param window Window handle affected by this operation.
 * @param filters Available file-dialog filters.
 * @param nfilters Number of entries in the filters array.
 * @param default_location Initial directory or path shown by the dialog.
 */
MOSS_API void Moss_ShowFileDialogWithProperties(Moss_DialogFileCallback callback, void* userdata, Moss_Window* window, const Moss_DialogFileFilter* filters, int nfilters, const char* default_location);
/**
 * @brief Displays a dialog for selecting one or more folders.
 * @param callback Callback function to register or invoke.
 * @param userdata User data passed through to the callback.
 * @param window Window handle affected by this operation.
 * @param filters Available file-dialog filters.
 * @param nfilters Number of entries in the filters array.
 * @param default_location Initial directory or path shown by the dialog.
 * @param allow_many Whether multiple selections are allowed.
 */
MOSS_API void Moss_ShowOpenFolderDialog(Moss_DialogFileCallback callback, void* userdata, Moss_Window* window, const Moss_DialogFileFilter* filters, int nfilters, const char* default_location, bool allow_many);
/**
 * @brief Displays a dialog for selecting one or more files.
 * @param callback Callback function to register or invoke.
 * @param userdata User data passed through to the callback.
 * @param window Window handle affected by this operation.
 * @param default_location Initial directory or path shown by the dialog.
 * @param allow_many Whether multiple selections are allowed.
 */
MOSS_API void Moss_ShowOpenFileDialog(Moss_DialogFileCallback callback, void* userdata, Moss_Window* window, const char* default_location, bool allow_many);
/**
 * @brief Displays a save-file dialog.
 * @param type Value for `type`.
 * @param callback Callback function to register or invoke.
 * @param userdata User data passed through to the callback.
 * @param props Properties handle used to configure the operation.
 */
MOSS_API void Moss_ShowSaveFileDialog(Moss_FileDialogType type, Moss_DialogFileCallback callback, void* userdata, Moss_PropertiesID props);


enum class Moss_GlobFlags : uint32_t {
    None              = 0,
    
    // Core search configurations
    Recursive         = 1 << 0,  // Search child directories recursively (often triggered by '**')
    CaseInsensitive   = 1 << 1,  // Ignore matching casing ('*.png' matches 'IMAGE.PNG')
    
    // Filter controls
    FilesOnly         = 1 << 2,  // Only return file paths, omit directory names
    DirectoriesOnly   = 1 << 3,  // Only return directory names, omit file paths
    IncludeHidden     = 1 << 4,  // Include system/hidden objects (e.g., '.DS_Store', '.git')
    
    // Safety & Validation
    FollowSymlinks    = 1 << 5,  // Resolve and traverse symbolic links
    FailOnError       = 1 << 6   // Instantly halt search/return null if access permissions fail
};

inline Moss_GlobFlags operator|(Moss_GlobFlags a, Moss_GlobFlags b) { return static_cast<Moss_GlobFlags>(static_cast<unsigned int>(a) | static_cast<unsigned int>(b)); }
inline Moss_GlobFlags operator&(Moss_GlobFlags a, Moss_GlobFlags b) { return static_cast<Moss_GlobFlags>(static_cast<unsigned int>(a) & static_cast<unsigned int>(b)); }

/**
 * @brief Closes the storage.
 * @param storage Value for `storage`.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_CloseStorage(Moss_Storage* storage);
/**
 * @brief Performs the copy storage file.
 * @param storage Value for `storage`.
 * @param oldpath Existing path.
 * @param newpath Destination path.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_CopyStorageFile(Moss_Storage* storage, const char *oldpath, const char* newpath);
/**
 * @brief Creates the storage directory.
 * @param storage Value for `storage`.
 * @param path Path to the file-system entry.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_CreateStorageDirectory(Moss_Storage* storage, const char* path);
/**
 * @brief Performs the enumerate storage directory.
 * @param storage Value for `storage`.
 * @param path Path to the file-system entry.
 * @param callback Callback function to register or invoke.
 * @param userdata User data passed through to the callback.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_EnumerateStorageDirectory(Moss_Storage* storage, const char* path, Moss_EnumerateDirectoryCallback callback, void* userdata);
/**
 * @brief Retrieves the storage file size.
 * @param storage Value for `storage`.
 * @param path Path to the file-system entry.
 * @param length Number of bytes or duration, as indicated by the parameter type.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_GetStorageFileSize(Moss_Storage* storage, const char *path, uint64* length);
/**
 * @brief Retrieves the storage space remaining.
 * @param storage Value for `storage`.
 * @return The value or handle requested by the function.
 */
MOSS_API uint64_t Moss_GetStorageSpaceRemaining(Moss_Storage* storage);
/**
 * @brief Retrieves the storage path info.
 * @param storage Value for `storage`.
 * @param path Path to the file-system entry.
 * @param info Metadata for the current path entry.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_GetStoragePathInfo(Moss_Storage* storage, const char *path, Moss_PathInfo* info);
/**
 * @brief Performs the glob storage directory.
 * @param storage Value for `storage`.
 * @param path Path to the file-system entry.
 * @param pattern Pattern used to match entries.
 * @param flags Options controlling the operation.
 * @param count Destination for the number of returned items.
 * @return A pointer to the requested string, or `nullptr` if unavailable.
 */
MOSS_API char** Moss_GlobStorageDirectory(Moss_Storage* storage, const char* path, const char* pattern, Moss_GlobFlags flags, int* count);
/**
 * @brief Opens the file storage.
 * @param path Path to the file-system entry.
 * @return A pointer to the requested resource, or `nullptr` if unavailable.
 */
MOSS_API Moss_Storage* Moss_OpenFileStorage(const char* path);
/**
 * @brief Opens the storage.
 * @param iface Value for `iface`.
 * @param userdata User data passed through to the callback.
 * @return A pointer to the requested resource, or `nullptr` if unavailable.
 */
MOSS_API Moss_Storage* Moss_OpenStorage(const Moss_Storage* iface, void* userdata);
/**
 * @brief Opens the title storage.
 * @param override Value for `override`.
 * @param props Properties handle used to configure the operation.
 * @return A pointer to the requested resource, or `nullptr` if unavailable.
 */
MOSS_API Moss_Storage* Moss_OpenTitleStorage(const char* override, Moss_PropertiesID props);
/**
 * @brief Opens the user storage.
 * @param org Organization name.
 * @param app Application name.
 * @param props Properties handle used to configure the operation.
 * @return A pointer to the requested resource, or `nullptr` if unavailable.
 */
MOSS_API Moss_Storage* Moss_OpenUserStorage(const char* org, const char* app, Moss_PropertiesID props);
/**
 * @brief Performs the read storage file.
 * @param storage Value for `storage`.
 * @param path Path to the file-system entry.
 * @param destination Destination buffer for the file contents.
 * @param length Number of bytes or duration, as indicated by the parameter type.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_ReadStorageFile(Moss_Storage* storage, const char* path, void* destination, uint64 length);
/**
 * @brief Performs the remove storage path.
 * @param storage Value for `storage`.
 * @param path Path to the file-system entry.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_RemoveStoragePath(Moss_Storage* storage, const char* path);
/**
 * @brief Performs the rename storage path.
 * @param storage Value for `storage`.
 * @param oldpath Existing path.
 * @param newpath Destination path.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_RenameStoragePath(Moss_Storage* storage, const char* oldpath, const char* newpath);
/**
 * @brief Performs the storage ready.
 * @param storage Value for `storage`.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_StorageReady(Moss_Storage* storage);
/**
 * @brief Performs the write storage file.
 * @param storage Value for `storage`.
 * @param path Path to the file-system entry.
 * @param source Source data or path, depending on the operation.
 * @param length Number of bytes or duration, as indicated by the parameter type.
 * @return `true` if the operation succeeds or the queried condition is met; otherwise, `false`.
 */
MOSS_API bool Moss_WriteStorageFile(Moss_Storage* storage, const char* path, const void* source, uint64 length);


// Mobile Android / IOS App Creation
#if defined(MOSS_PLATFORM_EMBEDDED)
/**
 * @brief Creates a platform-embedded window for supported mobile or embedded applications.
 * @return A handle to the created or opened resource, or `nullptr` on failure.
 */
MOSS_API Moss_Window* Moss_CreateEmbeddedWindow();
/**
 * @brief Terminates the platform-embedded window.
 */
MOSS_API void Moss_Terminate_EmbeddedWindow();
/**
 * @brief Notifies the embedded platform layer of an orientation or focus-state change.
 * @param focused Value for `focused`.
 */
MOSS_API void Moss_EmbeddedWindow_OnOrientationChanged(bool focused);
/**
 * @brief Notifies the embedded platform layer that the application has resumed.
 */
MOSS_API void Moss_EmbeddedWindow_OnResume();
/**
 * @brief Notifies the embedded platform layer that the application has paused.
 */
MOSS_API void Moss_EmbeddedWindow_OnPause();
#endif

// OpenGL / OpenGL-ES
#if defined(MOSS_GRAPHICS_OPENGL) || defined(MOSS_GRAPHICS_OPENGLES)
/**
 * @brief Makes the window's graphics context current on the calling thread.
 * @param window Window handle affected by this operation.
 */
MOSS_API void Moss_MakeContextCurrent(Moss_Window* window);
/**
 * @brief Presents the rendered frame by swapping the graphics buffers.
 */
MOSS_API void Moss_SwapBuffers();
/**
 * @brief Sets the buffer-swap interval.
 * @param interval Value for `interval`.
 */
MOSS_API void Moss_SwapBuffersInterval(int interval);
/**
 * @brief Retrieves the address of a graphics or platform procedure by name.
 * @param procname Name of the procedure to look up.
 * @return The requested handle or symbol address, or `nullptr` on failure.
 */
MOSS_API void* Moss_GetProcAddress(const char* procname);
#endif // MOSS_USE_OPENGL

// Vulkan
#if defined(MOSS_GRAPHICS_VULKAN)
/**
 * @brief Creates a Vulkan surface for the specified window.
 * @param window Window handle affected by this operation.
 * @param vk_instance Vulkan instance used to create the surface.
 * @param allocator Optional Vulkan allocation callbacks.
 * @param vk_surface Destination for the created Vulkan surface.
 * @return The value or handle requested by the function.
 */
MOSS_API VkResult Moss_CreateWindowSurface(Moss_Window* window, VkInstance vk_instance, const VkAllocationCallbacks* allocator, VkSurfaceKHR* vk_surface);
/**
 * @brief Checks whether Vulkan is supported by the current platform.
 * @return The value or handle requested by the function.
 */
MOSS_API int Moss_VulkanSupported(void);
/*! @brief Initializes the Vulkan loader with the supplied procedure-address function.
 * @param loader Vulkan loader function used to resolve Vulkan entry points.
 * @ingroup Vulkan
 */
//MOSS_API void Moss_InitVulkanLoader(PFN_vkGetInstanceProcAddr loader);
/**
 * @brief Returns the Vulkan instance extensions required by the platform.
 * @param count Destination for the number of returned items.
 * @return A pointer to the requested string, or `nullptr` if unavailable.
 * @note The lifetime and ownership of returned data depend on the platform implementation; consult the implementation before retaining the pointer.
 */
MOSS_API const char** Moss_GetRequiredInstanceExtensions(uint32_t* count);
/**
 * @brief Retrieves a Vulkan instance procedure address by name.
 * @param instance Value for `instance`.
 * @param procname Name of the procedure to look up.
 * @return The requested handle or symbol address, or `nullptr` on failure.
 */
MOSS_API void* Moss_GetInstanceProcAddress(VkInstance instance, const char* procname);
/**
 * @brief Checks whether a Vulkan queue family can present to the window.
 * @param window Window handle affected by this operation.
 * @param device Vulkan physical device to query.
 * @param queuefamily Index of the Vulkan queue family to query.
 * @return The value or handle requested by the function.
 */
MOSS_API int Moss_GetPhysicalDevicePresentationSupport(Moss_Window* window, VkPhysicalDevice device, uint32_t queuefamily);
#endif // MOSS_USE_VULKAN

// Metal
#if defined(MOSS_GRAPHICS_METAL)
typedef void* Moss_MetalView;
/**
 * @brief Performs the metal_create view.
 * @param window Window handle affected by this operation.
 * @return The value or handle requested by the function.
 */
MOSS_API Moss_MetalView Moss_Metal_CreateView(Moss_Window *window);
/**
 * @brief Performs the metal_destroy view.
 * @param view Value for `view`.
 */
MOSS_API void Moss_Metal_DestroyView(Moss_MetalView view);
/**
 * @brief Performs the metal_get layer.
 * @param view Value for `view`.
 * @return The value or handle requested by the function.
 */
MOSS_API void* Moss_Metal_GetLayer(Moss_MetalView view);
/**
 * @brief Performs the metal_resize.
 * @param handle Handle to the loaded library or platform resource.
 * @param width Width in pixels or screen coordinates, as applicable.
 * @param height Height in pixels or screen coordinates, as applicable.
 */
MOSS_API void Moss_Metal_Resize(Moss_MetalView handle, uint32_t width, uint32_t height);
#endif // MOSS_GRAPHICS_METAL

#endif // MOSS_PLATFORM_H
