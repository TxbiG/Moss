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

struct Moss_Window;
struct Moss_Monitor;
struct Moss_Curser;
struct Moss_GamepadBinding;

struct Moss_Storage;

struct Moss_HapticDirection;
struct Moss_HapticConstant;
struct Moss_HapticPeriodic;
struct Moss_HapticCondition;
struct Moss_HapticRamp;
struct Moss_HapticLeftRight;
struct Moss_HapticCustom; 
union Moss_HapticEffect;
struct Moss_Haptic;

enum class Moss_Keyboard : uint16_t {
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
    LAST_KEY = KEY_KP_SUBTRACT
};

// Mouse
enum class Moss_Mouse { 
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

    GAMEPAD_BUTTON_LAST = GAMEPAD_BUTTON_DPAD_LEFT

    GAMEPAD_BUTTON_CROSS  =  GAMEPAD_BUTTON_A,
    GAMEPAD_BUTTON_CIRCLE =  GAMEPAD_BUTTON_B,
    GAMEPAD_BUTTON_SQUARE =  GAMEPAD_BUTTON_X,
    GAMEPAD_BUTTON_TRIANGLE = GAMEPAD_BUTTON_Y,
};

enum class Moss_Joystick {
    GAMEPAD_AXIS_LEFT_X,            // Left Stick X Axis
    GAMEPAD_AXIS_LEFT_Y,            // Left Stick Y Axis
    GAMEPAD_AXIS_RIGHT_X,           // Right Stick X Axis
    GAMEPAD_AXIS_RIGHT_Y,           // Right Stick Y Axis
    GAMEPAD_AXIS_LEFT_TRIGGER,      // Left Trigger
    GAMEPAD_AXIS_RIGHT_TRIGGER,     // Right Trigger
    GAMEPAD_AXIS_TOUCHPAD_X,        // (PS4/PS5)
    GAMEPAD_AXIS_TOUCHPAD_Y,
    GAMEPAD_AXIS_GYRO_X,
    GAMEPAD_AXIS_GYRO_Y,
    GAMEPAD_AXIS_GYRO_Z,
    MOSS_JOY_AXIS_LAST = GAMEPAD_AXIS_GYRO_Z
};

// Cursor
enum class Moss_CursorMode {
    VISIBLE = 0,             //
    HIDDEN = 0,              //
    CAPTURED = 0,            //
    CONFINED = 0,            //
    CONFINED_HIDDEN = 0,     //
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

enum class Moss_GamepadBindingType { NONE = 0, BUTTON, AXIS, HAT };

////////////////////////////////////////////////////////////////
//          Haptic          
////////////////////////////////////////////////////////////////
enum class Moss_HapticFeedbackType {
    AUTOCENTER,
    CARTESIAN,
    CONSTANT,
    CUSTOM,
    DAMPER,
    FRICTION,
    GAIN,
    INERTIA,
    INFINITY,
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
}


enum class Moss_HapticEffectType {
    CONSTANT,
    PERIODIC,
    CONDITION,
    RAMP,
    LEFTRIGHT,
    CUSTOM 
};

////////////////////////////////////////////////////////////////
// Window
////////////////////////////////////////////////////////////////
enum class Moss_WindowFlags {
    NOTITLEBAR = 0x00000001,        // no window decoration
    RESIZE_DISABLED = 0,
    ALWAYS_ON_TOP = 20,
    TRANSPARENT = 3,
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
    
    ALWAYS_ON_TOP    = 0x00008000,   // window should always be above others
    KEYBOARD_GRABBED = 0x00100000,   // window has grabbed Moss_keyboard input

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

enum class InputEventType {
    NONE,
    KEYDOWN,
    KEYUP,
    MOUSEBUTTONDOWN,
    MOUSEBUTTONUP,
    MOUSEMOVE,
    CONTROLLERBUTTONDOWN,
    CONTROLLERBUTTONUP
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


struct Moss_GammaRamp { uint8_t* size, red, green, blue; };
struct Moss_VideoMode { int width, height, redBits, greenBits, blueBits, refreshRate; };
struct Moss_Image { int width, height; unsigned char* pixels; };


struct Moss_Locale {
    char* country;
    char* language;
};


/*! @param type The type of encoding. @param dir The encoded direction. */
typedef struct Moss_HapticDirection { uint8_t type; int32_t dir[3]; };

/*! @param type Header (HAPTIC_LEFTRIGHT). @param length Duration of the effect in milliseconds. (Replay) @param large_magnitude Control of the large controller motor. (Rumble) @param small_magnitude Control of the small controller motor. (Rumble) */
typedef struct Moss_HapticLeftRight { 
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
typedef struct Moss_HapticCustom { 
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

typedef struct Moss_HapticRamp {
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

typedef struct Moss_HapticConstant {
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

typedef struct Moss_HapticCondition {
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

typedef struct Moss_HapticPeriodic {
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

typedef union Moss_HapticEffect {
    uint16_t type;                        // Effect type.
    Moss_HapticConstant constant;       // Constant effect.
    Moss_HapticPeriodic periodic;       // Periodic effect.
    Moss_HapticCondition condition;     // Condition effect.
    Moss_HapticRamp ramp;               // Ramp effect.
    Moss_HapticLeftRight leftright;     // Left/Right effect.
    Moss_HapticCustom custom;           // Custom effect.
};


typedef struct Moss_Haptic {
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

struct Moss_PenAxisEvent {
    Moss_EventType type;     // Moss_EVENT_PEN_AXIS */
    uint32_t reserved;
    uint64 timestamp;       // In nanoseconds, populated using Moss_GetTicksNS() */
    Moss_PenID which;        // The pen instance id */
    Moss_PenInputFlags pen_state;   // Complete pen input state at time of event */
    float x;                // X coordinate, relative to window */
    float y;                // Y coordinate, relative to window */
    Moss_PenAxis axis;       // Axis that has changed */
    float value;            // New value of axis */
} Moss_PenAxisEvent;

struct Moss_PenButtonEvent {
    Moss_EventType type; // Moss_EVENT_PEN_BUTTON_DOWN or Moss_EVENT_PEN_BUTTON_UP */
    uint32_t reserved;
    uint64 timestamp;   // In nanoseconds, populated using Moss_GetTicksNS() */
    Moss_PenID which;        // The pen instance id */
    Moss_PenInputFlags pen_state;   // Complete pen input state at time of event */
    float x;                // X coordinate, relative to window */
    float y;                // Y coordinate, relative to window */
    uint8_t button;       // The pen button index (first button is 1). */
    bool down;      // true if the button is pressed */
};

struct Moss_PenMotionEvent {
    Moss_EventType type; // Moss_EVENT_PEN_MOTION */
    uint32_t reserved;
    uint64 timestamp;   // In nanoseconds, populated using Moss_GetTicksNS() */
    Moss_PenID which;        // The pen instance id */
    Moss_PenInputFlags pen_state;   // Complete pen input state at time of event */
    float x;                // X coordinate, relative to window */
    float y;                // Y coordinate, relative to window */
};

struct Moss_PenProximityEvent {
    Moss_EventType type; // Moss_EVENT_PEN_PROXIMITY_IN or Moss_EVENT_PEN_PROXIMITY_OUT */
    uint32_t reserved;
    uint64 timestamp;   // In nanoseconds, populated using Moss_GetTicksNS() */
    Moss_PenID which;        // The pen instance id */
};

struct Moss_PenTouchEvent {
    Moss_EventType type;     // Moss_EVENT_PEN_DOWN or Moss_EVENT_PEN_UP */
    uint32_t reserved;
    uint64 timestamp;       // In nanoseconds, populated using Moss_GetTicksNS() */
    Moss_PenID which;        // The pen instance id */
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

struct Moss_PathInfo {
    Moss_PathType type;      // the path type */
    uint64 size;            // the file size in bytes */
    Moss_Time create_time;   // the time when the path was created */
    Moss_Time modify_time;   // the last time the path was modified */
    Moss_Time access_time;   // the last time the path was read */

    bool readable;
    bool writable;
    bool executable;
};


// =================================================
//                 Callback Type Definitions
// =================================================
//! @brief Callback for framebuffer resize events. @param width  New framebuffer width, in pixels. @param height New framebuffer height, in pixels.
typedef void (*Moss_FramebufferResizeCallback)(int width, int height);
//! @brief Callback for logical window size changes. @param width  New window width, in screen coordinates. @param height New window height, in screen coordinates.
typedef void (*Moss_WindowSizeCallback)(int width, int height);
//! @brief Callback for window position changes on screen. @param xpos New X coordinate of the window’s top-left corner. @param ypos New Y coordinate of the window’s top-left corner.
typedef void (*Moss_WindowPositionCallback)(int xpos, int ypos);
//! @brief Callback for window focus events. @param focused True if the window gained focus; false if it lost focus.
typedef void (*Moss_WindowFocusCallback)(bool focused);
//! @brief Callback for content scale changes (e.g., HiDPI scaling). @param xscale X-axis content scale factor. @param yscale Y-axis content scale factor.
typedef void (*Moss_WindowContentScaleCallback)(float xscale, float yscale);
//! @brief Callback for general window resize notifications (platform-driven). @param width  New window width in pixels. @param height New window height in pixels.
typedef void (*Moss_WindowResizeCallback)(int width, int height);
//! @brief Callback for monitor configuration changes (e.g. hotplug events). @param monitorName Name or ID of the monitor that changed. @param connected True if the monitor was connected; false if disconnected.
typedef void (*Moss_MonitorCallback)(const char* monitorName, bool connected);
//! @brief X. @param width X. @param X. */
typedef void (MOSS_CALL* Moss_DialogFileCallback)(void *userdata, const char * const *filelist, int filter);
//! @brief X. @param width X. @param X. */
typedef bool (*Moss_DirectoryIterateFn)(const Moss_PathInfo* info, const char* path, void* user_data);


/*! @brief Initialization of Moss. Must be called before anything else. @param X X. @ingroup Moss */
//MOSS_API bool Moss_Init();
/*! @brief Terminates Moss. Must be called at the end.  @param X X. @ingroup Moss */
//MOSS_API void Moss_Terminate();


//void Moss_WindowHint(int hint, int value);
/*! @brief Creates a window. @param title window title. @param width window width. @param height window height. @param monitor monitor in which the window presents. @param Moss_Window* shared window. @ingroup window */
MOSS_API Moss_Window* Moss_CreateWindow(const char* title, int width, int height, Moss_Monitor* monitor, Moss_Window* share);
/*! @brief Destorys the window. @param Moss_Window* window. @ingroup window */
MOSS_API void Moss_TerminateWindow(Moss_Window* window);
/*! @brief Creates a window popup. @param title popup window title. @param message message inside the popup 
    @param flags Popup window flags. e.g. | AbortEntryIgnore | CancelTryContinue | Help | Ok | OkCancel | RetryCancel | YesNo | YesNoCancel |. 
    @param Moss_Window* window. @ingroup window */
MOSS_API bool Moss_CreateMessageBox(const char* title, const char* message, Moss_MessageBoxFlags flags, Moss_Window* window);
/*! @brief Used in while loop if window should close. @param X X. @ingroup window */
MOSS_API bool Moss_ShouldWindowClose(Moss_Window* window);
/*! @brief Pollevents. @ingroup window */
MOSS_API void Moss_PollEvents(void);
/*! @brief X. @param X X.*/
MOSS_API int Moss_GetWindowWidth();
/*! @brief X. @param X X.*/
MOSS_API int Moss_GetWindowHeight();
/*! @brief X. @param X X @ingroup Window */
MOSS_API void Moss_SetWindowTitle(Moss_Window* window, const char* title);
/*! @brief X. @param X X @ingroup Window */
MOSS_API void Moss_SetWindowIcon(Moss_Window* window, Moss_Image image);
/*! @brief This is to call for closing a window. @param Moss_Window* window @ingroup Window */
MOSS_API void Moss_CloseWindow(Moss_Window* window);

//MOSS_API void Moss_WindowShow(Moss_Window* window, bool show); 
//MOSS_API void Moss_WindowClose(Moss_Window* window); 
//MOSS_API void Moss_WindowSetOpacity(Moss_Window* window, float opacity);


/*! @brief X. @param X X @ingroup Monitor */
MOSS_API Moss_Monitor* Moss_MonitorGetPrimary();
/*! @brief X. @param X X @ingroup Monitor @returns returns monitor unless if theres not a second will return nullptr. */
MOSS_API Moss_Monitor* Moss_MonitorGetSecondary();
/*! @brief X. @param X X @ingroup Monitor */
MOSS_API void Moss_MonitorGetPhysicalSize(Moss_Monitor monitor, int* width_mm, int* height_mm);
/*! @brief X. @param X X @ingroup Monitor */
MOSS_API void Moss_MonitorGetContentScale(Moss_Monitor monitor, float* xscale, float* yscale);
/*! @brief X. @param X X @ingroup Monitor */
MOSS_API void Moss_MonitorGetPosition(Moss_Monitor monitor, int* x, int* y);
/*! @brief X. @param X X @ingroup Monitor */
MOSS_API const char* Moss_MonitorGetName(Moss_Monitor monitor);
/*! @brief X. @param X X @ingroup Monitor */
MOSS_API void Moss_MonitorSetGammaRamp(Moss_Monitor monitor, const Moss_GammaRamp* gammaRamp);
/*! @brief X. @param X X @ingroup Monitor */
MOSS_API Moss_GammaRamp* Moss_MonitorGetGammaRamp(Moss_Monitor monitor);
/*! @brief X. @param X X @ingroup Monitor */
MOSS_API void Moss_MonitorSetGamma(Moss_Monitor monitor, float gamma);



MOSS_API bool Moss_IsKeyPressed(Moss_Key key);
MOSS_API bool Moss_IsKeyJustPressed(Moss_Key key);
MOSS_API bool Moss_IsKeyJustReleased(Moss_Key key);
MOSS_API Moss_Keyboard Moss_InputGetKey();

MOSS_API bool Moss_IsMousePressed(Moss_MouseButton button);
MOSS_API bool Moss_IsMouseJustPressed(Moss_MouseButton button);
MOSS_API bool Moss_IsMouseJustReleased(Moss_MouseButton button);
MOSS_API Moss_Keyboard Moss_InputGetMouse();
MOSS_API void Moss_GetMousePosition(int* x, int* y);
MOSS_API void Moss_SetMousePosition(int x, int y);
MOSS_API void Moss_SetMouseVisible(bool visible);


// Gamepad management
MOSS_API int Moss_GetNumGamepads(void);
MOSS_API Moss_Gamepad* Moss_OpenGamepad(Moss_GamepadID id);
MOSS_API void Moss_CloseGamepad(Moss_Gamepad* gp);
MOSS_API bool Moss_GamepadConnected(Moss_Gamepad* gp);
MOSS_API void Moss_UpdateGamepads(void); // poll / refresh all gamepads

// Button & axis
MOSS_API bool Moss_IsGamepadButtonPressed(Moss_Gamepad* gp, Moss_GamepadButton button);
MOSS_API bool Moss_IsGamepadButtonJustPressed(Moss_Gamepad* gp, Moss_GamepadButton button);
MOSS_API bool Moss_IsGamepadButtonJustReleased(Moss_Gamepad* gp, Moss_GamepadButton button);
MOSS_API float Moss_GetGamepadAxis(Moss_Gamepad* gp, Moss_Joystick axis);

// Rumble / LED
MOSS_API bool Moss_RumbleGamepad(Moss_Gamepad* gp, uint16_t low, uint16_t high, uint32_t duration_ms);
MOSS_API bool Moss_RumbleGamepadTriggers(Moss_Gamepad* gp, uint16_t left, uint16_t right, uint32_t duration_ms);
MOSS_API bool Moss_SetGamepadLED(Moss_Gamepad* gp, uint8_t r, uint8_t g, uint8_t b);

// Metadata
MOSS_API const char* Moss_GetGamepadName(Moss_Gamepad* gp);
MOSS_API Moss_GamepadID Moss_GetGamepadID(Moss_Gamepad* gp);
MOSS_API int Moss_GetGamepadPlayerIndex(Moss_Gamepad* gp);
MOSS_API Moss_PowerState Moss_GetGamepadPowerInfo(Moss_Gamepad* gp, int* percent);
MOSS_API int Moss_GetNumGamepadTouchpads(Moss_Gamepad* gp);
MOSS_API int Moss_GetNumGamepadTouchpadFingers(Moss_Gamepad* gp);
MOSS_API bool Moss_GetGamepadTouchpadFinger(Moss_Gamepad* gp, int pad, int finger, bool* down, float* x, float* y, float* pressure);

// Mapping & type
MOSS_API const char* Moss_GetGamepadMapping(Moss_Gamepad* gp);
MOSS_API bool Moss_SetGamepadMapping(Moss_Gamepad* gp, const char* mapping);
MOSS_API void Moss_ReloadGamepadMappings(void);

MOSS_API Moss_GamepadButton Moss_InputGetGamepadButton();
MOSS_API Moss_GamepadAxis Moss_InputGetGamepadAxis();

MOSS_API Moss_PenDeviceType Moss_GetPenDeviceType(Moss_PenID instance_id);
MOSS_API const char* Moss_GetTouchDeviceName(Moss_TouchID touchID);
MOSS_API Moss_TouchID* Moss_GetTouchDevices(int *count);
MOSS_API Moss_TouchDeviceType Moss_GetTouchDeviceType(Moss_TouchID touchID);
MOSS_API Moss_Finger** Moss_GetTouchFingers(Moss_TouchID touchID, int *count);

/*            Haptic Feedback          */
/*! @brief X. @param X X.*/
MOSS_API Moss_Haptic* Moss_OpenHaptic(Moss_HapticID id);
/*! @brief X. @param X X.*/
MOSS_API void Moss_CloseHaptic(Moss_Haptic* haptic);
/*! @brief X. @param X X.*/
MOSS_API Moss_HapticID Moss_CreateHapticEffect(Moss_Haptic* haptic);
/*! @brief X. @param X X.*/
MOSS_API void Moss_DestroyHapticEffect(Moss_Haptic* haptic);
/*! @brief X. @param X X.*/
MOSS_API bool Moss_GetHapticEffectStatus(Moss_Haptic* haptic);
/*! @brief X. @param X X.*/
MOSS_API uint32_t Moss_GetHapticFeatures(Moss_Haptic* haptic);
/*! @brief X. @param X X.*/
MOSS_API Moss_Haptic* Moss_GetHapticFromID(Moss_Haptic* haptic);
/*! @brief X. @param X X.*/
MOSS_API Moss_HapticID* Moss_GetHapticID(Moss_Haptic* haptic);
/*! @brief X. @param X X.*/
MOSS_API const char* Moss_GetHapticName(Moss_Haptic* haptic);
/*! @brief X. @param X X.*/
MOSS_API const char* Moss_GetHapticNameForID(Moss_Haptic* haptic);
/*! @brief X. @param X X.*/
MOSS_API Moss_HapticID* Moss_GetHaptics(Moss_Haptic* haptic);
/*! @brief X. @param X X.*/
MOSS_API int Moss_GetMaxHapticEffects(Moss_Haptic* haptic);
/*! @brief X. @param X X.*/
MOSS_API int Moss_GetMaxHapticEffectsPlaying(Moss_Haptic* haptic);
/*! @brief X. @param X X.*/
MOSS_API int Moss_GetNumHapticAxes(Moss_Haptic* haptic);
/*! @brief X. @param X X.*/
MOSS_API bool Moss_HapticEffectSupported(Moss_Haptic* haptic);
/*! @brief X. @param X X.*/
MOSS_API bool Moss_HapticRumbleSupported(Moss_Haptic* haptic);
/*! @brief X. @param X X.*/
MOSS_API bool Moss_InitHapticRumble(Moss_Haptic* joystick);
/*! @brief X. @param X X.*/
MOSS_API bool Moss_IsJoystickHaptic(Moss_Joystick);
/*! @brief X. @param X X.*/
MOSS_API bool Moss_IsMouseHaptic(void);
/*! @brief X. @param X X.*/
MOSS_API Moss_Haptic* Moss_OpenHapticFromJoystick(Moss_Joystick* joystick);
/*! @brief X. @param X X.*/
MOSS_API Moss_Haptic* Moss_OpenHapticFromMouse(void);
/*! @brief X. @param X X.*/
MOSS_API bool Moss_PauseHaptic(Moss_Haptic* haptic);
/*! @brief X. @param X X.*/
MOSS_API bool Moss_PlayHapticRumble(Moss_Haptic* haptic, float strength, uint32_t length);
/*! @brief X. @param X X.*/
MOSS_API bool Moss_ResumeHaptic(Moss_Haptic* haptic);
/*! @brief X. @param X X.*/
MOSS_API bool Moss_RunHapticEffect(Moss_Haptic* haptic, uint32_t iterations);
/*! @brief X. @param X X.*/
MOSS_API bool Moss_SetHapticAutocenter(Moss_Haptic* haptic, int center);
/*! @brief X. @param X X.*/
MOSS_API bool Moss_SetHapticGain(Moss_Haptic* haptic, int gain);
/*! @brief X. @param X X.*/
MOSS_API bool Moss_StopHapticEffect(Moss_Haptic* haptic, Moss_HapticEffectID effect);
/*! @brief X. @param X X.*/
MOSS_API bool Moss_StopHapticEffects(Moss_Haptic* haptic);
/*! @brief X. @param X X.*/
MOSS_API bool Moss_StopHapticRumble(Moss_Haptic* haptic);
/*! @brief X. @param X X.*/
MOSS_API bool Moss_UpdateHapticEffect(Moss_Haptic* haptic, Moss_HapticEffectID effect, const Moss_HapticEffect* data);


/*              CPU Info          */
/*! @brief Get the number of logical CPU cores available. @ingroup Platform Misc. */
MOSS_API int Moss_GetAvailableCPUCores(void);
/*! @brief Determine the L1 cache line size of the CPU. @ingroup Platform Misc. */
MOSS_API int Moss_GetCPUCacheLineSize(void);
/*! @brief Get the amount of RAM configured in the system. @ingroup Platform Misc. */
MOSS_API int Moss_GetSystemRAM(void);


/*             OS Spesific        */
/*! @brief URL to a website link. Supported on PC and mobile. @param url URL link. @ingroup Platform Misc. */
MOSS_API bool Moss_OpenURL(const char *url);
/*! @brief Get Locale of the Operating system. @return Country "UK" for United Kingdom and language "en" for English. @ingroup Platform Misc. */
MOSS_API Moss_Locale* Moss_GetLocale();
/*! @brief */
MOSS_API Moss_PowerState Moss_GetPowerInfo(int *seconds, int *percent);
/*! @brief Detct if an executible is running. @param executable_path exe name - e.g. obs.exe. */
MOSS_API bool Moss_IsProcessRunningByName(const char* executable_path);
/*! @brief Loads a dynamic library from the given path. @param lib_path Path to the library file. @return Handle to the loaded library, or NULL on failure. @note The returned handle must be released using Moss_UnloadDynamicLibrary(). @ingroup Platform Misc.*/
MOSS_API void* Moss_LoadDynamicLibrary(const char* lib_path);
/*! @brief Retrieves a symbol from a loaded library. @param handle Handle to the loaded library. @param symbol_name Name of the symbol to retrieve. @return Pointer to the symbol, or NULL if not found.  @return A pointer to the requested symbol, or NULL if not found.  @warning The returned pointer must be cast to the appropriate function or data type. @ingroup Platform Misc.*/
MOSS_API void* Moss_GetLibrarySymbol(void* handle, const char* symbol_name);
/*! @brief Unloads a previously loaded dynamic library. @param handle Handle to the library to unload. @note After unloading, the handle should not be used again. @ingroup Platform Misc. */
MOSS_API void Moss_UnloadDynamicLibrary(void* handle);

// =================================================
//              Callback Registration API
// =================================================

/*! @brief Sets the framebuffer resize callback. @param callback Pointer to a function to be invoked when framebuffer size changes. @ingroup Window */
MOSS_API void Moss_SetFramebufferResizeCallback(Moss_FramebufferResizeCallback callback);
/*! @brief Sets the window size callback. @param callback Pointer to a function invoked when the window size changes. @ingroup Window */
MOSS_API void Moss_SetWindowSizeCallback(Moss_WindowSizeCallback callback);
/*! @brief Sets the window resize callback (platform-level, e.g. minimize/maximize). @param callback Pointer to a function invoked when window resizing events occur. @ingroup Window */
MOSS_API void Moss_SetWindowResizeCallback(Moss_WindowResizeCallback callback);
/*! @brief Sets the window position callback. @param callback Pointer to a function invoked when the window position changes. @ingroup Window */
MOSS_API void Moss_SetWindowPositionCallback(Moss_WindowPositionCallback callback);
/*! @brief Sets the window focus callback. @param callback Pointer to a function invoked when the window focus changes. @ingroup Window */
MOSS_API void Moss_SetWindowFocusCallback(Moss_WindowFocusCallback callback);
/*! @brief Sets the window content scale callback (for HiDPI / Retina support). @param callback Pointer to a function invoked when the content scale changes. @ingroup Window */
MOSS_API void Moss_SetWindowContentScaleCallback(Moss_WindowContentScaleCallback callback);
/*! @brief Sets the monitor connection or configuration callback. @param callback Pointer to a function invoked when a monitor is connected or disconnected. @ingroup Monitor */
MOSS_API void Moss_SetMonitorCallback(Moss_MonitorCallback callback);


/*          Camera          */
enum class Moss_CameraPermissionState { DENIED = -1, PENDING, APPROVED };
enum class Moss_CameraPosition { UNKNOWN, FRONT_FACING, BACK_FACING };

typedef struct Moss_Camera Moss_Camera; // Camera Device Dont use as the rendering camera

struct Moss_CameraSpec {
    PixelFormat format;         // Frame format
    Colorspace colorspace;      // Frame colorspace
    int width;                  // Frame width
    int height;                 // Frame height
    int framerate_numerator;    // Frame rate numerator ((num / denom) == FPS, (denom / num) == duration in seconds)
    int framerate_denominator;  // Frame rate demoninator ((num / denom) == FPS, (denom / num) == duration in seconds)
} Moss_CameraSpec;

MOSS_API Moss_CameraID* Moss_GetCameras(int* count);
/* Returned array valid until next call or shutdown */

MOSS_API const char* Moss_GetCameraName(Moss_CameraID camera_id);
MOSS_API Moss_CameraPosition Moss_GetCameraPosition(Moss_CameraID camera_id);
MOSS_API const char* Moss_GetCurrentCameraDriver(void);
MOSS_API int Moss_GetNumCameraDrivers(void);
MOSS_API const Moss_CameraSpec* Moss_GetCameraSupportedFormats(Moss_CameraID camera_id, int* count);
MOSS_API Moss_Surface* Moss_AcquireCameraFrame(Moss_Camera* camera, uint64_t* timestamp_ns);
MOSS_API void Moss_ReleaseCameraFrame(Moss_Camera* camera, Moss_Surface* frame);
MOSS_API bool Moss_GetCameraFormat(Moss_Camera* camera, Moss_CameraSpec* out_spec);
MOSS_API Moss_CameraPermissionState Moss_GetCameraPermissionState(Moss_Camera* camera);
MOSS_API Moss_PropertiesID Moss_GetCameraProperties(Moss_Camera* camera);
MOSS_API void Moss_CloseCamera(Moss_Camera *camera);
MOSS_API Moss_CameraID Moss_GetCameraID(Moss_Camera *camera);
MOSS_API Moss_PropertiesID Moss_GetCameraProperties(Moss_Camera *camera);
MOSS_API Moss_Camera * Moss_OpenCamera(Moss_CameraID instance_id, const Moss_CameraSpec *spec);
/*! @brief X. @param X X @ingroup Video Capture. */
MOSS_API Moss_VideoCapture* Moss_OpenVideoCapture(Moss_VideoCaptureID captureID);
/*! @brief X. @param X X @ingroup Video Capture. */
MOSS_API void Moss_CloseVideoCapture(Moss_VideoCapture* cap);
/*! @brief X. @param X X @ingroup Video Capture. */
MOSS_API uint8_t* Moss_VideoCaptureReadFrame(Moss_VideoCapture* cap);



MOSS_API bool Moss_CopyFile(const char* src_path, const char* dst_path, bool overwrite);
MOSS_API bool Moss_CreateDirectory(const char* path, bool recursive);
MOSS_API bool Moss_RemovePath(const char* path, bool recursive);
MOSS_API bool Moss_RenamePath(const char* old_path, const char* new_path, bool overwrite);


MOSS_API bool Moss_GetPathInfo(const char* path, Moss_PathInfo* out_info);
MOSS_API bool Moss_GetCurrentDirectory(char* out_path, int max_len);
MOSS_API bool Moss_GetBasePath(char* out_path, int max_len);

MOSS_API bool Moss_GetUserFolder(Moss_UserFolder folder, char* out_path, int max_len);
MOSS_API bool Moss_GetPrefPath(const char* org_name, const char* app_name, char* out_path, int max_len);
MOSS_API bool Moss_EnumerateDirectory( const char* path, bool recursive, Moss_DirectoryIterateFn callback, void* user_data);
MOSS_API bool Moss_GlobDirectory(const char* pattern, Moss_DirectoryIterateFn callback, void* user_data);


MOSS_API void Moss_ShowFileDialogWithProperties(Moss_DialogFileCallback callback, void *userdata, Moss_Window *window, const Moss_DialogFileFilter *filters, int nfilters, const char *default_location);
MOSS_API void Moss_ShowOpenFileDialog(Moss_DialogFileCallback callback, void *userdata, Moss_Window *window, const char *default_location, bool allow_many);
MOSS_API void Moss_ShowOpenFolderDialog(Moss_DialogFileCallback callback, void *userdata, Moss_Window *window, const Moss_DialogFileFilter *filters, int nfilters, const char *default_location, bool allow_many);
MOSS_API void Moss_ShowSaveFileDialog(Moss_FileDialogType type, Moss_DialogFileCallback callback, void *userdata, Moss_PropertiesID props);

MOSS_API bool Moss_CloseStorage(Moss_Storage *storage);
MOSS_API bool Moss_CopyStorageFile(Moss_Storage *storage, const char *oldpath, const char *newpath);
MOSS_API bool Moss_CreateStorageDirectory(Moss_Storage *storage, const char *path);
MOSS_API bool Moss_EnumerateStorageDirectory(Moss_Storage *storage, const char *path, Moss_EnumerateDirectoryCallback callback, void *userdata);
MOSS_API bool Moss_GetStorageFileSize(Moss_Storage *storage, const char *path, uint64 *length);
MOSS_API uint64_t Moss_GetStorageSpaceRemaining(Moss_Storage *storage);
MOSS_API bool Moss_GetStoragePathInfo(Moss_Storage *storage, const char *path, Moss_PathInfo *info);
MOSS_API char** Moss_GlobStorageDirectory(Moss_Storage *storage, const char *path, const char *pattern, Moss_GlobFlags flags, int *count);
MOSS_API Moss_Storage* Moss_OpenFileStorage(const char *path);
MOSS_API Moss_Storage* Moss_OpenStorage(const Moss_Storage *iface, void *userdata);
MOSS_API Moss_Storage* Moss_OpenTitleStorage(const char *override, Moss_PropertiesID props);
MOSS_API Moss_Storage* Moss_OpenUserStorage(const char *org, const char *app, Moss_PropertiesID props);
MOSS_API bool Moss_ReadStorageFile(Moss_Storage *storage, const char *path, void *destination, uint64 length);
MOSS_API bool Moss_RemoveStoragePath(Moss_Storage *storage, const char *path);
MOSS_API bool Moss_RenameStoragePath(Moss_Storage *storage, const char *oldpath, const char *newpath);
MOSS_API bool Moss_StorageReady(Moss_Storage *storage);
MOSS_API bool Moss_WriteStorageFile(Moss_Storage *storage, const char *path, const void *source, uint64 length);


// Mobile Android / IOS App Creation
#if defined(MOSS_PLATFORM_ANDROID) || defined(MOSS_PLATFORM_IOS) || defined(MOSS_PLATFORM_TVOS) || defined(MOSS_PLATFORM_XBOXSERIES) || defined(MOSS_PLATFORM_XBOXONE)
/*! @brief Embedded window intended for creating mobile, consoles and tv applications. @param X X. @ingroup window */
MOSS_API Moss_Window* Moss_CreateEmbeddedWindow(struct android_app* app);
MOSS_API void Moss_Terminate_MobileWindow(void);
MOSS_API void Moss_Mobile_OnOrientationChanged(bool focused);
MOSS_API void Moss_Mobile_OnResume();
MOSS_API void Moss_Mobile_OnPause();
#endif

// OpenGL / OpenGL-ES
#if defined(MOSS_GRAPHICS_OPENGL) || defined(MOSS_GRAPHICS_OPENGLES)
/*! @brief Sets window as current.  @param X X. @ingroup window */
MOSS_API void Moss_MakeContextCurrent(Moss_Window* window);
/*! @brief Swapbuffers used to swapbuffers for opengl. @ingroup window */
MOSS_API void Moss_SwapBuffers();
/*! @brief Swapbuffers but with a delay in seconds. @param X X. @ingroup window. */
MOSS_API void Moss_SwapBuffersInterval(int interval);
/*! @brief X. @param X X. @ingroup window. */
MOSS_API void* Moss_GetProcAddress(const char* procname);
#endif // MOSS_USE_OPENGL

// Vulkan
#if defined(MOSS_GRAPHICS_VULKAN)
/*! @brief Creates the window for vulkan @param X X @ingroup Vulkan. */
MOSS_API VkResult Moss_CreateWindowSurface(Moss_Window* window, VkInstance vk_instance, const VkAllocationCallbacks *allocator, VkSurfaceKHR* vk_surface);
/*! @brief X @param X X @ingroup Vulkan. */
MOSS_API int Moss_VulkanSupported(void);
/*! @brief X @param X X @ingroup Vulkan. */
//MOSS_API void Moss_InitVulkanLoader(PFN_vkGetInstanceProcAddr loader);
/*! @brief X @param X X @ingroup Vulkan. */
MOSS_API const char** Moss_GetRequiredInstanceExtensions(uint32_t* count);
/*! @brief X @param X X @ingroup Vulkan. */
MOSS_API void* Moss_GetInstanceProcAddress(VkInstance instance, const char* procname);
/*! @brief X @param X X @ingroup Vulkan. */
MOSS_API int Moss_GetPhysicalDevicePresentationSupport(Moss_Window* window, VkPhysicalDevice device, uint32_t queuefamily);
#endif // MOSS_USE_VULKAN

// Metal
#if defined(MOSS_GRAPHICS_METAL)
typedef void *Moss_MetalView;
MOSS_API Moss_MetalView Moss_Metal_CreateView(Moss_Window *window);
MOSS_API void Moss_Metal_DestroyView(Moss_MetalView view);
MOSS_API void* Moss_Metal_GetLayer(Moss_MetalView view);
MOSS_API void Moss_Metal_Resize(Moss_MetalView handle, uint32_t width, uint32_t height);
#endif // MOSS_GRAPHICS_METAL

#endif // MOSS_PLATFORM_H