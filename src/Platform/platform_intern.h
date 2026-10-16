#ifndef MOSS_PLATFORM_INTERNAL_H
#define MOSS_PLATFORM_INTERNAL_H

// 1. Forward-declare structures so line 105 knows they exist
struct Moss_Capture;
struct Moss_Surface;
struct Moss_Window;
struct Moss_Monitor;
struct Moss_Image;

#include <atomic>
#include <cstdint>
#include <cmath>
#include <algorithm>

#include <Moss/Moss_stdinc.h>
#include <Moss/Moss_Platform.h>

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
#ifndef NOMINMAX
#define NOMINMAX
#endif
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
    bool buttons_prev[static_cast<size_t>(Gamepad::COUNT)] = {}; // Added missing array
    float axes[static_cast<int>(GamepadAxis::COUNT)] = {};
    bool keys_prev[static_cast<size_t>(Gamepad::COUNT)] = {};    // Added missing array

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

typedef void (*Moss_CaptureFrameResult)(Moss_Capture *device, Moss_Surface *frame, uint64_t *timestampNS, float *rotation);


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

struct Moss_GamepadSensorState
{
    bool supported[2] = {};
    bool enabled[2] = {};
    bool has_sample[2] = {};

    Moss_GamepadSensorData latest[2] = {};

    float orientation[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // W, X, Y, Z
    float gravity[3]     = {0.0f, 0.0f, 0.0f};       // X, Y, Z
    float smooth_accel[3]= {0.0f, 0.0f, 0.0f};       // X, Y, Z
    float shakiness      = 0.0f;

    float calibration_offset[3] = {0.0f, 0.0f, 0.0f}; // Active Gyro Biases (X, Y, Z)
    
    // Min/Max window bounds for tracking stillness thresholds
    float min_gyro[3]  = {0.0f, 0.0f, 0.0f};
    float max_gyro[3]  = {0.0f, 0.0f, 0.0f};
    float mean_gyro[3] = {0.0f, 0.0f, 0.0f};
    
    float min_accel[3] = {0.0f, 0.0f, 0.0f};
    float max_accel[3] = {0.0f, 0.0f, 0.0f};
    float mean_accel[3]= {0.0f, 0.0f, 0.0f};

    int   window_samples = 0;
    float window_time    = 0.0f;
    float steady_time    = 0.0f;
};


struct MossMotionTracker {
    // Current orientation and processed gravity/acceleration matching Julian's logic
    float orientationW = 1.0f, orientationX = 0.0f, orientationY = 0.0f, orientationZ = 0.0f;
    float gravX = 0.0f, gravY = 0.0f, gravZ = 0.0f;
    float smoothAccelX = 0.0f, smoothAccelY = 0.0f, smoothAccelZ = 0.0f;
    float shakiness = 0.0f;

    // Hardcoded default filters from GamepadMotionSettings
    const float ShortSteadinessHalfTime = 0.25f;
    const float GravityCorrectionStillSpeed = 1.0f;
    const float GravityCorrectionShakySpeed = 0.1f;
    const float GravityCorrectionGyroFactor = 0.1f;
    const float GravityCorrectionMinimumSpeed = 0.01f;

    // Standalone implementation of Julian Smart's Motion::Update matching Moss coordinate rules
    void Update(float gyroX, float gyroY, float gyroZ, 
                float accelX, float accelY, float accelZ, float deltaTime) 
    {
        // 1. Calculate Angular Rotation Speed
        float gyroLength = std::sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);
        float angleSpeed = gyroLength * 3.14159265f / 180.0f;
        float angle = angleSpeed * deltaTime;

        // 2. Create Axis-Angle Quaternion rotation step
        float sinHalf = std::sin(angle * 0.5f);
        float cosHalf = std::cos(angle * 0.5f);
        float rotW = cosHalf;
        float rotX = (gyroLength > 0.0f) ? (gyroX / gyroLength) * sinHalf : 0.0f;
        float rotY = (gyroLength > 0.0f) ? (gyroY / gyroLength) * sinHalf : 0.0f;
        float rotZ = (gyroLength > 0.0f) ? (gyroZ / gyroLength) * sinHalf : 0.0f;

        // Local Quaternion Multiplication: orientation = orientation * local_rotation
        float tw = orientationW * rotW - orientationX * rotX - orientationY * rotY - orientationZ * rotZ;
        float tx = orientationW * rotX + orientationX * rotW + orientationY * rotZ - orientationZ * rotY;
        float ty = orientationW * rotY - orientationX * rotZ + orientationY * rotW + orientationZ * rotX;
        float tz = orientationW * rotZ + orientationX * rotY - orientationY * rotX + orientationZ * rotW;
        orientationW = tw; orientationX = tx; orientationY = ty; orientationZ = tz;

        float accelMag = std::sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
        if (accelMag > 0.0f) {
            float axNorm = accelX / accelMag;
            float ayNorm = accelY / accelMag;
            float azNorm = accelZ / accelMag;

            // Rotate existing tracking systems inversely matching the physical flip
            // (Simulates Quat * Vec * InverseQuat tracking)
            float invRotX = -rotX; float invRotY = -rotY; float invRotZ = -rotZ;
            // [Engine Math Library simplifies these transformations dramatically]

            // 3. Track Shakiness & Extract Pure Linear Acceleration
            float smoothFactor = (ShortSteadinessHalfTime <= 0.0f) ? 0.0f : std::exp2f(-deltaTime / ShortSteadinessHalfTime);
            shakiness *= smoothFactor;
            
            float diffX = accelX - smoothAccelX;
            float diffY = accelY - smoothAccelY;
            float diffZ = accelZ - smoothAccelZ;
            float diffLength = std::sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ);
            shakiness = std::max(shakiness, diffLength);

            smoothAccelX = accelX + (smoothAccelX - accelX) * smoothFactor;
            smoothAccelY = accelY + (smoothAccelY - accelY) * smoothFactor;
            smoothAccelZ = accelZ + (smoothAccelZ - accelZ) * smoothFactor;

            // 4. Close the drift error window using the gravity correction loop
            float gravCorrectionSpeed = GravityCorrectionStillSpeed + (GravityCorrectionShakySpeed - GravityCorrectionStillSpeed) * std::clamp((shakiness - 0.01f) / (0.4f - 0.01f), 0.0f, 1.0f);
            float gyroLimit = std::max(angleSpeed * GravityCorrectionGyroFactor, GravityCorrectionMinimumSpeed);
            
            if (gravCorrectionSpeed > gyroLimit) {
                gravCorrectionSpeed = gyroLimit; 
            }

            // Apply correction over delta time to align up vector safely
            gravX += (axNorm * -accelMag - gravX) * gravCorrectionSpeed * deltaTime;
            gravY += (ayNorm * -accelMag - gravY) * gravCorrectionSpeed * deltaTime;
            gravZ += (azNorm * -accelMag - gravZ) * gravCorrectionSpeed * deltaTime;
        }

        // Normalize Quaternion output to prevent floating-point drift mutations
        float qMag = std::sqrt(orientationW*orientationW + orientationX*orientationX + orientationY*orientationY + orientationZ*orientationZ);
        orientationW /= qMag; orientationX /= qMag; orientationY /= qMag; orientationZ /= qMag;
    }
};

inline void Moss_InternalProcessSensorFusion(Moss_GamepadSensorState& state, 
                                      float gyroX, float gyroY, float gyroZ, 
                                      float accelX, float accelY, float accelZ, 
                                      float deltaTime) 
{
    // Hardcoded framework-level filter defaults
    const float ShortSteadinessHalfTime = 0.25f;
    const float GravityCorrectionStillSpeed = 1.0f;
    const float GravityCorrectionGyroFactor = 0.1f;
    const float GravityCorrectionMinimumSpeed = 0.01f;

    // 1. Calculate Angular Rotation Speed
    float gyroLength = std::sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);
    float angleSpeed = gyroLength * 3.14159265f / 180.0f;
    float angle = angleSpeed * deltaTime;

    // 2. Compute Axis-Angle local rotation transformations
    float sinHalf = std::sin(angle * 0.5f);
    float cosHalf = std::cos(angle * 0.5f);
    float rotW = cosHalf;
    float rotX = (gyroLength > 0.0f) ? (gyroX / gyroLength) * sinHalf : 0.0f;
    float rotY = (gyroLength > 0.0f) ? (gyroY / gyroLength) * sinHalf : 0.0f;
    float rotZ = (gyroLength > 0.0f) ? (gyroZ / gyroLength) * sinHalf : 0.0f;

    // Quaternion Mutation: state.orientation = state.orientation * local_rotation step
    float tw = state.orientation[0] * rotW - state.orientation[1] * rotX - state.orientation[2] * rotY - state.orientation[3] * rotZ;
    float tx = state.orientation[0] * rotX + state.orientation[1] * rotW + state.orientation[2] * rotZ - state.orientation[3] * rotY;
    float ty = state.orientation[0] * rotY - state.orientation[1] * rotZ + state.orientation[2] * rotW + state.orientation[3] * rotX;
    float tz = state.orientation[0] * rotZ + state.orientation[1] * rotY - state.orientation[2] * rotX + state.orientation[3] * rotW;
    state.orientation[0] = tw; state.orientation[1] = tx; state.orientation[2] = ty; state.orientation[3] = tz;

    float accelMag = std::sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
    if (accelMag > 0.0f) {
        float axNorm = accelX / accelMag;
        float ayNorm = accelY / accelMag;
        float azNorm = accelZ / accelMag;

        // 3. Process frame shakiness profiles 
        float smoothFactor = (ShortSteadinessHalfTime <= 0.0f) ? 0.0f : std::exp2f(-deltaTime / ShortSteadinessHalfTime);
        state.shakiness *= smoothFactor;
        
        float diffX = accelX - state.smooth_accel[0];
        float diffY = accelY - state.smooth_accel[1];
        float diffZ = accelZ - state.smooth_accel[2];
        state.shakiness = std::max(state.shakiness, std::sqrt(diffX*diffX + diffY*diffY + diffZ*diffZ));

        state.smooth_accel[0] = accelX + (state.smooth_accel[0] - accelX) * smoothFactor;
        state.smooth_accel[1] = accelY + (state.smooth_accel[1] - accelY) * smoothFactor;
        state.smooth_accel[2] = accelZ + (state.smooth_accel[2] - accelZ) * smoothFactor;

        // 4. Calculate drift correction limits
        float gravCorrectionSpeed = GravityCorrectionStillSpeed + (0.1f - GravityCorrectionStillSpeed) * std::clamp((state.shakiness - 0.01f) / (0.4f - 0.01f), 0.0f, 1.0f);
        float gyroLimit = std::max(angleSpeed * GravityCorrectionGyroFactor, GravityCorrectionMinimumSpeed);
        
        if (gravCorrectionSpeed > gyroLimit) {
            gravCorrectionSpeed = gyroLimit; 
        }

        // Apply real-time gravity filtering directly down the array pipeline
        state.gravity[0] += (axNorm * -accelMag - state.gravity[0]) * gravCorrectionSpeed * deltaTime;
        state.gravity[1] += (ayNorm * -accelMag - state.gravity[1]) * gravCorrectionSpeed * deltaTime;
        state.gravity[2] += (azNorm * -accelMag - state.gravity[2]) * gravCorrectionSpeed * deltaTime;
    }

    // Stabilize the quaternion vector array to drop calculation rounding mutations
    float qMag = std::sqrt(state.orientation[0]*state.orientation[0] + state.orientation[1]*state.orientation[1] + state.orientation[2]*state.orientation[2] + state.orientation[3]*state.orientation[3]);
    state.orientation[0] /= qMag; state.orientation[1] /= qMag; state.orientation[2] /= qMag; state.orientation[3] /= qMag;
}

bool GetSensorIndex(Moss_GamepadSensorType sensor, std::size_t& outIndex);
bool IsValidSample(const Moss_GamepadSensorData& sample);

void PublishGamepadSensorSample(Moss_Gamepad* gp, Moss_GamepadSensorType sensor, const Moss_GamepadSensorData& sample) {
    std::size_t index = 0;
    if (gp == nullptr || !GetSensorIndex(sensor, index) || !IsValidSample(sample)) { return; }

    std::lock_guard<std::mutex> lock(g_sensorMutex);
    const auto gamepadIt = g_gamepadSensors.find(gp);
    if (gamepadIt == g_gamepadSensors.end()) return;

    SensorState& state = gamepadIt->second[index];
    if (!state.supported || !state.enabled) return;
    if (state.hasSample && sample.timestamp_ns < state.latest.timestamp_ns) { return; }

    // Calculate dynamic delta time since the last recorded hardware timestamp tick
    float deltaTime = (state.latest.timestamp_ns > 0) ? (float)(sample.timestamp_ns - state.latest.timestamp_ns) / 1e9f : 0.016f;

    state.latest = sample;
    state.hasSample = true;

    // --- EXECUTE THE NEW INLINE SENSOR FUSION ENGINE HERE ---
    // Extract a reference to our updated mathematical tracking engine state blocks
    auto& trackingState = g_gamepadSensors[gp]; // References the GamepadSensorStates container
    
    // Pass along both sensor data tracking arrays seamlessly
    Moss_InternalProcessSensorFusion(
        trackingState, // Pass the tracker array block 
        g_gamepadSensors[gp][1].latest.x, g_gamepadSensors[gp][1].latest.y, g_gamepadSensors[gp][1].latest.z, // Gyro readings
        g_gamepadSensors[gp][0].latest.x, g_gamepadSensors[gp][0].latest.y, g_gamepadSensors[gp][0].latest.z, // Accel readings
        deltaTime
    );
}

inline void UpdateControllerInput(Moss_Gamepad* activeGamepad, MossMotionTracker& motionTracker, float deltaTime) {
    if (activeGamepad == nullptr) return;

    Moss_GamepadSensorData gyroData{};
    Moss_GamepadSensorData accelData{};

    // Retrieve raw readings directly via the Moss pipeline
    if (Moss_GetGamepadSensorData(activeGamepad, Moss_GamepadSensorType::GYROSCOPE, &gyroData) && Moss_GetGamepadSensorData(activeGamepad, Moss_GamepadSensorType::ACCELEROMETER, &accelData))  {
        // Feed the hardware telemetry directly to your inline sensor equations
        motionTracker.Update(gyroData.x,  gyroData.y,  gyroData.z, accelData.x, accelData.y, accelData.z, deltaTime);
    }
}

#endif // MOSS_PLATFORM_INTERNAL_H