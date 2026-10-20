#include "platform_intern.h"

constexpr std::size_t SensorCount = 2;

struct SensorState {
    bool supported = false;
    bool enabled = false;
    bool hasSample = false;

    Moss_GamepadSensorData latest{};
};

using GamepadSensorStates = std::array<SensorState, SensorCount>;

std::mutex g_sensorMutex;

std::unordered_map<Moss_Gamepad*, GamepadSensorStates> g_gamepadSensors;

bool GetSensorIndex(Moss_GamepadSensorType sensor, std::size_t& index) {
    switch (sensor) {
        case Moss_GamepadSensorType::ACCELEROMETER:
            index = 0;
            return true;
        case Moss_GamepadSensorType::GYROSCOPE:
            index = 1;
            return true;
        default:
            return false;
    }
}

bool IsValidSample(const Moss_GamepadSensorData& sample) { return std::isfinite(sample.x) && std::isfinite(sample.y) && std::isfinite(sample.z); }

void ClearSample(SensorState& state) { state.hasSample = false; state.latest = {}; }

// Checks whether the requested sensor is supported.
bool GamepadHasSensor(Moss_Gamepad* gp, Moss_GamepadSensorType sensor) {
    std::size_t index = 0;

    if (gp == nullptr || !GetSensorIndex(sensor, index))
        return false;

    const bool supported = Moss_BackendHasGamepadSensor(gp, sensor);

    std::lock_guard<std::mutex> lock(g_sensorMutex);

    SensorState& state = g_gamepadSensors[gp][index];
    state.supported = supported;

    if (!supported) {
        state.enabled = false;
        ClearSample(state);
    }

    return supported;
}

// Enables or disables the requested sensor.
bool SetGamepadSensorEnabled(Moss_Gamepad* gp, Moss_GamepadSensorType sensor, bool enabled) {
    std::size_t index = 0;

    if (gp == nullptr || !GetSensorIndex(sensor, index))
        return false;

    if (!GamepadHasSensor(gp, sensor))
        return false;

    SensorState previous;

    {   // Lock #1: snapshot + prepare the cache
        std::lock_guard<std::mutex> lock(g_sensorMutex);

        SensorState& state = g_gamepadSensors[gp][index];
        previous = state;
        state.enabled = enabled;

        if (enabled && !previous.enabled)
            ClearSample(state);
    }   // released before calling native code

    const bool success = Moss_BackendSetGamepadSensorEnabled(gp, sensor, enabled);

    {   // Lock #2: commit or roll back
        std::lock_guard<std::mutex> lock(g_sensorMutex);

        SensorState& state = g_gamepadSensors[gp][index];

        if (!success) {
            state = previous;
            return false;
        }

        state.supported = true;
        state.enabled = enabled;

        if (!enabled)
            ClearSample(state);
    }

    return true;
}

// Retrieves the latest valid sensor sample.
bool GetGamepadSensorData(Moss_Gamepad* gp, Moss_GamepadSensorType sensor, Moss_GamepadSensorData* data) {
    std::size_t index = 0;

    if (gp == nullptr || data == nullptr || !GetSensorIndex(sensor, index)) { return false; }

    std::lock_guard<std::mutex> lock(g_sensorMutex);

    const auto gamepadIt = g_gamepadSensors.find(gp);

    if (gamepadIt == g_gamepadSensors.end())
        return false;

    const SensorState& state = gamepadIt->second[index];

    if (!state.supported || !state.enabled || !state.hasSample)
        return false;

    *data = state.latest;
    return true;
}

// Clears cached data when a gamepad disconnects or is destroyed.
// Stop native sensor callbacks before calling this function.
void ResetGamepadSensorState(Moss_Gamepad* gp) {
    if (gp == nullptr)
        return;

    std::lock_guard<std::mutex> lock(g_sensorMutex);
    g_gamepadSensors.erase(gp);
}

void PublishGamepadSensorSample(Moss_Gamepad* gp, Moss_GamepadSensorType sensor, const Moss_GamepadSensorData& sample) {
    std::size_t index = 0;
    if (gp == nullptr || !GetSensorIndex(sensor, index) || !IsValidSample(sample)) { return; }
    
    std::lock_guard<std::mutex> lock(g_sensorMutex);
    const auto gamepadIt = g_gamepadSensors.find(gp);
    if (gamepadIt == g_gamepadSensors.end()) return;

    // Fix: gamepadIt->second gives us a Moss_GamepadSensorState[2] array.
    // We isolate our specific active sensor structure via [index].
    Moss_GamepadSensorState& state = gamepadIt->second[index];
    if (!state.supported[index] || !state.enabled[index]) return;
    if (state.has_sample[index] && sample.timestamp_ns < state.latest[index].timestamp_ns) { return; }

    float deltaTime = (state.latest[index].timestamp_ns > 0) ? (float)(sample.timestamp_ns - state.latest[index].timestamp_ns) / 1e9f : 0.016f;
    state.latest[index] = sample;
    state.has_sample[index] = true;

    // Fix: Extract explicit struct array indexes from the map pointer reference block.
    // Index [1] holds Gyro telemetry data, Index [0] holds Accelerometer telemetry data.
    auto& gyroStruct  = gamepadIt->second[1];
    auto& accelStruct = gamepadIt->second[0];

    // Pass the state variable structure alongside the individual data elements
    // to cleanly conform to Moss_InternalProcessSensorFusion's arguments signature.
    Moss_InternalProcessSensorFusion(
        state,
        gyroStruct.latest[index].x,  gyroStruct.latest[index].y,  gyroStruct.latest[index].z,   // Gyro elements
        accelStruct.latest[index].x, accelStruct.latest[index].y, accelStruct.latest[index].z,  // Accel elements
        deltaTime
    );
}


// Public Moss API implementations.

bool Moss_GamepadHasSensor(Moss_Gamepad* gp, Moss_GamepadSensorType sensor) { return GamepadHasSensor(gp, sensor); }
bool Moss_SetGamepadSensorEnabled(Moss_Gamepad* gp, Moss_GamepadSensorType sensor, bool enabled) { return SetGamepadSensorEnabled(gp, sensor, enabled); }
bool Moss_GetGamepadSensorData(Moss_Gamepad* gp, Moss_GamepadSensorType sensor, Moss_GamepadSensorData* data) { return GetGamepadSensorData(gp, sensor, data); }
bool Moss_GetGamepadOrientation(Moss_Gamepad* gp, float* w, float* x, float* y, float* z) {
    if (gp == nullptr || w == nullptr || x == nullptr || y == nullptr || z == nullptr) return false;
    std::lock_guard<std::mutex> lock(g_sensorMutex);
    
    const auto it = g_gamepadSensors.find(gp);
    if (it == g_gamepadSensors.end()) return false;
    
    *w = it->second[1].orientation[0]; // Retain W from the active state tracking matrix
    *x = it->second[1].orientation[1];
    *y = it->second[1].orientation[2];
    *z = it->second[1].orientation[3];
    return true;
}
void Moss_GetGamepadPlayerSpaceGyro(Moss_Gamepad* gp, float* outX, float* outY, const float yawRelaxFactor) {
    if (gp == nullptr || outX == nullptr || outY == nullptr) return;
    
    Moss_GamepadSensorData gyroData{};
    std::lock_guard<std::mutex> lock(g_sensorMutex);
    
    const auto it = g_gamepadSensors.find(gp);
    if (it == g_gamepadSensors.end()) return;
    
    // Index 1 = Gyroscope, Index 0 = Accelerometer
    gyroData = it->second[1].latest; 
    const float* grav = it->second[0].gravity; 
    
    // Compute Player Space projection matching Julian Smart's engine tracking matrix
    const float worldYaw = -(grav[1] * gyroData.y + grav[2] * gyroData.z);
    const float worldYawSign = (worldYaw < 0.f) ? -1.f : 1.f;
    const float gyroMagnitude = std::sqrt(gyroData.y * gyroData.y + gyroData.z * gyroData.z);
    
    *outX = gyroData.x;
    *outY = worldYawSign * std::min(std::abs(worldYaw) * yawRelaxFactor, gyroMagnitude);
}

void Moss_GetGamepadWorldSpaceGyro(Moss_Gamepad* gp, float* outX, float* outY, const float sideReductionThreshold) {
    if (gp == nullptr || outX == nullptr || outY == nullptr) return;
    
    std::lock_guard<std::mutex> lock(g_sensorMutex);
    const auto it = g_gamepadSensors.find(gp);
    if (it == g_gamepadSensors.end()) return;
    
    const float* grav = it->second[0].gravity;
    const Moss_GamepadSensorData& gyro = it->second[1].latest;
    
    const float worldYaw = -grav[0] * gyro.x - grav[1] * gyro.y - grav[2] * gyro.z;
    const float gravDotPitchAxis = grav[0];
    
    float pitchAxisX = 1.0f - grav[0] * gravDotPitchAxis;
    float pitchAxisY = -grav[1] * gravDotPitchAxis;
    float pitchAxisZ = -grav[2] * gravDotPitchAxis;
    
    float pLenSq = pitchAxisX*pitchAxisX + pitchAxisY*pitchAxisY + pitchAxisZ*pitchAxisZ;
    if (pLenSq > 0.0f) {
        float pLen = std::sqrt(pLenSq);
        pitchAxisX /= pLen; pitchAxisY /= pLen; pitchAxisZ /= pLen;
        
        float flatness = std::abs(grav[1]);
        float upness = std::abs(grav[2]);
        float maxAxis = std::max(flatness, upness);
        
        float sideReduction = (sideReductionThreshold <= 0.f) ? 1.f : 
            std::clamp((maxAxis - sideReductionThreshold) / sideReductionThreshold, 0.f, 1.f);
            
        *outX = sideReduction * (pitchAxisX * gyro.x + pitchAxisY * gyro.y + pitchAxisZ * gyro.z);
    } else { *outX = 0.0f; }
    *outY = worldYaw;
}

void Moss_ProcessStillnessCalibration(Moss_GamepadSensorState& state, float gyroX, float gyroY, float gyroZ, float accelX, float accelY, float accelZ, float deltaTime)  {
    // Threshold profiles mapped from Julian Smart's production presets
    const int   MinStillnessSamples        = 10;
    const float MinStillnessCollectionTime = 0.5f; // Must be still for 0.5s to track values
    const float MinStillnessCorrectionTime = 2.0f; // Must stay still for 2.0s to begin filtering
    const float MaxGyroDeltaTolerance      = 2.0f; // Degrees/second variance cutoff
    const float MaxAccelDeltaTolerance     = 0.25f;// G-Force variance cutoff

    // Prevent corrupt initialization ticks or disconnected data segments
    if (gyroX == 0.0f && gyroY == 0.0f && gyroZ == 0.0f && accelX == 0.0f && accelY == 0.0f && accelZ == 0.0f) {
        Moss_ResetStillnessWindow(state);
        state.window_samples = 0;
        state.window_time = 0.0f;
        state.steady_time = 0.0f;
        return;
    }

    // 1. Initialize or append metrics inside the sliding sampling window
    if (state.window_samples == 0) {
        state.min_gyro[0]  = state.max_gyro[0]  = state.mean_gyro[0]  = gyroX;
        state.min_gyro[1]  = state.max_gyro[1]  = state.mean_gyro[1]  = gyroY;
        state.min_gyro[2]  = state.max_gyro[2]  = state.mean_gyro[2]  = gyroZ;
        
        state.min_accel[0] = state.max_accel[0] = state.mean_accel[0] = accelX;
        state.min_accel[1] = state.max_accel[1] = state.mean_accel[1] = accelY;
        state.min_accel[2] = state.max_accel[2] = state.mean_accel[2] = accelZ;
        
        state.window_samples = 1;
        state.window_time = deltaTime;
        return;
    }

    // Update boundary envelopes
    state.max_gyro[0] = std::max(state.max_gyro[0], gyroX); state.min_gyro[0] = std::min(state.min_gyro[0], gyroX);
    state.max_gyro[1] = std::max(state.max_gyro[1], gyroY); state.min_gyro[1] = std::min(state.min_gyro[1], gyroY);
    state.max_gyro[2] = std::max(state.max_gyro[2], gyroZ); state.min_gyro[2] = std::min(state.min_gyro[2], gyroZ);

    state.max_accel[0] = std::max(state.max_accel[0], accelX); state.min_accel[0] = std::min(state.min_accel[0], accelX);
    state.max_accel[1] = std::max(state.max_accel[1], accelY); state.min_accel[1] = std::min(state.min_accel[1], accelY);
    state.max_accel[2] = std::max(state.max_accel[2], accelZ); state.min_accel[2] = std::min(state.min_accel[2], accelZ);

    state.window_samples++;
    state.window_time += deltaTime;

    // Standard online rolling mean tracking equations
    float invSamples = 1.0f / (float)state.window_samples;
    state.mean_gyro[0]  += (gyroX - state.mean_gyro[0]) * invSamples;
    state.mean_gyro[1]  += (gyroY - state.mean_gyro[1]) * invSamples;
    state.mean_gyro[2]  += (gyroZ - state.mean_gyro[2]) * invSamples;
    state.mean_accel[0] += (accelX - state.mean_accel[0]) * invSamples;
    state.mean_accel[1] += (accelY - state.mean_accel[1]) * invSamples;
    state.mean_accel[2] += (accelZ - state.mean_accel[2]) * invSamples;

    // 2. Evaluate delta shifts across window thresholds
    float gDeltaX = state.max_gyro[0] - state.min_gyro[0];
    float gDeltaY = state.max_gyro[1] - state.min_gyro[1];
    float gDeltaZ = state.max_gyro[2] - state.min_gyro[2];

    float aDeltaX = state.max_accel[0] - state.min_accel[0];
    float aDeltaY = state.max_accel[1] - state.min_accel[1];
    float aDeltaZ = state.max_accel[2] - state.min_accel[2];

    if (state.window_samples >= MinStillnessSamples && state.window_time >= MinStillnessCollectionTime) {
        
        // Confirm if variations remain under hard physical thresholds
        if (gDeltaX <= MaxGyroDeltaTolerance && gDeltaY <= MaxGyroDeltaTolerance && gDeltaZ <= MaxGyroDeltaTolerance &&
            aDeltaX <= MaxAccelDeltaTolerance && aDeltaY <= MaxAccelDeltaTolerance && aDeltaZ <= MaxAccelDeltaTolerance) 
        {
            if (state.window_time >= MinStillnessCorrectionTime) {
                state.steady_time = std::min(state.steady_time + deltaTime, 3.0f);
                float easeInFactor = state.steady_time / 3.0f; // Scale smoothing linearly on ease-in windows

                // Perform exponential blend factor step to drift existing offsets toward newly discovered baselines
                float lerpFactor = std::exp2f(-easeInFactor * deltaTime / 0.1f);
                state.calibration_offset[0] = state.mean_gyro[0] + (state.calibration_offset[0] - state.mean_gyro[0]) * lerpFactor;
                state.calibration_offset[1] = state.mean_gyro[1] + (state.calibration_offset[1] - state.mean_gyro[1]) * lerpFactor;
                state.calibration_offset[2] = state.mean_gyro[2] + (state.calibration_offset[2] - state.mean_gyro[2]) * lerpFactor;
            }
        } 
        else {
            // Player moved or adjusted the controller. Flush collection window context instantly.
            state.steady_time = 0.0f;
            state.window_samples = 0;
            state.window_time = 0.0f;
        }
    }
}

void Moss_ResetGamepadCalibration(Moss_Gamepad* gp) {
    if (gp == nullptr) return;

    std::lock_guard<std::mutex> lock(g_sensorMutex);
    
    const auto it = g_gamepadSensors.find(gp);
    if (it != g_gamepadSensors.end()) {
        auto& state = it->second; // References the active GamepadSensorStates block

        // 1. Flush the historical calibration math offsets back to zero
        state.calibration_offset[0] = 0.0f;
        state.calibration_offset[1] = 0.0f;
        state.calibration_offset[2] = 0.0f;

        // 2. Clear out the sliding stillness validation windows
        state.min_gyro[0]   = state.min_gyro[1]   = state.min_gyro[2]   = 0.0f;
        state.max_gyro[0]   = state.max_gyro[1]   = state.max_gyro[2]   = 0.0f;
        state.mean_gyro[0]  = state.mean_gyro[1]  = state.mean_gyro[2]  = 0.0f;
        
        state.min_accel[0]  = state.min_accel[1]  = state.min_accel[2]  = 0.0f;
        state.max_accel[0]  = state.max_accel[1]  = state.max_accel[2]  = 0.0f;
        state.mean_accel[0] = state.mean_accel[1] = state.mean_accel[2] = 0.0f;

        // 3. Reset the tracking counters so it starts gathering a fresh baseline sample
        state.window_samples = 0;
        state.window_time    = 0.0f;
        state.steady_time    = 0.0f;

        // 4. Reset the underlying orientation quaternion back to identity
        state.orientation[0] = 1.0f; // W
        state.orientation[1] = 0.0f; // X
        state.orientation[2] = 0.0f; // Y
        state.orientation[3] = 0.0f; // Z
    }
}