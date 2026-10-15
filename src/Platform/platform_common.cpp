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

// Stores a new sample received from a native backend.
void PublishGamepadSensorSample(Moss_Gamepad* gp, Moss_GamepadSensorType sensor, const Moss_GamepadSensorData& sample) {
    std::size_t index = 0;

    if (gp == nullptr || !GetSensorIndex(sensor, index) || !IsValidSample(sample)) { return; }

    std::lock_guard<std::mutex> lock(g_sensorMutex);

    const auto gamepadIt = g_gamepadSensors.find(gp);

    if (gamepadIt == g_gamepadSensors.end())
        return;

    SensorState& state = gamepadIt->second[index];

    if (!state.supported || !state.enabled)
        return;

    // Do not replace a newer sample with an older asynchronous sample.
    if (state.hasSample && sample.timestamp_ns < state.latest.timestamp_ns) { return; }

    state.latest = sample;
    state.hasSample = true;
}

// Clears cached data when a gamepad disconnects or is destroyed.
// Stop native sensor callbacks before calling this function.
void ResetGamepadSensorState(Moss_Gamepad* gp) {
    if (gp == nullptr)
        return;

    std::lock_guard<std::mutex> lock(g_sensorMutex);
    g_gamepadSensors.erase(gp);
}

// Public Moss API implementations.

bool Moss_GamepadHasSensor(Moss_Gamepad* gp, Moss_GamepadSensorType sensor) { return GamepadHasSensor(gp, sensor); }
bool Moss_SetGamepadSensorEnabled(Moss_Gamepad* gp, Moss_GamepadSensorType sensor, bool enabled) { return SetGamepadSensorEnabled(gp, sensor, enabled); }
bool Moss_GetGamepadSensorData(Moss_Gamepad* gp, Moss_GamepadSensorType sensor, Moss_GamepadSensorData* data) { return GetGamepadSensorData(gp, sensor, data); }