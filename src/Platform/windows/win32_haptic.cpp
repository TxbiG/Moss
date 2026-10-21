#include "win32_platform.h"


Moss_Haptic* Moss_OpenHaptic(Moss_HapticID id) {
    return nullptr;
}

void Moss_CloseHaptic(Moss_Haptic* haptic) {
    
}

Moss_HapticID Moss_CreateHapticEffect(Moss_Haptic* haptic) {
    return 0;
}

void Moss_DestroyHapticEffect(Moss_Haptic* haptic) {
    
}

bool Moss_GetHapticEffectStatus(Moss_Haptic* haptic) {
    return false;
}

uint32_t Moss_GetHapticFeatures(Moss_Haptic* haptic) {
    return 0;
}

Moss_Haptic* Moss_GetHapticFromID(Moss_Haptic* haptic) {
    return nullptr;
}

Moss_HapticID* Moss_GetHapticID(Moss_Haptic* haptic) {
    return 0;
}

const char* Moss_GetHapticName(Moss_Haptic* haptic) {
    return "Hello";
}

const char* Moss_GetHapticNameForID(Moss_Haptic* haptic) {
    return "Hello";
}

Moss_HapticID* Moss_GetHaptics(Moss_Haptic* haptic) {
    return 0;
}

int Moss_GetMaxHapticEffects(Moss_Haptic* haptic) {
    return 0;
}

int Moss_GetMaxHapticEffectsPlaying(Moss_Haptic* haptic) {
    return 0;
}

int Moss_GetNumHapticAxes(Moss_Haptic* haptic) {
    
}

bool Moss_HapticEffectSupported(Moss_Haptic* haptic) {
    return false;
}

bool Moss_HapticRumbleSupported(Moss_Haptic* haptic) {
    return false;
}

bool Moss_InitHapticRumble(Moss_Haptic* joystick) {
    return false;
}

bool Moss_IsJoystickHaptic(Moss_Gamepad* joystick) {
    if (!joystick) {
        return false;
    }

    return joystick->connected;
}

bool Moss_IsMouseHaptic(void) {
    return false;
}

Moss_Haptic* Moss_OpenHapticFromJoystick(Moss_Gamepad* joystick) {
    if (!joystick) {
        return nullptr;
    }

    return nullptr; // replace with actual haptic object when implemented
}

Moss_Haptic* Moss_OpenHapticFromMouse(void) {
    return nullptr;
}

bool Moss_PauseHaptic(Moss_Haptic* haptic) {
    return false;
}

bool Moss_PlayHapticRumble(Moss_Haptic* haptic, float strength, uint32_t length) {
    return false;
}

bool Moss_ResumeHaptic(Moss_Haptic* haptic) {
    return false;
}

bool Moss_RunHapticEffect(Moss_Haptic* haptic, uint32_t iterations) {
    return false;
}

bool Moss_SetHapticAutocenter(Moss_Haptic* haptic, int center) {
    return false;
}

bool Moss_SetHapticGain(Moss_Haptic* haptic, int gain) {
    return false;
}

bool Moss_StopHapticEffect(Moss_Haptic* haptic, Moss_HapticEffectID effect) {
    return false;
}

bool Moss_StopHapticEffects(Moss_Haptic* haptic) {
    return false;
}

bool Moss_StopHapticRumble(Moss_Haptic* haptic) {
    return false;
}

bool Moss_UpdateHapticEffect(Moss_Haptic* haptic, Moss_HapticEffectID effect, const Moss_HapticEffect* data) {
    return false;
}