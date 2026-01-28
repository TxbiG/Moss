#include <Moss/Platform/Windows/win32_platform.h>







Moss_Haptic* Moss_OpenHaptic(Moss_HapticID id) {
    
}

void Moss_CloseHaptic(Moss_Haptic* haptic) {
    
}

Moss_HapticID Moss_CreateHapticEffect(Moss_Haptic* haptic) {
    
}

void Moss_DestroyHapticEffect(Moss_Haptic* haptic) {
    
}

bool Moss_GetHapticEffectStatus(Moss_Haptic* haptic) {
    
}

uint32_t Moss_GetHapticFeatures(Moss_Haptic* haptic) {
    
}

Moss_Haptic* Moss_GetHapticFromID(Moss_Haptic* haptic) {
    
}

Moss_HapticID* Moss_GetHapticID(Moss_Haptic* haptic) {
    
}

const char* Moss_GetHapticName(Moss_Haptic* haptic) {
    
}

const char* Moss_GetHapticNameForID(Moss_Haptic* haptic) {
    
}

Moss_HapticID* Moss_GetHaptics(Moss_Haptic* haptic) {
    
}

int Moss_GetMaxHapticEffects(Moss_Haptic* haptic) {
    
}

int Moss_GetMaxHapticEffectsPlaying(Moss_Haptic* haptic) {
    
}

int Moss_GetNumHapticAxes(Moss_Haptic* haptic) {
    
}

bool Moss_HapticEffectSupported(Moss_Haptic* haptic) {
    
}

bool Moss_HapticRumbleSupported(Moss_Haptic* haptic) {
    
}

bool Moss_InitHapticRumble(Moss_Haptic* joystick) {
    
}

bool Moss_IsJoystickHaptic(Moss_Joystick) {
    
}

bool Moss_IsMouseHaptic(void) {
    
}

Moss_Haptic* Moss_OpenHapticFromJoystick(Moss_Joystick* joystick) {
    
}

Moss_Haptic* Moss_OpenHapticFromMouse(void) {
    
}

bool Moss_PauseHaptic(Moss_Haptic* haptic) {
    
}

bool Moss_PlayHapticRumble(Moss_Haptic* haptic, float strength, uint32_t length) {
    
}

bool Moss_ResumeHaptic(Moss_Haptic* haptic) {
    
}

bool Moss_RunHapticEffect(Moss_Haptic* haptic, uint32_t iterations) {
    
}

bool Moss_SetHapticAutocenter(Moss_Haptic* haptic, int center) {
    
}

bool Moss_SetHapticGain(Moss_Haptic* haptic, int gain) {
    
}

bool Moss_StopHapticEffect(Moss_Haptic* haptic, Moss_HapticEffectID effect) {
    
}

bool Moss_StopHapticEffects(Moss_Haptic* haptic) {
    
}

bool Moss_StopHapticRumble(Moss_Haptic* haptic) {
    
}

bool Moss_UpdateHapticEffect(Moss_Haptic* haptic, Moss_HapticEffectID effect, const Moss_HapticEffect* data) {
    
}