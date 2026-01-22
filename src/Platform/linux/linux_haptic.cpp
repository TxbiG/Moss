#include <Moss/Platform/linux/linux_platform.h>






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