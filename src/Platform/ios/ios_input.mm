


// ios_input.mm
// Control the vibrating < helps with HapticFeedback
// Input controlls  < Touch input

#import <UIKit/UIKit.h>
#import <AudioToolbox/AudioToolbox.h>

static inline void Moss_PlayHapticEffect(const Moss_HapticEffect *effect) {
    if (!effect) return;

#if defined(__APPLE__)
    if (@available(iOS 10.0, *)) {
        switch (effect->type) {
            case HAPTIC_CONSTANT:
            case HAPTIC_LEFTRIGHT: {
                uint16_t strength = (effect->type == HAPTIC_CONSTANT)  ? (uint16_t)(effect->constant.level + 32767) / 65535  : (effect->leftright.large_magnitude + effect->leftright.small_magnitude) / 2;

                UIImpactFeedbackStyle style;
                if (strength < 85) style = UIImpactFeedbackStyleLight;
                else if (strength < 170) style = UIImpactFeedbackStyleMedium;
                else style = UIImpactFeedbackStyleHeavy;

                UIImpactFeedbackGenerator *gen = [[UIImpactFeedbackGenerator alloc] initWithStyle:style];
                [gen prepare];
                [gen impactOccurred];
                break;
            }
            case HAPTIC_PERIODIC:
                Moss_PlayNotificationHaptic(1); // Map periodic to "success" style
                break;
            case HAPTIC_CUSTOM:
                Moss_PlayNotificationHaptic(2); // Map custom to "warning" style
                break;
            default:
                break;
        }
    } else {
        AudioServicesPlaySystemSound(kSystemSoundID_Vibrate);
    }
}

static inline void Moss_PlayNotificationHaptic(uint8_t type) {
    if (@available(iOS 10.0, *)) {
        UINotificationFeedbackGenerator *gen = [[UINotificationFeedbackGenerator alloc] init];
        [gen prepare];
        if (type == 1) [gen notificationOccurred:UINotificationFeedbackTypeSuccess];
        else if (type == 2) [gen notificationOccurred:UINotificationFeedbackTypeWarning];
        else [gen notificationOccurred:UINotificationFeedbackTypeError];
    }
}