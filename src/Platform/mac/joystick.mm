

#import <IOKit/hid/IOHIDLib.h>

static IOHIDManagerRef hidManager = nullptr;


static void DeviceAdded(void* context, IOReturn result, void* sender, IOHIDDeviceRef device) {
    NSLog(@"Gamepad was plugged in");
}

static void DeviceRemoved(void* context, IOReturn result, void* sender, IOHIDDeviceRef device) {
    NSLog(@"Gamepad disconnected");
}

static void InputCallback(void* context, IOReturn result, void* sender, IOHIDValueRef value) {
    IOHIDElementRef elem = IOHIDValueGetElement(value);
    CFIndex usage      = IOHIDElementGetUsage(elem);
    CFIndex usagePage  = IOHIDElementGetUsagePage(elem);
    long pressed       = IOHIDValueGetIntegerValue(value);

    // Convert usage+page to your Gamepad enum and update currentGamepad
}

void Moss_SetupGamepadHID() {
    hidManager = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone);

    NSMutableDictionary* criteria = [NSMutableDictionary dictionary];
    criteria[(NSString*)CFSTR(kIOHIDDeviceUsagePageKey)] = @((int)kHIDPage_GenericDesktop);
    criteria[(NSString*)CFSTR(kIOHIDDeviceUsageKey)]     = @((int)kHIDUsage_GD_GamePad);

    IOHIDManagerSetDeviceMatching(hidManager, (CFDictionaryRef)criteria);
    IOHIDManagerRegisterDeviceMatchingCallback(hidManager, DeviceAdded, NULL);
    IOHIDManagerRegisterDeviceRemovalCallback(hidManager, DeviceRemoved, NULL);
    IOHIDManagerRegisterInputValueCallback(hidManager, InputCallback, NULL);

    IOHIDManagerScheduleWithRunLoop(hidManager, CFRunLoopGetCurrent(), kCFRunLoopDefaultMode);
    IOHIDManagerOpen(hidManager, kIOHIDOptionsTypeNone);
}


Gamepad MapUsageToGamepad(CFIndex usage, CFIndex page) {
    if (page == kHIDPage_Button) {
        switch (usage) {
            case 1: return Gamepad::GAMEPAD_BUTTON_A;
            case 2: return Gamepad::GAMEPAD_BUTTON_B;
            // ... up to your COUNT
            default: break;
        }
    } else if (page == kHIDPage_GenericDesktop) {
        if (usage == kHIDUsage_GD_DPadUp)    return Gamepad::GAMEPAD_BUTTON_DPAD_UP;
        // map axes and dpad directions similarly
    }
    return Gamepad::COUNT;
}


Gamepad button = MapUsageToGamepad(usage, usagePage);
if (button != Gamepad::COUNT) {
    bool pressed = pressedValue > 0;
    currentGamepad[static_cast<size_t>(button)] = pressed;
    if (g_gamepadButtonCallback) g_gamepadButtonCallback(button, pressed);
}
