
#include <Moss/Platform/android/platform_android.h>
#include <android/looper.h>
#include <android/input.h>
#include <android/sensor.h>

struct android_poll_source* source;

void Moss_PollEvents(void) {
    if (!moss_android.app) return;

    int32_t ident;
    int32_t events;

    // Poll all pending events without blocking (0 timeout)
    while ((ident = ALooper_pollAll(0, NULL, &events, (void**)&source)) >= 0) {

        // If we have a source, process it
        if (source) {
            source->process(moss_android.app, source);
        }

        // Handle lifecycle events
        switch (ident) {
            case LOOPER_ID_MAIN: // Main thread events
                break;

            case ALOOPER_POLL_WAKE: // Woken up, no action needed
            case ALOOPER_POLL_CALLBACK:
            case ALOOPER_POLL_TIMEOUT:
            case ALOOPER_POLL_ERROR:
                break;
        }

        // Process input events
        if (moss_android.app->inputQueue) {
            AInputEvent* event = NULL;
            while (AInputQueue_getEvent(moss_android.app->inputQueue, &event) >= 0) {
                if (AInputQueue_preDispatchEvent(moss_android.app->inputQueue, event)) continue;

                int32_t handled = Moss_Android_HandleInput(moss_android.app, event);
                AInputQueue_finishEvent(moss_android.app->inputQueue, event, handled);
            }
        }
    }
}