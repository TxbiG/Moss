



// ios_window.mm
// Create window    <
// Pollevents       <
// 




Moss_Window* Moss_CreateWindow(const char* title, int width, int height, Moss_Monitor* monitor, Moss_Window* share) {
    Moss_Window* win = &ios_app;
    memset(&win, 0, sizeof(Moss_Window));

    // iOS ignores title, monitor, share — but we store width/height for parity
    win->screenWidth  = width;
    win->screenHeight = height;
    win->running      = true;

    // Start UIApplication in a detached thread so we can keep control
    dispatch_async(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0), ^{
        @autoreleasepool {
            int argc = 0;
            char *argv[] = { NULL };
            UIApplicationMain(argc, argv, nil, NSStringFromClass([IOS_AppDelegate class]));
        }
    });

    // Wait until UIWindow is ready
    while (!ios_app.glContext) {
        [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode beforeDate:[NSDate dateWithTimeIntervalSinceNow:0.01]];
    }

#if defined(MOSS_USE_METAL)
    id<MTLDevice> device = MTLCreateSystemDefaultDevice();
    if (!device) return 0;

    CAMetalLayer* metalLayer = [CAMetalLayer layer];
    metalLayer.device = device;
    metalLayer.pixelFormat = MTLPixelFormatBGRA8Unorm;
    metalLayer.framebufferOnly = YES;
    metalLayer.frame = view.layer.frame;
    view.layer = metalLayer;

    win->metalDevice = (__bridge_retained void*)device;
    win->metalLayer  = (__bridge_retained void*)metalLayer;
#else
    win->glContext = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];
    if (!win->glContext) return 0;
    [EAGLContext setCurrentContext:win->glContext];
#endif // MOSS_USE_METAL or MOSS_USE_GLES2

    return win;
}


static inline void PollEvents() {
    @autoreleasepool {
        // Run the runloop briefly to process events
        [[NSRunLoop currentRunLoop] runMode:NSDefaultRunLoopMode beforeDate:[NSDate dateWithTimeIntervalSinceNow:0.001]];
    }
}


void Moss_TerminateWindow(Moss_Window* window) {
    if (window)
        free(window);
    // Any extra cleanup for Metal/GL contexts should go here

    // On iOS, forcefully exiting is unusual — do this for dev only (change exit as we dont do that)
    dispatch_async(dispatch_get_main_queue(), ^{ exit(0); });
}


int Moss_ShouldWindowClose(Moss_Window* window) { return (window && !window->running); } // Return true if the window is flagged for closing


#ifdef MOSS_GRAPHICS_VULKAN
VkResult Moss_CreateWindowSurface(Moss_Window* window, VkInstance vk_instance, const VkAllocationCallbacks *allocator, VkSurfaceKHR* vk_surface) {
    VkMetalSurfaceCreateInfoEXT surfaceInfo{};
    surfaceInfo.sType = VK_STRUCTURE_TYPE_METAL_SURFACE_CREATE_INFO_EXT;
    surfaceInfo.pLayer = myCAMetalLayer;  // MTKView.layer on iOS/macOS

    vkCreateMetalSurfaceEXT(vk_instance, &surfaceInfo, nullptr, &vk_surface);
    return res;
}
#endif