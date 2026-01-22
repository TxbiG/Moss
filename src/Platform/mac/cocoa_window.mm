#import <Moss/Platform/Mac/cocoa_platform.h>

#import <Cocoa/Cocoa.h>
#import <QuartzCore/CAMetalLayer.h>

#ifdef MOSS_USE_OPENGL
#import <OpenGL/gl3.h>
#import <AppKit/NSOpenGLView.h>
#endif

// App delegate for handling NSApplication lifecycle
@interface MossAppDelegate : NSObject <NSApplicationDelegate>
@end

@implementation MossAppDelegate
- (void)applicationDidFinishLaunching:(NSNotification *)notification {
    // App launched — no-op for now
}
@end

static bool initializedApp = false;
static MossAppDelegate* appDelegate = nil;

Moss_Window* Moss_CreateWindow(const char* title, int width, int height, Moss_Monitor* monitor, Moss_Window* share) {
    if (!initializedApp) {
        [NSApplication sharedApplication];
        appDelegate = [[MossAppDelegate alloc] init];
        [NSApp setDelegate:appDelegate];
        [NSApp finishLaunching];
        initializedApp = true;
    }

    Moss_Window* window = new Moss_Window();
    window->width = width;
    window->height = height;

    NSRect frame = NSMakeRect(100, 100, width, height);
    NSUInteger style = NSWindowStyleMaskTitled |
                       NSWindowStyleMaskClosable |
                       NSWindowStyleMaskResizable |
                       NSWindowStyleMaskMiniaturizable;

    NSString* nsTitle = [NSString stringWithUTF8String:title];

    NSWindow* nsWindow = [[NSWindow alloc] initWithContentRect:frame
                                                      styleMask:style
                                                        backing:NSBackingStoreBuffered
                                                          defer:NO];
    [nsWindow setTitle:nsTitle];
    [nsWindow makeKeyAndOrderFront:nil];
    window->object = (id)nsWindow;

    //
    // Rendering view setup
    //

#ifdef MOSS_USE_OPENGL
    // Create OpenGL pixel format
    NSOpenGLPixelFormatAttribute attrs[] =
    {
        NSOpenGLPFAOpenGLProfile, NSOpenGLProfileVersion3_2Core,
        NSOpenGLPFAColorSize, 24,
        NSOpenGLPFAAlphaSize, 8,
        NSOpenGLPFADepthSize, 24,
        NSOpenGLPFAStencilSize, 8,
        NSOpenGLPFADoubleBuffer,
        NSOpenGLPFAAccelerated,
        0
    };
    
    NSOpenGLPixelFormat* pixelFormat = [[NSOpenGLPixelFormat alloc] initWithAttributes:attrs];
    NSOpenGLView* glView = [[NSOpenGLView alloc] initWithFrame:frame pixelFormat:pixelFormat];
    
    [glView setWantsBestResolutionOpenGLSurface:YES];
    [glView prepareOpenGL];

    [nsWindow setContentView:glView];

    window->view = (id)glView;
    window->layer = nil;

#elif defined(MOSS_USE_METAL)
    // Use a plain NSView with a CAMetalLayer
    NSView* metalView = [[NSView alloc] initWithFrame:frame];
    [metalView setWantsLayer:YES];

    CAMetalLayer* metalLayer = [CAMetalLayer layer];
    metalLayer.pixelFormat = MTLPixelFormatBGRA8Unorm;
    metalLayer.contentsScale = [metalView.window backingScaleFactor];

    metalView.layer = metalLayer;
    [nsWindow setContentView:metalView];

    window->view = (id)metalView;
    window->layer = (id)metalLayer;
#else
#warning "No rendering backend defined! Define MOSS_USE_OPENGL or MOSS_USE_METAL."
#endif

    return window;
}

void Moss_TerminateWindow(Moss_Window* window) {
    if (window && window->object) {
        NSWindow* nsWindow = (__bridge NSWindow*)window->object;
        [nsWindow close];
        window->object = nullptr;
        window->view = nullptr;
        window->layer = nullptr;
    }

    delete window;
}


void Moss_PollEvents() {
    Moss_UpdateInputStates();
    NSEvent* event;
    while ((event = [NSApp nextEventMatchingMask:NSEventMaskAny
                                        untilDate:nil
                                           inMode:NSDefaultRunLoopMode
                                          dequeue:YES])) {
        [NSApp sendEvent:event];
    }
}


void Moss_ToggleFullscreen(Moss_Window* window) {
    if (window && window->object) {
        NSWindow* nsWindow = (__bridge NSWindow*)window->object;
        [nsWindow toggleFullScreen:nil];
    }
}





#import <Metal/Metal.h>
#import <QuartzCore/CAMetalLayer.h>
#import <Cocoa/Cocoa.h>

struct MossMetalView
{
    NSView* view;
    CAMetalLayer* layer;
};

Moss_MetalView Moss_Metal_CreateView(Moss_Window* window)
{
    // Get NSWindow from your Moss_Window
    NSWindow* nsWindow = (NSWindow*)Moss_GetNativeWindow(window);

    NSView* contentView = [nsWindow contentView];

    CAMetalLayer* metalLayer = [CAMetalLayer layer];
    metalLayer.device = MTLCreateSystemDefaultDevice();
    metalLayer.pixelFormat = MTLPixelFormatBGRA8Unorm;
    metalLayer.framebufferOnly = YES;

    // Retina correct scaling
    CGFloat scale = [nsWindow backingScaleFactor];
    metalLayer.contentsScale = scale;

    metalLayer.frame = contentView.bounds;
    [contentView setWantsLayer:YES];
    [contentView setLayer:metalLayer];

    MossMetalView* view = new MossMetalView{};
    view->view = contentView;
    view->layer = metalLayer;

    return (Moss_MetalView)view;
}

void Moss_Metal_DestroyView(Moss_MetalView handle)
{
    if (!handle)
        return;

    MossMetalView* view = (MossMetalView*)handle;

    // Do NOT release NSView owned by NSWindow
    view->layer = nil;

    delete view;
}

void* Moss_Metal_GetLayer(Moss_MetalView handle)
{
    MossMetalView* view = (MossMetalView*)handle;
    return (void*)view->layer;
}

void Moss_Metal_Resize(Moss_MetalView handle, uint32_t width, uint32_t height)
{
    MossMetalView* view = (MossMetalView*)handle;

    CGFloat scale = view->view.window.backingScaleFactor;
    view->layer.drawableSize = CGSizeMake(width * scale, height * scale);
}


#ifdef MOSS_GRAPHICS_VULKAN
VkResult Moss_CreateWindowSurface(Moss_Window* window, VkInstance vk_instance, const VkAllocationCallbacks *allocator, VkSurfaceKHR* vk_surface) {
    VkMetalSurfaceCreateInfoEXT surfaceInfo{};
    surfaceInfo.sType = VK_STRUCTURE_TYPE_METAL_SURFACE_CREATE_INFO_EXT;
    surfaceInfo.pLayer = myCAMetalLayer;  // MTKView.layer on iOS/macOS

    vkCreateMetalSurfaceEXT(vk_instance, &surfaceInfo, nullptr, &vk_surface);
    return res;
}
#endif