#ifndef MOSS_PLATFORM_IOS_H
#define MOSS_PLATFORM_IOS_H

// ios_platform.h


#include <Moss/Platform/platform_intern.h>


#include <UIKit/UIKit.h>
#include <QuartzCore/CADisplayLink.h>
#include <AVFoundation/AVFoundation.h>

#ifdef MOSS_USE_METAL
#include <Metal/Metal.h>
#include <QuartzCore/CAMetalLayer.h>
#else   // OPENGL ES 2 if not metal
#include <OpenGLES/ES2/gl.h>
#include <OpenGLES/EAGL.h>
#endif

// ============================
// C-Style Callback Definitions
// ============================

// Lifecycle
typedef void (*IOS_OnInit)();
typedef void (*IOS_OnUpdate)(float deltaTime);
typedef void (*IOS_OnShutdown)();

// Window / Screen
typedef void (*IOS_OnResize)(int width, int height, UIInterfaceOrientation orientation);

// Touch
typedef void (*IOS_OnTouch)(int fingerId, float x, float y, int phase); 
// phase: 0=began, 1=moved, 2=ended, 3=canceled

// Pause / Resume
typedef void (*IOS_OnPause)();
typedef void (*IOS_OnResume)();

// ============================
// App Globals
// ============================
struct Moss_Window {
    IOS_OnInit       onInit;
    IOS_OnUpdate     onUpdate;
    IOS_OnShutdown   onShutdown;
    IOS_OnResize     onResize;
    IOS_OnTouch      onTouch;
    IOS_OnPause      onPause;
    IOS_OnResume     onResume;

    int screenWidth;
    int screenHeight;
    bool running;

    CADisplayLink *displayLink;

#ifdef MOSS_USE_METAL
    void* metalDevice; // id<MTLDevice>
    void* metalLayer;  // CAMetalLayer*
#else
    EAGLContext *glContext;
#endif
};
static Moss_Window ios_app;

// ============================
// File Access Helpers
// ============================
static inline const char* IOS_GetDocumentsPath() {
    static char buffer[512];
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    snprintf(buffer, sizeof(buffer), "%s", [[paths firstObject] UTF8String]);
    return buffer;
}

static inline const char* IOS_GetCachesPath() {
    static char buffer[512];
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSCachesDirectory, NSUserDomainMask, YES);
    snprintf(buffer, sizeof(buffer), "%s", [[paths firstObject] UTF8String]);
    return buffer;
}

// ============================
// App Delegate & View
// ============================
@interface IOS_View : UIView
@end
@implementation IOS_View
- (void)touchesBegan:(NSSet<UITouch *> *)touches withEvent:(UIEvent *)event {
    for (UITouch *touch in touches) {
        CGPoint p = [touch locationInView:self];
        if (ios_app.onTouch) ios_app.onTouch((int)touch.hash, p.x, p.y, 0);
    }
}
- (void)touchesMoved:(NSSet<UITouch *> *)touches withEvent:(UIEvent *)event {
    for (UITouch *touch in touches) {
        CGPoint p = [touch locationInView:self];
        if (ios_app.onTouch) ios_app.onTouch((int)touch.hash, p.x, p.y, 1);
    }
}
- (void)touchesEnded:(NSSet<UITouch *> *)touches withEvent:(UIEvent *)event {
    for (UITouch *touch in touches) {
        CGPoint p = [touch locationInView:self];
        if (ios_app.onTouch) ios_app.onTouch((int)touch.hash, p.x, p.y, 2);
    }
}
- (void)touchesCancelled:(NSSet<UITouch *> *)touches withEvent:(UIEvent *)event {
    for (UITouch *touch in touches) {
        CGPoint p = [touch locationInView:self];
        if (ios_app.onTouch) ios_app.onTouch((int)touch.hash, p.x, p.y, 3);
    }
}
@end

@interface IOS_AppDelegate : UIResponder <UIApplicationDelegate>
@property (strong, nonatomic) UIWindow *window;
@property (strong, nonatomic) IOS_View *view;
@end

@implementation IOS_AppDelegate
- (BOOL)application:(UIApplication *)application didFinishLaunchingWithOptions:(NSDictionary *)launchOptions {
    ios_app.running = true;

    // Window & View
    self.window = [[UIWindow alloc] initWithFrame:[[UIScreen mainScreen] bounds]];
    self.view = [[IOS_View alloc] initWithFrame:self.window.bounds];
    self.view.multipleTouchEnabled = YES;

    UIViewController *vc = [UIViewController new];
    vc.view = self.view;
    self.window.rootViewController = vc;
    [self.window makeKeyAndVisible];

    // GL Context
    ios_app.glContext = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];
    [EAGLContext setCurrentContext:ios_app.glContext];

    // Init Callback
    if (ios_app.onInit) ios_app.onInit();

    // Display Link (Game Loop)
    ios_app.displayLink = [CADisplayLink displayLinkWithTarget:self selector:@selector(gameLoop)];
    [ios_app.displayLink addToRunLoop:[NSRunLoop mainRunLoop] forMode:NSDefaultRunLoopMode];

    return YES;
}
- (void)gameLoop {
    static double lastTime = CACurrentMediaTime();
    double now = CACurrentMediaTime();
    float deltaTime = (float)(now - lastTime);
    lastTime = now;

    if (ios_app.onUpdate) ios_app.onUpdate(deltaTime);

    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glClearColor(0,0,0,1);
    glClear(GL_COLOR_BUFFER_BIT);
    // Present frame if needed
}
- (void)applicationWillResignActive:(UIApplication *)application {
    if (ios_app.onPause) ios_app.onPause();
}
- (void)applicationDidBecomeActive:(UIApplication *)application {
    if (ios_app.onResume) ios_app.onResume();
}
- (void)applicationWillTerminate:(UIApplication *)application {
    if (ios_app.onShutdown) ios_app.onShutdown();
}
@end


#endif // MOSS_PLATFORM_IOS_H