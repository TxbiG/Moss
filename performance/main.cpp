#pragma once

//#define Moss_Include_OpenGL
//#define Moss_Include_Vulkan   < still needs implemented
//#define Moss_Include_DirectX  < still needs implemented


#include <Moss/Moss.h>
#include <Moss/Moss_Platform.h> // Successful dont touch
#include <Moss/Moss_Audio.h> // Successful dont touch
#include <Moss/Moss_Renderer.h> // Successful dont touch
//#include <Moss/Moss_GUI.h> // Successful dont touch (Still needs more testing)
#include <stdio.h>


/*
Physics:
    Stress test.
    new/updated implementations.
Renderer:
    Stress Test.
    New implemenations.
Audio:
    Play Audio File.
    Play audio file spacial 2D and 3D.
    Spacial Audio.
    Play Different Effects.
*/

Moss_Window* window;
// Scenes

// Add XR, Navigation, Network, GUI
enum class Scenes {MENU, AUDIO, RENDERER, PHYSICS, NETWORK, XR, NAVIGATION, UI };
enum class AudioScenes : uint8_t {AUDIOMENU, NonSpatial, Spatial2D, Spatial3D};
enum class RenderScenes : uint8_t {RENDERMENU, RENDER2D, RENDER3D};
enum class PhyscisScenes : uint8_t {PHYSICSMENU, PHYSICS2, PHYSICS3};
enum class XRScenes {XRMENU, VR, AR, MR};
enum class NavigationScenes {NAVMENU, Nav2D, Nav3D};

Scenes sceneType = Scenes::MENU;
AudioScenes AudioSceneType = AudioScenes::AUDIOMENU;
RenderScenes RenderSceneType = RenderScenes::RENDERMENU;
PhyscisScenes PhysicsSceneType = PhyscisScenes::PHYSICSMENU;

// Audio
//Moss_AudioMixer* master = Moss_Audio_Create_Channel();
//Moss_AudioStream* Music = Moss_CreateAudioStreamPlayer("");
//Moss_AudioStream* SFX2D = Moss_CreateAudioStreamPlayer2D("");
//Moss_AudioStream* SFX3D = Moss_CreateAudioStreamPlayer3D("");

//Camera2 cam2 = new Camera2()
//Camera3* cam3 = new Camera3()
//Moss_AudioStreamPlayer_Play(Moss_AudioStream* stream);
//Moss_AudioStreamPlayer_Pause(Moss_AudioStream* stream);
//Moss_AudioStreamPlayer_Stop(Moss_AudioStream* stream);
//Moss_AudioStreamPlayer_UpdatePosition2D(Moss_AudioStream* stream, float x, float y);
//Moss_AudioStreamPlayer_UpdatePosition3D(Moss_AudioStream* stream, float x, float y, float z);
//Moss_SetAudioStreamPlayer_Volume(Moss_AudioStream* stream, float db);
//Moss_SetAudioStreamPlayer_Pitch(Moss_AudioStream* stream, float pitch);
//Moss_GetAudioStreamPlayer_Volume(Moss_AudioStream* stream);
//Moss_GetAudioStreamPlayer_Pitch(Moss_AudioStream* stream);
//Audio Listener (for spatial audio)
//Moss_SetAudioListener2D(float x, float y);
//Moss_SetAudioListener3D(float x, float y, float z);


void MainScene()
{
    // MAIN
    if (sceneType == Scenes::MENU)
    {

    }

    // Audio
    if (AudioSceneType == AudioScenes::AUDIOMENU){}

    // Renderer
    if (RenderSceneType == RenderScenes::RENDERMENU){}

    // Physics
    if (PhysicsSceneType == PhyscisScenes::PHYSICSMENU){}
}

void loop()
{
    while (Moss_ShouldWindowClose(window))
    {
        //Moss_RendererBeginFrame();
        //Wav* gunshotWav = loadWav("sound.wav");
        //AudioStream2D explosion(*gunshotWav);
        //AudioStream3D explosion(*gunshotWav);
        /*
        explosion.position      = { 00.f, -30.f, 0.f };
        explosion.velocity      = { 0.f, 0.f, 0.f };
        explosion.maxDistance   = 50.f;

        Float3 listenerPos      = { 0.f, 0.f, 0.f };
        Float3 listenerVel      = { 0.f, 0.f, 0.f };
        SetAudioListener3D(&listenerPos);
        SetAudioListenerVelocity3D(&listenerVel);
        explosion.play();
        */
        switch (sceneType)
        {
        case Scenes::MENU:
            MainScene();
            break;
        case Scenes::AUDIO:
            switch (AudioSceneType)
            {
                case AudioScenes::AUDIOMENU:
                    MainScene();
                    break;
                case AudioScenes::NonSpatial:
                    break;
                case AudioScenes::Spatial2D:
                    break;
                case AudioScenes::Spatial3D:
                    break;
                default:
                    break;
            }
            break;
        case Scenes::RENDERER:
            switch (RenderSceneType)
            {
                case RenderScenes::RENDERMENU:
                    MainScene();
                    break;
                case RenderScenes::RENDER2D:
                    MainScene();
                    break;
                case RenderScenes::RENDER3D:
                    MainScene();
                    break;
                default:
                    break;
            }
            break;
        case Scenes::PHYSICS:
            switch (PhysicsSceneType)
            {
                case PhyscisScenes::PHYSICSMENU:
                    MainScene();
                    break;
                case PhyscisScenes::PHYSICS2:
                    MainScene();
                    break;
                case PhyscisScenes::PHYSICS3:
                    MainScene();
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
        }
        //Moss_PresentRenderer(renderer);
    }
}

#include <thread>

int main()
{
    //Moss_Init();
    Moss_Init_Audio();

    //if (!Moss_Init_Audio()) { printf("Failed to create audio!\n"); }
    window = Moss_CreateWindow("Moss Framework Test", 780, 420, NULL, NULL);
    if (!window) { printf("Failed to create window!\n"); return 0; }
    
    //Moss_MakeContextCurrent(window);
    Moss_Renderer* renderer = Moss_CreateRenderer(window);
    if (!renderer) { printf("Failed to create renderer!\n"); return 0; }
    /*
    Camera2 camera;
    //Moss_RendererActivateCamera2(renderer, camera);
    //camera.position = Float2(0.0f, 0.0f);
    Texture tex("Tex.png", TextureType::Texture2D);
    Surface surface(0.0f, 0.0f, 100.0f, 100.0f, tex); // x, y, width, height
    //Surface surface(0, 0, 1, 1, Color(1, 0, 0, 1));
    //surface.color = Color(0.0f, 1.0f, 0.0f, 1.0f);  // green, for example
    //Texture texture("Tex.png"); // or any Texture2D
    //Camera3 camera;
    Font font;
    if (!font.load("C:/Windows/Fonts/arial.ttf", 50.0f)) {
        printf("Font failed to load!");
    }

    DeltaTime time;

    const float targetFPS = 120.0f;
    bool limitFPS = (targetFPS > 0);
    const float targetFrameTime = (limitFPS) ? (1.0f / targetFPS) : 0.0f;
    */
    while (!Moss_ShouldWindowClose(window)) {
        Moss_RendererBeginFrame(renderer);
        Moss_RendererEndFrame(renderer);
        Moss_PollEvents();
        /*
        time.StartFrame();
        float delta = time.GetDeltaTime();
        //surface.rect.x += 0.0001f;
        //camera.position.x += 0.01f;
        if (IsPressed(Keyboard::KEY_W)) { surface.position.y -= 0.2f; }  // move camera up
        if (IsPressed(Keyboard::KEY_A)) { surface.position.x -= 0.2f; }  // move camera left
        if (IsPressed(Keyboard::KEY_S)) { surface.position.y += 0.2f; }  // move camera down
        if (IsPressed(Keyboard::KEY_D)) { surface.position.x += 0.2f; }  // move camera right


        surface.update();
        camera.update(renderer->width, renderer->height);
        Moss_RendererBeginFrame(renderer);

        std::string fpsText = "FPS: " + std::to_string(time.GetFPS());
        font.renderText(fpsText.c_str(), 10, 10, camera.getViewProjectionMatrix());

        font.renderText("Hello, stb!", 100.5f, 100.5f, camera.getViewProjectionMatrix());
        surface.draw(camera.getViewProjectionMatrix());
        //surface.draw(camera.getViewProjectionMatrix());

        //quadRenderer.render(texture);

        //Rect screen = { 50, 50, 800, 600 };
        //BeginCenterContainer(screen);
            //Rect rootRect = { -100, -150, 200, 300 };
            //BeginVContainer(screen, 50.0f);
                //if (Button("Play", nullptr, camera.getViewProjectionMatrix())) { printf("h"); }
                //if (Button("Quit", nullptr, camera.getViewProjectionMatrix())) { printf("h"); }
            //EndVContainer();
        ////EndCenterContainer();
        //camera.draw();
        Moss_RendererEndFrame(renderer);
        Moss_PollEvents();

        if (limitFPS) {
            float frameDuration = time.GetDeltaTime();
            float sleepTime = targetFrameTime - frameDuration;

            if (sleepTime > 0.001f) {
                std::this_thread::sleep_for(std::chrono::duration<float>(sleepTime));
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        */
    }

    //loop();

    /*          Audio        */
    //DeleteAudioStream(Music);
    //DeleteAudioStream(SFX2D);
    //DeleteAudioStream(SFX3D);
    //Moss_Audio_Delete_Channel(master);
    //RemoveWav(gunshotWav);

    Moss_Terminate_Audio();
    Moss_TerminateRenderer(renderer);
    Moss_TerminateWindow(window);

    return 0;
}