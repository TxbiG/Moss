#pragma once

#include <Moss/Moss_Platform.h>

Moss_Window* window;
//Moss_Renderer* renderer;
Moss_Event events;


enum class Scenes : uint8_t {MAINMENU, GAMEOVER, GAME};

/*
struct Bird {
    Vec2 position;
    float rotation;
    Texture texture;
    // Collision
};

struct Piller {
    Vec2 position;
    Texture texture;
    int minHeight int maxHeight;
    // Collision
};


// Globals
Piller pillarPool[6];
Scenes scene = MAINMENU;
*/


void input(Moss_Event event) {
    while (Moss_PollEvents(&event))
    {
        switch(event.type)
        {
            case InputEventType::EVENT_KEYDOWN:
                // handle key down
                break;

            case InputEventType::EVENT_KEYUP:
                // handle key up
                break;

            case InputEventType::EVENT_MOUSEBUTTONDOWN:
                // optional: handle mouse down
                break;

            case InputEventType::EVENT_MOUSEBUTTONUP:
                // optional: handle mouse up
                break;

            case InputEventType::EVENT_MOUSEMOVE:
                // optional: handle mouse move
                break;

            case InputEventType::EVENT_CONTROLLERBUTTONDOWN:
                // optional: handle controller button down
                break;

            case InputEventType::EVENT_CONTROLLERBUTTONUP:
                // optional: handle controller button up
                break;

            case InputEventType::EVENT_NONE:
                // optionally ignore
                break;
        }
    }
}
void update() {

}
/*
void render(Renderer& renderer)
{
    //Moss_RendererBeginFrame();

    //Moss_PresentRenderer(renderer);
}*/

int main()
{
    //Moss_Init();

    window = Moss_CreateWindow("Flappy bird", 780, 420, NULL, NULL);

    Moss_MakeContextCurrent(window);

    //Moss_Init_Audio(window);

    //renderer = Moss_Create_Renderer(window);


    while (Moss_ShouldWindowClose(window))
    {
        input(events);         // Input
        update();              // Update
        //render(renderer);    // Render
    }


    //Moss_Terminate_Renderer(renderer);
    //Moss_Terminate_Audio(window);
    Moss_TerminateWindow(window);
    //Moss_Terminate();

    return 0;
}