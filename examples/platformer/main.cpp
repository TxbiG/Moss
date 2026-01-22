#pragma once

#include <Moss/Moss_Platform.h>

Moss_Window* window;
Moss_Event events;
//Moss_Renderer* renderer;

enum class Scenes : uint8_t {MAINMENU, GAMEOVER, GAME};
// Support

void loop()
{
    
}



int main()
{
    //Moss_Init();

    window = Moss_CreateWindow("2D Platformer", 780, 420, NULL, NULL);

    Moss_MakeContextCurrent(window);

    //Moss_Init_Audio(window);

    //renderer = Moss_Create_Renderer(window);


    while (Moss_ShouldWindowClose(window))
    {
        Moss_PollEvents(&events);

        //loop();

        ////Moss_RendererBeginFrame();
        //Moss_PresentRenderer(renderer);
    }


    //Moss_Terminate_Renderer(renderer);
    //Moss_Terminate_Audio(window);
    Moss_TerminateWindow(window);
    //Moss_Terminate();

    return 0;
}