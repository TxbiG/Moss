//                        MIT License
//
//                  Copyright (c) 2026 Toby
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//     /$$      /$$        
//    | $$$    /$$$         
//    | $$$$  /$$$$  /$$$$$$   /$$$$$$$ /$$$$$$$
//    | $$ $$/$$ $$ /$$__  $$ /$$_____//$$_____/
//    | $$  $$$| $$| $$  \ $$|  $$$$$$|  $$$$$$
//    | $$\  $ | $$| $$  | $$ \____  $$\____  $$
//    | $$ \/  | $$|  $$$$$$/ /$$$$$$$//$$$$$$$/
//    |__/     |__/ \______/ |_______/|_______/
//

#pragma once


/*
Celeste In C++ - https://youtube.com/playlist?list=PLFAIgTeqcARmowCzcOMil78OxcPNsac70&si=vbuU5P9kpJoOikIH
C++ OpenGL 3D Tutorial by Suraj Sharma - https://youtube.com/playlist?list=PL6xSOsbVA1eYSZTKBxnoXYboy7wc4yg-Z&si=it66RuUf0umytGNN
Intermediate OpenGL - https://youtube.com/playlist?list=PLA0dXqQjCx0TKQiXRyQU62KQgcTE6E92f&si=bP8_RuutQt73ZGM3
Vulkan (c++) Game Engine Tutorials - https://youtube.com/playlist?list=PL8327DO66nu9qYVKLDmdLW_84-yE4auCR&si=U9ndcQglXMR4dfht
Vulkan For Beginners - https://youtube.com/playlist?list=PLA0dXqQjCx0RntJy1pqje9uHRF1Z5vZgA&si=y691h7dLcGmCeOG2
OpenGL Tutorials - https://youtube.com/playlist?list=PLPaoO-vpZnumdcb4tZc4x5Q-v7CkrQ6M-&si=0EJSV4jd2hKEu-KB

https://youtu.be/x2FHHU50ktQ?si=1zYR1lmBug518mDj
https://youtu.be/oETErEkFICE?si=hiUsH7HcI1ES35mm
https://youtu.be/Uc8FEI5bg3w?si=bPz9i05X0zCh9Dz2

Improving Learn OpenGL's Text Rendering Example | Adventures in Coding - https://youtu.be/S0PyZKX4lyI?si=ezp1-jTYYxw6wJY2
Bindless Textures // Intermediate OpenGL Series - https://youtu.be/Gk3JsDCBp1o?si=fMMsROe6ELQuSx9d
Level Of Detail // Terrain Rendering episode #7 - https://youtu.be/W6PqTiWihFM?si=IFJEkCN34fYyeN8T
Modern OpenGL Tutorial - Direct State Access (DSA) - https://youtu.be/cadzqhqPqVA?si=w0kVKMHfTQYNDAgV
OpenGL Tutorial 20 - Geometry Shader - https://youtu.be/BZw2olDmZo4?si=m17vaRHiNoC23-XY
OpenGL Tutorial 22 - Anti-Aliasing (MSAA) - https://youtu.be/oHVh8htoGKw?si=hEDeD8wwXXMEEDQB
Quad Tessellation // OpenGL Tutorial #51 - https://youtu.be/_aeVGwKzVqo?si=BG16ILmmwPqeN-EZ
Advanced OpenGL Tutorial – Skeletal Animations with Assimp - https://youtu.be/GZQkwx10p-8?si=TRdS3GA0N2FPyWqL
Advanced OpenGL - Crash Course - https://youtu.be/GJFHqK_-ARA?si=4A0ypCNAPS09yBkN
Interactive Graphics 24 - Refractions, Transparency, Blending, & Alpha Testing - https://youtu.be/LTzhxLEgldA?si=bjz158YetxE8_Enf
Once Upon a Pixel: How Crysis Changed Real-Time Lighting Forever - https://youtu.be/xq5hD4RGcYw?si=CCNe3iPwx7AGumYv
OpenGL Tutorial 30 - Bloom - https://youtu.be/um9iCPUGyU4?si=JgZNfqRZ-2yqWrjO
Post processing tutorial for beginners - https://youtu.be/x2jKZgmFVq4?si=uqMKqWKKT2IAKRpL
OpenGL Tutorial 18 - Framebuffer & Post-processing - https://youtu.be/QQ3jr-9Rc1o?si=wLQBOmSm7oEfP1k_
The LOD Manager // Terrain Rendering episode #8 - https://youtu.be/EYxtuE2r7Us?si=BAM2pA4IiCddBvOW
Adding global illumination to my game engine w/ DDGI [Voxel Devlog #23] - https://youtu.be/L1vhle74AEU?si=zhFtWSDV0hkSOS-p
Skybox // Terrain Rendering episode #11 - https://youtu.be/L1NOYs0iPug?si=EcyG9auE6S1uq0Pn
Skydome // Terrain Rendering episode #12 - https://youtu.be/I0jI_d1jORc?si=F8y8O0ojAju6kXio
Terrain Tessellation Shaders // Terrain Rendering Episode 13 - https://youtu.be/GgW3MVOP8_A?si=Q2ffEUepm0RGe6WL
Indirect Rendering // Intermediate OpenGL Series - https://youtu.be/oETErEkFICE?si=hldc_LsN_0FxheJt
Skeletal Animation Using Assimp - Part 1/5 // OpenGL Tutorial #24 - https://youtu.be/r6Yv_mh79PI?si=C6fMw1U-AfWu680s
Texture Mapping // Vulkan For Beginners #18 - https://youtu.be/yps8SGA02ZE?si=zwZI5Qhk_ocHFDhj
Kohi Vulkan Graphics Pipeline (Vulkan Game Engine Series) - https://youtu.be/KanKG-vd9GA?si=rjxDswhPxkoaTjdd
Uniform Buffers // Vulkan For Beginners #17 - https://youtu.be/i6NpPdX4M4w?si=jSQVlUCifxleEhZz
https://youtu.be/kpA5X6eI6fM?si=yvA86N9AHoOAuoBJ
Deriving 3D Rigid Body Physics and implementing it in C/C++ (with intuitions) - https://youtu.be/4r_EvmPKOvY?si=I4PWbyXfvgudb4Cu
Building a Physics Engine with C++ and Simulating Machines - https://youtu.be/TtgS-b191V0?si=i74pfZTFMfHpZG4p
https://youtu.be/lp9cqlngCZQ?si=RGvsJZKpSMFw0WXk
https://wiki.libsdl.org/SDL3/CategoryGPU
https://wiki.libsdl.org/SDL3/CategoryRender
*/


#include <Moss/Moss_stdinc.h>
#include <Moss/Core/Variants/TArray.h>
#include <Moss/Core/Variants/Vector/Vec4.h>
#include <Moss/Core/Variants/Matrix/Mat44.h>
#include <Moss/Core/Variants/Math/Real.h>


#include <Moss/Moss_Platform.h>             // Contains creating window, device input, threads, timer, and time.
#include <Moss/Moss_Audio.h>                  // Contains the code for setting up Audio and drivers.
#include <Moss/Moss_Renderer.h>             // Contains the rendering side of the framework.
//#include <Moss/Moss_GUI.h>
#include <Moss/Moss_Physics.h>              // Contains all the physics side for 2D & 3D
#include <Moss/Moss_Network.h>              // Contains all network libraries needed by enet6.
#include <Moss/Moss_XR.h>
#include <Moss/Moss_Navigation.h>