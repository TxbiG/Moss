# Moss Framework [Development]
Moss Framework is a lightweight, modular, and high-performance game development framework designed for creating both 2D and 3D applications across multiple platforms.


## Features
- Physics - 2D and 3D simulation and collision.
- Audio System - integrated audio playback and spatial sound support.
- Rendering APIs - Supports OpenGL, OpenGL-ES, Vulkan, Metal, DirectX 12.
- Cross-Platform - Windows, Linux, MacOS, IOS, Android.
- Input handling and Haptic Feedback support.
- Integrated OpenXR support for VR/AR/MR devices
- Multiplayer support.
- Navigation support for 2D and 3D.

## Documentation
Comprehensive documentation is available in the [Docs](https://github.com/TxbiG/MossFramework/tree/main/docs#docs)

## Required CPU features
- x86/x64
  - Minimum: SSE2.
  - Optional compile targets: SSE4.1, SSE4.2, AVX, AVX2, or AVX512.
- ARM32/ARM64 
  - NEON and FP16. 
  - ARM32 can be built without any special CPU extensions.
- Other Architectures
  - RISC-V64/LoongArch64/PowerPC64LE support

## Compiling
- C++ 17.

## Folder structure
- ```docs``` - Project documentation and developer guides.
- ```examples``` - Example projects demonstrating framework usage.
- ```external``` - Third-party libraries and source code.
- ```include``` - 	Core header code of the framework.
- ```src``` - 	Core source code of the framework.
- ```performance``` - Benchmarks and profiling tools.
- ```test``` - Unit and integration test.

## License
The project is distributed under the [MIT license](LICENSE).