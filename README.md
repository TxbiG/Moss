[![Build](https://github.com/TxbiG/Moss/actions/workflows/build.yml/badge.svg)](https://github.com/TxbiG/Moss/actions/workflows/build.yml)

# Moss Framework
**Moss Framework** is a modular, high-performance game development framework designed for building **2D and 3D applications** across multiple platforms.

It provides a flexible architecture for rendering, physics, audio, input, networking, and real-time simulation systems.

> **Status:** Active development

## Overview

Moss Framework is designed to be a **low-level, extensible foundation for games and simulation development**.
It aims to give developers control over performance-critical systems while maintaining portability across major platforms.

The framework supports modern graphics APIs and is structured for scalability from small 2D projects to complex 3D applications and multiplayer simulations.

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

Comprehensive documentation is available in the [`docs/`](./docs) directory:

* Architecture overview
* Rendering system design
* Physics system design
* Audio pipeline
* Input system
* Networking model
* Platform backend architecture
* Performance guidelines

- [API cheatsheet](docs/API_Cheatsheet.md)
- [Roadmap](docs/roadmap.md)


## Required CPU features
- x86/x64
  - Minimum: SSE2.
  - Optional compile targets: SSE4.1, SSE4.2, AVX, AVX2, or AVX512.
- ARM32/ARM64 
  - NEON and FP16. 
  - ARM32 can be built without any special CPU extensions.
- Other Architectures
  - RISC-V64/LoongArch64/PowerPC64LE support

---

# Bindings
- [C - CMoss](https://github.com/TxbiG/CMoss)
- [C# - MossSharp](https://github.com/TxbiG/MossSharp)
- [Zig - MossZig](https://github.com/TxbiG/CMoss)
- [Rust - MossRS](https://github.com/TxbiG/CMoss)
- [Java - JavaMoss](https://github.com/TxbiG/CMoss)
- [JavaScript - MossScript](https://github.com/TxbiG/CMoss)
- [Lua - LuaMoss](https://github.com/TxbiG/CMoss)
- [Python - PyMoss](https://github.com/TxbiG/CMoss)

## Compiling
- CMake 3.20 or newer.
- C++ 17.

## Design Philosophy
- Modularity: systems are separable and replaceable.
- Performance: real-time systems stay explicit and low overhead.
- Portability: public APIs avoid backend ownership leaks where practical.
- Control: callers can choose low-level GPU/platform paths or higher-level renderer/component helpers.
- Incremental stability: new public APIs favor C-like handles and descriptors while older C++ drafts are migrated.

## Current Focus
- Make CMake configuration and install/export reliable.
- Finish backend parity across renderer/GPU, audio, platform, XR, and GUI.
- Continue shrinking large public headers behind simpler Moss-level APIs.
- Expand tests and examples for every stable public subsystem.

## Repository Structure
```text
.
|-- docs/         Documentation and developer guides
|-- examples/     Example projects
|-- external/     Third-party dependencies
|-- include/      Public framework headers
|-- src/          Framework source code
|-- performance/  Benchmarks and profiling tools
|-- tests/        Unit and smoke tests
```

## License
The project is distributed under the [MIT license](LICENSE).
