[![Build](https://github.com/TxbiG/Moss/actions/workflows/build.yml/badge.svg)](https://github.com/TxbiG/Moss/actions/workflows/build.yml)
# Moss Framework
**Moss Framework** is a modular, high-performance game development framework designed for building **2D and 3D applications** across multiple platforms.

It provides a flexible architecture for rendering, physics, audio, input, networking, and real-time simulation systems.

> **Status:** Active development

## Overview

Moss Framework is designed to be a **low-level, extensible foundation for game and simulation development**.
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

## Design Philosophy

Moss Framework is built around:

* **Modularity** — systems are independent and replaceable
* **Performance** — designed for real-time applications
* **Portability** — multiple platforms and architectures supported
* **Low-level control** — minimal abstraction overhead
* **Scalability** — from simple 2D games to complex 3D engines

---

## Roadmap Highlights

* Fully stable rendering abstraction layer
* Expanded Vulkan and DirectX 12 backend support
* Physics engine stabilization
* Multiplayer networking stack
* Editor tooling (future)
* Asset pipeline system
* ECS architecture expansion

---

## Compiling
- C++ 17.

## Repository Structure

```text id="moss1"
.
├─ docs/           # Documentation and developer guides
├─ examples/       # Example projects
├─ external/       # Third-party dependencies
├─ include/        # Public framework headers
├─ src/            # Core framework source code
├─ performance/    # Benchmarks and profiling tools
└─ test/           # Unit and integration tests
```

## License
The project is distributed under the [MIT license](LICENSE).
