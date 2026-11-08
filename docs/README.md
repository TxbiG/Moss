# Moss Framework Documentation

## Start Here
- [Introduction](#introduction) — What is MossFramework?
- [Architecture Overview](#architecture-overview) — Key systems/components.
- [API Cheatsheet](API_Cheatsheet.md)
- [Roadmap](roadmap.md)
- [Thirdparties](/docs/Thirdparties.md)
- [Licensing](LICENSE)


## Core Systems
- [Platform](Platform.md): windows, monitors, input, gamepads, haptics, dialogs, clipboard, cursors, high DPI, video capture.
- [GPU](Renderer.md): device, swapchain, command buffers, shaders, buffers, textures, pipelines, barriers, debug markers.
- [Renderer](Renderer.md): renderer lifecycle, materials, meshes, models, sprites, cameras, debug draw, post-processing, shadows.
- [Audio](Audio.md): speakers, microphones, streams, channels, DSP, spatial audio.
- [GUI](UI.md): draw data, widgets, layout, input focus, styling.
- [Physics](Physics.md): physics utilities, debug draw, serialization, known limitations.
- [Navigation](Navigation.md): nav meshes, 2D navigation, path queries, crowds, obstacles, serialization, debug draw.
- [Network](Network.md): ENet-style host/client APIs, packets, channels, reliability, polling, replication helpers.
- [XR](XR.md): OpenXR instances, sessions, swapchains, frame loop, actions, haptics, extension-gated features.
- [Resources](Resources.md): JSON, virtual paths, PCK/PAK, hot reload.
- [Variants](Variants.md): common scalar, vector, matrix, color, rectangle, container, and math types.

## Tutorials
- [2D game lessons](2DGame/README.md)
- [3D game lessons](3DGame/README.md)


## Public API Header Map
- `include/Moss/Moss_Platform.h`
- `include/Moss/Moss_GPU.h`
- `include/Moss/Moss_Renderer.h`
- `include/Moss/Moss_Audio.h`
- `include/Moss/Moss_GUI.h`
- `include/Moss/Moss_Physics.h`
- `include/Moss/Moss_Navigation.h`
- `include/Moss/Moss_Network.h`
- `include/Moss/Moss_XR.h`



# Introduction
X

## Documentation Rules
- Prefer relative links between docs pages.
- Use real anchors only; avoid empty `#` links.
- Keep examples short and compile-oriented.
- Document backend status explicitly as supported, partial, experimental, or compatibility-only.


# Architecture Overview
X


## Design Philosophy
Moss Framework is built around:

* **Modularity** — systems are independent and replaceable
* **Performance** — designed for real-time applications
* **Portability** — multiple platforms and architectures supported
* **Low-level control** — minimal abstraction overhead
* **Scalability** — from simple 2D games to complex 3D engines

---

# Roadmap
# Moss Framework – Development Roadmap

## Phase 1 – Core Foundation (v1.0–v1.1)

- [ ] Finalise core framework architecture.
- [ ] Stabilise memory management.
- [ ] Improve resource management.
- [ ] Improve application lifecycle management.
- [ ] Improve threading and job systems.
- [ ] Improve event handling.
- [ ] Improve logging and diagnostics.
- [ ] Improve error handling.
- [ ] Finalise platform abstraction.
- [ ] Establish automated testing.
- [ ] Establish continuous integration (CI/CD).
- [ ] Complete API documentation.

## Phase 2 – 2D Development (v1.2)

- [ ] Improve 2D rendering.
- [ ] Implement sprite rendering and batching.
- [ ] Implement sprite animation system.
- [ ] Implement tilemap system.
- [ ] Optimise tilemap rendering.
- [ ] Implement 2D camera system.
- [ ] Implement parallax scrolling.
- [ ] Improve 2D collision detection.
- [ ] Integrate 2D physics.
- [ ] Implement particle system.
- [ ] Improve text rendering.
- [ ] Improve UI rendering.
- [ ] Improve 2D audio integration.
- [ ] Create 2D development examples.

## Phase 3 – Asset and Resource Management (v1.3)

- [ ] Implement centralised asset management.
- [ ] Implement texture loading and management.
- [ ] Implement audio resource management.
- [ ] Implement model and mesh importing.
- [ ] Implement material management.
- [ ] Implement asset caching.
- [ ] Implement asset dependency tracking.
- [ ] Implement asset packaging.
- [ ] Implement hot reloading.
- [ ] Implement asset metadata.
- [ ] Develop asset import pipeline.
- [ ] Optimise resource management.

## Phase 4 – Developer Tools (v1.4)

- [ ] Implement performance profiler.
- [ ] Implement memory profiler.
- [ ] Implement rendering debugger.
- [ ] Implement physics debugger.
- [ ] Implement collision visualisation.
- [ ] Implement logging console.
- [ ] Implement resource inspector.
- [ ] Improve application diagnostics.
- [ ] Implement crash reporting.
- [ ] Implement automated testing utilities.
- [ ] Implement benchmarking tools.

## Phase 5 – 3D Development (v1.5)

- [ ] Improve 3D rendering.
- [ ] Implement mesh management.
- [ ] Implement material system.
- [ ] Implement lighting system.
- [ ] Implement shadow rendering.
- [ ] Implement 3D camera system.
- [ ] Improve transform system.
- [ ] Implement skeletal animation.
- [ ] Integrate 3D physics.
- [ ] Implement environment rendering.
- [ ] Implement post-processing.
- [ ] Improve particle systems.
- [ ] Create 3D development examples.

## Phase 6 – Platform Expansion (v1.6)

- [ ] Improve Windows support.
- [ ] Improve Linux support.
- [ ] Add macOS support.
- [ ] Add Android support.
- [ ] Add iOS support.
- [ ] Improve WebAssembly support.
- [ ] Implement platform-specific input handling.
- [ ] Improve platform-specific window management.
- [ ] Implement cross-platform build presets.
- [ ] Establish platform compatibility testing.
- [ ] Establish long-term support releases.