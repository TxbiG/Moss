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
X
