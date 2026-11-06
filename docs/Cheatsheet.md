# Moss Framework Cheatsheet

> Practical entry points for the public Moss headers. This document is a quick guide, not a replacement for the complete declaration index in [API Reference](API_Reference.md).

## Source scope

Prepared from the header tree supplied for this update. The archive contains **270 `.h` headers**, **19 `.inl` implementation headers**, and **60 `.cpp` files** under the `Moss/` include tree. The header inventory and export-marked declaration listing are in the API Reference. Backend-specific declarations remain conditional on the macros and platform configuration used by the build.

## 1. Include Moss

For a framework-level include:

```cpp
#include <Moss/Moss.h>
```

`Moss.h` includes `Moss_stdinc.h`, selected math/container headers, and the subsystem headers for Platform, Audio, GPU, Renderer, GUI, Physics, Network, XR, and Navigation. It does **not** include every lower-level header in the tree. Include specialised Core, Physics, Renderer, Geometry, Resources, Skeleton, TriangleSplitter, or Variants headers explicitly when you use their APIs.

For smaller translation units, include only what you need:

```cpp
#include <Moss/Moss_Platform.h>
#include <Moss/Moss_Renderer.h>
#include <Moss/Variants/Vector/Vec2.h>
```

Use the include root that contains the `Moss/` directory, and link the Moss targets/libraries configured by your project. The header archive alone does not define the CMake target names or third-party link requirements.

## 2. Main subsystem map

| Area | Main header | What it provides |
|---|---|---|
| Common types and macros | `Moss/Moss_stdinc.h` | Platform/compiler detection, fixed-width aliases, export/calling-convention macros, assertions and logging macros. |
| Platform | `Moss/Moss_Platform.h` | Windows, monitors, input, gamepads, touch/pen, haptics, cameras, filesystem, storage, and file dialogs. |
| GPU | `Moss/Moss_GPU.h` | GPU devices, buffers, textures, shaders, pipelines, command buffers, barriers, swapchains, fences, semaphores, and caches. |
| Renderer | `Moss/Moss_Renderer.h` | Renderer lifecycle, materials, pipelines, frame lifecycle, meshes/models, fonts, sprites, cameras, debug drawing and post-processing. |
| Audio | `Moss/Moss_Audio.h` | Audio subsystem, source loading, channels, effects, streams, spatial listeners, ray-based acoustics and microphone capture. |
| GUI | `Moss/Moss_GUI.h` | Contexts, windows, child windows, containers, drawing widgets, buttons, sliders and text/scalar inputs. |
| Physics | `Moss/Moss_Physics.h` | Physics-facing type declarations, collision layer/filter helpers, contact queries, collision dispatch and geometry helpers. |
| Navigation | `Moss/Moss_Navigation.h` | Recast/Detour navmesh data, queries, corridors, crowds, tile cache and obstacle avoidance. |
| Networking | `Moss/Moss_Network.h` | ENet-style hosts, peers, packets, transports, HTTP requests/server, TLS, WebSocket and WebRTC declarations. |
| XR | `Moss/Moss_XR.h` | XR lifecycle, sessions, frame timing, views, action sets, controller actions, haptics, hand/body/face tracking. |

## 3. Minimal platform event loop

This Platform-only sample uses declarations from `Moss_Platform.h`. It was syntax-checked against the supplied snapshot with Clang 17 in C++20 mode:

```cpp
#include <Moss/Moss_Platform.h>

int main()
{
    Moss_Window* window = Moss_CreateWindow(
        "Moss application",
        1280,
        720,
        nullptr,
        nullptr);

    if (window == nullptr)
        return 1;

    while (!Moss_ShouldWindowClose(window))
        Moss_PollEvents();

    Moss_TerminateWindow(window);
    return 0;
}
```

This demonstrates window ownership and event polling only. It does not initialize or draw with the renderer, choose a graphics backend, load assets, configure a camera, or demonstrate resize handling. The renderer declarations are indexed below, but the standalone renderer header did not pass the isolated syntax check recorded in the API Reference.

### Common renderer calls

```cpp
// Frame lifecycle
Moss_RendererBeginFrame(renderer);
Moss_RendererEndFrame(renderer);

// 2D debug drawing
Vec2 from(20.0f, 20.0f);
Vec2 to(240.0f, 120.0f);
Moss_RendererDrawLine2D(renderer, &from, &to, Color(0.2f, 0.8f, 1.0f, 1.0f), 2.0f);
```

The signatures above are present in `Moss_Renderer.h`; renderer usage is not presented as a compile-verified sample until the header dependency issues noted in the API Reference are resolved. Other renderer entry points include `Moss_RendererDrawMesh`, `Moss_RendererDrawModel`, `Moss_RendererDrawText`, `Moss_RendererDrawSprite`, `Moss_RendererDrawSpriteBatch`, `Moss_RendererDrawDebugList`, `Moss_RendererSetCamera`, and shadow-map operations. The full signatures are indexed by header in the API Reference.

## 4. Common API entry points

### Platform and input

```cpp
Moss_Window* Moss_CreateWindow(const char* title, int width, int height,
                               Moss_Monitor* monitor, Moss_Window* share);
bool Moss_ShouldWindowClose(Moss_Window* window);
void Moss_PollEvents(void);
void Moss_TerminateWindow(Moss_Window* window);

bool Moss_IsKeyPressed(Keyboard key);
bool Moss_IsKeyJustPressed(Keyboard key);
bool Moss_IsMousePressed(Mouse button);
void Moss_GetMousePosition(int* x, int* y);
```

Gamepad handling uses `Moss_GetNumGamepads`, `Moss_OpenGamepad`, `Moss_CloseGamepad`, `Moss_UpdateGamepads`, button/axis queries and optional sensor/haptic functions. Poll events regularly; update gamepad state as appropriate for the application loop.

### GPU device and resource lifecycle

```cpp
Moss_GPUDevice* Moss_CreateGPUDevice(const Moss_GPUDeviceDesc* desc);
void Moss_DestroyGPUDevice(Moss_GPUDevice* device);

Moss_GPUBuffer* Moss_GPUBufferCreate(Moss_GPUDevice* device,
                                     const Moss_GPUBufferDesc* desc);
void Moss_GPUBufferUpload(Moss_GPUDevice* device, Moss_GPUBuffer* buffer,
                          const void* data, uint64_t size, uint64_t offset);
void Moss_GPUBufferDestroy(Moss_GPUDevice* device, Moss_GPUBuffer* buffer);
```

The GPU API also exposes command buffers, compute/copy/render passes, texture uploads, shader and pipeline creation, resource sets, swapchain acquisition/presentation, fences, semaphores, and pipeline caches. Use the `Moss_GPUDeviceDesc` and related descriptor structures rather than assuming a backend-specific default.

### Audio lifecycle and streams

```cpp
int Moss_Init_Audio();
void Moss_AudioUpdate(float deltaTime);
Moss_AudioSource* Moss_AudioLoadWavFile(const char* filename);
void Moss_AudioSourceDestroy(Moss_AudioSource* source);
void Moss_Terminate_Audio();

AudioStream* Moss_AudioStreamCreate();
void Moss_AudioStreamPlay(AudioStream* audiostream);
void Moss_AudioStreamStop(AudioStream* audiostream);
void Moss_AudioStreamRemove(AudioStream* audiostream);
```

Call the audio update function from the application update loop. Destroy streams/sources and shut down the subsystem during teardown. Audio also contains 2D/3D streams, listener activation, effects, channels, ray-traced acoustics and microphone capture.

### Physics

The main public types are `PhysicsSystem`, `Body`, `BodyID`, `BodyInterface`, shape settings and shape classes, collision filters, contact listeners, constraint classes, character controllers, and soft-body/vehicle/ragdoll systems. `PhysicsSystem` is a C++ class defined in `Moss/Physics/PhysicsSystem.h`; include that header (or the specific lower-level physics header) for its full method surface.

The physics header family is built around Jolt-derived C++ interfaces. It is not a single `Moss_PhysicsInit()` C-style API. Set up `PhysicsSystem::Init` with the broadphase interface and collision filters, create bodies through the body interfaces/settings, and advance the simulation through `PhysicsSystem::Update` using the job/temporary-allocator interfaces required by the selected overload. Consult the exact class declarations before wiring this into an engine loop.

### Networking

```cpp
int Moss_Init_Network(void);
void Moss_TerminateNetwork(void);

ENetHost* enet_host_create(ENetAddressType type, const ENetAddress* address,
                           size_t peerCount, size_t channelLimit,
                           uint32_t incomingBandwidth, uint32_t outgoingBandwidth);
int enet_host_service(ENetHost* host, ENetEvent* event, uint32_t timeout);
void enet_host_destroy(ENetHost* host);
```

The same header also declares transport extensions, secure-host creation, asynchronous HTTP, an HTTP server and optional WebSocket/WebRTC facilities. The implementation/status documentation is not equivalent to the existence of a declaration; check the configured backend and implementation before depending on a transport feature.

### XR lifecycle

```cpp
bool Moss_XR_Initialize(Moss_Renderer* renderer);
MossXR_Session* Moss_XR_CreateSession(void);
bool Moss_XR_BeginFrame(MossXR_Session* session);
void Moss_XR_EndFrame(MossXR_Session* session);
void Moss_XR_DestroySession(MossXR_Session* session);
void Moss_XR_Shutdown(void);
```

The frame loop also needs `Moss_XR_HandleEvents`, the predicted display time/views, and action synchronization where used. The supplied XR header itself warns that the implementation has not been tested with a physical headset; validate against the target runtime and device.

### GUI

The GUI API uses `CreateContext`, `DestroyContext`, `GetCurrentContext`, and `SetCurrentContext`. Windows and containers use paired begin/end calls (for example `BeginWindow` / `EndWindow`, `BeginWindowChild` / `EndWindowChild`, and `BeginVContainer` / `EndVContainer`). Widgets include `Text`, `Button`, `Checkbox`, scalar/vector sliders, drag controls, and text/scalar input functions.

## 5. Foundational types and conventions

| Type family | Headers | Common uses |
|---|---|---|
| `Vec2`, `Vec3`, `Vec4` and integer/double/float variants | `Variants/Vector/` | Vector math and spatial data. |
| `Mat22`, `Mat33`, `Mat44`, `DMat44`, `TMatrix` | `Variants/Matrix/` | Matrix and transform operations. |
| `Quat`, `Basis` | `Variants/` | Rotation and orientation. |
| `AABB2`, `AABB3`, `OBB2`, `OBB3`, `Plane`, `Rect` | `Variants/` | Bounds, collision and geometric regions. |
| `Color` | `Variants/Color.h` | RGBA color values; fields are `float` members in the supplied header. |
| `TArray`, `TStaticArray`, `TMap`, `TSet` | `Variants/` | Moss containers. |
| `Result<T>`, `Ref<T>`, `RefConst<T>` | `Core/` | Result/error and reference-counting utilities. |
| `JobSystem`, `JobSystemThreadPool`, `JobSystemWithBarrier` | `Core/` | Scheduling and synchronizing worker jobs. |

The project uses global declarations in many public headers rather than placing every API in a single `Moss` namespace. Do not assume the namespace from the repository name; use the declaration as it appears in the header.

## 6. Important lifecycle rules

- Pair resource creation with the matching destroy/release function. Examples: renderer create/destroy, GPU buffer create/destroy, audio source load/destroy, XR session create/destroy.
- Keep descriptor data, callbacks, filters, listeners, and user-data alive for as long as the subsystem may reference them. Several physics interfaces explicitly require caller-owned objects to outlive `PhysicsSystem`.
- Respect begin/end pairs for renderer frames, GUI windows/containers, render passes, and XR frames.
- Treat returned pointers as nullable unless the declaration and implementation guarantee otherwise; check results from create/load/initialize calls.
- Do not mix resources from different GPU devices or renderers unless the API explicitly supports it.
- The `MOSS_API` marker is an export/import decoration, not a feature-availability guarantee. Preprocessor conditions and the compiled backend determine which declarations are usable.

## 7. Header families outside the main subsystem headers

- `Core/`: allocators, memory, containers/helpers, locks, job systems, profiler, RTTI, references, results, streams, strings and timers.
- `Geometry/`: convex support/hull, clipping, closest-point, GJK/EPA and triangle algorithms.
- `Physics/`: body management, characters, collision, shapes, constraints, ragdolls, soft bodies, hair, vehicles, AABB trees and state recording.
- `Renderer/`: backend-specific renderer resources, fonts, debug renderers, material/model/mesh/texture types, and managers.
- `Resources/`: JSON/configuration, PCK, timers/tweens and object-stream serialization.
- `Skeleton/` and `TriangleSplitter/`: skeleton/animation data and triangle subdivision.
- `Variants/`: vector/matrix/math types, bounds, colors and generic containers.

For exact header paths and an index of export-marked signatures, see [API Reference](API_Reference.md).

## 8. Declaration review notes from this header snapshot

These are observations about the supplied declarations, not speculative corrections:

1. `Moss_AudioStream2DSetPlaybackRate(AudioStream2D*, float)` is declared without the `MOSS_API` marker, while neighbouring stream functions carry it. Check symbol visibility when building/consuming a shared library.
2. `Moss_AudioStream3DSetPosition`, `Moss_AudioStream3DSetVelocity`, and `Moss_AudioStream3DSetMaxDistance` have overloads declared with `AudioStream2D*` and also overloads with `AudioStream3D*`; neither form accepts the position, velocity, or distance value. The signatures therefore do not expose those settings as parameters.
3. `Color.h` describes `Color` as 8-bit-per-component, but its public storage is four `float` members (`r`, `g`, `b`, `a`). The implementation layout should be treated as authoritative until the comment is corrected.
4. `Moss_Navigation.h` primarily exposes class methods and typedefs rather than `MOSS_API`-prefixed free functions. The export-signature appendix therefore cannot be used as its sole API index; consult the class declarations in that header.
5. `Moss.h` includes the main subsystem headers but not every specialised header in the 270-file tree. A symbol being present in the ZIP does not mean it is transitively included by `Moss.h`.
6. A standalone syntax check of several top-level headers fails in this snapshot. See the compiler-check results in the API Reference; do not assume the umbrella include is currently a clean, self-contained include until those issues have been corrected or the project's required pre-include/build configuration is documented.

---

The API Reference is the declaration-level companion to this cheatsheet. Update both when public headers change.
