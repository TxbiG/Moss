# Renderer
## Overview
The Moss Renderer supports a wide range of graphics APIs such as ```OpenGL```, ```Vulkan```, ```Metal```, and ```DirectX12```. The Renderer is written in a C like style but is completely made in C++.

Renderer supports ```Windows```, ```Linux```, ```MacOS```, ```FreeBSD```, ```IOS``` and ```Android```.

> [!NOTE]  
> When working with Vulkan, ensure your graphics card supports it and you've installed [Vulkan SDK](https://vulkan.lunarg.com/sdk/home)

> [!TIP]
> For noise textures I recommend using this website [opengameart.org/content/700-noise-textures](https://opengameart.org/content/700-noise-textures)
> 
> Moss does support making noise texureing using FastNoiseLite


## Choosing a Renderer
- ```Compatibility```: Web, Older / low-end. Desktop, Consoles, Mobile.
- ```Mobile```: Newer / high-end Mobile devices.
- ```Forward+```: Newer / high-end Desktop, Consoles.

## Comparison
| Features | Compatibility | Mobile | Forward+ |
| ------------- | ------------- | ------------- | ------------- |
| XR Support |✔️ | ✔️ | ✔️ |
| Web Support | ✔️ | ❌ | ❌ |
| Driver | OpenGL | Vulkan, Metal | Vulkan, DirectX 12, Metal |
| 2D Games |✔️ | ✔️ | ✔️ |
| 3D Games |✔️ | ✔️ | ✔️ |

### Lighting and shadows
| Features | Compatibility | Mobile | Forward+ |
| ------------- | ------------- | ------------- | ------------- |
| Approach | Forward | Forward | Forward+ | Forward or Forward+ |
| Content Cell | Content Cell | Content Cell | Content Cell | Content Cell |

### Global Illumination
| Features | Compatibility | Mobile | Forward+ |
| ------------- | ------------- | ------------- | ------------- |
| Content Cell | Content Cell | Content Cell | Content Cell |
| Content Cell | Content Cell | Content Cell | Content Cell |

### Environment and post-processing
| Features | Compatibility | Mobile | Forward+ |
| ------------- | ------------- | ------------- | ------------- |
| Fog (Depth and Height) | ✔️ | ✔️ | ✔️ |
| Volumetric Fog | ✔️ (Local Fog Volumes)| ✔️ | ✔️ |
| Tonemapping | ✔️ | ✔️ | ✔️ |
| Screen-Space Reflections | ✔️ | ✔️ | ✔️ |
| Screen-Space Ambient Occlusion (SSAO) | ✔️ | ✔️ | ✔️ |
| Glow | ✔️ | ✔️ | ✔️ |
| Adjustments | ✔️ | ✔️ | ✔️ |

### Antialiasing
| Features | Compatibility | Mobile | Forward+ |
| ------------- | ------------- | ------------- | ------------- |
| MSAA 2D | ❌ | ✔️ | ✔️ | ✔️ |
| MSAA 3D | ✔️ | ✔️ | ✔️ |
| TAA | ❌ | ✔️ | ✔️ | ❌ |
| FXAA | ❌ | ✔️ | ✔️ | ✔️ |
| SMAA | ❌ | ✔️ | ✔️ | ✔️ |
| SSAA | ✔️ | ✔️ | ✔️ | ✔️ |


### Matrix views
| Types | Description |
| ------------- | ------------- |
| u_proj | Projection matrix (orthographic / perspective) |
| u_local | Object/model transform (local space to world space) |
| u_world | World/scene transform (optional, often combined with model) |
| u_view | View/camera transform (world space → camera space) |
| u_mvp | Combined matrix: Projection X View X Model |
| u_normal | Normal matrix (inverse-transpose of model or model-view) |


## Defines
```cpp
// Provided by Moss
#define MOSS_GRAPHICS_OPENGL
#define MOSS_GRAPHICS_OPENGLES
#define MOSS_GRAPHICS_VULKAN
#define MOSS_GRAPHICS_DIRECTX
#define MOSS_GRAPHICS_METAL
```

## Renderer Life-cycles
[Add Text here]
```cpp
Moss_Renderer* Moss_CreateRenderer(Moss_Window& window);
```
```cpp
void Moss_RendererBeginFrame(Moss_Renderer& renderer);
```
```cpp
void Moss_PresentRenderer(Moss_Renderer& renderer);
```
```cpp
void Moss_TerminateRenderer(Moss_Renderer& renderer);
```

// Add Clear colour

Exmaple
```cpp
...

renderer = Moss_CreateRenderer(window);

while (!Moss_ShouldWindowClose(window))
    {

        ...

        Moss_RendererBeginFrame(renderer);

        ...

        Moss_PresentRenderer(renderer);

        ...
    };


Moss_TerminateRenderer(renderer);

...
```

## Camera 2D & 3D
[Add Text here]
```cpp
```
## Viewport & SubViewports
[Add Text here]
```cpp
```
## Skybox
This is used for 3D
[Add Text here]
```cpp
```
## Textures
[Add Text here]
```cpp
```

## Shaders
[Add Text here]
```cpp
```

## Mesh
[Add Text here]
```cpp
```
## Particle Effects

### Particle Effect 2D
```cpp
```

#### Example
```cpp
ParticleSystem2 g_particleSystem2D;

uint32 explosionID = g_particleSystem2D.CreateParticleEffect(explosionTransform);
g_particleSystem2D.CreateParticleEmitter(); // attaches emitters to effect


Transform2 t;
t.position = pos;

uint32 effectID = g_particleSystem2D.CreateParticleEffect(t);
ParticleEmitterID emitter = g_particleSystem2D.CreateParticleEmitter();

// Configure emitter
ParticleEmitter2 explosionEmitter;
explosionEmitter.m_transform = t.ToMat44();
explosionEmitter.amount = 100;      // 100 sparks
explosionEmitter.spawnTimer = 0;
g_particleSystem2D.m_activeEmitterBuckets[emitter] = ParticleEmitterBucket2(explosionEmitter);


g_particleSystem2D.Update(currentTime, deltaTime);

for (auto& [id, emitter] : g_particleSystem2D.m_activeEmitterBuckets) {
    emitter.draw(); // 2D = no view
}

```
### Particle Effect 3D
```cpp
```

#### Example
```cpp
ParticleSystem3 g_particleSystem3D;


Transform2 t;
t.position = pos;

uint32 effectID = g_particleSystem3D.CreateParticleEffect(t);
ParticleEmitterID emitter = g_particleSystem3D.CreateParticleEmitter();

// Configure emitter
ParticleEmitter2 explosionEmitter;
explosionEmitter.m_transform = t.ToMat44();
explosionEmitter.amount = 100;      // 100 sparks
explosionEmitter.spawnTimer = 0;
g_particleSystem2D.m_activeEmitterBuckets[emitter] = ParticleEmitterBucket2(explosionEmitter);

g_particleSystem3D.Update(currentTime, deltaTime);

for (auto& [id, emitter] : g_particleSystem3D.m_activeEmitterBuckets) {
    emitter.draw();
}

```
## Post-processing
[Add Text here]
```cpp
```

## OpenGL and OpenGLES Specific
[Add Text here]

## Vulkan Spesific
[Add Text here]

## DirectX 12 Spesific
[Add Text here]

## Metal Spesific
[Add Text here]
