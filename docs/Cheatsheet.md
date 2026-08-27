# API Cheatsheet

This page lists the main public C-like functions by subsystem. See each subsystem page for behavior notes and backend limitations.

## Platform
```cpp
Moss_Init();
Moss_Terminate();
Moss_Window* Moss_CreateWindow(const Moss_WindowDesc* desc);
void Moss_DestroyWindow(Moss_Window* window);
bool Moss_ShouldWindowClose(Moss_Window* window);
void Moss_PollEvents();
const char* Moss_GetClipboardText();
bool Moss_SetClipboardText(const char* text);
```

## GPU
```cpp
Moss_GPUDevice* Moss_CreateGPUDevice(const Moss_GPUDeviceDesc* desc);
void Moss_DestroyGPUDevice(Moss_GPUDevice* device);
Moss_GPUCommandBuffer* Moss_GPUCommandBufferBegin(Moss_GPUDevice* device, ECommandQueue queue);
void Moss_GPUCommandBufferEnd(Moss_GPUCommandBuffer* cmd);
void Moss_GPUCommandBufferSubmit(Moss_GPUDevice* device, Moss_GPUCommandBuffer* cmd);
Moss_GPUBuffer* Moss_GPUBufferCreate(Moss_GPUDevice* device, const Moss_GPUBufferDesc* desc);
Moss_Texture* Moss_TextureCreate(Moss_GPUDevice* device, const Moss_TextureDesc* desc);
Moss_GPUShader* Moss_CreateGPUShader(Moss_GPUDevice* device, const Moss_GPUShaderCreateInfo* create_info);
Moss_ComputePipelineState* Moss_CreateGPUComputePipeline(Moss_GPUDevice* device, const Moss_GPUComputePipelineCreateInfo* create_info);
Moss_GPUClothSolver* Moss_GPUClothSolverCreate(Moss_GPUDevice* device, const Moss_GPUClothSolverDesc* desc);
```

## Renderer
```cpp
Moss_Renderer* Moss_RendererCreate(const Moss_RendererDesc* desc);
void Moss_RendererDestroy(Moss_Renderer* renderer);
void Moss_RendererBeginFrameEx(Moss_Renderer* renderer, const Moss_RenderFrameDesc* desc);
void Moss_PresentRenderer(Moss_Renderer* renderer);
Moss_Material* Moss_RendererCreateMaterial(Moss_Renderer* renderer, const Moss_MaterialDesc* desc);
Moss_Mesh* Moss_RendererCreateMesh(Moss_Renderer* renderer, const Moss_MeshDesc* desc);
void Moss_RendererDrawMesh(Moss_Renderer* renderer, const Moss_DrawMeshDesc* desc);
Moss_DebugDrawList* Moss_RendererCreateDebugDrawList(Moss_Renderer* renderer, uint32_t capacity);
void Moss_RendererDrawDebugList(Moss_Renderer* renderer, const Moss_DebugDrawList* list);
```

## Audio
```cpp
int Moss_Init_Audio();
void Moss_Terminate_Audio();
void Moss_AudioUpdate(float deltaTime);
Moss_Microphone* Moss_MicrophoneOpen(const Moss_MicrophoneDesc* desc);
void Moss_MicrophoneClose(Moss_Microphone* mic);
bool Moss_MicrophoneStart(Moss_Microphone* mic);
void Moss_MicrophoneStop(Moss_Microphone* mic);
uint32_t Moss_MicrophoneRead(Moss_Microphone* mic, float* frames, uint32_t frame_count);
Moss_MicrophoneLevels Moss_MicrophoneGetLevels(Moss_Microphone* mic);
```

## Components
```cpp
Moss_Terrain* Moss_TerrainCreate(const Moss_TerrainDesc* desc);
Moss_OpenWorldTerrain* Moss_OpenWorldTerrainCreate(const Moss_OpenWorldTerrainDesc* desc);
Moss_ParticleEffect* Moss_ParticleEffectCreate(const Moss_ParticleEmitterDesc* desc);
Moss_Tilemap* Moss_TilemapCreate(const Moss_TilemapDesc* desc);
Moss_Gridmap* Moss_GridmapCreate(const Moss_GridmapDesc* desc);
Moss_Landscape* Moss_LandscapeCreate(const Moss_LandscapeDesc* desc);
Moss_OpenSea* Moss_OpenSeaCreate(const Moss_OpenSeaDesc* desc);
size_t Moss_ComponentSettingsSerialize(Moss_ComponentKind kind, const void* component, char* buffer, size_t buffer_size);
```

## Physics
```cpp
void Moss_PhysicsGetLimitations(Moss_PhysicsLimitations* limitations);
uint32_t Moss_PhysicsDebugDrawBodies(Moss_DebugDrawList* list, const Moss_PhysicsDebugBodyDesc* bodies, uint32_t body_count, uint32_t flags);
uint32_t Moss_PhysicsDebugDrawContacts(Moss_DebugDrawList* list, const Moss_PhysicsDebugContactDesc* contacts, uint32_t contact_count);
size_t Moss_PhysicsShapeSerialize(const Moss_PhysicsShapeSerializeDesc* shape, char* buffer, size_t buffer_size);
size_t Moss_PhysicsSceneSerialize(const Moss_PhysicsSceneSerializeDesc* scene, char* buffer, size_t buffer_size);
```

## Navigation
```cpp
Moss_NavMesh* Moss_NavMeshCreate(const Moss_NavMeshDesc* desc);
bool Moss_NavMeshBakeTriangles(Moss_NavMesh* nav, const Moss_NavMeshBuildDesc* desc);
bool Moss_NavMeshFindPath(Moss_NavMesh* nav, const Moss_NavPathQueryDesc* query, Moss_NavPath* out_path);
Moss_NavCrowd* Moss_NavCrowdCreate(Moss_NavMesh* nav, const Moss_NavCrowdDesc* desc);
void Moss_NavCrowdUpdate(Moss_NavCrowd* crowd, float delta_time);
```

## Network
```cpp
int moss_net_initialize();
void moss_net_deinitialize();
Moss_NetworkHost* Moss_NetworkHostCreate(const Moss_NetworkHostDesc* desc);
Moss_NetworkPeer* Moss_NetworkConnect(Moss_NetworkHost* host, const Moss_NetworkAddress* address, uint32_t channel_count);
bool Moss_NetworkSendReliable(Moss_NetworkPeer* peer, uint8_t channel, const void* data, size_t size);
bool Moss_NetworkSendUnreliable(Moss_NetworkPeer* peer, uint8_t channel, const void* data, size_t size);
int Moss_NetworkPoll(Moss_NetworkHost* host, Moss_NetworkEvent* event, uint32_t timeout_ms);
```

## XR
```cpp
Moss_XRInstance* Moss_XRCreateInstance(const Moss_XRInstanceDesc* desc);
bool Moss_XRBeginSession(Moss_XRSession* session);
bool Moss_XRBeginFrame(Moss_XRSession* session, Moss_XRFrameState* state);
bool Moss_XRLocateViews(Moss_XRSession* session, Moss_XRView* views, uint32_t view_capacity, uint32_t* view_count);
bool Moss_XREndFrame(Moss_XRSession* session, const Moss_XRFrameEndInfo* info);
```

## Resources
```cpp
Moss_AssetManager* Moss_AssetManagerCreate(const Moss_AssetManagerDesc* desc);
void Moss_AssetManagerDestroy(Moss_AssetManager* manager);
Moss_AssetHandle Moss_AssetLoad(Moss_AssetManager* manager, const Moss_AssetLoadDesc* desc);
bool Moss_AssetReload(Moss_AssetManager* manager, Moss_AssetHandle handle);
void Moss_AssetUnload(Moss_AssetManager* manager, Moss_AssetHandle handle);
```
