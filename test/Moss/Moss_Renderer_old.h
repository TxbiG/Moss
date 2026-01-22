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


/*!
 * @file Moss_Renderer.h
 * @brief Core rendering interface for the Moss Framework.
 *
 * The renderer module provides the abstraction layer for all 2D and 3D graphics
 * operations. It serves as a unified, API-agnostic rendering backend capable of
 * targeting multiple graphics APIs (Vulkan, OpenGL, OpenGL ES, Metal, and DirectX 12).
 * 
 * ### Renderer Types
 * - Forward+       - Vulkan, DirectX 12, Metal (Modern Gen PC and consoles)
 * - Mobile         - Vulkan, OpenGL ES v2.0 & v3.0, Metal (IOS & Android)
 * - Compatibility  - OpenGL v3.3 (Older Gen PC and consoles)
 * 
 * ### Anti Aliasing
 * - Forward+       - MSAA, SSAA, FXAA, SMAA, SSRL, TAA, FSR2
 * - Mobile         - MSAA, SSAA, FXAA, SMAA, SSRL
 * - Compatibility  - MSAA, SSAA
 * 
 * ### Macros
 * - MOSS_GRAPHICS_OPENGL 	- OpenGL v3.3
 * - MOSS_GRAPHICS_OPENGLES - OpenGL ES v2.0 or v3.0
 * - MOSS_GRAPHICS_VULKAN 	- Vulkan
 * - MOSS_GRAPHICS_DIRECTX 	- DirectX 12
 * - MOSS_GRAPHICS_METAL 	- Metal
 * - MOSS_RENDERER_FORWARD 	- Forward+ Renderer
 * - MOSS_RENDERER_MOBILE 	- Mobile Renderer
 * - MOSS_RENDERER_COMPATI 	- Compatibility Renderer
 * 
 * ### Primary Responsibilities:
 * - **2D and 3D Rendering** — Handles scene rendering for both worlds and UI layers.
 * - **Debug Visualization** — Supports drawing collision bounds, gizmos, and profiling overlays.
 * - **Lighting & Shadows** — Manages dynamic and baked lighting, real-time shadow mapping, and reflections.
 * - **Post-Processing Pipeline** — Provides a customizable chain for effects such as bloom, tone mapping, FXAA/TAA, and color grading.
 * - **Reflection Probes & Global Illumination** — Enables environment-based lighting and reflections.
 * - **Compositor (Planned)** — Future abstraction for multi-pass rendering and post-fx composition.
 * - **Graphics API Abstraction Layers** — Full separation of renderer logic from API-specific backends.
 * - **Level of Detail (LOD) System** — Mesh simplification and distance-based swapping.
 *
 * ### Planned Features / TODO:
 * - **Multithreaded Render Submission** — Asynchronous job-based rendering pipeline (future optimization).
 * - Reflections
 * - Record capture of screen. (Record/Captrue gameplay)
 * - Display VideoCapture of videos
 *
 * The renderer acts as the core visual system of Moss Renderering and debugging utils, directly integrated with physics visualisation and UI rendering layers and navigation.
 */


// Add Font, Noise Texture
// Smol-V is used to "bake" the shaders will need to make a BAKE define
// Create a tool/functions for baking/pre-processing assets from source formats and shader compression

#ifndef MOSS_RENDERER_H
#define MOSS_RENDERER_H


#ifdef MOSS_GRAPHICS_VULKAN
#include <vulkan/vulkan.h>
#endif // MOSS_GRAPHICS_VULKAN

#include <Moss/Moss_stdinc.h>
#include <Moss/Moss_Platform.h>

#define MOSS_API		// < Delete this

enum class AntiAliasing : uint8_t {
    None = 0,
    // Spatial (geometry / raster)
    MSAA_X2,
    MSAA_X4,
    MSAA_X8,
    SSAA,
    // Post-process
    FXAA,
    SMAA,
    // Temporal
    TAA,
    // Upscale / Temporal AA
    FSR2
};

/// Describes the input layout of the vertex shader
enum class EInputDescription {
	Position,						///< 3 float position
	Color,							///< 4 uint8 color
	Normal,							///< 3 float normal
	TexCoord,						///< 2 float texture coordinate
	InstanceColor,					///< 4 uint8 per instance color
	InstanceTransform,				///< 4x4 float per instance transform
	InstanceInvTransform,			///< 4x4 float per instance inverse transform
};

// Global Illumination use the type Static, Rigid, Dynamic (Should be placed in the 2D and 3D Light managers)
struct GlobalIllumination;
class ReflectionProbe;

struct Bone 				{ int index; std::string Name; Bone Parent; };
struct SkeletalWeight 	{ int index; float Bias; Float3 Position };
struct SkeletalVertex 	{ TArray<SkeletalWeight> Weights; };
struct SkeletalPose 		{ TArray<Mat44> transformations; };
struct Skeleton 			{ TArray<Bone> Bones; SkeletalPose Identity; SkeletalPose BindPose; SkeletalPose InverseBindPose; };

class [[nodiscard]] Plane2D {
public:
	Plane2D() = default;
	explicit Plane2D();
	Plane2D();

private:
	Vec2 normal;
	float offset;
};

class [[nodiscard]] Frustum2D {
public:
	MOSS_OVERRIDE_NEW_DELETE

    Frustum2D() = default;
    Frustum2D(const Vec2& bottomLeft, const Vec2& topRight);

    void SetBounds(const Vec2& bottomLeft, const Vec2& topRight);
    bool Overlaps(const AABB2& box) const;
    const AABB2& GetBounds() const;

private:
    AABB2 m_bounds; // 2D camera rect
};


class [[nodiscard]] Plane {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Constructor
	Plane() = default;
	explicit Plane(Vec4Arg inNormalAndConstant);
	Plane(Vec3Arg inNormal, float inConstant);

	// Create from point and normal
	static Plane sFromPointAndNormal(Vec3Arg inPoint, Vec3Arg inNormal);

	// Create from point and normal, double precision version that more accurately calculates the plane constant
	static Plane sFromPointAndNormal(DVec3Arg inPoint, Vec3Arg inNormal);

	// Create from 3 counter clockwise points
	static Plane sFromPointsCCW(Vec3Arg inV1, Vec3Arg inV2, Vec3Arg inV3);

	// Properties
	Vec3 GetNormal() const;
	void SetNormal(Vec3Arg inNormal);
	float GetConstant() const;
	void SetConstant(float inConstant);

	// Offset the plane (positive value means move it in the direction of the plane normal)
	Plane Offset(float inDistance) const;

	// Transform the plane by a matrix
	inline Plane GetTransformed(Mat44Arg inTransform) const;

	// Scale the plane, can handle non-uniform and negative scaling
	inline Plane Scaled(Vec3Arg inScale) const;

	// Distance point to plane
	float SignedDistance(Vec3Arg inPoint) const;

	// Project inPoint onto the plane
	Vec3 ProjectPointOnPlane(Vec3Arg inPoint) const;

	// Returns intersection point between 3 planes
	static bool	sIntersectPlanes(const Plane &inP1, const Plane &inP2, const Plane &inP3, Vec3 &outPoint);

private:
	Vec4			mNormalAndConstant;													// XYZ = normal, W = constant, plane: x . normal + constant = 0
};

/// A camera frustum containing of 6 planes (left, right, top, bottom, near, far) pointing inwards
class [[nodiscard]] Frustum3D {
public:
	/// Empty constructor
	Frustum3D() = default;

	/// Construct frustum from position, forward, up, field of view x and y and near plane.
	/// Note that inUp does not need to be perpendicular to inForward but cannot be collinear.
	inline Frustum3D(Vec3 inPosition, Vec3 inForward, Vec3 inUp, float inFOVX, float inFOVY, float inNear);

	/// Test if frustum overlaps with axis aligned box. Note that this is a conservative estimate and can return true if the
	/// frustum doesn't actually overlap with the box. This is because we only test the plane axis as separating axis
	/// and skip checking the cross products of the edges of the frustum
	inline bool Overlaps(const AABox &inBox) const;

private:
	static constexpr int PLANE_COUNT = 5; // or 6 if you include the far plane
	Plane			m_Planes[PLANE_COUNT]; // Planes that form the frustum
};

class Camera2D {
public:
	Camera2D();


	const Frustum2D& GetFrustum() const { return m_frustum; }

    Float2 position;
	Float2 offset;
    float zoom = 1.0f;
    float rotation = 0.0f;

	float rotation_smoothing_speed;
	float rotation_smoothing_enabled;
	float position_smoothing_speed;
	bool position_smoothing_enabled;

	int limit_top, limit_right, limit_left, limit_bottom;
	float drag_left_margin, drag_right_margin, drag_top_margin, drag_bottom_margin;

	bool drag_horizontal_enabled, drag_vertical_enabled;
	float drag_horizontal_offset, drag_vertical_offset;

	void update(float width, float height);
    Mat44 getViewProjectionMatrix() const;
	void draw();

	LightMask light_mask;
	VisibleLayer visible_layer;

#ifdef MOSS_DEBUG
	bool editor_draw_drag_margin, editor_draw_limits, editor_draw_screen;
#endif  // MOSS_DEBUG
private:
    Mat44 u_viewProj;
	Frustum2D m_frustum;
};

class Camera3D {
public:
	Camera3D();

	const Frustum3D& GetFrustum() const { return m_frustum; }

    Float3 position;
	Float3 up = Float3(0.0f, 1.0f, 0.0f);    // Y is up
    Float3 target;
	Float3 Orientation = Float3(0.0f, 0.0f, -1.0f);

    float speed = 0.1f;
    float sensitivity = 100;

    float fov = 45.0f;          // Field of view (degrees)
    float aspectRatio;          // avoid divide-by-zero
    float nearPlane = 0.1f;
    float farPlane = 1000.0f;

    void update(float width, float height);
    Mat44 getViewProjectionMatrix() const;
	void draw();
    CullMask cullMask;
	LightMask light_mask;
	//int width; int height; Stores the width and height of the window
	// bool firstClick = true; Prevents the camera from jumping around when first clicking left click
private:
    Mat44 u_viewProj;
	Frustum3D m_frustum;
};

/*! @brief Creates a Renderer. 
	@attention Renderer Extentions: SurfaceManager, MeshManager, ModelManager, SurfaceInstanceManager, MeshInstanceManager, ModelInstanceManager, LightManager2D, LightManager3D, DecalManager, SkeletonManager.
	 */ 
class [[nodiscard]] MOSS_API Moss_Renderer {
public:
	~RendererGL() override = default;

	void CreateRenderer(Moss_Window* inWindow) override;
	bool BeginFrame() override;
	void EndFrame() override;

	/*! @brief */
	Camera2D* CreateCamera2D(Float2 position = Float2(0.0f, 0.0f), float zoom = 1.0f, float rotation = 0.0f, CullMask cullMask = CullMask::Layer_1);
	/*! @brief */
	Camera3D* CreateCamera3D(Vec3 position = Vec3(0.0f,0.0f,0.0f), Vec3 target, float speed = 0.1f, float sensitivity = 100, CullMask cullMask = CullMask::Layer_1);

	void SetProjectionMode() override;
	void SetOrthoMode() override;

	// Settings
	SetAntiAliasing();

	Ref<Texture> CreateTexture(const Surface* inSurface);

	// Shaders
	Ref<VertexShader> 		CreateVertexShader(const char* inName) override;
	Ref<PixelShader> 		CreatePixelShader(const char* inName) override;
	Ref<ComputeShader>		CreateComputeShader(const char *inName) override;

	// Shadow
	void					EndShadowPass() override;
	Texture*				GetShadowMap() const override { return mShadowMap.GetPtr(); }

	// PostProcessing
	Ref<PostProcess> 		CreatePostProcessing();
    void 					AddPostProcessing(Ref<PostProcess> pass);
    void 					RemovePostProcessing(Ref<PostProcess> pass);

	// Pipeline
	unique_ptr<PipelineState> CreatePipelineState(const VertexShader* inVertexShader, const PipelineState::EInputDescription* inInputDescription, uint inInputDescriptionCount, 
		const PixelShader* inPixelShader, PipelineState::EDrawPass inDrawPass, PipelineState::EFillMode inFillMode, PipelineState::ETopology inTopology, PipelineState::EDepthTest inDepthTest, 
		PipelineState::EBlendMode inBlendMode, PipelineState::ECullMode inCullMode) override;
	
	unique_ptr<PipelineState> CreatePipelineStateEx(const VertexShader* inVertexShader, const PipelineState::EInputDescription* inInputDescription, uint inInputDescriptionCount, 
		const PixelShader* inPixelShader, PipelineState::EDrawPass inDrawPass, PipelineState::EFillMode inFillMode, PipelineState::ETopology inTopology, PipelineState::EDepthTest inDepthTest, 
		PipelineState::EBlendMode inBlendMode, PipelineState::ECullMode inCullMode, ComputeShader* computeShader) override;

	// Pipeline Extentions
	unique_ptr<PipelineState> CreatePipelineStateEx2(const VertexShader* inVertexShader, const PipelineState::EInputDescription* inInputDescription, uint inInputDescriptionCount, 
		const PixelShader* inPixelShader, PipelineState::EDrawPass inDrawPass, PipelineState::EFillMode inFillMode, PipelineState::ETopology inTopology, PipelineState::EDepthTest inDepthTest, 
		PipelineState::EBlendMode inBlendMode, PipelineState::ECullMode inCullMode, 
		GeometryShader* gemoShader, TesselationControlShader* TessControl, TesselationEvaluationShader* TessEval) override;
	RenderPrimitive* CreateRenderPrimitive(PipelineState::ETopology inType) override;
	RenderInstances* CreateRenderInstances() override;
	Texture* GetShadowMap() const override;

// Renderer Spesific Graphics Renderer functions
#if defined(MOSS_GRAPHICS_OPENGL)
#ifdef MOSS_DEBUG
#endif  // MOSS_DEBUG
#elif defined(MOSS_GRAPHICS_VULKAN)
	VkDevice						GetDevice() const												{ return mDevice; }
	VkDescriptorPool				GetDescriptorPool() const										{ return mDescriptorPool; }
	VkDescriptorSetLayout			GetDescriptorSetLayoutTexture() const							{ return mDescriptorSetLayoutTexture; }
	VkSampler						GetTextureSamplerRepeat() const									{ return mTextureSamplerRepeat; }
	VkSampler						GetTextureSamplerShadow() const									{ return mTextureSamplerShadow; }
	VkRenderPass					GetRenderPassShadow() const										{ return mRenderPassShadow; }
	VkRenderPass					GetRenderPass() const											{ return mRenderPass; }
	VkPipelineLayout				GetPipelineLayout() const										{ return mPipelineLayout; }
	VkCommandBuffer					GetCommandBuffer()												{ MOSS_ASSERT(mInFrame); return mCommandBuffers[mFrameIndex]; }
	VkCommandBuffer					StartTempCommandBuffer();
	void							EndTempCommandBuffer(VkCommandBuffer inCommandBuffer);
	void							AllocateMemory(VkDeviceSize inSize, uint32 inMemoryTypeBits, VkMemoryPropertyFlags inProperties, VkDeviceMemory &outMemory);
	void							FreeMemory(VkDeviceMemory inMemory, VkDeviceSize inSize);
	void							CreateBuffer(VkDeviceSize inSize, VkBufferUsageFlags inUsage, VkMemoryPropertyFlags inProperties, BufferVK &outBuffer);
	void							CopyBuffer(VkBuffer inSrc, VkBuffer inDst, VkDeviceSize inSize);
	void							CreateDeviceLocalBuffer(const void *inData, VkDeviceSize inSize, VkBufferUsageFlags inUsage, BufferVK &outBuffer);
	void							FreeBuffer(BufferVK &ioBuffer);
	unique_ptr<ConstantBufferVK>	CreateConstantBuffer(VkDeviceSize inBufferSize);
	void							CreateImage(uint32 inWidth, uint32 inHeight, VkFormat inFormat, VkImageTiling inTiling, VkImageUsageFlags inUsage, VkMemoryPropertyFlags inProperties, VkImage &outImage, VkDeviceMemory &outMemory);
	void							DestroyImage(VkImage inImage, VkDeviceMemory inMemory);
	VkImageView						CreateImageView(VkImage inImage, VkFormat inFormat, VkImageAspectFlags inAspectFlags);
	VkFormat						FindDepthFormat();
#ifdef MOSS_DEBUG
#endif  // MOSS_DEBUG
#elif defined(MOSS_GRAPHICS_METAL)
	MTKView*						GetView() const													{ return mView; }
	id<MTLDevice>					GetDevice() const												{ return mView.device; }
	id<MTLRenderCommandEncoder>		GetRenderEncoder() const										{ return mRenderEncoder; }
#ifdef MOSS_DEBUG
#endif  // MOSS_DEBUG
#elif defined(MOSS_GRAPHICS_DIRECTX)
	ID3D12Device*					GetDevice()							{ return mDevice.Get(); }
	ID3D12RootSignature*			GetRootSignature()					{ return mRootSignature.Get(); }
	ID3D12GraphicsCommandList*		GetCommandList()					{ JPH_ASSERT(mInFrame); return mCommandList.Get(); }
	CommandQueueDX12&				GetUploadQueue()					{ return mUploadQueue; }
	DescriptorHeapDX12&				GetDSVHeap()						{ return mDSVHeap; }
	DescriptorHeapDX12&				GetSRVHeap()						{ return mSRVHeap; }
#ifdef MOSS_DEBUG
#endif  // MOSS_DEBUG
#endif
private:
	Ref<TextureGL> mShadowMap; // Optional: For shadow rendering

	// Cameras
	std::vector<Camera2D*> m_camera2d;
	std::vector<Camera3D*> m_camera3d;
	std::vector<SubViewport*> m_subViewports;

	// Shaders
	std::vector<VertexShader*> m_VertexShader;
	std::vector<PixelShader*> m_PixelShader;
	std::vector<ComputeShader*> m_ComputeShader;


#if defined(MOSS_GRAPHICS_OPENGL)
#ifdef MOSS_DEBUG
#endif  // MOSS_DEBUG
#elif defined(MOSS_GRAPHICS_VULKAN)
	uint32							FindMemoryType(uint32 inTypeFilter, VkMemoryPropertyFlags inProperties);
	void							FreeBufferInternal(BufferVK &ioBuffer);
	VkSurfaceFormatKHR				SelectFormat(VkPhysicalDevice inDevice);
	void							CreateSwapChain(VkPhysicalDevice inDevice);
	void							DestroySwapChain();
	void							UpdateViewPortAndScissorRect(uint32 inWidth, uint32 inHeight);
	VkSemaphore						AllocateSemaphore();
	void							FreeSemaphore(VkSemaphore inSemaphore);

	VkInstance						mInstance = VK_NULL_HANDLE;
	VkPhysicalDevice				mPhysicalDevice = VK_NULL_HANDLE;
	VkPhysicalDeviceMemoryProperties mMemoryProperties;
	VkDevice						mDevice = VK_NULL_HANDLE;
	uint32							mGraphicsQueueIndex = 0;
	uint32							mPresentQueueIndex = 0;
	VkQueue							mGraphicsQueue = VK_NULL_HANDLE;
	VkQueue							mPresentQueue = VK_NULL_HANDLE;
	VkSurfaceKHR					mSurface = VK_NULL_HANDLE;
	VkSwapchainKHR					mSwapChain = VK_NULL_HANDLE;
	bool							mSubOptimalSwapChain = false;
	TArray<VkImage>					mSwapChainImages;
	VkFormat						mSwapChainImageFormat;
	VkExtent2D						mSwapChainExtent;
	TArray<VkImageView>				mSwapChainImageViews;
	VkImage							mDepthImage = VK_NULL_HANDLE;
	VkDeviceMemory					mDepthImageMemory = VK_NULL_HANDLE;
	VkImageView						mDepthImageView = VK_NULL_HANDLE;
	VkDescriptorSetLayout			mDescriptorSetLayoutUBO = VK_NULL_HANDLE;
	VkDescriptorSetLayout			mDescriptorSetLayoutTexture = VK_NULL_HANDLE;
	VkDescriptorPool				mDescriptorPool = VK_NULL_HANDLE;
	VkDescriptorSet					mDescriptorSets[cFrameCount];
	VkDescriptorSet					mDescriptorSetsOrtho[cFrameCount];
	VkSampler						mTextureSamplerShadow = VK_NULL_HANDLE;	
	VkSampler						mTextureSamplerRepeat = VK_NULL_HANDLE;	
	VkRenderPass					mRenderPassShadow = VK_NULL_HANDLE;
	VkRenderPass					mRenderPass = VK_NULL_HANDLE;
	VkPipelineLayout				mPipelineLayout = VK_NULL_HANDLE;
	VkFramebuffer					mShadowFrameBuffer = VK_NULL_HANDLE;
	TArray<VkFramebuffer>			mSwapChainFramebuffers;
	uint32							mImageIndex = 0;
	VkCommandPool					mCommandPool = VK_NULL_HANDLE;
	VkCommandBuffer					mCommandBuffers[cFrameCount];
	TArray<VkSemaphore>				mAvailableSemaphores;
	TArray<VkSemaphore>				mImageAvailableSemaphores;
	TArray<VkSemaphore>				mRenderFinishedSemaphores;
	VkFence							mInFlightFences[cFrameCount];
	Ref<TextureVK>					mShadowMap;
	unique_ptr<ConstantBufferVK>	mVertexShaderConstantBufferProjection[cFrameCount];
	unique_ptr<ConstantBufferVK>	mVertexShaderConstantBufferOrtho[cFrameCount];
	unique_ptr<ConstantBufferVK>	mPixelShaderConstantBuffer[cFrameCount];

	struct Key {
		bool operator == (const Key &inRHS) const { return mSize == inRHS.mSize && mUsage == inRHS.mUsage && mProperties == inRHS.mProperties;}
		VkDeviceSize				mSize;
		VkBufferUsageFlags			mUsage;
		VkMemoryPropertyFlags		mProperties;
	};

	MOSS_MAKE_HASH_STRUCT(Key, KeyHasher, t.mSize, t.mUsage, t.mProperties)

	// We try to recycle buffers from frame to frame
	using BufferCache = TMap<Key, TArray<BufferVK>, KeyHasher>;

	BufferCache						mFreedBuffers[cFrameCount];
	BufferCache						mBufferCache;

	// Smaller allocations (from cMinAllocSize to cMaxAllocSize) will be done in blocks of cBlockSize bytes.
	// We do this because there is a limit to the number of allocations that we can make in Vulkan.
	static constexpr VkDeviceSize	cMinAllocSize = 512;
	static constexpr VkDeviceSize	cMaxAllocSize = 65536;
	static constexpr VkDeviceSize	cBlockSize = 524288;

	MOSS_MAKE_HASH_STRUCT(Key, MemKeyHasher, t.mUsage, t.mProperties, t.mSize)

	struct Memory {
		VkDeviceMemory				mMemory;
		VkDeviceSize				mOffset;
	};

	using MemoryCache = TMap<Key, TArray<Memory>, KeyHasher>;

	MemoryCache						mMemoryCache;
	uint32							mNumAllocations = 0;
	uint32							mMaxNumAllocations = 0;
	VkDeviceSize					mTotalAllocated = 0;
	VkDeviceSize					mMaxTotalAllocated = 0;
#ifdef MOSS_DEBUG
	VkDebugUtilsMessengerEXT		mDebugMessenger = VK_NULL_HANDLE;
#endif  // MOSS_DEBUG
#elif defined(MOSS_GRAPHICS_METAL)
	MTKView*						mView;
	MTLRenderPassDescriptor*		mShadowRenderPass;
	Ref<TextureMTL>					mShadowMap;
	id<MTLLibrary>					mShaderLibrary;
	id<MTLCommandQueue>				mCommandQueue;
	id<MTLCommandBuffer> 			mCommandBuffer;
	id<MTLRenderCommandEncoder>		mRenderEncoder;
#ifdef MOSS_DEBUG
#endif  // MOSS_DEBUG
#elif defined(MOSS_GRAPHICS_DIRECTX)
	// DirectX interfaces
	ComPtr<IDXGIFactory4>			mDXGIFactory;
	ComPtr<ID3D12Device>			mDevice;
	DescriptorHeapDX12				mRTVHeap;							///< Render target view heap
	DescriptorHeapDX12				mDSVHeap;							///< Depth stencil view heap
	DescriptorHeapDX12				mSRVHeap;							///< Shader resource view heap
	ComPtr<IDXGISwapChain3>			mSwapChain;
	ComPtr<ID3D12Resource>			mRenderTargets[cFrameCount];		///< Two render targets (we're double buffering in order for the CPU to continue while the GPU is rendering)
	D3D12_CPU_DESCRIPTOR_HANDLE		mRenderTargetViews[cFrameCount];	///< The two render views corresponding to the render targets
	ComPtr<ID3D12Resource>			mDepthStencilBuffer;				///< The main depth buffer
	D3D12_CPU_DESCRIPTOR_HANDLE		mDepthStencilView { 0 };			///< A view for binding the depth buffer
	ComPtr<ID3D12CommandAllocator>	mCommandAllocators[cFrameCount];	///< Two command allocator lists (one per frame)
	ComPtr<ID3D12CommandQueue>		mCommandQueue;						///< The command queue that will execute commands (there's only 1 since we want to finish rendering 1 frame before moving onto the next)
	ComPtr<ID3D12GraphicsCommandList> mCommandList;						///< The command list
	ComPtr<ID3D12RootSignature>		mRootSignature;						///< The root signature, we have a simple application so we only need 1, which is suitable for all our shaders
	Ref<TextureDX12>				mShadowMap;							///< Used to render shadow maps
	CommandQueueDX12				mUploadQueue;						///< Queue used to upload resources to GPU memory
	unique_ptr<ConstantBufferDX12>	mVertexShaderConstantBufferProjection[cFrameCount];
	unique_ptr<ConstantBufferDX12>	mVertexShaderConstantBufferOrtho[cFrameCount];
	unique_ptr<ConstantBufferDX12>	mPixelShaderConstantBuffer[cFrameCount];

	// Synchronization objects used to finish rendering and swapping before reusing a command queue
	HANDLE							mFenceEvent;						///< Fence event to wait for the previous frame rendering to complete (in order to free 1 of the buffers)
	ComPtr<ID3D12Fence>				mFence;								///< Fence object, used to signal the end of a frame
	UINT64							mFenceValues[cFrameCount] = {};		///< Values that were used to signal completion of one of the two frames

	using ResourceCache = UnorderedMap<uint64, Array<ComPtr<ID3D12Resource>>>;

	ResourceCache					mResourceCache;						///< Cache items ready to be reused
	ResourceCache					mDelayCached[cFrameCount];			///< List of reusable ID3D12Resources that are potentially referenced by the GPU so can be used only when the GPU finishes
	Array<ComPtr<ID3D12Object>>		mDelayReleased[cFrameCount];		///< Objects that are potentially referenced by the GPU so can only be freed when the GPU finishes
	bool							mIsExiting = false;					///< When exiting we don't want to add references too buffers
#ifdef MOSS_DEBUG
#endif  // MOSS_DEBUG
#endif
};


// Extenstions

class [[nodiscard]] MOSS_API Subviewport {
public:
    enum class CameraType { None, Camera2D, Camera3D };
    enum class SceneType  { None, Surface2D, Mesh3D };

    // Constructor
    Subviewport(float width, float height) : resolution_width(width), resolution_height(height), cameraType(CameraType::None), sceneType(SceneType::None){
        // Automatically create render target texture
        texture = CreateRenderTexture(resolution_width, resolution_height);
    }

    ~Subviewport() {
        // Cameras are not owned externally, so don't delete raw pointers
        camera2 = nullptr;
        camera3 = nullptr;
        surface = nullptr;
        mesh = nullptr;
    }

    // Set camera
    void SetCamera(Camera2D* cam) { camera2 = cam; cameraType = CameraType::Camera2D; }
    void SetCamera(Camera3D* cam) { camera3 = cam; cameraType = CameraType::Camera3D; }

    // Set target scene
    void SetTarget(Surface* s) { surface = s; sceneType = SceneType::Surface2D; }
    void SetTarget(Mesh* m)    { mesh    = m; sceneType = SceneType::Mesh3D; }

    // Resize the viewport and its render target
    void SetResolution(float width, float height) {
        resolution_width = width;
        resolution_height = height;
        texture = CreateRenderTexture(resolution_width, resolution_height);
    }

    // Draw the subviewport content
    void draw() {
        if (!texture) return;

        // Update camera view-projection
        if (cameraType == CameraType::Camera2D && camera2) {
            u_viewProj = camera2->getViewProjectionMatrix();
        } else if (cameraType == CameraType::Camera3D && camera3) {
            u_viewProj = camera3->getViewProjectionMatrix();
        }

        // Bind render target
        BindRenderTarget(texture);

        // Bind pipeline if exists
        if (m_pipeline) m_pipeline->bind();

        // Draw the assigned object
        switch (sceneType) {
            case SceneType::Surface2D:
                if (surface) surface->Draw(u_viewProj);
                break;
            case SceneType::Mesh3D:
                if (mesh) mesh->Draw(u_viewProj);
                break;
            default: break;
        }

        if (m_pipeline) m_pipeline->unbind();

        // Unbind render target (return to main framebuffer)
        UnbindRenderTarget();
    }

    // Accessors
    float GetWidth() const { return resolution_width; }
    float GetHeight() const { return resolution_height; }
    Ref<Texture> GetTexture() const { return texture; }

private:
    float resolution_width, resolution_height;
    Ref<Texture> texture;         // Render target texture
    Mat44 u_viewProj;             // Cached view-projection matrix
    unique_ptr<PipelineState> m_pipeline;

    CameraType cameraType = CameraType::None;
    union {
        Camera2D* camera2;
        Camera3D* camera3;
    };

    SceneType sceneType = SceneType::None;
    union {
        Surface* surface;
        Mesh* mesh;
    };

    // Helpers
    Ref<Texture> CreateRenderTexture(float width, float height) {
        // TODO: Implement platform-specific texture creation
        // For example, create a 2D render target
        return Ref<Texture>{ /* create texture here */ };
    }

    void BindRenderTarget(Ref<Texture> tex) {
        // TODO: Bind tex as framebuffer/render target
    }

    void UnbindRenderTarget() {
        // TODO: Restore previous framebuffer (main screen)
    }
};

/*! @brief */
class MOSS_API LightManager2D {
public:
    explicit LightManager2D(Moss_Renderer* renderer);
    ~LightManager2D() = default;

    void setDirectionalLight(const DirectionalLight2D& light);
    void addPointLight(const PointLight2D& light);
    void addTextureLight(const TextureLight2D& light);

	void AddGI(GlobalIllumination2D gi);

    void clear();

    // Should upload light data to active shader
    void draw() const;

private:
    Moss_Renderer* m_renderer;
    Ref<StorageBuffer> m_lightBuffer;
    unique_ptr<PipelineState> mPipeline;

	// Light 2D data
	DirectionalLight2D directionalLight;
    int hasDirectional = 0;
    std::vector<PointLight2D> pointLights;
    std::vector<TextureLight2D> textureLights;

	std::vector<GlobalIllumination2D> m_gi;
};

/*! @brief */
class MOSS_API LightManager3D {
public:
    LightManager3D(Moss_Renderer* renderer);
    ~LightManager3D() = default;

    void setDirectionalLight(const DirectionalLight3D& light);
    void addSpotLight(const SpotLight3D& light);
    void addOmniLight(const OmniLight3D& light);
    void addTextureLight(const TextureLight3D& light);


	void AddGI(std::unique_ptr<GlobalIllumination> gi);
    void AddReflectionProbe(std::unique_ptr<ReflectionProbe> probe);

    const auto& GetGIs() const { return m_globalIllumination; }
    const auto& GetReflectionProbes() const { return m_reflectionProbes; }

    void clear();

    // Upload light and shadow data to a backend-specific shader
    void draw() const;

    // Optional for engines that support shadows
    void renderShadows(const class Scene& scene);
private:
    Moss_Renderer* m_renderer;
    Ref<StorageBuffer> m_lightBuffer;
    unique_ptr<PipelineState> mPipeline;

	// Light 3D data
	DirectionalLight3D directionalLight;
    int hasDirectional = 0;
    std::vector<SpotLight3D> spotLights;
    std::vector<OmniLight3D> omniLights;
    std::vector<TextureLight3D> textureLights;

	std::vector<std::unique_ptr<GlobalIllumination>> m_globalIllumination;
    std::vector<std::unique_ptr<ReflectionProbe>> m_reflectionProbes;
};


// Mesh intented for 3D meshes
class [[nodiscard]] MOSS_API MeshManager3D {
public:
	MeshManager3D(Moss_Renderer* renderer);
	~MeshManager3D();

	Mesh* CreateMesh();
private:
    struct MeshEntry {
        std::string name;
        std::vector<Mesh> lods;
    };

    std::unordered_map<std::string, MeshEntry> m_meshes;
	unique_ptr<PipelineState> pipeline;
}

// Model intented for 3D models
class [[nodiscard]] MOSS_API ModelManager {
public:
	ModelManager3D(Moss_Renderer* renderer);
	~ModelManager3D();

	Model* CreateModel();
private:
    struct MeshEntry {
        std::string name;
        std::vector<Mesh> lods;
    };

    std::unordered_map<std::string, MeshEntry> m_meshes;
	unique_ptr<PipelineState> pipeline;
}

// Surface
class [[nodiscard]] MOSS_API SurfaceManager {
public:
	SurfaceManager(Moss_Renderer* renderer);
	~SurfaceManager();

	Surface* CreateSurface();
private:
    struct MeshEntry {
        std::string name;
        std::vector<Mesh> lods;
    };

    std::unordered_map<std::string, MeshEntry> m_meshes;
	Moss_Renderer* m_renderer;
	unique_ptr<PipelineState> pipeline;
}

// Instaces
class [[nodiscard]] MOSS_API MeshInstanceManager {
public:
	MeshInstanceManager(Moss_Renderer* renderer);
	~MeshInstanceManager();
	MeshInstance* CreateMeshInstance();
private:
    struct MeshEntry {
        std::string name;
        std::vector<Mesh> lods;
		std::vector<Mat44> instanceMatrix;
    };

    std::unordered_map<std::string, MeshEntry> m_meshes;
	Moss_Renderer* m_renderer;
	unique_ptr<PipelineState> pipeline;
}

class [[nodiscard]] MOSS_API SurfaceInstanceManager {
public:
	SurfaceInstanceManager(Moss_Renderer* renderer);
	~SurfaceInstanceManager();
	SurfaceInstance* CreateSurfaceInstance();
private:
    struct MeshEntry {
        std::string name;
        std::vector<Mesh> lods;
		std::vector<Mat44> instanceMatrix;
    };

    std::unordered_map<std::string, MeshEntry> m_meshes;
	Moss_Renderer* m_renderer;
	unique_ptr<PipelineState> pipeline;
}


class [[nodiscard]] MOSS_API SkyBox : public RefTarget<SkyBox> {
public:
    SkyBox(Moss_Renderer* renderer, Ref<Texture> inTexture) : mRenderer(inRenderer), m_texture(std::move(inTexture))  {
        const PipelineState::EInputDescription vertex_desc[] = { PipelineState::EInputDescription::Position };

        // Load vertex and pixel shaders
        Ref<VertexShader> vtx = mRenderer->CreateVertexShader("SkyBoxVertexShader");
        Ref<PixelShader> pix = mRenderer->CreatePixelShader("SkyBoxPixelShader");

        // Create pipeline state (Skybox usually uses FrontFace to render inside of cube)
        pipeline = mRenderer->CreatePipelineState(vtx, vertex_desc, std::size(vertex_desc), pix, PipelineState::EDrawPass::Normal, PipelineState::EFillMode::Solid, PipelineState::ETopology::Triangle, 
		PipelineState::EDepthTest::On, PipelineState::EBlendMode::AlphaBlend, PipelineState::ECullMode::FrontFace);

        // Create cube primitive for the skybox
        primitive = mRenderer->CreateRenderPrimitive(PipelineState::ETopology::Triangle);
        // Assume primitive is filled elsewhere or uses a cube mesh
    }

    ~SkyBox() = default;

    void Draw(const Mat44& matrixProjection) {
        m_texture->Bind();            // You may need to bind to a slot
        pipeline->Activate();
        primitive->Draw();            // You may need to set transform before this
        m_texture->Unbind();
    }

private:
    Moss_Renderer* m_Renderer = nullptr;
    unique_ptr<PipelineState> pipeline;
    Ref<Mesh> primitive;
    Ref<Texture> m_texture;
};


class [[nodiscard]] MOSS_API FogVolumeManager : public RefTarget<FogVolume> {
public:
    FogVolumeManager(Moss_Renderer* renderer) : mRenderer(inRenderer) {
        const PipelineState::EInputDescription vertex_desc[] = { PipelineState::EInputDescription::Position PipelineState::EInputDescription::Color };

        // Load shaders that do volumetric raymarching
        Ref<VertexShader> vtx = mRenderer->CreateVertexShader("FogVolumeVertexShader");
        Ref<PixelShader> pix = mRenderer->CreatePixelShader("FogVolumePixelShader");

        pipeline = mRenderer->CreatePipelineState( vtx, vertex_desc, std::size(vertex_desc), pix, PipelineState::EDrawPass::Normal, PipelineState::EFillMode::Solid, PipelineState::ETopology::Triangle,
            PipelineState::EDepthTest::On, PipelineState::EBlendMode::AlphaBlend, PipelineState::ECullMode::Backface);

        // Fog box mesh or primitive (usually a cube)
        primitive = mRenderer->CreateRenderPrimitive(PipelineState::ETopology::Triangle);
        // Setup cube geometry here...
    }
	~FogVolumeManager();

    void Draw(const Mat44& modelMatrix)
    {
        // Set shader uniforms like density, color, step size...
        pipeline->Activate();
        primitive->Draw(); // Draws the fog volume cube
    }

private:
    Moss_Renderer* mRenderer = nullptr;
    unique_ptr<PipelineState> pipeline;
    Ref<RenderPrimitive> primitive;
};


class [[nodiscard]] MOSS_API ReflectionProbeManager {
public:
	ReflectionProbeManager(Moss_Renderer* renderer);
	~ReflectionProbeManager();
	ReflectionProbe* CreateReflectionProbe();

private:
	Moss_Renderer* renderer = nullptr;
	std::vector<ReflectionProbe*> m_probes;
}

class [[nodiscard]] MOSS_API DecalManager {
public:
    DecalManager(Moss_Renderer* renderer);
    ~DecalManager() = default;

    // Create decal using a transform instead of separate position/rotation/scale
    Decal* CreateDecal(const Mat44& model, Ref<Texture> albedo, std::optional<Ref<Texture>> normal = {}, std::optional<Ref<Texture>> orm = {}, 
		std::optional<Ref<Texture>> emission = {}, Color color = Color(1,1,1,1),float emission_energy = 0.0f, float blendFactor = 1.0f, CullFilter filter = CullFilter::Layer_1);

    void ClearDecals() { m_decals.clear(); }

    // Update decal buffer for GPU
    void UpdateDecalBuffer();

    // Draw decals
    void DrawDecals(const Mat44& viewProj);

private:
    struct VertexAttribute {
        std::string semantic;
        uint32_t location;
        uint32_t size;
        uint32_t stride;
        uint32_t offset;
    };

    Moss_Renderer* renderer = nullptr;
    unique_ptr<PipelineState> pipeline;
    std::vector<Decal> m_decals;
    Ref<StorageBuffer> decalBuffer;

    void UploadToGPU(); // Internal helper
};

class [[nodiscard]] MOSS_API SkeletonManager {
public:
	SkeletonManager(Moss_Renderer* renderer);
	~SkeletonManager();
	/*! @brief */
	Bone* CreateBone(const std::string& name, int parentIndex = -1);
	Skeleton* CreateSkeleton(const std::vector<Bone*>& bones);
private:
	Moss_Renderer* renderer = nullptr;
	unique_ptr<PipelineState> pipeline;
}

class [[nodiscard]] MOSS_API SubViewportManager {
public:
	SubViewportManager(Moss_Renderer* renderer);
	~SubViewportManager();
	/*! @brief */
	SubViewport* CreateSubViewport(Camera2D camera, Rect rect, int resolution_width, int resolution_height);
	SubViewport* CreateSubViewport(Camera3D camera, Rect rect, int resolution_width, int resolution_height);

private:
	Moss_Renderer* renderer = nullptr;
	unique_ptr<PipelineState> pipeline;
}



/*
enum class GIUpdateType : uint8_t {
    Static,   // baked once
    Rigid,    // transform changes only
    Dynamic   // fully re-evaluated
};

struct GlobalIllumination {
    GIUpdateType updateType = GIUpdateType::Static;

    bool affectsStaticGeometry  = true;
    bool affectsDynamicGeometry = true;

    float intensity = 1.0f;
    float indirectBounceFactor = 1.0f;

    virtual ~GlobalIllumination() = default;
};
struct VoxelGI : public GlobalIllumination {
    AABB bounds;

    uint32_t resolution = 128;
    uint32_t cascadeCount = 1;

    bool temporalAccumulation = true;
    float historyWeight = 0.95f;
};
struct ProbeGI : public GlobalIllumination {
    Vec3 probeSpacing = Vec3(2.0f);
    uint32_t maxProbes = 4096;

    bool interpolateIrradiance = true;
};
struct GlobalIllumination2D : public GlobalIllumination {
    AABB2 bounds;
    float bounceRadius = 10.0f;
};
class ReflectionProbe {
public:
    enum class Type : uint8_t {
        Box,
        Sphere,
        Planar
    };

    ReflectionProbe(Type inType, GIUpdateType inUpdateType)
        : mType(inType), mUpdateType(inUpdateType) {}

    virtual ~ReflectionProbe() = default;

    Type        GetType() const { return mType; }
    GIUpdateType GetUpdateType() const { return mUpdateType; }

    float GetIntensity() const { return mIntensity; }
    void  SetIntensity(float v) { mIntensity = v; }

protected:
    Type mType;
    GIUpdateType mUpdateType;

    float mIntensity = 1.0f;
};

class BoxReflectionProbe final : public ReflectionProbe {
public:
    BoxReflectionProbe()
        : ReflectionProbe(Type::Box, GIUpdateType::Static) {}

    AABB bounds;
    float blendDistance = 1.0f;
};
class SphereReflectionProbe final : public ReflectionProbe {
public:
    SphereReflectionProbe()
        : ReflectionProbe(Type::Sphere, GIUpdateType::Static) {}

    Vec3 position;
    float radius = 10.0f;
};
class PlanarReflectionProbe final : public ReflectionProbe {
public:
    PlanarReflectionProbe()
        : ReflectionProbe(Type::Planar, GIUpdateType::Rigid) {}

    Plane reflectionPlane;
    float clipBias = 0.01f;
};

*/


#endif // MOSS_RENDERER_H