
#include "renderer_gl.h"


// Compatibility

struct Texture {

};

struct Font {

};






RendererGL::~RendererGL() override = default;

void RendererGL::Initialize(ApplicationWindow* inWindow) override {
		mWindow = inWindow;

		// Init OpenGL context (assuming window system already created it)
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_CULL_FACE);
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
}

// Mobile
bool Moss_CreateRenderer(Moss_Renderer* renderer, Moss_Window* window) {
    renderer->window = window;

    // 1. Create OpenGL ES context (platform-specific)
    if (!CreateGLESContext(window)) {
        return false;
    }

    // 2. Make context current
    MakeContextCurrent(window);

    // 3. Query device info
    renderer->deviceInfo.vendor   = (const char*)glGetString(GL_VENDOR);
    renderer->deviceInfo.renderer = (const char*)glGetString(GL_RENDERER);
    renderer->deviceInfo.version  = (const char*)glGetString(GL_VERSION);

    // 4. Setup default GL state
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);

    // 5. Create persistent resources
    CreateDefaultShaders(renderer);
    CreateDefaultBuffers(renderer);
    CreateDefaultTextures(renderer);

    return true;
}


void Moss_TerminateRenderer(Moss_Renderer* renderer) {
    DestroyShaders(renderer);
    DestroyBuffers(renderer);
    DestroyTextures(renderer);

    DestroyGLESContext(renderer->window);
}

bool RendererGL::BeginFrame(const CameraState& inCamera, float inWorldScale) override {
	mInFrame = true;
	mCameraState = inCamera;
	mFrameIndex = (mFrameIndex + 1) % cFrameCount;

	// Calculate camera frustum here if needed
	mCameraFrustum = Frustum::FromCamera(inCamera);

	// Bind default framebuffer
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	glViewport(0, 0, mWindow->GetWidth(), mWindow->GetHeight());
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	/*
	Moss_WindowGetFramebufferSize(r->window, &width, &height);

    glViewport(0, 0, renderer->window->width, renderer->window->height);

    glBindFramebuffer(GL_FRAMEBUFFER, 0); // Default framebuffer

    glClearColor(renderer->clearColor.r, renderer->clearColor.g, renderer->clearColor.b, renderer->clearColor.a);

    GLbitfield clearMask = GL_COLOR_BUFFER_BIT;

    if (r->enableDepthTest) {
        glEnable(GL_DEPTH_TEST);
        clearMask |= GL_DEPTH_BUFFER_BIT;
    }

    glClear(clearMask);
	*/

	return true;
}

void RendererGL::EndShadowPass() override {
		// Optionally unbind shadow framebuffer here
}

void RendererGL::EndFrame() override {
	mInFrame = false;

	mWindow->SwapBuffers(); // Assuming this wraps glfwSwapBuffers or similar

	glFlush(); // optional
    SwapBuffers(renderer->window); // eglSwapBuffers / CAEAGLLayer / Android
}

void RendererGL::SetProjectionMode() override {
	// 3D mode: Load from mVSBuffer
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(&mVSBuffer.mProjection(0, 0));
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(&mVSBuffer.mView(0, 0));
}

void RendererGL::SetOrthoMode() override
{
	// 2D mode: Load from mVSBufferOrtho
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(&mVSBufferOrtho.mProjection(0, 0));
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(&mVSBufferOrtho.mView(0, 0));
}

Ref<Texture> RendererGL::CreateTexture(const Surface* inSurface) override
{
	return MakeRef<TextureGL>(inSurface);
}

Ref<VertexShader> RendererGL::CreateVertexShader(const char* inName) override
{
	auto shader = MakeRef<VertexShaderGL>();
	if (shader->LoadFromFile(inName))
		return shader;
	return nullptr;
}

Ref<PixelShader> RendererGL::CreatePixelShader(const char* inName) override
{
	auto shader = MakeRef<PixelShaderGL>();
	if (shader->LoadFromFile(inName))
		return shader;
	return nullptr;
}

unique_ptr<PipelineState> RendererGL::CreatePipelineState(
	const VertexShader* inVertexShader,
	const PipelineState::EInputDescription* inInputDescription,
	uint32 inInputDescriptionCount,
	const PixelShader* inPixelShader,
	PipelineState::EDrawPass inDrawPass,
	PipelineState::EFillMode inFillMode,
	PipelineState::ETopology inTopology,
	PipelineState::EDepthTest inDepthTest,
	PipelineState::EBlendMode inBlendMode,
	PipelineState::ECullMode inCullMode) override {
	return make_unique<PipelineStateGL>(
		this,
		static_cast<const VertexShaderGL*>(inVertexShader),
		inInputDescription,
		inInputDescriptionCount,
		static_cast<const PixelShaderGL*>(inPixelShader),
		inDrawPass,
		inFillMode,
		inTopology,
		inDepthTest,
		inBlendMode,
		inCullMode);
}

RenderPrimitive* RendererGL::CreateRenderPrimitive(PipelineState::ETopology inType) override { return new RenderPrimitiveGL(inType); }

RenderInstances* RendererGL::CreateRenderInstances() override { return new RenderInstancesGL(); }

Texture* RendererGL::GetShadowMap() const override {
	// Optional: Return real shadow texture if implemented
	return nullptr;
}

void RendererGL::OnWindowResize() override
{
	glViewport(0, 0, mWindow->GetWidth(), mWindow->GetHeight());
}