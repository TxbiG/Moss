#include <Moss/Renderer/Renderer_intern.h>


MOSS_API void Moss_GPUFatalError(Moss_Result result);

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

void Moss_RendererBeginFrame(Moss_Renderer* renderer) {
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
}

void Moss_RendererEndFrame(Moss_Renderer* renderer) {
    glFlush(); // optional
    SwapBuffers(renderer->window); // eglSwapBuffers / CAEAGLLayer / Android
}



void RendererGL::EndShadowPass() override {
		// Optionally unbind shadow framebuffer here
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
	uint inInputDescriptionCount,
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