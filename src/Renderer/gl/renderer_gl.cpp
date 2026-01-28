#include <Moss/Renderer/Renderer_intern.h>

// Compatibility


RendererGL::~RendererGL() override = default;

void RendererGL::Initialize(ApplicationWindow* inWindow) override {
		mWindow = inWindow;

		// Init OpenGL context (assuming window system already created it)
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_CULL_FACE);
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
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

	return true;
}

void RendererGL::EndShadowPass() override {
		// Optionally unbind shadow framebuffer here
}

void RendererGL::EndFrame() override {
	mInFrame = false;

	mWindow->SwapBuffers(); // Assuming this wraps glfwSwapBuffers or similar
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