float skyboxVertices[] = {
	//   Coordinates
	-1.0f, -1.0f,  1.0f,//		7--------6
	 1.0f, -1.0f,  1.0f,//	   /|	    /|
	 1.0f, -1.0f, -1.0f,//	  4--------5 |
	-1.0f, -1.0f, -1.0f,//	  | |	   | |
	-1.0f,  1.0f,  1.0f,//	  | 3------|-2
	 1.0f,  1.0f,  1.0f,//	  |/	   |/
	 1.0f,  1.0f, -1.0f,//	  0--------1
	-1.0f,  1.0f, -1.0f
};

unsigned int skyboxIndices[] =
{
	// Right
	1, 2, 6,
	6, 5, 1,
	// Left
	0, 4, 7,
	7, 3, 0,
	// Top
	4, 5, 6,
	6, 7, 4,
	// Bottom
	0, 3, 2,
	2, 1, 0,
	// Back
	0, 1, 5,
	5, 4, 0,
	// Front
	3, 7, 6,
	6, 2, 3
};




SkyBox::SkyBox(Renderer* inRenderer, Ref<Texture> inTexture) : mRenderer(inRenderer), m_texture(std::move(inTexture))  {
	const PipelineState::EInputDescription vertex_desc[] = { PipelineState::EInputDescription::Position };

	// Load vertex and pixel shaders
	Ref<VertexShader> vtx = mRenderer->CreateVertexShader("skybox");
	Ref<PixelShader> pix = mRenderer->CreatePixelShader("skybox");

	// Create pipeline state
	pipeline = mRenderer->CreatePipelineState(vtx,vertex_desc, std::size(vertex_desc),
	pix, PipelineState::EDrawPass::Normal, PipelineState::EFillMode::Solid, PipelineState::ETopology::Triangle,
	PipelineState::EDepthTest::On, PipelineState::EBlendMode::AlphaBlend, PipelineState::ECullMode::FrontFace  // Skybox usually uses FrontFace to render inside of cube
	);

	// Create cube primitive for the skybox
	primitive = mRenderer->CreateRenderPrimitive(PipelineState::ETopology::Triangle);
	

    // Create Mesh

    // Place texture on mesh
}


void Draw(const Mat44& matrixProjection) {
    m_texture->Bind();            // You may need to bind to a slot
    pipeline->Activate();
    primitive->Draw();            // You may need to set transform before this
    m_texture->Unbind();
}