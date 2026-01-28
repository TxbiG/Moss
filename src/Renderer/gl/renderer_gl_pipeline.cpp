#include <Moss/Renderer/Renderer_intern.h>



typedef struct Moss_PipelineState {
    GLuint program;         // OpenGL shader program
} Moss_PipelineState;

typedef struct Moss_Framebuffer {
    int width, height;

    GLuint fbo;              // OpenGL framebuffer object
    GLuint color_attachment; // Color texture/renderbuffer
    GLuint depth_attachment; // Depth texture/renderbuffer

    // Optional CPU-side buffers (if needed for readback)
    unsigned char* color_buffer;  
    float* depth_buffer;
} Moss_Framebuffer;

// GPU buffer abstraction
typedef struct Moss_GPUBuffer {
    GLuint buffer_id;        // OpenGL buffer object
    uint32_t size;           // Size in bytes
    GLenum target;           // GL_ARRAY_BUFFER, GL_ELEMENT_ARRAY_BUFFER, etc.
} Moss_GPUBuffer;

// GPU texture abstraction
typedef struct Moss_Texture {
    GLuint texture_id;
    GLenum target;           // GL_TEXTURE_2D, GL_TEXTURE_CUBE_MAP, etc.
    uint32_t width, height, depth;
    uint32_t mip_levels;
    GLenum internal_format;  // GL_RGBA8, GL_RGB16F, etc.
} Moss_Texture;

// Texture view abstraction (optional for OpenGL)
typedef struct Moss_TextureView {
    Moss_Texture* texture;
    GLenum target;           // GL_TEXTURE_2D, GL_TEXTURE_2D_ARRAY, etc.
    uint32_t base_mip;
    uint32_t mip_count;
    uint32_t base_layer;
    uint32_t layer_count;
} Moss_TextureView;

// Sampler abstraction
typedef struct Moss_GPUSampler {
    GLuint sampler_id;       // OpenGL sampler object
    GLenum min_filter;
    GLenum mag_filter;
    GLenum wrap_s;
    GLenum wrap_t;
    GLenum wrap_r;
    float max_anisotropy;
    float lod_bias;
} Moss_GPUSampler;

// Resource set abstraction (maps uniform buffers / textures / samplers)
typedef struct Moss_ResourceSet {
    // OpenGL: simple uniform location cache
    GLuint program;          // Shader program this set binds to
    GLuint* uniform_locations; // Array of uniform locations
    uint32_t count;           // Number of resources in the set
} Moss_ResourceSet;

// Command buffer abstraction (mostly no-op for OpenGL)
typedef struct Moss_GPUCommandBuffer {
    // OpenGL executes commands immediately; can store function pointers for batching if needed
    void** commands;
    uint32_t command_count;
} Moss_GPUCommandBuffer;

// Render graph placeholders
typedef struct Moss_RenderGraph {
    // Placeholder: OpenGL executes passes immediately
    void* dummy;
} Moss_RenderGraph;

typedef struct Moss_RGPass {
    const char* name;
    void (*record)(Moss_GPUCommandBuffer* cmd, void* user_data);
    void* user_data;
} Moss_RGPass;

typedef struct Moss_RGTexture {
    Moss_Texture* texture;
} Moss_RGTexture;

typedef struct Moss_RGBuffer {
    Moss_GPUBuffer* buffer;
} Moss_RGBuffer;

typedef struct Moss_GPUDevice {
    // For OpenGL, device is global context
    void* context;
} Moss_GPUDevice;

typedef struct Moss_Shader {
    GLuint program;  // OpenGL program
	EShaderStage type;
} Moss_Shader;


Moss_PipelineState* Moss_RendererCreatePipeline(Moss_Renderer* renderer, const Moss_PipelineDesc* desc) {

}


void Moss_PipelineSetUniform(Moss_PipelineState* pipeline, const char* name, const void* data, uint32_t size) {
    if (!pipeline || !pipeline->program) return;

    GLint location = glGetUniformLocation(pipeline->program, name);
    if (location == -1) return; // uniform not found

    switch(size) {
        case 4:  glUniform1fv(location, 1, (const GLfloat*)data); break;
        case 8:  glUniform2fv(location, 1, (const GLfloat*)data); break;
        case 12: glUniform3fv(location, 1, (const GLfloat*)data); break;
        case 16: glUniform4fv(location, 1, (const GLfloat*)data); break;
        case 64: glUniformMatrix4fv(location, 1, GL_FALSE, (const GLfloat*)data); break;
        default: /* handle other types or log error */ break;
    }
}


void Moss_PipelineBind(Moss_PipelineState* pipeline) {
	glUseProgram(mProgramID);
	// Apply OpenGL state settings

	switch ()
	if (depthTest == EDepthTest::On) {
		glEnable(GL_DEPTH_TEST);
		glDepthMask(GL_TRUE);
	} else {
		glDisable(GL_DEPTH_TEST);
		glDepthMask(GL_FALSE);
	}

	switch (blendMode) {
        case EBlendMode::Opaque:
            glDisable(GL_BLEND);
            break;
        case EBlendMode::Alpha:
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            break;
        case EBlendMode::Additive:
            glEnable(GL_BLEND);
            glBlendFunc(GL_ONE, GL_ONE);
            break;
    }


    if (cullMode != ECullMode::None) {
        glEnable(GL_CULL_FACE);
        glCullFace(cullMode == ECullMode::Backface ? GL_BACK : GL_FRONT);
    }


	// Fill mode
	glPolygonMode(GL_FRONT_AND_BACK, fillMode == EFillMode::Wireframe ? GL_LINE : GL_FILL);
}

void Moss_PipelineUnbind(Moss_PipelineState* pipeline) {
	if (depthTest == EDepthTest::On) {
		glDisable(GL_DEPTH_TEST);
		glDepthMask(GL_FALSE);
	}

	switch (blendMode) {
        case EBlendMode::Opaque:
            glDisable(GL_BLEND);
            break;
        case EBlendMode::Alpha:
            glDisable(GL_BLEND);
            break;
        case EBlendMode::Additive:
            glDisable(GL_BLEND);
            break;
    }

    if (cullMode != ECullMode::None) { glDisable(GL_CULL_FACE); } 
	else {
        glEnable(GL_CULL_FACE);
        glCullFace(cullMode == ECullMode::Backface ? GL_BACK : GL_FRONT);
    }

	// Bind shaders (you'd likely have a ShaderProgramGL class for this)
	// vertexShader->Bind();
	// pixelShader->Bind();

	glUseProgram(0);
}


PipelineStateGL::PipelineStateGL(RendererGL* inRenderer, 
    const VertexShaderGL* inVertexShader, const EInputDescription* inInputDescription, uint32_t inInputDescriptionCount, 
    const PixelShaderGL* inPixelShader, EDrawPass inDrawPass, EFillMode inFillMode, ETopology inTopology, EDepthTest inDepthTest, EBlendMode inBlendMode, ECullMode inCullMode) : 
    renderer(inRenderer), vertexShader(inVertexShader), pixelShader(inPixelShader),
	drawPass(inDrawPass), fillMode(inFillMode), topology(inTopology), depthTest(inDepthTest), blendMode(inBlendMode), cullMode(inCullMode) {
		
	// Copy input layout description
	inputDescription.assign(inInputDescription, inInputDescription + inInputDescriptionCount);


		// Create and link program
	mProgramID = glCreateProgram();
	glAttachShader(mProgramID, inVertexShader->GetID());
	glAttachShader(mProgramID, inPixelShader->GetID());
	glLinkProgram(mProgramID);
    mVertexShaderID = mFragmentShaderID = 0;


	mUniformLocations["u_ModelViewProj"] = glGetUniformLocation(mProgramID, "u_ModelViewProj");
	mUniformLocations["u_Texture"] = glGetUniformLocation(mProgramID, "u_Texture");
	mUniformLocations["u_Color"] = glGetUniformLocation(mProgramID, "u_Color");

		// Assume vao, vertexBuffer, instanceBuffer are created and valid
	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
		
	GLsizei stride = sizeof(Vertex);
	GLuint attribIndex = 0;
	for (size_t i = 0; i < inputDescCount; ++i) {
		switch (inputDesc[i]) {
			case PipelineState::EInputDescription::Position:
				glEnableVertexAttribArray(attribIndex);
					// 3 floats for position, not normalized, offset = offsetof(Vertex, position)
				glVertexAttribPointer(
					attribIndex,        // Attribute index in the shader
					3,                  // Number of components (x,y,z)
					GL_FLOAT,           // Data type
					GL_FALSE,           // Normalized?
					stride,             // Byte stride between vertices
					(void*)offsetof(Vertex, position) // Offset of this attribute in the struct
				);
				attribIndex++;
				break;

			case PipelineState::EInputDescription::TexCoord:
				glEnableVertexAttribArray(attribIndex);
				// 2 floats for texcoords, not normalized, offset = offsetof(Vertex, texCoord)
				glVertexAttribPointer(attribIndex,2,GL_FLOAT,GL_FALSE,stride,(void*)offsetof(Vertex, texCoord));
				attribIndex++;
				break;

			case PipelineState::EInputDescription::Color:
					glEnableVertexAttribArray(attribIndex);
					// 4 unsigned bytes for color, normalized to float in shader
				glVertexAttribPointer(
					attribIndex,
					4,
					GL_UNSIGNED_BYTE,
					GL_TRUE, // normalized = true (maps 0-255 to 0.0-1.0)
					stride,
					(void*)offsetof(Vertex, color)
				);
				attribIndex++;
				break;
			case PipelineState::EInputDescription::Normal:
				glEnableVertexAttribArray(attribIndex);
				glVertexAttribPointer(attribIndex, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
				glVertexAttribDivisor(attribIndex, 0);
				attribIndex++;
				break;
			case PipelineState::EInputDescription::InstanceColor:
				if (!instanceBufferBound) {
					glBindBuffer(GL_ARRAY_BUFFER, instanceBuffer);
					instanceBufferBound = true;
				}
				glEnableVertexAttribArray(attribIndex);
				glVertexAttribPointer(attribIndex, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(InstanceData), (void*)offsetof(InstanceData, instanceColor));
				glVertexAttribDivisor(attribIndex, 1); // per instance
				attribIndex++;
				break;
				
				case PipelineState::EInputDescription::InstanceTransform:
				if (!instanceBufferBound) {
					glBindBuffer(GL_ARRAY_BUFFER, instanceBuffer);
					instanceBufferBound = true;
				}
				for (int col = 0; col < 4; ++col) {
					glEnableVertexAttribArray(attribIndex + col);
					glVertexAttribPointer(attribIndex + col, 4, GL_FLOAT, GL_FALSE, sizeof(InstanceData), (void*)(offsetof(InstanceData, instanceTransform) + sizeof(float) * 4 * col));
					glVertexAttribDivisor(attribIndex + col, 1);
				}
				attribIndex += 4;
			    break;

			case PipelineState::EInputDescription::InstanceInvTransform:
				if (!instanceBufferBound) {
					glBindBuffer(GL_ARRAY_BUFFER, instanceBuffer);
					instanceBufferBound = true;
				}
				for (int col = 0; col < 4; ++col) {
					glEnableVertexAttribArray(attribIndex + col);
					glVertexAttribPointer(attribIndex + col, 4, GL_FLOAT, GL_FALSE, sizeof(InstanceData), (void*)(offsetof(InstanceData, instanceInvTransform) + sizeof(float) * 4 * col));
					glVertexAttribDivisor(attribIndex + col, 1);
				}
				attribIndex += 4;
				break;
			default:
				// ignore
				break;
		}
	}
	// Optionally compile/link shaders or setup VAO/VBO here
	// Not shown for brevity, but could be a good place to call CreatePipeline()
}



GLenum PipelineStateGL::GetGLPrimitive() const {
	switch (topology) {
	case ETopology::Triangle: return GL_TRIANGLES;
	case ETopology::Line:     return GL_LINES;
	default:                  return GL_TRIANGLES;
	}
}