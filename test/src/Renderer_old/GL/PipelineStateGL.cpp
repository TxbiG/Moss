#include <Moss/Renderer/GL/PipelineStateGL.h>

const char* ppvert = R"(
#version 330 core
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec2 aTexCoord;

out vec2 TexCoords;

void main() {
    TexCoords = aTexCoord;
    gl_Position = vec4(aPos.xy, 0.0, 1.0);
})";

const char* ppfrag = R"(
#version 330 core
in vec2 TexCoords;
out vec4 FragColor;

uniform sampler2D u_Scene;

void main() {
    vec3 color = texture(u_Scene, TexCoords).rgb;
    // Simple gamma correction
    color = pow(color, vec3(1.0 / 2.2));
    FragColor = vec4(color, 1.0);
}
)";


PostProcess::PostProcess(int w, int h) : width(w), height(h), shader(ppvert, ppfrag, true) {
        // FBO + color texture
        glGenFramebuffers(1, &fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);

        glGenTextures(1, &fboTexture);
        glBindTexture(GL_TEXTURE_2D, fboTexture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16F, width, height, 0, GL_RGB, GL_FLOAT, nullptr); // HDR ready
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fboTexture, 0);

        // Optional: depth/stencil
        glGenRenderbuffers(1, &rbo);
        glBindRenderbuffer(GL_RENDERBUFFER, rbo);
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);

        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            printf("PostProcess: Framebuffer not complete!\n");

        
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // Set up quads
        float quadVertices[] = {
            // positions   // texCoords
            -1.0f,  1.0f,  0.0f, 1.0f,
            -1.0f, -1.0f,  0.0f, 0.0f,
            1.0f, -1.0f,  1.0f, 0.0f,

            -1.0f,  1.0f,  0.0f, 1.0f,
            1.0f, -1.0f,  1.0f, 0.0f,
            1.0f,  1.0f,  1.0f, 1.0f
        };

        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glBindVertexArray(vao);

        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);
        
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
        
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));

        glBindVertexArray(0);
    }

PostProcess::~PostProcess() {
        glDeleteFramebuffers(1, &fbo);
        glDeleteTextures(1, &fboTexture);
        glDeleteRenderbuffers(1, &rbo);
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &vbo);
    }

void PostProcess::bind() {
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glViewport(0, 0, width, height);
     glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void PostProcess::render(GLuint inputTexture) {
        glDisable(GL_DEPTH_TEST);
        shader.bind();

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, inputTexture);
        shader.SetUniformi("u_Scene", 0); // must match GLSL

        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glBindVertexArray(0);

        glEnable(GL_DEPTH_TEST);
        shader.unbind();
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


void PipelineStateGL::bind() override {
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


void PipelineStateGL::unbind() override {
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

GLenum PipelineStateGL::GetGLPrimitive() const {
	switch (topology) {
	case ETopology::Triangle: return GL_TRIANGLES;
	case ETopology::Line:     return GL_LINES;
	default:                  return GL_TRIANGLES;
	}
}