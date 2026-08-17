#include <Moss/Renderer/Renderer_intern.h>


typedef struct Moss_PipelineState {
    GLuint program;         // OpenGL shader program
} Moss_PipelineState;


Moss_PipelineState* Moss_RendererCreatePipeline(Moss_Renderer* renderer, const Moss_PipelineDesc* desc) {

}


void Moss_PipelineSetUniform(Moss_PipelineState* pipeline, const char* name, const void* data, uint32_t size) {
    if (!pipeline || !pipeline->program) return;

    GLint location = glGetUniformLocation(pipeline->program, name);
    if (location == -1) return; // uniform not found

    switch(size) {
        // FLOAT types
        case GL_FLOAT:       glUniform1fv(uniform->location, 1, (const GLfloat*)data); break;
        case GL_FLOAT_VEC2:  glUniform2fv(uniform->location, 1, (const GLfloat*)data); break;
        case GL_FLOAT_VEC3:  glUniform3fv(uniform->location, 1, (const GLfloat*)data); break;
        case GL_FLOAT_VEC4:  glUniform4fv(uniform->location, 1, (const GLfloat*)data); break;

        // INTEGER types
        case GL_INT:         glUniform1iv(uniform->location, 1, (const GLint*)data); break;
        case GL_INT_VEC2:    glUniform2iv(uniform->location, 1, (const GLint*)data); break;
        case GL_INT_VEC3:    glUniform3iv(uniform->location, 1, (const GLint*)data); break;
        case GL_INT_VEC4:    glUniform4iv(uniform->location, 1, (const GLint*)data); break;

        // UNSIGNED INTEGER types (OpenGL 3.3 / ES 3.0 only)
        case GL_UNSIGNED_INT:    glUniform1uiv(uniform->location, 1, (const GLuint*)data); break;
        case GL_UNSIGNED_INT_VEC2: glUniform2uiv(uniform->location, 1, (const GLuint*)data); break;
        case GL_UNSIGNED_INT_VEC3: glUniform3uiv(uniform->location, 1, (const GLuint*)data); break;
        case GL_UNSIGNED_INT_VEC4: glUniform4uiv(uniform->location, 1, (const GLuint*)data); break;

        // MATRIX types
        case GL_FLOAT_MAT2:      glUniformMatrix2fv(uniform->location, 1, GL_FALSE, (const GLfloat*)data); break;
        case GL_FLOAT_MAT3:      glUniformMatrix3fv(uniform->location, 1, GL_FALSE, (const GLfloat*)data); break;
        case GL_FLOAT_MAT4:      glUniformMatrix4fv(uniform->location, 1, GL_FALSE, (const GLfloat*)data); break;
        case GL_FLOAT_MAT2x3:    glUniformMatrix2x3fv(uniform->location, 1, GL_FALSE, (const GLfloat*)data); break;
        case GL_FLOAT_MAT3x2:    glUniformMatrix3x2fv(uniform->location, 1, GL_FALSE, (const GLfloat*)data); break;
        case GL_FLOAT_MAT2x4:    glUniformMatrix2x4fv(uniform->location, 1, GL_FALSE, (const GLfloat*)data); break;
        case GL_FLOAT_MAT4x2:    glUniformMatrix4x2fv(uniform->location, 1, GL_FALSE, (const GLfloat*)data); break;
        case GL_FLOAT_MAT3x4:    glUniformMatrix3x4fv(uniform->location, 1, GL_FALSE, (const GLfloat*)data); break;
        case GL_FLOAT_MAT4x3:    glUniformMatrix4x3fv(uniform->location, 1, GL_FALSE, (const GLfloat*)data); break;

        case GL_SAMPLER_1D:
        case GL_SAMPLER_2D:
        case GL_SAMPLER_3D:
        case GL_SAMPLER_CUBE:
        case GL_SAMPLER_2D_SHADOW:
        case GL_INT_SAMPLER_1D:
        case GL_INT_SAMPLER_2D:
        case GL_INT_SAMPLER_3D:
        case GL_INT_SAMPLER_CUBE:
        case GL_UNSIGNED_INT_SAMPLER_2D:
        case GL_UNSIGNED_INT_SAMPLER_3D:
        case GL_UNSIGNED_INT_SAMPLER_CUBE: glUniform1iv(uniform->location, 1, (const GLint*)data); // Pass texture unit index
        break;

        default: /* handle other types or log error */ break;
    }
}


void Moss_PipelineBind(Moss_PipelineState* pipeline) {
	glUseProgram(pipeline->program);
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
        case EBlendMode::Opaque: glDisable(GL_BLEND); break;
        case EBlendMode::Alpha: glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); break;
        case EBlendMode::Additive: glEnable(GL_BLEND); glBlendFunc(GL_ONE, GL_ONE); break;
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
        case EBlendMode::Opaque: glDisable(GL_BLEND); break;
        case EBlendMode::Alpha: glDisable(GL_BLEND); break;
        case EBlendMode::Additive: glDisable(GL_BLEND); break;
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