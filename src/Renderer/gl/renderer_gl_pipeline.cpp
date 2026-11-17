// OpenGL / OpenGL ES 3.0+ pipeline implementation.
//
// A Moss_PipelineState wraps a linked GL program plus all the fixed-function
// state that goes with it. GL has no pipeline object, so:
//   - Moss_PipelineBind applies the *complete* state set every time, which is
//     why Moss_PipelineUnbind has nothing to restore.
//   - Uniforms are reflected once at link time. Moss_PipelineSetUniform writes
//     into a CPU shadow copy and Moss_PipelineFlush / Bind upload what changed,
//     which matches how the Vulkan / D3D12 / Metal backends treat constants.
//   - The vertex layout is stored on the pipeline and applied to a VAO with
//     MossGL_PipelineApplyVertexLayout (GL keeps attribute state in the VAO).

#include "renderer_gl.h"

#include <cstdio>
#include <cstring>

// Bound pipeline as tracked by this file. Only valid while all program changes
// go through Moss_PipelineBind / Moss_PipelineUnbind / MossGL_DestroyPipeline.
static thread_local const Moss_PipelineState* sBoundPipeline = nullptr;

// ---------------------------------------------------------------------------
// Shaders
// ---------------------------------------------------------------------------

GLuint MossGL_CompileShader(GLenum stage, const char* source) {
    if (!source) return 0;

    GLuint shader = glCreateShader(stage);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint ok = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        GLint len = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);
        std::vector<char> log((size_t)(len > 1 ? len : 1), 0);
        glGetShaderInfoLog(shader, (GLsizei)log.size(), nullptr, log.data());
        MOSS_ERROR("GL shader compile failed:\n%s\n", log.data());
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

// ---------------------------------------------------------------------------
// Uniform reflection helpers
// ---------------------------------------------------------------------------

// Byte size of one element of a uniform type, 0 if unsupported (images, etc).
static uint32_t UniformTypeSize(GLenum t) {
    switch (t) {
    case GL_FLOAT:                 return 4;
    case GL_FLOAT_VEC2:            return 8;
    case GL_FLOAT_VEC3:            return 12;
    case GL_FLOAT_VEC4:            return 16;
    case GL_INT:
    case GL_BOOL:
    case GL_UNSIGNED_INT:          return 4;
    case GL_INT_VEC2:
    case GL_BOOL_VEC2:
    case GL_UNSIGNED_INT_VEC2:     return 8;
    case GL_INT_VEC3:
    case GL_BOOL_VEC3:
    case GL_UNSIGNED_INT_VEC3:     return 12;
    case GL_INT_VEC4:
    case GL_BOOL_VEC4:
    case GL_UNSIGNED_INT_VEC4:     return 16;
    case GL_FLOAT_MAT2:            return 16;
    case GL_FLOAT_MAT3:            return 36;
    case GL_FLOAT_MAT4:            return 64;
    case GL_FLOAT_MAT2x3:
    case GL_FLOAT_MAT3x2:          return 24;
    case GL_FLOAT_MAT2x4:
    case GL_FLOAT_MAT4x2:          return 32;
    case GL_FLOAT_MAT3x4:
    case GL_FLOAT_MAT4x3:          return 48;
    case GL_SAMPLER_2D:
    case GL_SAMPLER_3D:
    case GL_SAMPLER_CUBE:
    case GL_SAMPLER_2D_SHADOW:
    case GL_SAMPLER_2D_ARRAY:
    case GL_SAMPLER_2D_ARRAY_SHADOW:
    case GL_SAMPLER_CUBE_SHADOW:
    case GL_INT_SAMPLER_2D:
    case GL_INT_SAMPLER_3D:
    case GL_INT_SAMPLER_CUBE:
    case GL_INT_SAMPLER_2D_ARRAY:
    case GL_UNSIGNED_INT_SAMPLER_2D:
    case GL_UNSIGNED_INT_SAMPLER_3D:
    case GL_UNSIGNED_INT_SAMPLER_CUBE:
    case GL_UNSIGNED_INT_SAMPLER_2D_ARRAY: return 4; // texture unit index
#ifdef GL_SAMPLER_1D
    case GL_SAMPLER_1D:
    case GL_SAMPLER_1D_SHADOW:     return 4;
#endif
    default:                       return 0;
    }
}

// Upload one uniform (program must be in use).
static void UploadUniform(const Moss_GLUniform& u, const uint8_t* data) {
    const GLint   l = u.location;
    const GLsizei n = u.count;
    switch (u.type) {
    case GL_FLOAT:             glUniform1fv(l, n, (const GLfloat*)data); break;
    case GL_FLOAT_VEC2:        glUniform2fv(l, n, (const GLfloat*)data); break;
    case GL_FLOAT_VEC3:        glUniform3fv(l, n, (const GLfloat*)data); break;
    case GL_FLOAT_VEC4:        glUniform4fv(l, n, (const GLfloat*)data); break;

    case GL_UNSIGNED_INT:      glUniform1uiv(l, n, (const GLuint*)data); break;
    case GL_UNSIGNED_INT_VEC2: glUniform2uiv(l, n, (const GLuint*)data); break;
    case GL_UNSIGNED_INT_VEC3: glUniform3uiv(l, n, (const GLuint*)data); break;
    case GL_UNSIGNED_INT_VEC4: glUniform4uiv(l, n, (const GLuint*)data); break;

    case GL_INT:
    case GL_BOOL:              glUniform1iv(l, n, (const GLint*)data); break;
    case GL_INT_VEC2:
    case GL_BOOL_VEC2:         glUniform2iv(l, n, (const GLint*)data); break;
    case GL_INT_VEC3:
    case GL_BOOL_VEC3:         glUniform3iv(l, n, (const GLint*)data); break;
    case GL_INT_VEC4:
    case GL_BOOL_VEC4:         glUniform4iv(l, n, (const GLint*)data); break;

    case GL_FLOAT_MAT2:        glUniformMatrix2fv  (l, n, GL_FALSE, (const GLfloat*)data); break;
    case GL_FLOAT_MAT3:        glUniformMatrix3fv  (l, n, GL_FALSE, (const GLfloat*)data); break;
    case GL_FLOAT_MAT4:        glUniformMatrix4fv  (l, n, GL_FALSE, (const GLfloat*)data); break;
    case GL_FLOAT_MAT2x3:      glUniformMatrix2x3fv(l, n, GL_FALSE, (const GLfloat*)data); break;
    case GL_FLOAT_MAT3x2:      glUniformMatrix3x2fv(l, n, GL_FALSE, (const GLfloat*)data); break;
    case GL_FLOAT_MAT2x4:      glUniformMatrix2x4fv(l, n, GL_FALSE, (const GLfloat*)data); break;
    case GL_FLOAT_MAT4x2:      glUniformMatrix4x2fv(l, n, GL_FALSE, (const GLfloat*)data); break;
    case GL_FLOAT_MAT3x4:      glUniformMatrix3x4fv(l, n, GL_FALSE, (const GLfloat*)data); break;
    case GL_FLOAT_MAT4x3:      glUniformMatrix4x3fv(l, n, GL_FALSE, (const GLfloat*)data); break;

    default:                   glUniform1iv(l, n, (const GLint*)data); break; // samplers: unit index
    }
}

// Program must be in use.
static void UploadDirty(Moss_PipelineState* p) {
    if (!p->anyDirty) return;
    for (Moss_GLUniform& u : p->uniforms) {
        if (!u.dirty) continue;
        UploadUniform(u, p->shadow.data() + u.offset);
        u.dirty = false;
    }
    p->anyDirty = false;
}

static void ReflectUniforms(Moss_PipelineState* p) {
    GLint count = 0, maxLen = 0;
    glGetProgramiv(p->program, GL_ACTIVE_UNIFORMS, &count);
    glGetProgramiv(p->program, GL_ACTIVE_UNIFORM_MAX_LENGTH, &maxLen);
    std::vector<char> buf((size_t)(maxLen > 1 ? maxLen : 1), 0);

    uint32_t offset = 0;
    for (GLint i = 0; i < count; ++i) {
        GLint size = 0; GLenum type = 0; GLsizei len = 0;
        glGetActiveUniform(p->program, (GLuint)i, (GLsizei)buf.size(), &len, &size, &type, buf.data());

        const uint32_t elem = UniformTypeSize(type);
        if (elem == 0) continue;                       // unsupported type
        if (std::strncmp(buf.data(), "gl_", 3) == 0) continue;

        const GLint loc = glGetUniformLocation(p->program, buf.data());
        if (loc < 0) continue;                         // uniform-block member

        std::string name(buf.data(), (size_t)len);
        if (name.size() > 3 && name.compare(name.size() - 3, 3, "[0]") == 0) name.resize(name.size() - 3);

        Moss_GLUniform u;
        u.location = loc;
        u.type     = type;
        u.count    = size;
        u.elemSize = elem;
        u.offset   = offset;
        offset += elem * (uint32_t)size;

        p->uniformLookup[name] = (uint32_t)p->uniforms.size();
        p->uniforms.push_back(u);
    }
    p->shadow.assign(offset, 0);
}

// ---------------------------------------------------------------------------
// Vertex layout
// ---------------------------------------------------------------------------

static bool BuildVertexLayout(Moss_PipelineState* p, const Moss_GLInput* inputs, uint32_t inputCount) {
    GLint maxAttribs = 16;
    glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, &maxAttribs);
    const uint32_t limit = maxAttribs < 16 ? (uint32_t)maxAttribs : 16u;

    uint32_t vOff = 0, iOff = 0, loc = 0;

    auto add = [&](GLint comps, GLenum type, GLboolean norm, bool instanced, uint32_t bytes) -> bool {
        if (loc >= limit) return false;
        Moss_GLVertexAttrib& a = p->attribs[loc];
        a.location   = loc;
        a.components = comps;
        a.type       = type;
        a.normalized = norm;
        a.divisor    = instanced ? 1u : 0u;
        a.offset     = instanced ? iOff : vOff;
        (instanced ? iOff : vOff) += bytes;
        ++loc;
        return true;
    };

    for (uint32_t i = 0; i < inputCount; ++i) {
        bool ok = true;
        switch (inputs[i]) {
        case Moss_GLInput::Position:  ok = add(3, GL_FLOAT,         GL_FALSE, false, 12); break;
        case Moss_GLInput::Normal:    ok = add(3, GL_FLOAT,         GL_FALSE, false, 12); break;
        case Moss_GLInput::TexCoord:  ok = add(2, GL_FLOAT,         GL_FALSE, false, 8);  break;
        case Moss_GLInput::Color:     ok = add(4, GL_UNSIGNED_BYTE, GL_TRUE,  false, 4);  break;
        case Moss_GLInput::InstanceColor:
            ok = add(4, GL_UNSIGNED_BYTE, GL_TRUE, true, 4);
            break;
        case Moss_GLInput::InstanceTransform:
        case Moss_GLInput::InstanceInvTransform:
            for (int c = 0; c < 4 && ok; ++c) ok = add(4, GL_FLOAT, GL_FALSE, true, 16);
            break;
        default: ok = false; break;
        }
        if (!ok) {
            MOSS_ERROR("GL pipeline: vertex inputs exceed %u attribute slots\n", limit);
            return false;
        }
    }
    p->attribCount     = loc;
    p->vertexStride    = vOff;
    p->instanceStride  = iOff;
    return true;
}

void MossGL_PipelineApplyVertexLayout(const Moss_PipelineState* p, GLuint vertexBuffer, GLsizei vertexStride, GLuint instanceBuffer) {
    if (!p) return;
    const GLsizei vStride = vertexStride ? vertexStride : (GLsizei)p->vertexStride;
    const GLsizei iStride = (GLsizei)p->instanceStride;

    for (uint32_t i = 0; i < p->attribCount; ++i) {
        const Moss_GLVertexAttrib& a = p->attribs[i];
        const bool instanced = a.divisor != 0;
        const GLuint buffer = instanced ? instanceBuffer : vertexBuffer;
        if (!buffer) {
            MOSS_ERROR("GL pipeline: no %s buffer supplied for attribute %u\n", instanced ? "instance" : "vertex", a.location);
            glDisableVertexAttribArray(a.location);
            continue;
        }
        glBindBuffer(GL_ARRAY_BUFFER, buffer);
        glEnableVertexAttribArray(a.location);
        glVertexAttribPointer(a.location, a.components, a.type, a.normalized,
                              instanced ? iStride : vStride, (const void*)(uintptr_t)a.offset);
        glVertexAttribDivisor(a.location, a.divisor);
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

// ---------------------------------------------------------------------------
// Create / destroy
// ---------------------------------------------------------------------------

Moss_PipelineState* Moss_CreatePipeline(Moss_GPUDevice* device, const Moss_GLPipelineInfo* info) {
    if (!info || !info->vertexShader || !info->fragmentShader) {
        MOSS_ERROR("GL pipeline: vertex and fragment shaders are required\n");
        return nullptr;
    }
    if (info->inputCount && !info->inputs) {
        MOSS_ERROR("GL pipeline: inputCount set but inputs is null\n");
        return nullptr;
    }

    GLuint program = glCreateProgram();
    glAttachShader(program, info->vertexShader);
    glAttachShader(program, info->fragmentShader);
    glLinkProgram(program);
    glDetachShader(program, info->vertexShader);   // lets the caller delete them
    glDetachShader(program, info->fragmentShader);

    GLint linked = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &linked);
    if (!linked) {
        GLint len = 0;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &len);
        std::vector<char> log((size_t)(len > 1 ? len : 1), 0);
        glGetProgramInfoLog(program, (GLsizei)log.size(), nullptr, log.data());
        MOSS_ERROR("GL program link failed:\n%s\n", log.data());
        glDeleteProgram(program);
        return nullptr;
    }

    Moss_PipelineState* p = new Moss_PipelineState();
    p->device        = device;
    p->program       = program;
    p->topology      = info->topology;
    p->depthTest     = info->depthTest;
    p->depthWrite    = info->depthWrite;
    p->depthFunc     = info->depthFunc;
    p->blending      = info->blendEnable;
    p->blendSrcColor = info->blendSrcColor;
    p->blendDstColor = info->blendDstColor;
    p->blendSrcAlpha = info->blendSrcAlpha;
    p->blendDstAlpha = info->blendDstAlpha;
    p->blendEquation = info->blendEquation;
    p->cullFace      = info->cullEnable;
    p->cullMode      = info->cullFace;
    p->frontFace     = info->frontFace;
    p->wireframe     = info->wireframe;
    p->colorWrite    = info->colorWrite;

    if (!BuildVertexLayout(p, info->inputs, info->inputCount)) {
        glDeleteProgram(program);
        delete p;
        return nullptr;
    }
    ReflectUniforms(p);
    return p;
}

void MossGL_DestroyPipeline(Moss_PipelineState* p) {
    if (!p) return;
    if (sBoundPipeline == p) {
        glUseProgram(0);
        sBoundPipeline = nullptr;
    }
    if (p->program) glDeleteProgram(p->program);
    delete p;
}

GLenum MossGL_PipelineGetTopology(const Moss_PipelineState* p) { return p ? p->topology : GL_TRIANGLES; }

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

// ---- Moss_PipelineDesc adapter ---------------------------------------------
// Moss_PipelineDesc lives in Moss_GPU.h, which was not available when this was
// written. The member and enum names below are ASSUMED (they follow the old
// PipelineState constructor). This function is the only place that touches
// them; adjust it to the real descriptor.
//   desc->vertex_shader / fragment_shader : Moss_Shader*  (shaderHandle = compiled GL shader)
//   desc->inputs, desc->input_count       : EInputDescription list
//   desc->topology   : ETopology::{Triangle, Line}
//   desc->depth_test : EDepthTest::{Off, On}
//   desc->blend_mode : EBlendMode::{Opaque, Alpha, Additive}
//   desc->cull_mode  : ECullMode::{None, Backface, Frontface}
//   desc->fill_mode  : EFillMode::{Solid, Wireframe}
static bool MapInput(EInputDescription in, Moss_GLInput& out) {
    switch (in) {
    case EInputDescription::Position:             out = Moss_GLInput::Position;             return true;
    case EInputDescription::Normal:               out = Moss_GLInput::Normal;               return true;
    case EInputDescription::TexCoord:             out = Moss_GLInput::TexCoord;             return true;
    case EInputDescription::Color:                out = Moss_GLInput::Color;                return true;
    case EInputDescription::InstanceColor:        out = Moss_GLInput::InstanceColor;        return true;
    case EInputDescription::InstanceTransform:    out = Moss_GLInput::InstanceTransform;    return true;
    case EInputDescription::InstanceInvTransform: out = Moss_GLInput::InstanceInvTransform; return true;
    default:                                      return false;
    }
}

Moss_PipelineState* Moss_RendererCreatePipeline(Moss_Renderer* renderer, const Moss_PipelineDesc* desc) {
    if (!renderer || !desc || !desc->vertex_shader || !desc->fragment_shader) {
        MOSS_ERROR("Moss_RendererCreatePipeline: renderer, desc and both shaders are required\n");
        return nullptr;
    }

    std::vector<Moss_GLInput> inputs;
    inputs.reserve(desc->input_count);
    for (uint32_t i = 0; i < desc->input_count; ++i) {
        Moss_GLInput in;
        if (!MapInput(desc->inputs[i], in)) {
            MOSS_ERROR("Moss_RendererCreatePipeline: unsupported vertex input %u\n", i);
            return nullptr;
        }
        inputs.push_back(in);
    }

    Moss_GLPipelineInfo info;
    info.vertexShader   = desc->vertex_shader->shaderHandle;
    info.fragmentShader = desc->fragment_shader->shaderHandle;
    info.inputs         = inputs.data();
    info.inputCount     = (uint32_t)inputs.size();
    info.topology       = desc->topology == ETopology::Line ? GL_LINES : GL_TRIANGLES;

    info.depthTest  = desc->depth_test == EDepthTest::On;
    info.depthWrite = info.depthTest;

    switch (desc->blend_mode) {
    case EBlendMode::Opaque:   info.blendEnable = false; break;
    case EBlendMode::Alpha:    info.blendEnable = true; info.blendSrcColor = GL_SRC_ALPHA; info.blendDstColor = GL_ONE_MINUS_SRC_ALPHA; break;
    case EBlendMode::Additive: info.blendEnable = true; info.blendSrcColor = GL_ONE;       info.blendDstColor = GL_ONE;                  break;
    }
    if (desc->blend_mode == EBlendMode::Additive) { info.blendSrcAlpha = GL_ONE; info.blendDstAlpha = GL_ONE; }

    info.cullEnable = desc->cull_mode != ECullMode::None;
    info.cullFace   = desc->cull_mode == ECullMode::Frontface ? GL_FRONT : GL_BACK;
    info.wireframe  = desc->fill_mode == EFillMode::Wireframe;

    return MossGL_CreatePipeline(Moss_RendererGetGPUDevice(renderer), &info);
}

void Moss_PipelineBind(Moss_PipelineState* pipeline) {
    if (!pipeline) return;

    if (sBoundPipeline != pipeline) glUseProgram(pipeline->program);
    sBoundPipeline = p;

    // Depth
    if (pipeline->depthTest) { glEnable(GL_DEPTH_TEST); glDepthFunc(pipeline->depthFunc); }
    else              { glDisable(GL_DEPTH_TEST); }
    glDepthMask(pipeline->depthWrite ? GL_TRUE : GL_FALSE);

    // Blend
    if (pipeline->blending) {
        glEnable(GL_BLEND);
        glBlendFuncSeparate(pipeline->blendSrcColor, pipeline->blendDstColor, pipeline->blendSrcAlpha, pipeline->blendDstAlpha);
        glBlendEquation(pipeline->blendEquation);
    } else { glDisable(GL_BLEND); }
	/*
	switch (blendMode) {
        case EBlendMode::Opaque: glDisable(GL_BLEND); break;
        case EBlendMode::Alpha: glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); break;
        case EBlendMode::Additive: glEnable(GL_BLEND); glBlendFunc(GL_ONE, GL_ONE); break;
    }
	*/

    // Cull
    if (pipeline->cullFace) { glEnable(GL_CULL_FACE); glCullFace(pipeline->cullMode); glFrontFace(pipeline->frontFace); }
    else             { glDisable(GL_CULL_FACE); }

    glColorMask(p->colorWrite, p->colorWrite, p->colorWrite, p->colorWrite);

#if defined(MOSS_GRAPHICS_OPENGL)
    glPolygonMode(GL_FRONT_AND_BACK, p->wireframe ? GL_LINE : GL_FILL);   // not in GLES
#endif

    UploadDirty(p);
}

void Moss_PipelineUnbind(Moss_PipelineState* p) {
    // Bind applies every state it owns, so there is nothing to restore.
    if (sBoundPipeline == p || !p) {
        glUseProgram(0);
        sBoundPipeline = nullptr;
    }
}

void Moss_PipelineSetUniform(Moss_PipelineState* p, const char* name, const void* data, uint32_t size) {
    if (!p || !name || !data) return;

    auto it = p->uniformLookup.find(name);
    if (it == p->uniformLookup.end()) return;      // optimised out or not present: not an error in GL

    Moss_GLUniform& u = p->uniforms[it->second];
    const uint32_t total = u.elemSize * (uint32_t)u.count;
    const bool exact   = size == total;
    const bool partial = u.count > 1 && size > 0 && size < total && (size % u.elemSize) == 0;
    if (!exact && !partial) {
        MOSS_ERROR("Moss_PipelineSetUniform('%s'): got %u bytes, expected %u\n", name, size, total);
        return;
    }

    uint8_t* dst = p->shadow.data() + u.offset;
    if (u.written && std::memcmp(dst, data, size) == 0) return;   // unchanged

    std::memcpy(dst, data, size);
    u.written = true;
    u.dirty   = true;
    p->anyDirty = true;
}

void Moss_PipelineFlush(Moss_PipelineState* pipeline) {
    if (!pipeline || !pipeline->anyDirty) return;

    if (sBoundPipeline == pipeline) {
        UploadDirty(pipeline);
        return;
    }
    // GL 3.3 has no program-uniform upload without the program in use.
    const GLuint previous = sBoundPipeline ? sBoundPipeline->program : 0;
    glUseProgram(pipeline->program);
    UploadDirty(pipeline);
    glUseProgram(previous);
}

// Returns 1 if the pipeline is usable, 0 otherwise.
int Moss_PipelineValidate(Moss_PipelineState* pipeline) {
    if (!pipeline || !pipeline->program) return 0;

    GLint linked = GL_FALSE;
    glGetProgramiv(pipeline->program, GL_LINK_STATUS, &linked);
    if (!linked) return 0;

    // glValidateProgram checks against current GL state (and needs a VAO in core
    // profile), so it is only meaningful when this pipeline is bound with a VAO.
    GLint vao = 0;
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &vao);
    if (sBoundPipeline == pipeline && vao != 0) {
        glValidateProgram(pipeline->program);
        GLint ok = GL_FALSE;
        glGetProgramiv(pipeline->program, GL_VALIDATE_STATUS, &ok);
        if (!ok) {
            GLint len = 0;
            glGetProgramiv(pipeline->program, GL_INFO_LOG_LENGTH, &len);
            std::vector<char> log((size_t)(len > 1 ? len : 1), 0);
            glGetProgramInfoLog(pipeline->program, (GLsizei)log.size(), nullptr, log.data());
            MOSS_ERROR("GL pipeline validation failed:\n%s\n", log.data());
            return 0;
        }
    }
    return 1;
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