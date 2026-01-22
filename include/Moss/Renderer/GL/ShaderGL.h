#ifndef MOSS_SHADER_GL_H
#define MOSS_SHADER_GL_H


#include <Moss/Renderer/Shader.h>
#include <Moss/Renderer/GL/glad.h>

#include <Moss/Core/Variants/Vector/Float2.h>
#include <Moss/Core/Variants/Vector/Float3.h>
#include <Moss/Core/Variants/Vector/Float4.h>
#include <Moss/Core/Variants/Vector/Int2.h>
#include <Moss/Core/Variants/Vector/Int3.h>
#include <Moss/Core/Variants/Vector/Int4.h>
#include <Moss/Core/Variants/Color.h>

struct Moss_Renderer;

/*
class VertexShaderGL : public VertexShader
{
public:
	VertexShaderGL(const char* vertexFile);
    GLuint GetID() { return ID; }

private:
	GLuint		ID;										///< The compiled shader
};

class PixelShaderGL : public PixelShader
{
public:
	PixelShaderGL(const char* fragFile);
    GLuint GetID() { return ID; }

private:
	GLuint		ID;										///< The compiled shader
};

class ComputeShaderGL : public ComputeShader
{
public:
	ComputeShaderGL(const char* computeFile) : mShader(inShader) {
        unsigned int compute;
        // compute shader
        compute = glCreateShader(GL_COMPUTE_SHADER);
        glShaderSource(compute, 1, &cShaderCode, NULL);
        glCompileShader(compute);
        checkCompileErrors(compute, "COMPUTE");

        // shader Program
        ID = glCreateProgram();
        glAttachShader(ID, compute);
        glLinkProgram(ID);
        checkCompileErrors(ID, "PROGRAM");
     }


    void bind() {
        glUseProgram(computeProgram);
		glDispatchCompute(ceil(SCREEN_WIDTH / 8), ceil(SCREEN_HEIGHT / 4), 1);
		glMemoryBarrier(GL_ALL_BARRIER_BITS);
    }
    GLuint GetID() { return ID; }

private:
	GLuint		ID;										///< The compiled shader
};
*/
class [[nodiscard]] MOSS_API ShaderGL
{
public:
    ShaderGL() = default;
    ShaderGL(const char* vertexFile, const char* fragmentFile, bool isRawGLSL); // vert and frag
    ShaderGL(const char* vertexFile, const char* fragmentFile, const char* geometryFile, bool isRawGLSL);  // geomatry
    ~ShaderGL();

    void bind();
    void unbind();
    GLuint ID() { return mProgramID; }

    void SetUniform(const char* name, const float value);
    void SetUniform(const char* name, const int value);

    void SetUniform(const char* name, const Color& value);

    // Vector Uniforms
    void SetUniform(const char* name, const Float2& value);
    void SetUniform(const char* name, const Int2& value);
    void SetUniform(const char* name, const Float3& value);
    void SetUniform(const char* name, const Int3& value);
    void SetUniform(const char* name, const Float4& value);
    void SetUniform(const char* name, const Int4& value);

    // Matrix Uniforms
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param value Mat2 type. */
    //void SetUniformMat2(const char* name, const Mat2& value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param value Mat2x3 type. */
    //void SetUniformMat2x3(const char* name, const Mat2x3& value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param value Mat2x4 type. */
    //void SetUniformMat2x4(const char* name, const Mat2x4& value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param value Mat3 type. */
    //void SetUniformMat3(const char* name, const Mat3& value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param value Mat3x2 type. */
    //void SetUniformMat3x2(const char* name, const Mat3x2& value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param value Mat3x4 type. */
    ///void SetUniformMat3x4(const char* name, const Mat3x4& value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param value Mat4 type. */
    void SetUniform(const char* name, const Mat44& value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param value Mat4x2 type. */
    //void SetUniformMat4x2(const char* name, const Mat4x2& value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param value Mat4x3 type. */
    //void SetUniformMat4x3(const char *name, const Mat4x3& value);
    // Unsigned int
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param value uint32 type. */
    //void SetUniform(const char* name, unsigned int value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param value0 uint32 type. @param value1 uint32 type. */
    //void SetUniform(const char* name, unsigned int value0, unsigned int value1);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param value0 uint32 type. @param value1 uint32 type. @param value2 uint32 type. */
    //void SetUniform(const char* name, unsigned int value0, unsigned int value1, unsigned int value2);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param value0 uint32 type. @param value1 uint32 type. @param value2 uint32 type. @param value3 uint32 type. */
    //void SetUniform(const char* name, unsigned int value0, unsigned int value1, unsigned int value2, unsigned int value3);

    // Unsigned Integer Array Uniforms
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count int type. @param value uint32 type. */
    //void SetUniformInt(const char* name, int count, uint32* value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count int type. @param value uint32 type. */
    //void SetUniformInt2(const char* name, int count, uint32* value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count int type. @param value uint32 type. */
    //void SetUniformInt3(const char* name, int count, uint32* value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count int type. @param value uint32 type. */
    //void SetUniformInt4(const char* name, int count, uint32* value);

    // Float Array Uniforms
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count int type. @param value float* type. */
    //void SetUniformArrayf(const char* name, int count, const float* value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count int type. @param value float* type. */
    //void SetUniformArrayf2(const char* name, int count, const float* value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count int type. @param value float* type. */
    //void SetUniformArrayf3(const char* name, int count, const float* value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count int type. @param value float* type. */
    //void SetUniformArrayf4(const char* name, int count, const float* value);

    // Integer Array Uniforms
    /*! @brief Tell OpenGL to get ShaderGL uniform content. Can be used for Sampler types too. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count. @param value const int32 type. */
    //void SetUniformArrayi(const char* name, int count, const int* value);
    /*! @brief Tell OpenGL to get ShaderGL uniform content. Can be used for Sampler types too. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count. @param value const int32 type. */
    //void SetUniformArrayi2(const char* name, int count, const int* value);
    /*! @brief Tell OpenGL to get ShaderGL uniform content. Can be used for Sampler types too. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count. @param value const int32 type. */
    //void SetUniformArrayi3(const char* name, int count, const int* value);
    /*! @brief Tell OpenGL to get ShaderGL uniform content. Can be used for Sampler types too. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count. @param value const int32 type. */
    //void SetUniformArrayi4(const char* name, int count, const int* value);

private:
    GLint getUniformLocation(const char* name) {
        auto it = uniformLocationCache.find(name);
        if (it != uniformLocationCache.end())
            return it->second;
        GLint location = glGetUniformLocation(this->mProgramID, name);
        uniformLocationCache[name] = location;
        return location;
    }
    GLuint mProgramID;
    GLuint mVertexShaderID;
    GLuint mFragmentShaderID;
    GLuint mGeometryShaderID;

    std::unordered_map<std::string, GLint> uniformLocationCache;
};
#endif // MOSS_SHADER_GL_H