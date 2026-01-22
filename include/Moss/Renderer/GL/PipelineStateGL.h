#ifndef MOSS_PIPELINE_GL_H
#define MOSS_PIPELINE_GL_H

#include <vector>
#include <Moss/Renderer/PipelineState.h>
#include <Moss/Renderer/GL/glad.h>
#include <Moss/Renderer/GL/ShaderGL.h>

struct Moss_Renderer;
class VertexShaderGL;
class PixelShaderGL;

class PipelineStateGL : public PipelineState
{
public:
	PipelineStateGL(Moss_Renderer* inRenderer, 
        const VertexShaderGL* inVertexShader, const EInputDescription* inInputDescription, uint32_t inInputDescriptionCount, 
        const PixelShaderGL* inPixelShader, EDrawPass inDrawPass, EFillMode inFillMode, ETopology inTopology, EDepthTest inDepthTest, EBlendMode inBlendMode, ECullMode inCullMode);

	~PipelineStateGL() { if (mProgramID) glDeleteProgram(mProgramID); };

	void bind() override;

	void unbind() { glUseProgram(0); }

	GLenum GetGLPrimitive() const;
	GLuint GetProgramID() const { return mProgramID; }


	void SetUniform(const char* name, const float value);
    void SetUniform(const char* name, const int value);

    void SetUniform(const char* name, const Color& value);
    //void SetUniform(const char* name, const Texture& value);          // 1D, 2D, 3D, Cube, etc.

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
    void SetUniform(const char* name, unsigned int value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param value0 uint32 type. @param value1 uint32 type. */
    void SetUniform(const char* name, unsigned int value0, unsigned int value1);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param value0 uint32 type. @param value1 uint32 type. @param value2 uint32 type. */
    void SetUniform(const char* name, unsigned int value0, unsigned int value1, unsigned int value2);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param value0 uint32 type. @param value1 uint32 type. @param value2 uint32 type. @param value3 uint32 type. */
    void SetUniform(const char* name, unsigned int value0, unsigned int value1, unsigned int value2, unsigned int value3);

    // Unsigned Integer Array Uniforms
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count int type. @param value uint32 type. */
    //void SetUniform(const char* name, int count, uint32* value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count int type. @param value uint32 type. */
    //void SetUniform(const char* name, int count, uint32* value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count int type. @param value uint32 type. */
    //void SetUniform(const char* name, int count, uint32* value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count int type. @param value uint32 type. */
    //void SetUniform(const char* name, int count, uint32* value);

    // Float Array Uniforms
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count int type. @param value float* type. */
    //void SetUniform(const char* name, int count, const float* value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count int type. @param value float* type. */
    //void SetUniform(const char* name, int count, const float* value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count int type. @param value float* type. */
    //void SetUniform(const char* name, int count, const float* value);
    /*! @brief Tell ShaderGL to get uniform content. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count int type. @param value float* type. */
    //void SetUniform(const char* name, int count, const float* value);

    // Integer Array Uniforms
    /*! @brief Tell OpenGL to get ShaderGL uniform content. Can be used for Sampler types too. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count. @param value const int32 type. */
    //void SetUniform(const char* name, int count, const int* value);
    /*! @brief Tell OpenGL to get ShaderGL uniform content. Can be used for Sampler types too. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count. @param value const int32 type. */
    //void SetUniform(const char* name, int count, const int* value);
    /*! @brief Tell OpenGL to get ShaderGL uniform content. Can be used for Sampler types too. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count. @param value const int32 type. */
    //void SetUniform(const char* name, int count, const int* value);
    /*! @brief Tell OpenGL to get ShaderGL uniform content. Can be used for Sampler types too. @param ShaderGL ShaderGL struct. @param name Name of uniform. @param count. @param value const int32 type. */
    //void SetUniform(const char* name, int count, const int* value);
    

    void SetUniformBlock(const std::string& blockName, const void* data, size_t size) override
	{
	}

    //FlushUniforms

    void SetUniformBuffer(const std::string& blockName, const void* data, size_t size) override {
        GLuint blockIndex = glGetUniformBlockIndex(mProgramID, blockName.c_str());
		if (blockIndex == GL_INVALID_INDEX) return;

		GLuint ubo;
		if (uniformBlocks.find(blockName) == uniformBlocks.end()) {
			glGenBuffers(1, &ubo);
			uniformBlocks[blockName] = ubo;
		} else {
			ubo = uniformBlocks[blockName];
		}

		glBindBuffer(GL_UNIFORM_BUFFER, ubo);
		glBufferData(GL_UNIFORM_BUFFER, size, data, GL_DYNAMIC_DRAW);
		glBindBufferBase(GL_UNIFORM_BUFFER, blockIndex, ubo);
    }

    //FlushUniformBuffers
private:
	Moss_Renderer* renderer = nullptr;
	const VertexShaderGL* vertexShader = nullptr;
	const PixelShaderGL* pixelShader = nullptr;
	std::vector<EInputDescription> inputDescription;

	EDrawPass drawPass = EDrawPass::Normal;
	EFillMode fillMode = EFillMode::Solid;
	ETopology topology = ETopology::Triangle;
	EDepthTest depthTest = EDepthTest::On;
	EBlendMode blendMode = EBlendMode::Write;
	ECullMode cullMode = ECullMode::Backface;

	GLuint mProgramID;
	std::unordered_map<std::string, GLint> mUniformLocations;	// Cache the uniforms
};
#endif //MOSS_PIPELINE_GL_H