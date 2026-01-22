#ifndef MOSS_PIPELINE_STATE_H
#define MOSS_PIPELINE_STATE_H

typedef enum {
    BLEND_ALPHA = 0,                // Blend textures considering alpha (default)
    BLEND_ADDITIVE,                 // Blend textures adding colors
    BLEND_MULTIPLIED,               // Blend textures multiplying colors
    BLEND_ADD_COLORS,               // Blend textures adding colors (alternative)
    BLEND_SUBTRACT_COLORS,          // Blend textures subtracting colors (alternative)
    BLEND_ALPHA_PREMULTIPLY,        // Blend premultiplied textures considering alpha
    BLEND_CUSTOM,                   // Blend textures using custom src/dst factors (use rlSetBlendFactors())
    BLEND_CUSTOM_SEPARATE           // Blend textures using custom rgb/alpha separate src/dst factors (use rlSetBlendFactorsSeparate())
} BlendMode;

class PipelineState
{
public:
	/// Describes the input layout of the vertex shader
	enum class EInputDescription
	{
		Position,						///< 3 float position
		Color,							///< 4 uint8 color
		Normal,							///< 3 float normal
		TexCoord,						///< 2 float texture coordinate
		InstanceColor,					///< 4 uint8 per instance color
		InstanceTransform,				///< 4x4 float per instance transform
		InstanceInvTransform,			///< 4x4 float per instance inverse transform
	};

	enum class EDrawPass { Shadow, Normal, Lighting, Transparent }; // In which draw pass to use this pipeline state
	enum class ETopology { Triangle, Line, Point };					// The type of topology to emit
	enum class EFillMode { Solid, Wireframe };						// Fill mode of the triangles (<-Should be in Renderer Settings)
	enum class EDepthTest { Off, On };								// If depth write / depth test is on
	enum class EBlendMode { Opaque, Alpha, Additive};			    // How to blend the pixel from the shader in the back buffer
	enum class ECullMode { None, Backface, FrontFace };				// How to cull triangles

    //enum class StencilPass { NONE, LESS, LEQUAL, GREATER, GEQUAL, EQUAL, NOTEQUAL, ALWAYS };Off, ReadEqual, ReadGreaterEqual, ReadWrite, Max

	/// Destructor
	virtual	~PipelineState() = default;

	/// Make this pipeline state active (any primitives rendered after this will use this state)
	virtual void bind() = 0;
	virtual void unbind() = 0;

	virtual void SetUniform(const char* name, const float value) = 0;
    virtual void SetUniform(const char* name, const int value) = 0;

    //void SetUniform(const char* name, const Texture& value);          // 1D, 2D, 3D, Cube, etc.
    virtual void SetUniform(const char* name, const Color& value) = 0;

    // Vector Uniforms
    virtual void SetUniform(const char* name, const Float2& value) = 0;
    virtual void SetUniform(const char* name, const Int2& value) = 0;
    virtual void SetUniform(const char* name, const Float3& value) = 0;
    virtual void SetUniform(const char* name, const Int3& value) = 0;
    virtual void SetUniform(const char* name, const Float4& value) = 0;
    virtual void SetUniform(const char* name, const Int4& value) = 0;

    // Matrix Uniforms
    //virtual void SetUniformMat2(const char* name, const Mat2& value) = 0;
    //virtual void SetUniformMat2x3(const char* name, const Mat2x3& value) = 0;
    //virtual void SetUniformMat2x4(const char* name, const Mat2x4& value) = 0;
    //virtual void SetUniformMat3(const char* name, const Mat3& value) = 0;
    //virtual void SetUniformMat3x2(const char* name, const Mat3x2& value) = 0;
    //virtual void SetUniformMat3x4(const char* name, const Mat3x4& value) = 0;
    virtual void SetUniform(const char* name, const Mat44& value) = 0;
    //virtual void SetUniformMat4x2(const char* name, const Mat4x2& value) = 0;
    //virtual void SetUniformMat4x3(const char *name, const Mat4x3& value) = 0;

    // Unsigned int
    //virtual void SetUniform(const char* name, unsigned int value) = 0;
    //virtual void SetUniform(const char* name, unsigned int value0, unsigned int value1) = 0;
    //virtual void SetUniform(const char* name, unsigned int value0, unsigned int value1, unsigned int value2) = 0;
    //virtual void SetUniform(const char* name, unsigned int value0, unsigned int value1, unsigned int value2, unsigned int value3) = 0;

    // Unsigned Integer Array Uniforms
    //virtual void SetUniformInt(const char* name, int count, uint32* value) = 0;
    //virtual void SetUniformInt2(const char* name, int count, uint32* value) = 0;
    //virtual void SetUniformInt3(const char* name, int count, uint32* value) = 0;
    //virtual void SetUniformInt4(const char* name, int count, uint32* value) = 0;

    // Float Array Uniforms
    //virtual void SetUniformArrayf(const char* name, int count, const float* value) = 0;
    //virtual void SetUniformArrayf2(const char* name, int count, const float* value) = 0;
    //virtual void SetUniformArrayf3(const char* name, int count, const float* value) = 0;
    //virtual void SetUniformArrayf4(const char* name, int count, const float* value) = 0;

    // Integer Array Uniforms
    //virtual void SetUniformArrayi(const char* name, int count, const int* value) = 0;
    //virtual void SetUniformArrayi2(const char* name, int count, const int* value) = 0;
    //virtual void SetUniformArrayi3(const char* name, int count, const int* value) = 0;
    //virtual void SetUniformArrayi4(const char* name, int count, const int* value) = 0;

    virtual void SetUniformBlock(const std::string& blockName, const void* data, size_t size) = 0;
	virtual void FlushUniforms() = 0;
};

#endif // MOSS_PIPELINE_STATE_H