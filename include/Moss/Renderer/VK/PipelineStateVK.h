#ifndef MOSS_PIPELINE_VK_H
#define MOSS_PIPELINE_VK_H

#include <Moss/Renderer/PipelineState.h>
#include <Moss/Renderer/VK/ShaderVK.h>

class RendererVK;

/// Vulkan pipeline state object
class PipelineStateVK : public PipelineState
{
public:
	/// Constructor
										PipelineStateVK(RendererVK *inRenderer, const VertexShaderVK *inVertexShader, const EInputDescription *inInputDescription, uint inInputDescriptionCount, const PixelShaderVK *inPixelShader, EDrawPass inDrawPass, EFillMode inFillMode, ETopology inTopology, EDepthTest inDepthTest, EBlendMode inBlendMode, ECullMode inCullMode);
	virtual								~PipelineStateVK() override;

	/// Make this pipeline state active (any primitives rendered after this will use this state)
	virtual void bind() override;

private:
	RendererVK *						mRenderer;
	RefConst<VertexShaderVK>			mVertexShader;
	RefConst<PixelShaderVK>				mPixelShader;

	VkPipeline							mGraphicsPipeline;
};

#endif //MOSS_PIPELINE_VK_H