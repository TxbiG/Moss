#ifndef MOSS_SHADER_VK_H
#define MOSS_SHADER_VK_H

#include <Moss/Renderer/Shader.h>

/*! [WARNING]: Shaders dont support SPIR-V. */
class PixelShaderVK {
public:
    PixelShaderVK(const std::vector<char>& code) {
        VkShaderModuleCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        createInfo.codeSize = code.size();
        createInfo.pCode = reinterpret_cast<const uint32_t*>(code.data());

        return vkCreateShaderModule(renderer->GetDevice(), &createInfo, nullptr, frag) == VK_SUCCESS;
    }
    ~PixelShaderVK() { vkDestroyShaderModule(renderer->GetDevice(), frag, nullptr); };

    VkPipelineShaderStageCreateInfo GetInfo() {
        VkPipelineShaderStageCreateInfo stageInfo{};
        stageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        stageInfo.module = frag;
        stageInfo.pName = "main";
    }
private:
    Moss_RendererVK renderer;
    VkShaderModule frag  = VK_NULL_HANDLE;
};

class VertexShaderVK {
public:
    VertexShaderVK(const std::vector<char>& code) {
        VkShaderModuleCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        createInfo.codeSize = code.size();
        createInfo.pCode = reinterpret_cast<const uint32_t*>(code.data());

        return vkCreateShaderModule(renderer->GetDevice(), &createInfo, nullptr, shaderModule) == VK_SUCCESS;
    }
    ~VertexShaderVK() { vkDestroyShaderModule(renderer->GetDevice(), vert, nullptr); }

    VkPipelineShaderStageCreateInfo GetInfo() {
        VkPipelineShaderStageCreateInfo stageInfo{};
        stageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
        stageInfo.module = vert;
        stageInfo.pName = "main";
    }
private:
    Moss_RendererVK renderer;
    VkShaderModule vert  = VK_NULL_HANDLE;
};

class ComputeShaderVK  {
    ComputeShaderVK(const std::vector<char>& code) {
        VkShaderModuleCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        createInfo.codeSize = code.size();
        createInfo.pCode = reinterpret_cast<const uint32_t*>(code.data());

        return vkCreateShaderModule(renderer->GetDevice(), &createInfo, nullptr, shaderModule) == VK_SUCCESS;
    }
    ~ComputeShaderVK() { vkDestroyShaderModule(renderer->GetDevice(), compute, nullptr); }

    VkPipelineShaderStageCreateInfo GetInfo() {
        VkPipelineShaderStageCreateInfo stageInfo{};
        stageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
        stageInfo.module = compute;
        stageInfo.pName = "main";
    }

private:
    Moss_RendererVK renderer;
    VkShaderModule compute  = VK_NULL_HANDLE;
};

/*class GeometryShaderVK     {};

class TessCtrlShaderVK {
    TessCtrlShaderVK();
    ~TessCtrlShaderVK();

    VkPipelineShaderStageCreateInfo GetInfo() const {
        VkPipelineShaderStageCreateInfo stageInfo{};
        stageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stageInfo.stage = VK_SHADER_STAGE_TESSELLATION_CONTROL_BIT;
        stageInfo.module = TessCtrlShader;
        stageInfo.pName = "main";
        return stageInfo;
    }
private:
    Moss_RendererVK renderer;
    VkShaderModule compute  = VK_NULL_HANDLE;
};

class TessEvalShaderVK {
    VkPipelineShaderStageCreateInfo GetInfo() const {
        VkPipelineShaderStageCreateInfo stageInfo{};
        stageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stageInfo.stage = VK_SHADER_STAGE_TESSELLATION_EVALUATION_BIT;
        stageInfo.module = TessEvalShader;
        stageInfo.pName = "main";
        return stageInfo;
    }
private:
    Moss_RendererVK renderer;
    VkShaderModule compute  = VK_NULL_HANDLE;
};*/

#endif // MOSS_SHADER_VK_H