#include <Moss/Renderer/Shader.h>
#include <Moss/Renderer/gl/glad.h>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>


Shader::Shader(VkDevice device, const std::vector<char>& spirvVertex, const std::vector<char>& spirvFragment)
    : mDevice(device) {

    // 1. Create shader modules
    auto createModule = [&](const std::vector<char>& code, VkShaderModule& module) {
        VkShaderModuleCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        createInfo.codeSize = code.size();
        createInfo.pCode = reinterpret_cast<const uint32_t*>(code.data());

        if (vkCreateShaderModule(mDevice, &createInfo, nullptr, &module) != VK_SUCCESS) { MOSS_ERROR("Failed to create shader module."); }
    };

    createModule(spirvVertex, mVertModule);
    createModule(spirvFragment, mFragModule);

    // 2. Create uniform buffer
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);

    VkBufferCreateInfo bufferInfo{};
    bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufferInfo.size = bufferSize;
    bufferInfo.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
    bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    if (vkCreateBuffer(mDevice, &bufferInfo, nullptr, &mUniformBuffer) != VK_SUCCESS) {
        MOSS_ERROR("Failed to create uniform buffer.");
    }

    VkMemoryRequirements memRequirements;
    vkGetBufferMemoryRequirements(mDevice, mUniformBuffer, &memRequirements);

    VkMemoryAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocInfo.allocationSize = memRequirements.size;
    allocInfo.memoryTypeIndex = 0; // 👈 We'll need to set this properly

    // You must implement this function for your platform!
    extern uint32_t FindMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties);
    allocInfo.memoryTypeIndex = FindMemoryType(memRequirements.memoryTypeBits,
                                               VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    if (vkAllocateMemory(mDevice, &allocInfo, nullptr, &mUniformMemory) != VK_SUCCESS) { MOSS_ERROR("Failed to allocate uniform buffer memory."); }

    vkBindBufferMemory(mDevice, mUniformBuffer, mUniformMemory, 0);

    // 3. Create descriptor set layout
    VkDescriptorSetLayoutBinding uboLayoutBinding{};
    uboLayoutBinding.binding = 0;
    uboLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    uboLayoutBinding.descriptorCount = 1;
    uboLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    uboLayoutBinding.pImmutableSamplers = nullptr;

    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 1;
    layoutInfo.pBindings = &uboLayoutBinding;

    if (vkCreateDescriptorSetLayout(mDevice, &layoutInfo, nullptr, &mDescriptorSetLayout) != VK_SUCCESS) { MOSS_ERROR("Failed to create descriptor set layout."); }

    // 4. Create descriptor pool
    VkDescriptorPoolSize poolSize{};
    poolSize.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSize.descriptorCount = 1;

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.poolSizeCount = 1;
    poolInfo.pPoolSizes = &poolSize;
    poolInfo.maxSets = 1;

    if (vkCreateDescriptorPool(mDevice, &poolInfo, nullptr, &mDescriptorPool) != VK_SUCCESS) { MOSS_ERROR("Failed to create descriptor pool."); }

    // 5. Allocate descriptor set
    VkDescriptorSetAllocateInfo allocInfoDS{};
    allocInfoDS.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfoDS.descriptorPool = mDescriptorPool;
    allocInfoDS.descriptorSetCount = 1;
    allocInfoDS.pSetLayouts = &mDescriptorSetLayout;

    if (vkAllocateDescriptorSets(mDevice, &allocInfoDS, &mDescriptorSet) != VK_SUCCESS) { MOSS_ERROR("Failed to allocate descriptor set."); }

    // 6. Bind buffer to descriptor set
    VkDescriptorBufferInfo bufferInfoDS{};
    bufferInfoDS.buffer = mUniformBuffer;
    bufferInfoDS.offset = 0;
    bufferInfoDS.range = sizeof(UniformBufferObject);

    VkWriteDescriptorSet descriptorWrite{};
    descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrite.dstSet = mDescriptorSet;
    descriptorWrite.dstBinding = 0;
    descriptorWrite.dstArrayElement = 0;
    descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    descriptorWrite.descriptorCount = 1;
    descriptorWrite.pBufferInfo = &bufferInfoDS;

    vkUpdateDescriptorSets(mDevice, 1, &descriptorWrite, 0, nullptr);
}

Shader::~Shader() {
    if (mUniformMemory) { vkFreeMemory(mDevice, mUniformMemory, nullptr);}
    if (mUniformBuffer) { vkDestroyBuffer(mDevice, mUniformBuffer, nullptr); }
    if (mDescriptorPool) { vkDestroyDescriptorPool(mDevice, mDescriptorPool, nullptr); }
    if (mDescriptorSetLayout) { vkDestroyDescriptorSetLayout(mDevice, mDescriptorSetLayout, nullptr); }
    if (mVertModule) { vkDestroyShaderModule(mDevice, mVertModule, nullptr); }
    if (mFragModule) { vkDestroyShaderModule(mDevice, mFragModule, nullptr); }
}

void Shader::SetUniformf(const char *name, const float value) { mUBO.intVal = value; needsUpdate = true; }
void Shader::SetUniformi(const char *name, const int value) { mUBO.floatVal = value; needsUpdate = true; }
void Shader::SetUniformVec2f(const char *name, const Float2 &value) { mUBO.vec2 = value; needsUpdate = true; }
void Shader::SetUniformVec2i(const char *name, const Int2 &value) { mUBO.int2 = value; needsUpdate = true; }
void Shader::SetUniformVec3f(const char *name, const Float3 &value) { mUBO.vec3 = value; needsUpdate = true; }
void Shader::SetUniformVec3i(const char *name, const Int3 &value) { mUBO.int3 = value; needsUpdate = true; }
void Shader::SetUniformVec4f(const char *name, const Float4 &value) { mUBO.vec4 = value; needsUpdate = true; }
void Shader::SetUniformVec4i(const char *name, const Int4 &value) { mUBO.int4 = value; needsUpdate = true; }

/*
void Shader::SetUniformMat2(const char *name, const Mat2 &value)
{

}

void Shader::SetUniformMat2x3(const char *name, const Mat2x3 &value)
{

}

void Shader::SetUniformMat2x4(const char *name, const Mat2x4 &value)
{

}

void Shader::SetUniformMat3(const char *name, const Mat3 &value)
{
}

void Shader::SetUniformMat3x2(const char *name, const Mat3x2 &value)
{

}

void Shader::SetUniformMat3x4(const char *name, const Mat3x4 &value)
{

}
*/
void Shader::SetUniformMat4(const char *name, const Mat44 &value) { mUBO.mat = value; needsUpdate = true; }
/*
void Shader::SetUniformMat4x2(const char *name, const Mat4x2 &value)
{

}

void Shader::SetUniformMat4x3(const char *name, const Mat4x3& value)
{

}*/

void Shader::SetUniformUint(const char *name, unsigned int value)
{

}

void Shader::SetUniformUint2(const char *name, unsigned int value0, unsigned int value1)
{

}

void Shader::SetUniformUint3(const char *name, unsigned int value0, unsigned int value1, unsigned int value2)
{

}

void Shader::SetUniformUint4(const char *name, unsigned int value0, unsigned int value1, unsigned int value2, unsigned int value3)
{

}

void Shader::SetUniformInt(const char *name, int count, uint32 *value)
{

}

void Shader::SetUniformInt2(const char *name, int count, uint32 *value)
{

}

void Shader::SetUniformInt3(const char *name, int count, uint32 *value)
{

}

void Shader::SetUniformInt4(const char *name, int count, uint32 *value)
{

}
void Shader::SetUniformArrayf(const char *name, int count, const float *value)
{

}
void Shader::SetUniformArrayf2(const char *name, int count, const float *value)
{

}
void Shader::SetUniformArrayf3(const char *name, int count, const float *value)
{

}
void Shader::SetUniformArrayf4(const char *name, int count, const float *value)
{

}

void Shader::SetUniformArrayi(const char *name, int count, const int *value)
{

}

void Shader::SetUniformArrayi2(const char *name, int count, const int *value)
{

}

void Shader::SetUniformArrayi3(const char *name, int count, const int *value)
{

}

void Shader::SetUniformArrayi4(const char *name, int count, const int *value)
{

}