


#include <Moss/Renderer/VK/ConstantBufferVK.h>
#include <Moss/Renderer/VK/Renderer_VK.h>
//#include <Renderer/VK/FatalErrorIfFailedVK.h>

//ConstantBufferVK::ConstantBufferVK(Moss_Renderer *inRenderer, VkDeviceSize inBufferSize) : mRenderer(inRenderer)
//{ }//mRenderer->CreateBuffer(inBufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, mBuffer); }

//ConstantBufferVK::~ConstantBufferVK() { mRenderer->FreeBuffer(mBuffer); }

void *ConstantBufferVK::MapInternal()
{
	void *data = nullptr;
	//FatalErrorIfFailed(
    vkMapMemory(mRenderer->m_device, mBuffer.mMemory, mBuffer.mOffset, mBuffer.mSize, 0, &data);//);
	return data;
}

void ConstantBufferVK::Unmap() { vkUnmapMemory(mRenderer->m_device, mBuffer.mMemory); }