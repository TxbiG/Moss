#pragma once

#include <vulkan/vulkan.h>

struct BufferVK {
	VkBuffer					mBuffer = VK_NULL_HANDLE;
	VkDeviceMemory				mMemory = VK_NULL_HANDLE;
	VkDeviceSize				mOffset = 0;
	VkDeviceSize				mSize = 0;

	VkBufferUsageFlags			mUsage;
	VkMemoryPropertyFlags		mProperties;
	VkDeviceSize				mAllocatedSize;
};

class Moss_Renderer;

/// A binary blob that can be used to pass constants to a shader
class MOSS_API ConstantBufferVK
{
public:
	/// Constructor
										//ConstantBufferVK(Moss_Renderer *inRenderer, VkDeviceSize inBufferSize);
										//~ConstantBufferVK();

	/// Map / unmap buffer (get pointer to data). This will discard all data in the buffer.
	template <typename T> T *			Map()											{ return reinterpret_cast<T *>(MapInternal()); }
	void								Unmap();

	VkBuffer							GetBuffer() const								{ return mBuffer.mBuffer; }

private:
	void *								MapInternal();

	Moss_Renderer*						mRenderer;
	BufferVK							mBuffer;
};