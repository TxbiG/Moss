#include <Moss/Renderer/StorageBuffer.h>
#include <vulkan/vulkan.h>

class StorageBufferVK : public StorageBuffer {
public:
    StorageBufferVK(VkDevice device, VkPhysicalDevice physical)  : mDevice(device), mPhysical(physical), mBuffer(VK_NULL_HANDLE), mMemory(VK_NULL_HANDLE) {}

    ~StorageBufferVK() { if (mBuffer) vkDestroyBuffer(mDevice, mBuffer, nullptr) if (mMemory) vkFreeMemory(mDevice, mMemory, nullptr); }

    void Create(uint32_t size, const void* initialData = nullptr) override;
    void Update(uint32_t offset, uint32_t size, const void* data) override;
    void Bind(uint32_t bindingPoint) override; // done via descriptor sets
    void* Map() override;
    void Unmap() override;

private:
    VkDevice mDevice;
    VkPhysicalDevice mPhysical;
    VkBuffer mBuffer;
    VkDeviceMemory mMemory;
    uint32_t mSize;
};