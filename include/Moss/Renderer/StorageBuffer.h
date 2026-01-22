#pragma once

#include <cstdint>
#include <memory>

class StorageBuffer {
public:
    virtual ~StorageBuffer() = default;

    virtual void Create(uint32_t size, const void* initialData = nullptr) = 0;
    virtual void Update(uint32_t offset, uint32_t size, const void* data) = 0;
    virtual void Bind(uint32_t bindingPoint) = 0;
    virtual void* Map() = 0;
    virtual void Unmap() = 0;
};