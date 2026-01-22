#ifndef FREEBLOACKALLOC_H
#define FREEBLOACKALLOC_H

#include <cstdlib>
#include <cassert>

// Low-level C-style allocator
struct FreeBlock {
    FreeBlock* next;
};

struct FreeBlockAllocator {
    FreeBlock* freeList;
    size_t blockSize;
    size_t capacity;
    char* buffer;

    void init(size_t blockSize, size_t blockCount) {
        this->blockSize = blockSize;
        this->capacity = blockSize * blockCount;
        buffer = (char*)malloc(this->capacity);

        // Initialize freelist
        freeList = (FreeBlock*)buffer;
        FreeBlock* current = freeList;
        for (size_t i = 1; i < blockCount; ++i) {
            current->next = (FreeBlock*)(buffer + i * blockSize);
            current = current->next;
        }
        current->next = nullptr;
    }

    void* allocate() {
        if (!freeList) return nullptr; // No free blocks
        void* block = freeList;
        freeList = freeList->next;
        return block;
    }

    void deallocate(void* ptr) {
        FreeBlock* block = (FreeBlock*)ptr;
        block->next = freeList;
        freeList = block;
    }

    void destroy() { free(buffer); }
};

// C++ Wrapper
class FreeBlockAllocatorWrapper {
    FreeBlockAllocator allocator;
public:
    FreeBlockAllocatorWrapper(size_t blockSize, size_t blockCount) {
        allocator.init(blockSize, blockCount);
    }
    ~FreeBlockAllocatorWrapper() { allocator.destroy(); }

    void* allocate() { return allocator.allocate(); }
    void deallocate(void* ptr) { allocator.deallocate(ptr); }
};
#endif // FREEBLOACKALLOC_H