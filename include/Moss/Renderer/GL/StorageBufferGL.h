
#include <Moss/Renderer/StorageBuffer.h>

class StorageBufferGL : public StorageBuffer {
public:
    StorageBufferGL() : mBuffer(0), mSize(0) {}
    ~StorageBufferGL() {
        if (mBuffer) glDeleteBuffers(1, &mBuffer);
    }

    void Create(uint32_t size, const void* initialData = nullptr) override {
        mSize = size;
        glGenBuffers(1, &mBuffer);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, mBuffer);
        glBufferData(GL_SHADER_STORAGE_BUFFER, size, initialData, GL_DYNAMIC_DRAW);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    }

    void Update(uint32_t offset, uint32_t size, const void* data) override {
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, mBuffer);
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, offset, size, data);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    }

    void Bind(uint32_t bindingPoint) override {
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, bindingPoint, mBuffer);
    }

    void* Map() override {
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, mBuffer);
        return glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_WRITE_ONLY);
    }

    void Unmap() override {
        glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    }

private:
    GLuint mBuffer;
    uint32_t mSize;
};
