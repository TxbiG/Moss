#include <Moss/Renderer/Renderer_intern.h>


typedef struct Moss_PipelineState {
    ID3D12Device* device;
    ID3D12PipelineState* pipelineState;
    ID3D12Resource* constantBuffer;   // Constant buffer for uniforms
    UINT64 constantBufferSize;
} Moss_PipelineState;



Moss_PipelineState* Moss_RendererCreatePipeline(Moss_Renderer* renderer, const Moss_PipelineDesc* desc) {

}


void Moss_PipelineSetUniform(Moss_PipelineState* pipeline, const char* name, const void* data, uint32_t size) {
    if (!pipeline || !pipeline->constantBuffer) return;

    void* mappedData = NULL;
    D3D12_RANGE readRange = {0, 0};
    pipeline->constantBuffer->Map(0, &readRange, &mappedData);
    memcpy(mappedData, data, size);
    pipeline->constantBuffer->Unmap(0, NULL);
}


void Moss_PipelineBind(Moss_PipelineState* pipeline) { }
void Moss_PipelineUnbind(Moss_PipelineState* pipeline) { }
void Moss_PipelineFlush(Moss_PipelineState* pipeline) { }
int Moss_PipelineValidate(Moss_PipelineState* pipeline) { }