#include "renderer_vk.h"

Moss_PipelineState* Moss_RendererCreatePipeline(Moss_Renderer* renderer, const Moss_PipelineDesc* desc) {

}


void Moss_PipelineSetUniform(Moss_PipelineState* pipeline, const char* name, const void* data, uint32_t size) {
    if (!pipeline || !pipeline->uniformBuffer) return;

    void* mapped;
    vkMapMemory(pipeline->device, pipeline->uniformMemory, 0, size, 0, &mapped);
    memcpy(mapped, data, size);
    vkUnmapMemory(pipeline->device, pipeline->uniformMemory);
}

void Moss_PipelineBind(Moss_PipelineState* pipeline) { }
void Moss_PipelineUnbind(Moss_PipelineState* pipeline) { }
void Moss_PipelineFlush(Moss_PipelineState* pipeline) { }
int Moss_PipelineValidate(Moss_PipelineState* pipeline) { }