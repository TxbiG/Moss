#include <Moss/Moss_GPU.h>
#include <Moss/Moss_Renderer.h>

#include <cmath>
#include <cstring>
#include <vector>

#if __has_include("meshoptomizer/meshoptimizer.h")
#include "meshoptomizer/meshoptimizer.h"
#define MOSS_GPU_EXTERNAL_HAS_MESHOPTIMIZER 1
#endif

#if __has_include("smol-v/smolv.h")
#include "smol-v/smolv.h"
#define MOSS_GPU_EXTERNAL_HAS_SMOLV 1
#endif

#if __has_include("ffx-fsr2/ffx_fsr2.h")
#include "ffx-fsr2/ffx_fsr2.h"
#define MOSS_GPU_EXTERNAL_HAS_FSR2 1
#endif

#if __has_include("ffx-fsr/ffx_fsr1.h")
#define MOSS_GPU_EXTERNAL_HAS_FSR1 1
#endif

#if defined(MOSS_USE_VULKAN) || defined(MOSS_GRAPHICS_VULKAN)
#if __has_include("vk_mem_alloc.h") || __has_include("VulkanMemoryAllocator/include/vk_mem_alloc.h")
#define MOSS_GPU_EXTERNAL_HAS_VMA 1
#endif
#endif

static uint32_t Moss_FloatBits(float value) {
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

uint32_t Moss_GPUGetExternalFeatureMask(void) {
    uint32_t mask = 0;
#if defined(MOSS_GPU_EXTERNAL_HAS_FSR1)
    mask |= MOSS_GPU_EXTERNAL_FEATURE_FSR1;
#endif
#if defined(MOSS_GPU_EXTERNAL_HAS_FSR2)
    mask |= MOSS_GPU_EXTERNAL_FEATURE_FSR2;
#endif
#if defined(MOSS_GPU_EXTERNAL_HAS_MESHOPTIMIZER)
    mask |= MOSS_GPU_EXTERNAL_FEATURE_MESHOPTIMIZER;
#endif
#if defined(MOSS_GPU_EXTERNAL_HAS_SMOLV)
    mask |= MOSS_GPU_EXTERNAL_FEATURE_SMOLV;
#endif
#if defined(MOSS_GPU_EXTERNAL_HAS_VMA)
    mask |= MOSS_GPU_EXTERNAL_FEATURE_VULKAN_MEMORY_ALLOCATOR;
#endif
    return mask;
}

bool Moss_GPUExternalFeatureAvailable(Moss_GPUExternalFeature feature) {
    return (Moss_GPUGetExternalFeatureMask() & static_cast<uint32_t>(feature)) != 0;
}

void Moss_GPUGetExternalFeatures(Moss_GPUExternalFeatures* out_features) {
    if (!out_features) return;
    const uint32_t mask = Moss_GPUGetExternalFeatureMask();
    out_features->fsr1 = (mask & MOSS_GPU_EXTERNAL_FEATURE_FSR1) != 0;
    out_features->fsr2 = (mask & MOSS_GPU_EXTERNAL_FEATURE_FSR2) != 0;
    out_features->meshoptimizer = (mask & MOSS_GPU_EXTERNAL_FEATURE_MESHOPTIMIZER) != 0;
    out_features->smolv = (mask & MOSS_GPU_EXTERNAL_FEATURE_SMOLV) != 0;
    out_features->vulkan_memory_allocator = (mask & MOSS_GPU_EXTERNAL_FEATURE_VULKAN_MEMORY_ALLOCATOR) != 0;
}

const char* Moss_GPUExternalFeatureName(Moss_GPUExternalFeature feature) {
    switch (feature) {
        case MOSS_GPU_EXTERNAL_FEATURE_FSR1: return "FidelityFX Super Resolution 1";
        case MOSS_GPU_EXTERNAL_FEATURE_FSR2: return "FidelityFX Super Resolution 2";
        case MOSS_GPU_EXTERNAL_FEATURE_MESHOPTIMIZER: return "meshoptimizer";
        case MOSS_GPU_EXTERNAL_FEATURE_SMOLV: return "SMOL-V";
        case MOSS_GPU_EXTERNAL_FEATURE_VULKAN_MEMORY_ALLOCATOR: return "Vulkan Memory Allocator";
        default: return "Unknown";
    }
}

void Moss_FSR1BuildEasuConstants(uint32_t out_constants16[16], float input_viewport_width, float input_viewport_height, float input_width, float input_height, float output_width, float output_height) {
    if (!out_constants16 || input_width <= 0.0f || input_height <= 0.0f || output_width <= 0.0f || output_height <= 0.0f) return;

    out_constants16[0] = Moss_FloatBits(input_viewport_width / output_width);
    out_constants16[1] = Moss_FloatBits(input_viewport_height / output_height);
    out_constants16[2] = Moss_FloatBits(0.5f * input_viewport_width / output_width - 0.5f);
    out_constants16[3] = Moss_FloatBits(0.5f * input_viewport_height / output_height - 0.5f);

    out_constants16[4] = Moss_FloatBits(1.0f / input_width);
    out_constants16[5] = Moss_FloatBits(1.0f / input_height);
    out_constants16[6] = Moss_FloatBits(1.0f / input_width);
    out_constants16[7] = Moss_FloatBits(-1.0f / input_height);

    out_constants16[8] = Moss_FloatBits(-1.0f / input_width);
    out_constants16[9] = Moss_FloatBits(2.0f / input_height);
    out_constants16[10] = Moss_FloatBits(1.0f / input_width);
    out_constants16[11] = Moss_FloatBits(2.0f / input_height);

    out_constants16[12] = Moss_FloatBits(0.0f / input_width);
    out_constants16[13] = Moss_FloatBits(4.0f / input_height);
    out_constants16[14] = 0;
    out_constants16[15] = 0;
}

float Moss_FSR2GetUpscaleRatio(Moss_FSR2QualityMode quality) {
#if defined(MOSS_GPU_EXTERNAL_HAS_FSR2)
    return ffxFsr2GetUpscaleRatioFromQualityMode(static_cast<FfxFsr2QualityMode>(quality));
#else
    switch (quality) {
        case MOSS_FSR2_QUALITY: return 1.5f;
        case MOSS_FSR2_BALANCED: return 1.7f;
        case MOSS_FSR2_PERFORMANCE: return 2.0f;
        case MOSS_FSR2_ULTRA_PERFORMANCE: return 3.0f;
        default: return 1.0f;
    }
#endif
}

bool Moss_FSR2GetRenderResolution(uint32_t display_width, uint32_t display_height, Moss_FSR2QualityMode quality, Moss_FSR2RenderResolution* out_resolution) {
    if (!out_resolution || display_width == 0 || display_height == 0) return false;
#if defined(MOSS_GPU_EXTERNAL_HAS_FSR2)
    uint32_t render_width = 0;
    uint32_t render_height = 0;
    if (ffxFsr2GetRenderResolutionFromQualityMode(&render_width, &render_height, display_width, display_height, static_cast<FfxFsr2QualityMode>(quality)) != FFX_OK) return false;
    out_resolution->width = render_width;
    out_resolution->height = render_height;
    return true;
#else
    const float ratio = Moss_FSR2GetUpscaleRatio(quality);
    out_resolution->width = static_cast<uint32_t>(std::ceil(display_width / ratio));
    out_resolution->height = static_cast<uint32_t>(std::ceil(display_height / ratio));
    return true;
#endif
}

int32_t Moss_FSR2GetJitterPhaseCount(int32_t render_width, int32_t display_width) {
#if defined(MOSS_GPU_EXTERNAL_HAS_FSR2)
    return ffxFsr2GetJitterPhaseCount(render_width, display_width);
#else
    if (render_width <= 0 || display_width <= 0) return 1;
    return static_cast<int32_t>(std::ceil(8.0f * static_cast<float>(display_width * display_width) / static_cast<float>(render_width * render_width)));
#endif
}

bool Moss_FSR2GetJitterOffset(float* out_x, float* out_y, int32_t index, int32_t phase_count) {
    if (!out_x || !out_y || phase_count <= 0) return false;
#if defined(MOSS_GPU_EXTERNAL_HAS_FSR2)
    return ffxFsr2GetJitterOffset(out_x, out_y, index, phase_count) == FFX_OK;
#else
    const float phase = static_cast<float>((index % phase_count) + 1);
    *out_x = phase / static_cast<float>(phase_count) - 0.5f;
    *out_y = 0.5f - phase / static_cast<float>(phase_count);
    return true;
#endif
}

size_t Moss_MeshGenerateVertexRemap(uint32_t* destination, const uint32_t* indices, size_t index_count, const void* vertices, size_t vertex_count, size_t vertex_size) {
#if defined(MOSS_GPU_EXTERNAL_HAS_MESHOPTIMIZER)
    if (!destination || !vertices || vertex_count == 0 || vertex_size == 0) return 0;
    return meshopt_generateVertexRemap(destination, indices, index_count, vertices, vertex_count, vertex_size);
#else
    (void)destination; (void)indices; (void)index_count; (void)vertices; (void)vertex_count; (void)vertex_size;
    return 0;
#endif
}

bool Moss_MeshOptimizeVertexCache(uint32_t* destination, const uint32_t* indices, size_t index_count, size_t vertex_count) {
#if defined(MOSS_GPU_EXTERNAL_HAS_MESHOPTIMIZER)
    if (!destination || !indices || index_count == 0 || vertex_count == 0) return false;
    meshopt_optimizeVertexCache(destination, indices, index_count, vertex_count);
    return true;
#else
    (void)destination; (void)indices; (void)index_count; (void)vertex_count;
    return false;
#endif
}

bool Moss_MeshOptimizeOverdraw(uint32_t* destination, const uint32_t* indices, size_t index_count, const float* vertex_positions, size_t vertex_count, size_t vertex_positions_stride, float threshold) {
#if defined(MOSS_GPU_EXTERNAL_HAS_MESHOPTIMIZER)
    if (!destination || !indices || !vertex_positions || index_count == 0 || vertex_count == 0 || vertex_positions_stride == 0) return false;
    meshopt_optimizeOverdraw(destination, indices, index_count, vertex_positions, vertex_count, vertex_positions_stride, threshold);
    return true;
#else
    (void)destination; (void)indices; (void)index_count; (void)vertex_positions; (void)vertex_count; (void)vertex_positions_stride; (void)threshold;
    return false;
#endif
}

size_t Moss_MeshOptimizeVertexFetch(void* destination, uint32_t* indices, size_t index_count, const void* vertices, size_t vertex_count, size_t vertex_size) {
#if defined(MOSS_GPU_EXTERNAL_HAS_MESHOPTIMIZER)
    if (!destination || !indices || !vertices || index_count == 0 || vertex_count == 0 || vertex_size == 0) return 0;
    return meshopt_optimizeVertexFetch(destination, indices, index_count, vertices, vertex_count, vertex_size);
#else
    (void)destination; (void)indices; (void)index_count; (void)vertices; (void)vertex_count; (void)vertex_size;
    return 0;
#endif
}

bool Moss_SMOLVEncode(const void* spirv_data, size_t spirv_size, void* out_smolv_data, size_t* inout_smolv_size, uint32_t flags) {
#if defined(MOSS_GPU_EXTERNAL_HAS_SMOLV)
    if (!spirv_data || spirv_size == 0 || !inout_smolv_size) return false;
    smolv::ByteArray encoded;
    if (!smolv::Encode(spirv_data, spirv_size, encoded, flags)) return false;
    const size_t required = encoded.size();
    if (!out_smolv_data || *inout_smolv_size < required) {
        *inout_smolv_size = required;
        return false;
    }
    std::memcpy(out_smolv_data, encoded.data(), required);
    *inout_smolv_size = required;
    return true;
#else
    (void)spirv_data; (void)spirv_size; (void)out_smolv_data; (void)inout_smolv_size; (void)flags;
    return false;
#endif
}

size_t Moss_SMOLVGetDecodedSize(const void* smolv_data, size_t smolv_size) {
#if defined(MOSS_GPU_EXTERNAL_HAS_SMOLV)
    if (!smolv_data || smolv_size == 0) return 0;
    return smolv::GetDecodedBufferSize(smolv_data, smolv_size);
#else
    (void)smolv_data; (void)smolv_size;
    return 0;
#endif
}

bool Moss_SMOLVDecode(const void* smolv_data, size_t smolv_size, void* out_spirv_data, size_t spirv_size, uint32_t flags) {
#if defined(MOSS_GPU_EXTERNAL_HAS_SMOLV)
    if (!smolv_data || smolv_size == 0 || !out_spirv_data || spirv_size == 0) return false;
    return smolv::Decode(smolv_data, smolv_size, out_spirv_data, spirv_size, flags);
#else
    (void)smolv_data; (void)smolv_size; (void)out_spirv_data; (void)spirv_size; (void)flags;
    return false;
#endif
}

bool Moss_GPUVulkanMemoryAllocatorAvailable(void) {
#if defined(MOSS_GPU_EXTERNAL_HAS_VMA)
    return true;
#else
    return false;
#endif
}

void Moss_RendererSetUpscaler(Moss_Renderer* renderer, Moss_Upscaler upscaler) {
    (void)renderer;
    (void)upscaler;
}

void Moss_RendererSetUpscalerDesc(Moss_Renderer* renderer, const Moss_RendererUpscalerDesc* desc) {
    if (!desc) return;
    Moss_RendererSetUpscaler(renderer, desc->upscaler);
}