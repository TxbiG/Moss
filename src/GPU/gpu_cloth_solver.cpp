//                        MIT License
//
//                  Copyright (c) 2026 Toby

#include <Moss/Moss_GPU.h>

#include <new>

struct Moss_GPUClothSolver {
    Moss_ComputePipelineState* pipeline = nullptr;
    bool owns_pipeline = false;
    uint32_t group_size = 64;
};

struct Moss_GPUClothParams {
    uint32_t particle_count = 0;
    uint32_t constraint_count = 0;
    uint32_t iterations = 1;
    float delta_time = 1.0f / 60.0f;
    float damping = 0.01f;
    float stiffness = 1.0f;
    uint32_t _padding0 = 0;
    uint32_t _padding1 = 0;
};

Moss_GPUClothSolver* Moss_GPUClothSolverCreate(Moss_GPUDevice* device, const Moss_GPUClothSolverDesc* desc) {
    if (!device || !desc) {
        return nullptr;
    }

    Moss_ComputePipelineState* pipeline = desc->pipeline;
    bool owns_pipeline = desc->take_pipeline_ownership;

    Moss_GPUShader* temporary_shader = nullptr;
    if (!pipeline && desc->shader.bytecode && desc->shader.bytecode_size > 0) {
        Moss_GPUShaderCreateInfo shader_info = desc->shader;
        shader_info.stage = EShaderStage::COMPUTE;
        temporary_shader = Moss_CreateGPUShader(device, &shader_info);
        if (!temporary_shader) {
            return nullptr;
        }

        Moss_GPUComputePipelineCreateInfo pipeline_info = desc->pipeline_info;
        pipeline_info.compute_shader = temporary_shader;
        if (!pipeline_info.debug_name) {
            pipeline_info.debug_name = "Moss GPU Cloth Solver";
        }
        pipeline = Moss_CreateGPUComputePipeline(device, &pipeline_info);
        Moss_ReleaseGPUShader(device, temporary_shader);
        owns_pipeline = true;
    }

    if (!pipeline) {
        return nullptr;
    }

    Moss_GPUClothSolver* solver = new (std::nothrow) Moss_GPUClothSolver();
    if (!solver) {
        if (owns_pipeline) {
            Moss_ReleaseGPUComputePipeline(device, pipeline);
        }
        return nullptr;
    }

    solver->pipeline = pipeline;
    solver->owns_pipeline = owns_pipeline;
    solver->group_size = desc->group_size > 0 ? desc->group_size : 64;
    return solver;
}

void Moss_GPUClothSolverDestroy(Moss_GPUDevice* device, Moss_GPUClothSolver* solver) {
    if (!solver) {
        return;
    }
    if (device && solver->owns_pipeline && solver->pipeline) {
        Moss_ReleaseGPUComputePipeline(device, solver->pipeline);
    }
    delete solver;
}

bool Moss_GPUClothSolverDispatch(Moss_GPUCommandBuffer* cmd, Moss_GPUClothSolver* solver, const Moss_GPUClothDispatchDesc* desc) {
    if (!cmd || !solver || !solver->pipeline || !desc || !desc->positions || !desc->previous_positions || !desc->velocities || desc->particle_count == 0) {
        return false;
    }

    Moss_GPUStorageBufferReadWriteBinding bindings[4]{};
    uint32_t binding_count = 0;
    bindings[binding_count++] = { desc->positions, 0, desc->positions_size };
    bindings[binding_count++] = { desc->previous_positions, 0, desc->previous_positions_size };
    bindings[binding_count++] = { desc->velocities, 0, desc->velocities_size };
    if (desc->constraints && desc->constraint_count > 0) {
        bindings[binding_count++] = { desc->constraints, 0, desc->constraints_size };
    }

    Moss_GPUClothParams params{};
    params.particle_count = desc->particle_count;
    params.constraint_count = desc->constraint_count;
    params.iterations = desc->iterations > 0 ? desc->iterations : 1;
    params.delta_time = desc->delta_time;
    params.damping = desc->damping;
    params.stiffness = desc->stiffness;

    Moss_BindGPUComputePipeline(cmd, solver->pipeline);
    Moss_BindGPUComputeStorageBuffers(cmd, 0, bindings, binding_count);
    Moss_PushGPUComputeUniformData(cmd, 0, &params, sizeof(params));

    const uint32_t group_size = solver->group_size > 0 ? solver->group_size : 64;
    const uint32_t group_count = (desc->particle_count + group_size - 1) / group_size;
    Moss_DispatchGPUCompute(cmd, group_count, 1, 1);
    return true;
}