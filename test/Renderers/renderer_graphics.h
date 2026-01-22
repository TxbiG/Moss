#ifndef GRAPHICS_H
#define GRAPHICS_H

#include "maths.h"

typedef struct {
    int width, height;
    unsigned char *color_buffer;
    float *depth_buffer;
} framebuffer_t;

typedef struct program program_t;
typedef vec4_t vertex_shader_t(void *attribs, void *varyings, void *uniforms);
typedef vec4_t fragment_shader_t(void *varyings, void *uniforms, int *discard, int backface);

/* framebuffer management */
framebuffer_t *framebuffer_create(int width, int height);
void framebuffer_release(framebuffer_t *framebuffer);
void framebuffer_clear_color(framebuffer_t *framebuffer, vec4_t color);
void framebuffer_clear_depth(framebuffer_t *framebuffer, float depth);

/* program management */
program_t *program_create(
    vertex_shader_t *vertex_shader, fragment_shader_t *fragment_shader,
    int sizeof_attribs, int sizeof_varyings, int sizeof_uniforms,
    int double_sided, int enable_blend);
void program_release(program_t *program);
void *program_get_attribs(program_t *program, int nth_vertex);
void *program_get_uniforms(program_t *program);

/* graphics pipeline */
void graphics_draw_triangle(framebuffer_t *framebuffer, program_t *program);



typedef struct mesh mesh_t;

typedef struct {
    vec3_t position;
    vec2_t texcoord;
    vec3_t normal;
    vec4_t tangent;
    vec4_t joint;
    vec4_t weight;
} vertex_t;

/* mesh loading/releasing */
mesh_t *mesh_load(const char *filename);
void mesh_release(mesh_t *mesh);

/* vertex retrieving */
int mesh_get_num_faces(mesh_t *mesh);
vertex_t *mesh_get_vertices(mesh_t *mesh);
vec3_t mesh_get_center(mesh_t *mesh);










typedef struct {
    float frame_time;
    float delta_time;
    vec3_t light_dir;
    vec3_t camera_pos;
    mat4_t light_view_matrix;
    mat4_t light_proj_matrix;
    mat4_t camera_view_matrix;
    mat4_t camera_proj_matrix;
    float ambient_intensity;
    float punctual_intensity;
    texture_t *shadow_map;
    int layer_view;
} perframe_t;

typedef struct model {
    mesh_t *mesh;
    program_t *program;
    mat4_t transform;
    /* for animation */
    skeleton_t *skeleton;
    int attached;
    /* for sorting */
    int opaque;
    float distance;
    /* polymorphism */
    void (*update)(struct model *model, perframe_t *perframe);
    void (*draw)(struct model *model, framebuffer_t *framebuffer,
                 int shadow_pass);
    void (*release)(struct model *model);
} model_t;

typedef struct {
    vec4_t background;
    model_t *skybox;
    model_t **models;
    /* light intensity */
    float ambient_intensity;
    float punctual_intensity;
    /* shadow mapping */
    framebuffer_t *shadow_buffer;
    texture_t *shadow_map;
} scene_t;

scene_t *scene_create(vec3_t background, model_t *skybox, model_t **models,
                      float ambient_intensity, float punctual_intensity,
                      int shadow_width, int shadow_height);
void scene_release(scene_t *scene);

#endif
