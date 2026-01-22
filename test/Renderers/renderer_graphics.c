#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "graphics.h"
#include "macro.h"
#include "maths.h"

/* framebuffer management */

framebuffer_t *framebuffer_create(int width, int height) {
    int color_buffer_size = width * height * 4;
    int depth_buffer_size = sizeof(float) * width * height;
    vec4_t default_color = {0, 0, 0, 1};
    float default_depth = 1;
    framebuffer_t *framebuffer;

    assert(width > 0 && height > 0);

    framebuffer = (framebuffer_t*)malloc(sizeof(framebuffer_t));
    framebuffer->width = width;
    framebuffer->height = height;
    framebuffer->color_buffer = (unsigned char*)malloc(color_buffer_size);
    framebuffer->depth_buffer = (float*)malloc(depth_buffer_size);

    framebuffer_clear_color(framebuffer, default_color);
    framebuffer_clear_depth(framebuffer, default_depth);

    return framebuffer;
}

void framebuffer_release(framebuffer_t *framebuffer) {
    free(framebuffer->color_buffer);
    free(framebuffer->depth_buffer);
    free(framebuffer);
}

void framebuffer_clear_color(framebuffer_t *framebuffer, vec4_t color) {
    int num_pixels = framebuffer->width * framebuffer->height;
    int i;
    for (i = 0; i < num_pixels; i++) {
        framebuffer->color_buffer[i * 4 + 0] = float_to_uchar(color.x);
        framebuffer->color_buffer[i * 4 + 1] = float_to_uchar(color.y);
        framebuffer->color_buffer[i * 4 + 2] = float_to_uchar(color.z);
        framebuffer->color_buffer[i * 4 + 3] = float_to_uchar(color.w);
    }
}

void framebuffer_clear_depth(framebuffer_t *framebuffer, float depth) {
    int num_pixels = framebuffer->width * framebuffer->height;
    int i;
    for (i = 0; i < num_pixels; i++) {
        framebuffer->depth_buffer[i] = depth;
    }
}

/* program management */

#define MAX_VARYINGS 10

struct program {
    vertex_shader_t *vertex_shader;
    fragment_shader_t *fragment_shader;
    int sizeof_attribs;
    int sizeof_varyings;
    int sizeof_uniforms;
    int double_sided;
    int enable_blend;
    /* for shaders */
    void *shader_attribs[3];
    void *shader_varyings;
    void *shader_uniforms;
    /* for clipping */
    vec4_t in_coords[MAX_VARYINGS];
    vec4_t out_coords[MAX_VARYINGS];
    void *in_varyings[MAX_VARYINGS];
    void *out_varyings[MAX_VARYINGS];
};

program_t *program_create(
        vertex_shader_t *vertex_shader, fragment_shader_t *fragment_shader,
        int sizeof_attribs, int sizeof_varyings, int sizeof_uniforms,
        int double_sided, int enable_blend) {
    program_t *program;
    int i;

    assert(sizeof_attribs > 0 && sizeof_varyings > 0 && sizeof_uniforms > 0);
    assert(sizeof_varyings % sizeof(float) == 0);

    program = (program_t*)malloc(sizeof(program_t));

    program->vertex_shader = vertex_shader;
    program->fragment_shader = fragment_shader;
    program->sizeof_attribs = sizeof_attribs;
    program->sizeof_varyings = sizeof_varyings;
    program->sizeof_uniforms = sizeof_uniforms;
    program->double_sided = double_sided;
    program->enable_blend = enable_blend;

    for (i = 0; i < 3; i++) {
        program->shader_attribs[i] = malloc(sizeof_attribs);
        memset(program->shader_attribs[i], 0, sizeof_attribs);
    }
    program->shader_varyings = malloc(sizeof_varyings);
    memset(program->shader_varyings, 0, sizeof_varyings);
    program->shader_uniforms = malloc(sizeof_uniforms);
    memset(program->shader_uniforms, 0, sizeof_uniforms);
    for (i = 0; i < MAX_VARYINGS; i++) {
        program->in_varyings[i] = malloc(sizeof_varyings);
        memset(program->in_varyings[i], 0, sizeof_varyings);
        program->out_varyings[i] = malloc(sizeof_varyings);
        memset(program->out_varyings[i], 0, sizeof_varyings);
    }

    return program;
}

void program_release(program_t *program) {
    int i;
    for (i = 0; i < 3; i++) {
        free(program->shader_attribs[i]);
    }
    free(program->shader_varyings);
    free(program->shader_uniforms);
    for (i = 0; i < MAX_VARYINGS; i++) {
        free(program->in_varyings[i]);
        free(program->out_varyings[i]);
    }
    free(program);
}

void *program_get_attribs(program_t *program, int nth_vertex) {
    assert(nth_vertex >= 0 && nth_vertex < 3);
    return program->shader_attribs[nth_vertex];
}

void *program_get_uniforms(program_t *program) {
    return program->shader_uniforms;
}

/* graphics pipeline */

/*
 * for triangle clipping, see
 * http://fabiensanglard.net/polygon_codec/
 * http://graphics.idav.ucdavis.edu/education/GraphicsNotes/Clipping.pdf
 */

typedef enum {
    POSITIVE_W,
    POSITIVE_X,
    NEGATIVE_X,
    POSITIVE_Y,
    NEGATIVE_Y,
    POSITIVE_Z,
    NEGATIVE_Z
} plane_t;

static int is_inside_plane(vec4_t coord, plane_t plane) {
    switch (plane) {
        case POSITIVE_W:
            return coord.w >= EPSILON;
        case POSITIVE_X:
            return coord.x <= +coord.w;
        case NEGATIVE_X:
            return coord.x >= -coord.w;
        case POSITIVE_Y:
            return coord.y <= +coord.w;
        case NEGATIVE_Y:
            return coord.y >= -coord.w;
        case POSITIVE_Z:
            return coord.z <= +coord.w;
        case NEGATIVE_Z:
            return coord.z >= -coord.w;
        default:
            assert(0);
            return 0;
    }
}

static float get_intersect_ratio(vec4_t prev, vec4_t curr, plane_t plane) {
    switch (plane) {
        case POSITIVE_W:
            return (prev.w - EPSILON) / (prev.w - curr.w);
        case POSITIVE_X:
            return (prev.w - prev.x) / ((prev.w - prev.x) - (curr.w - curr.x));
        case NEGATIVE_X:
            return (prev.w + prev.x) / ((prev.w + prev.x) - (curr.w + curr.x));
        case POSITIVE_Y:
            return (prev.w - prev.y) / ((prev.w - prev.y) - (curr.w - curr.y));
        case NEGATIVE_Y:
            return (prev.w + prev.y) / ((prev.w + prev.y) - (curr.w + curr.y));
        case POSITIVE_Z:
            return (prev.w - prev.z) / ((prev.w - prev.z) - (curr.w - curr.z));
        case NEGATIVE_Z:
            return (prev.w + prev.z) / ((prev.w + prev.z) - (curr.w + curr.z));
        default:
            assert(0);
            return 0;
    }
}

static int clip_against_plane(
        plane_t plane, int in_num_vertices, int varying_num_floats,
        vec4_t in_coords[MAX_VARYINGS], void *in_varyings[MAX_VARYINGS],
        vec4_t out_coords[MAX_VARYINGS], void *out_varyings[MAX_VARYINGS]) {
    int out_num_vertices = 0;
    int i, j;

    assert(in_num_vertices >= 3 && in_num_vertices <= MAX_VARYINGS);
    for (i = 0; i < in_num_vertices; i++) {
        int prev_index = (i - 1 + in_num_vertices) % in_num_vertices;
        int curr_index = i;
        vec4_t prev_coord = in_coords[prev_index];
        vec4_t curr_coord = in_coords[curr_index];
        float *prev_varyings = (float*)in_varyings[prev_index];
        float *curr_varyings = (float*)in_varyings[curr_index];
        int prev_inside = is_inside_plane(prev_coord, plane);
        int curr_inside = is_inside_plane(curr_coord, plane);

        if (prev_inside != curr_inside) {
            vec4_t *dest_coord = &out_coords[out_num_vertices];
            float *dest_varyings = (float*)out_varyings[out_num_vertices];
            float ratio = get_intersect_ratio(prev_coord, curr_coord, plane);

            *dest_coord = vec4_lerp(prev_coord, curr_coord, ratio);
            /*
             * since this computation is performed in clip space before
             * division by w, clipped varying values are perspective-correct
             */
            for (j = 0; j < varying_num_floats; j++) {
                dest_varyings[j] = float_lerp(prev_varyings[j],
                                              curr_varyings[j],
                                              ratio);
            }
            out_num_vertices += 1;
        }

        if (curr_inside) {
            vec4_t *dest_coord = &out_coords[out_num_vertices];
            float *dest_varyings = (float*)out_varyings[out_num_vertices];
            int sizeof_varyings = varying_num_floats * sizeof(float);

            *dest_coord = curr_coord;
            memcpy(dest_varyings, curr_varyings, sizeof_varyings);
            out_num_vertices += 1;
        }
    }
    assert(out_num_vertices <= MAX_VARYINGS);
    return out_num_vertices;
}

#define CLIP_IN2OUT(plane)                                                  \
    do {                                                                    \
        num_vertices = clip_against_plane(                                  \
            plane, num_vertices, varying_num_floats,                        \
            in_coords, in_varyings, out_coords, out_varyings);              \
        if (num_vertices < 3) {                                             \
            return 0;                                                       \
        }                                                                   \
    } while (0)

#define CLIP_OUT2IN(plane)                                                  \
    do {                                                                    \
        num_vertices = clip_against_plane(                                  \
            plane, num_vertices, varying_num_floats,                        \
            out_coords, out_varyings, in_coords, in_varyings);              \
        if (num_vertices < 3) {                                             \
            return 0;                                                       \
        }                                                                   \
    } while (0)

static int is_vertex_visible(vec4_t v) {
    return fabs(v.x) <= v.w && fabs(v.y) <= v.w && fabs(v.z) <= v.w;
}

static int clip_triangle(
        int sizeof_varyings,
        vec4_t in_coords[MAX_VARYINGS], void *in_varyings[MAX_VARYINGS],
        vec4_t out_coords[MAX_VARYINGS], void *out_varyings[MAX_VARYINGS]) {
    int v0_visible = is_vertex_visible(in_coords[0]);
    int v1_visible = is_vertex_visible(in_coords[1]);
    int v2_visible = is_vertex_visible(in_coords[2]);
    if (v0_visible && v1_visible && v2_visible) {
        out_coords[0] = in_coords[0];
        out_coords[1] = in_coords[1];
        out_coords[2] = in_coords[2];
        memcpy(out_varyings[0], in_varyings[0], sizeof_varyings);
        memcpy(out_varyings[1], in_varyings[1], sizeof_varyings);
        memcpy(out_varyings[2], in_varyings[2], sizeof_varyings);
        return 3;
    } else {
        int varying_num_floats = sizeof_varyings / sizeof(float);
        int num_vertices = 3;
        CLIP_IN2OUT(POSITIVE_W);
        CLIP_OUT2IN(POSITIVE_X);
        CLIP_IN2OUT(NEGATIVE_X);
        CLIP_OUT2IN(POSITIVE_Y);
        CLIP_IN2OUT(NEGATIVE_Y);
        CLIP_OUT2IN(POSITIVE_Z);
        CLIP_IN2OUT(NEGATIVE_Z);
        return num_vertices;
    }
}

/*
 * for facing determination, see subsection 3.5.1 of
 * https://www.khronos.org/registry/OpenGL/specs/es/2.0/es_full_spec_2.0.pdf
 *
 * this is the same as (but more efficient than)
 *     vec3_t ab = vec3_sub(b, a);
 *     vec3_t ac = vec3_sub(c, a);
 *     return vec3_cross(ab, ac).z <= 0;
 */
static int is_back_facing(vec3_t ndc_coords[3]) {
    vec3_t a = ndc_coords[0];
    vec3_t b = ndc_coords[1];
    vec3_t c = ndc_coords[2];
    float signed_area = a.x * b.y - a.y * b.x +
                        b.x * c.y - b.y * c.x +
                        c.x * a.y - c.y * a.x;
    return signed_area <= 0;
}

/*
 * for viewport transformation, see subsection 2.12.1 of
 * https://www.khronos.org/registry/OpenGL/specs/es/2.0/es_full_spec_2.0.pdf
 */
static vec3_t viewport_transform(int width, int height, vec3_t ndc_coord) {
    float x = (ndc_coord.x + 1) * 0.5f * (float)width;   /* [-1, 1] -> [0, w] */
    float y = (ndc_coord.y + 1) * 0.5f * (float)height;  /* [-1, 1] -> [0, h] */
    float z = (ndc_coord.z + 1) * 0.5f;                  /* [-1, 1] -> [0, 1] */
    return vec3_new(x, y, z);
}

typedef struct {int min_x, min_y, max_x, max_y;} bbox_t;

static int min_integer(int a, int b) {
    return a < b ? a : b;
}

static int max_integer(int a, int b) {
    return a > b ? a : b;
}

static bbox_t find_bounding_box(vec2_t abc[3], int width, int height) {
    vec2_t min = vec2_min(vec2_min(abc[0], abc[1]), abc[2]);
    vec2_t max = vec2_max(vec2_max(abc[0], abc[1]), abc[2]);
    bbox_t bbox;
    bbox.min_x = max_integer((int)floor(min.x), 0);
    bbox.min_y = max_integer((int)floor(min.y), 0);
    bbox.max_x = min_integer((int)ceil(max.x), width - 1);
    bbox.max_y = min_integer((int)ceil(max.y), height - 1);
    return bbox;
}

/*
 * for barycentric coordinates, see
 * http://blackpawn.com/texts/pointinpoly/
 *
 * solve
 *     P = A + s * AB + t * AC  -->  AP = s * AB + t * AC
 * then
 *     s = (AC.y * AP.x - AC.x * AP.y) / (AB.x * AC.y - AB.y * AC.x)
 *     t = (AB.x * AP.y - AB.y * AP.x) / (AB.x * AC.y - AB.y * AC.x)
 *
 * notice
 *     P = A + s * AB + t * AC
 *       = A + s * (B - A) + t * (C - A)
 *       = (1 - s - t) * A + s * B + t * C
 * then
 *     weight_A = 1 - s - t
 *     weight_B = s
 *     weight_C = t
 */
static vec3_t calculate_weights(vec2_t abc[3], vec2_t p) {
    vec2_t a = abc[0];
    vec2_t b = abc[1];
    vec2_t c = abc[2];
    vec2_t ab = vec2_sub(b, a);
    vec2_t ac = vec2_sub(c, a);
    vec2_t ap = vec2_sub(p, a);
    float factor = 1 / (ab.x * ac.y - ab.y * ac.x);
    float s = (ac.y * ap.x - ac.x * ap.y) * factor;
    float t = (ab.x * ap.y - ab.y * ap.x) * factor;
    vec3_t weights = vec3_new(1 - s - t, s, t);
    return weights;
}

/*
 * for depth interpolation, see subsection 3.5.1 of
 * https://www.khronos.org/registry/OpenGL/specs/es/2.0/es_full_spec_2.0.pdf
 */
static float interpolate_depth(float screen_depths[3], vec3_t weights) {
    float depth0 = screen_depths[0] * weights.x;
    float depth1 = screen_depths[1] * weights.y;
    float depth2 = screen_depths[2] * weights.z;
    return depth0 + depth1 + depth2;
}

/*
 * for perspective correct interpolation, see
 * https://www.comp.nus.edu.sg/~lowkl/publications/lowk_persp_interp_techrep.pdf
 * https://www.khronos.org/registry/OpenGL/specs/es/2.0/es_full_spec_2.0.pdf
 *
 * equation 15 in reference 1 (page 2) is a simplified 2d version of
 * equation 3.5 in reference 2 (page 58) which uses barycentric coordinates
 */
static void interpolate_varyings(
        void *src_varyings[3], void *dst_varyings,
        int sizeof_varyings, vec3_t weights, float recip_w[3]) {
    int num_floats = sizeof_varyings / sizeof(float);
    float *src0 = (float*)src_varyings[0];
    float *src1 = (float*)src_varyings[1];
    float *src2 = (float*)src_varyings[2];
    float *dst = (float*)dst_varyings;
    float weight0 = recip_w[0] * weights.x;
    float weight1 = recip_w[1] * weights.y;
    float weight2 = recip_w[2] * weights.z;
    float normalizer = 1 / (weight0 + weight1 + weight2);
    int i;
    for (i = 0; i < num_floats; i++) {
        float sum = src0[i] * weight0 + src1[i] * weight1 + src2[i] * weight2;
        dst[i] = sum * normalizer;
    }
}

static void draw_fragment(framebuffer_t *framebuffer, program_t *program,
                          int backface, int index, float depth) {
    vec4_t color;
    int discard;

    /* execute fragment shader */
    discard = 0;
    color = program->fragment_shader(program->shader_varyings,
                                     program->shader_uniforms,
                                     &discard,
                                     backface);
    if (discard) {
        return;
    }
    color = vec4_saturate(color);

    /* perform blending */
    if (program->enable_blend) {
        /* out_color = src_color * src_alpha + dst_color * (1 - src_alpha) */
        unsigned char dst_r = framebuffer->color_buffer[index * 4 + 0];
        unsigned char dst_g = framebuffer->color_buffer[index * 4 + 1];
        unsigned char dst_b = framebuffer->color_buffer[index * 4 + 2];
        color.x = color.x * color.w + float_from_uchar(dst_r) * (1 - color.w);
        color.y = color.y * color.w + float_from_uchar(dst_g) * (1 - color.w);
        color.z = color.z * color.w + float_from_uchar(dst_b) * (1 - color.w);
    }

    /* write color and depth */
    framebuffer->color_buffer[index * 4 + 0] = float_to_uchar(color.x);
    framebuffer->color_buffer[index * 4 + 1] = float_to_uchar(color.y);
    framebuffer->color_buffer[index * 4 + 2] = float_to_uchar(color.z);
    framebuffer->depth_buffer[index] = depth;
}

static int rasterize_triangle(framebuffer_t *framebuffer, program_t *program,
                              vec4_t clip_coords[3], void *varyings[3]) {
    int width = framebuffer->width;
    int height = framebuffer->height;
    vec3_t ndc_coords[3];
    vec2_t screen_coords[3];
    float screen_depths[3];
    float recip_w[3];
    int backface;
    bbox_t bbox;
    int i, x, y;

    /* perspective division */
    for (i = 0; i < 3; i++) {
        vec3_t clip_coord = vec3_from_vec4(clip_coords[i]);
        ndc_coords[i] = vec3_div(clip_coord, clip_coords[i].w);
    }

    /* back-face culling */
    backface = is_back_facing(ndc_coords);
    if (backface && !program->double_sided) {
        return 1;
    }

    /* reciprocals of w */
    for (i = 0; i < 3; i++) {
        recip_w[i] = 1 / clip_coords[i].w;
    }

    /* viewport mapping */
    for (i = 0; i < 3; i++) {
        vec3_t window_coord = viewport_transform(width, height, ndc_coords[i]);
        screen_coords[i] = vec2_new(window_coord.x, window_coord.y);
        screen_depths[i] = window_coord.z;
    }

    /* perform rasterization */
    bbox = find_bounding_box(screen_coords, width, height);
    for (x = bbox.min_x; x <= bbox.max_x; x++) {
        for (y = bbox.min_y; y <= bbox.max_y; y++) {
            vec2_t point = vec2_new((float)x + 0.5f, (float)y + 0.5f);
            vec3_t weights = calculate_weights(screen_coords, point);
            int weight0_okay = weights.x > -EPSILON;
            int weight1_okay = weights.y > -EPSILON;
            int weight2_okay = weights.z > -EPSILON;
            if (weight0_okay && weight1_okay && weight2_okay) {
                int index = y * width + x;
                float depth = interpolate_depth(screen_depths, weights);
                /* early depth testing */
                if (depth <= framebuffer->depth_buffer[index]) {
                    interpolate_varyings(varyings, program->shader_varyings,
                                         program->sizeof_varyings,
                                         weights, recip_w);
                    draw_fragment(framebuffer, program, backface, index, depth);
                }
            }
        }
    }

    return 0;
}

void graphics_draw_triangle(framebuffer_t *framebuffer, program_t *program) {
    int num_vertices;
    int i;

    /* execute vertex shader */
    for (i = 0; i < 3; i++) {
        vec4_t clip_coord = program->vertex_shader(program->shader_attribs[i],
                                                   program->in_varyings[i],
                                                   program->shader_uniforms);
        program->in_coords[i] = clip_coord;
    }

    /* triangle clipping */
    num_vertices = clip_triangle(program->sizeof_varyings,
                                 program->in_coords, program->in_varyings,
                                 program->out_coords, program->out_varyings);

    /* triangle assembly */
    for (i = 0; i < num_vertices - 2; i++) {
        int index0 = 0;
        int index1 = i + 1;
        int index2 = i + 2;
        vec4_t clip_coords[3];
        void *varyings[3];
        int is_culled;

        clip_coords[0] = program->out_coords[index0];
        clip_coords[1] = program->out_coords[index1];
        clip_coords[2] = program->out_coords[index2];
        varyings[0] = program->out_varyings[index0];
        varyings[1] = program->out_varyings[index1];
        varyings[2] = program->out_varyings[index2];

        is_culled = rasterize_triangle(framebuffer, program,
                                       clip_coords, varyings);
        if (is_culled) {
            break;
        }
    }
}







//////////////////////////////////////////////////
typedef struct {
    int joint_index;
    int parent_index;
    mat4_t inverse_bind;
    /* translations */
    int num_translations;
    float *translation_times;
    vec3_t *translation_values;
    /* rotations */
    int num_rotations;
    float *rotation_times;
    quat_t *rotation_values;
    /* scales */
    int num_scales;
    float *scale_times;
    vec3_t *scale_values;
    /* interpolated */
    mat4_t transform;
} joint_t;

struct skeleton {
    float min_time;
    float max_time;
    int num_joints;
    joint_t *joints;
    /* cached result */
    mat4_t *joint_matrices;
    mat3_t *normal_matrices;
    float last_time;
};

/* skeleton loading/releasing */

static void read_inverse_bind(FILE *file, joint_t *joint) {
    char line[LINE_SIZE];
    int items;
    int i;
    items = fscanf(file, " %s", line);
    assert(items == 1 && strcmp(line, "inverse-bind:") == 0);
    for (i = 0; i < 4; i++) {
        items = fscanf(file, " %f %f %f %f",
                       &joint->inverse_bind.m[i][0],
                       &joint->inverse_bind.m[i][1],
                       &joint->inverse_bind.m[i][2],
                       &joint->inverse_bind.m[i][3]);
        assert(items == 4);
    }
    UNUSED_VAR(items);
}

static void read_translations(FILE *file, joint_t *joint) {
    int items;
    int i;
    items = fscanf(file, " translations %d:", &joint->num_translations);
    assert(items == 1 && joint->num_translations >= 0);
    if (joint->num_translations > 0) {
        int time_size = sizeof(float) * joint->num_translations;
        int value_size = sizeof(vec3_t) * joint->num_translations;
        joint->translation_times = (float*)malloc(time_size);
        joint->translation_values = (vec3_t*)malloc(value_size);
        for (i = 0; i < joint->num_translations; i++) {
            items = fscanf(file, " time: %f, value: [%f, %f, %f]",
                           &joint->translation_times[i],
                           &joint->translation_values[i].x,
                           &joint->translation_values[i].y,
                           &joint->translation_values[i].z);
            assert(items == 4);
        }
    } else {
        joint->translation_times = NULL;
        joint->translation_values = NULL;
    }
    UNUSED_VAR(items);
}

static void read_rotations(FILE *file, joint_t *joint) {
    int items;
    int i;
    items = fscanf(file, " rotations %d:", &joint->num_rotations);
    assert(items == 1 && joint->num_rotations >= 0);
    if (joint->num_rotations > 0) {
        int time_size = sizeof(float) * joint->num_rotations;
        int value_size = sizeof(quat_t) * joint->num_rotations;
        joint->rotation_times = (float*)malloc(time_size);
        joint->rotation_values = (quat_t*)malloc(value_size);
        for (i = 0; i < joint->num_rotations; i++) {
            items = fscanf(file, " time: %f, value: [%f, %f, %f, %f]",
                           &joint->rotation_times[i],
                           &joint->rotation_values[i].x,
                           &joint->rotation_values[i].y,
                           &joint->rotation_values[i].z,
                           &joint->rotation_values[i].w);
            assert(items == 5);
        }
    } else {
        joint->rotation_times = NULL;
        joint->rotation_values = NULL;
    }
    UNUSED_VAR(items);
}

static void read_scales(FILE *file, joint_t *joint) {
    int items;
    int i;
    items = fscanf(file, " scales %d:", &joint->num_scales);
    assert(items == 1 && joint->num_scales >= 0);
    if (joint->num_scales > 0) {
        int time_size = sizeof(float) * joint->num_scales;
        int value_size = sizeof(vec3_t) * joint->num_scales;
        joint->scale_times = (float*)malloc(time_size);
        joint->scale_values = (vec3_t*)malloc(value_size);
        for (i = 0; i < joint->num_scales; i++) {
            items = fscanf(file, " time: %f, value: [%f, %f, %f]",
                           &joint->scale_times[i],
                           &joint->scale_values[i].x,
                           &joint->scale_values[i].y,
                           &joint->scale_values[i].z);
            assert(items == 4);
        }
    } else {
        joint->scale_times = NULL;
        joint->scale_values = NULL;
    }
    UNUSED_VAR(items);
}

static joint_t load_joint(FILE *file) {
    joint_t joint;
    int items;

    items = fscanf(file, " joint %d:", &joint.joint_index);
    assert(items == 1);
    items = fscanf(file, " parent-index: %d", &joint.parent_index);
    assert(items == 1);

    read_inverse_bind(file, &joint);
    read_translations(file, &joint);
    read_rotations(file, &joint);
    read_scales(file, &joint);

    UNUSED_VAR(items);
    return joint;
}

static void initialize_cache(skeleton_t *skeleton) {
    int joint_matrix_size = sizeof(mat4_t) * skeleton->num_joints;
    int normal_matrix_size = sizeof(mat3_t) * skeleton->num_joints;
    skeleton->joint_matrices = (mat4_t*)malloc(joint_matrix_size);
    skeleton->normal_matrices = (mat3_t*)malloc(normal_matrix_size);
    memset(skeleton->joint_matrices, 0, joint_matrix_size);
    memset(skeleton->normal_matrices, 0, normal_matrix_size);
    skeleton->last_time = -1;
}

static skeleton_t *load_ani(const char *filename) {
    skeleton_t *skeleton;
    FILE *file;
    int items;
    int i;

    skeleton = (skeleton_t*)malloc(sizeof(skeleton_t));

    file = fopen(filename, "rb");
    assert(file != NULL);

    items = fscanf(file, " joint-size: %d", &skeleton->num_joints);
    assert(items == 1 && skeleton->num_joints > 0);
    items = fscanf(file, " time-range: [%f, %f]",
                   &skeleton->min_time, &skeleton->max_time);
    assert(items == 2 && skeleton->min_time < skeleton->max_time);

    skeleton->joints = (joint_t*)malloc(sizeof(joint_t) * skeleton->num_joints);
    for (i = 0; i < skeleton->num_joints; i++) {
        joint_t joint = load_joint(file);
        assert(joint.joint_index == i);
        skeleton->joints[i] = joint;
    }

    fclose(file);

    initialize_cache(skeleton);

    UNUSED_VAR(items);
    return skeleton;
}

skeleton_t *skeleton_load(const char *filename) {
    const char *extension = private_get_extension(filename);
    if (strcmp(extension, "ani") == 0) {
        return load_ani(filename);
    } else {
        assert(0);
        return NULL;
    }
}

void skeleton_release(skeleton_t *skeleton) {
    int i;
    for (i = 0; i < skeleton->num_joints; i++) {
        joint_t *joint = &skeleton->joints[i];
        free(joint->translation_times);
        free(joint->translation_values);
        free(joint->rotation_times);
        free(joint->rotation_values);
        free(joint->scale_times);
        free(joint->scale_values);
    }
    free(skeleton->joints);
    free(skeleton->joint_matrices);
    free(skeleton->normal_matrices);
    free(skeleton);
}

/* joint updating/retrieving */

static vec3_t get_translation(joint_t *joint, float frame_time) {
    int num_translations = joint->num_translations;
    float *translation_times = joint->translation_times;
    vec3_t *translation_values = joint->translation_values;

    if (num_translations == 0) {
        return vec3_new(0, 0, 0);
    } else if (frame_time <= translation_times[0]) {
        return translation_values[0];
    } else if (frame_time >= translation_times[num_translations - 1]) {
        return translation_values[num_translations - 1];
    } else {
        int i;
        for (i = 0; i < num_translations - 1; i++) {
            float curr_time = translation_times[i];
            float next_time = translation_times[i + 1];
            if (frame_time >= curr_time && frame_time < next_time) {
                float t = (frame_time - curr_time) / (next_time - curr_time);
                vec3_t curr_translation = translation_values[i];
                vec3_t next_translation = translation_values[i + 1];
                return vec3_lerp(curr_translation, next_translation, t);
            }
        }
        assert(0);
        return vec3_new(0, 0, 0);
    }
}

static quat_t get_rotation(joint_t *joint, float frame_time) {
    int num_rotations = joint->num_rotations;
    float *rotation_times = joint->rotation_times;
    quat_t *rotation_values = joint->rotation_values;

    if (num_rotations == 0) {
        return quat_new(0, 0, 0, 1);
    } else if (frame_time <= rotation_times[0]) {
        return rotation_values[0];
    } else if (frame_time >= rotation_times[num_rotations - 1]) {
        return rotation_values[num_rotations - 1];
    } else {
        int i;
        for (i = 0; i < num_rotations - 1; i++) {
            float curr_time = rotation_times[i];
            float next_time = rotation_times[i + 1];
            if (frame_time >= curr_time && frame_time < next_time) {
                float t = (frame_time - curr_time) / (next_time - curr_time);
                quat_t curr_rotation = rotation_values[i];
                quat_t next_rotation = rotation_values[i + 1];
                return quat_slerp(curr_rotation, next_rotation, t);
            }
        }
        assert(0);
        return quat_new(0, 0, 0, 1);
    }
}

static vec3_t get_scale(joint_t *joint, float frame_time) {
    int num_scales = joint->num_scales;
    float *scale_times = joint->scale_times;
    vec3_t *scale_values = joint->scale_values;

    if (num_scales == 0) {
        return vec3_new(1, 1, 1);
    } else if (frame_time <= scale_times[0]) {
        return scale_values[0];
    } else if (frame_time >= scale_times[num_scales - 1]) {
        return scale_values[num_scales - 1];
    } else {
        int i;
        for (i = 0; i < num_scales - 1; i++) {
            float curr_time = scale_times[i];
            float next_time = scale_times[i + 1];
            if (frame_time >= curr_time && frame_time < next_time) {
                float t = (frame_time - curr_time) / (next_time - curr_time);
                vec3_t curr_scale = scale_values[i];
                vec3_t next_scale = scale_values[i + 1];
                return vec3_lerp(curr_scale, next_scale, t);
            }
        }
        assert(0);
        return vec3_new(1, 1, 1);
    }
}

void skeleton_update_joints(skeleton_t *skeleton, float frame_time) {
    frame_time = (float)fmod(frame_time, skeleton->max_time);
    if (frame_time != skeleton->last_time) {
        int i;
        for (i = 0; i < skeleton->num_joints; i++) {
            joint_t *joint = &skeleton->joints[i];
            vec3_t translation = get_translation(joint, frame_time);
            quat_t rotation = get_rotation(joint, frame_time);
            vec3_t scale = get_scale(joint, frame_time);
            mat4_t joint_matrix;
            mat3_t normal_matrix;

            joint->transform = mat4_from_trs(translation, rotation, scale);
            if (joint->parent_index >= 0) {
                joint_t *parent = &skeleton->joints[joint->parent_index];
                joint->transform = mat4_mul_mat4(parent->transform,
                                                 joint->transform);
            }

            joint_matrix = mat4_mul_mat4(joint->transform, joint->inverse_bind);
            normal_matrix = mat3_inverse_transpose(mat3_from_mat4(joint_matrix));
            skeleton->joint_matrices[i] = joint_matrix;
            skeleton->normal_matrices[i] = normal_matrix;
        }
        skeleton->last_time = frame_time;
    }
}

mat4_t *skeleton_get_joint_matrices(skeleton_t *skeleton) {
    return skeleton->joint_matrices;
}

mat3_t *skeleton_get_normal_matrices(skeleton_t *skeleton) {
    return skeleton->normal_matrices;
}




//////////////////////////////////////////////////////

scene_t *scene_create(vec3_t background, model_t *skybox, model_t **models,
                      float ambient_intensity, float punctual_intensity,
                      int shadow_width, int shadow_height) {
    scene_t *scene = (scene_t*)malloc(sizeof(scene_t));
    scene->background = vec4_from_vec3(background, 1);
    scene->skybox = skybox;
    scene->models = models;
    scene->ambient_intensity = ambient_intensity;
    scene->punctual_intensity = punctual_intensity;
    if (shadow_width > 0 && shadow_height > 0) {
        scene->shadow_buffer = framebuffer_create(shadow_width, shadow_height);
        scene->shadow_map = texture_create(shadow_width, shadow_height);
    } else {
        scene->shadow_buffer = NULL;
        scene->shadow_map = NULL;
    }
    return scene;
}

void scene_release(scene_t *scene) {
    int num_models = darray_size(scene->models);
    int i;
    if (scene->skybox) {
        model_t *skybox = scene->skybox;
        skybox->release(skybox);
    }
    for (i = 0; i < num_models; i++) {
        model_t *model = scene->models[i];
        model->release(model);
    }
    darray_free(scene->models);
    if (scene->shadow_buffer) {
        framebuffer_release(scene->shadow_buffer);
    }
    if (scene->shadow_map) {
        texture_release(scene->shadow_map);
    }
    free(scene);
}

///////////////////////////////////

struct mesh {
    int num_faces;
    vertex_t *vertices;
    vec3_t center;
};

/* mesh loading/releasing */

static mesh_t *build_mesh(
        vec3_t *positions, vec2_t *texcoords, vec3_t *normals,
        vec4_t *tangents, vec4_t *joints, vec4_t *weights,
        int *position_indices, int *texcoord_indices, int *normal_indices) {
    vec3_t bbox_min = vec3_new(+1e6, +1e6, +1e6);
    vec3_t bbox_max = vec3_new(-1e6, -1e6, -1e6);
    int num_indices = darray_size(position_indices);
    int num_faces = num_indices / 3;
    vertex_t *vertices;
    mesh_t *mesh;
    int i;

    assert(num_faces > 0 && num_faces * 3 == num_indices);
    assert(darray_size(position_indices) == num_indices);
    assert(darray_size(texcoord_indices) == num_indices);
    assert(darray_size(normal_indices) == num_indices);

    vertices = (vertex_t*)malloc(sizeof(vertex_t) * num_indices);
    for (i = 0; i < num_indices; i++) {
        int position_index = position_indices[i];
        int texcoord_index = texcoord_indices[i];
        int normal_index = normal_indices[i];
        assert(position_index >= 0 && position_index < darray_size(positions));
        assert(texcoord_index >= 0 && texcoord_index < darray_size(texcoords));
        assert(normal_index >= 0 && normal_index < darray_size(normals));
        vertices[i].position = positions[position_index];
        vertices[i].texcoord = texcoords[texcoord_index];
        vertices[i].normal = normals[normal_index];

        if (tangents) {
            int tangent_index = position_index;
            assert(tangent_index >= 0 && tangent_index < darray_size(tangents));
            vertices[i].tangent = tangents[tangent_index];
        } else {
            vertices[i].tangent = vec4_new(1, 0, 0, 1);
        }

        if (joints) {
            int joint_index = position_index;
            assert(joint_index >= 0 && joint_index < darray_size(joints));
            vertices[i].joint = joints[joint_index];
        } else {
            vertices[i].joint = vec4_new(0, 0, 0, 0);
        }

        if (weights) {
            int weight_index = position_index;
            assert(weight_index >= 0 && weight_index < darray_size(weights));
            vertices[i].weight = weights[weight_index];
        } else {
            vertices[i].weight = vec4_new(0, 0, 0, 0);
        }

        bbox_min = vec3_min(bbox_min, vertices[i].position);
        bbox_max = vec3_max(bbox_max, vertices[i].position);
    }

    mesh = (mesh_t*)malloc(sizeof(mesh_t));
    mesh->num_faces = num_faces;
    mesh->vertices = vertices;
    mesh->center = vec3_div(vec3_add(bbox_min, bbox_max), 2);

    return mesh;
}

static mesh_t *load_obj(const char *filename) {
    vec3_t *positions = NULL;
    vec2_t *texcoords = NULL;
    vec3_t *normals = NULL;
    vec4_t *tangents = NULL;
    vec4_t *joints = NULL;
    vec4_t *weights = NULL;
    int *position_indices = NULL;
    int *texcoord_indices = NULL;
    int *normal_indices = NULL;
    char line[LINE_SIZE];
    mesh_t *mesh;
    FILE *file;

    file = fopen(filename, "rb");
    assert(file != NULL);
    while (1) {
        int items;
        if (fgets(line, LINE_SIZE, file) == NULL) {
            break;
        } else if (strncmp(line, "v ", 2) == 0) {               /* position */
            vec3_t position;
            items = sscanf(line, "v %f %f %f",
                           &position.x, &position.y, &position.z);
            assert(items == 3);
            darray_push(positions, position);
        } else if (strncmp(line, "vt ", 3) == 0) {              /* texcoord */
            vec2_t texcoord;
            items = sscanf(line, "vt %f %f",
                           &texcoord.x, &texcoord.y);
            assert(items == 2);
            darray_push(texcoords, texcoord);
        } else if (strncmp(line, "vn ", 3) == 0) {              /* normal */
            vec3_t normal;
            items = sscanf(line, "vn %f %f %f",
                           &normal.x, &normal.y, &normal.z);
            assert(items == 3);
            darray_push(normals, normal);
        } else if (strncmp(line, "f ", 2) == 0) {               /* face */
            int i;
            int pos_indices[3], uv_indices[3], n_indices[3];
            items = sscanf(line, "f %d/%d/%d %d/%d/%d %d/%d/%d",
                           &pos_indices[0], &uv_indices[0], &n_indices[0],
                           &pos_indices[1], &uv_indices[1], &n_indices[1],
                           &pos_indices[2], &uv_indices[2], &n_indices[2]);
            assert(items == 9);
            for (i = 0; i < 3; i++) {
                darray_push(position_indices, pos_indices[i] - 1);
                darray_push(texcoord_indices, uv_indices[i] - 1);
                darray_push(normal_indices, n_indices[i] - 1);
            }
        } else if (strncmp(line, "# ext.tangent ", 14) == 0) {  /* tangent */
            vec4_t tangent;
            items = sscanf(line, "# ext.tangent %f %f %f %f",
                           &tangent.x, &tangent.y, &tangent.z, &tangent.w);
            assert(items == 4);
            darray_push(tangents, tangent);
        } else if (strncmp(line, "# ext.joint ", 12) == 0) {    /* joint */
            vec4_t joint;
            items = sscanf(line, "# ext.joint %f %f %f %f",
                           &joint.x, &joint.y, &joint.z, &joint.w);
            assert(items == 4);
            darray_push(joints, joint);
        } else if (strncmp(line, "# ext.weight ", 13) == 0) {   /* weight */
            vec4_t weight;
            items = sscanf(line, "# ext.weight %f %f %f %f",
                           &weight.x, &weight.y, &weight.z, &weight.w);
            assert(items == 4);
            darray_push(weights, weight);
        }
        UNUSED_VAR(items);
    }
    fclose(file);

    mesh = build_mesh(positions, texcoords, normals, tangents, joints, weights,
                      position_indices, texcoord_indices, normal_indices);
    darray_free(positions);
    darray_free(texcoords);
    darray_free(normals);
    darray_free(tangents);
    darray_free(joints);
    darray_free(weights);
    darray_free(position_indices);
    darray_free(texcoord_indices);
    darray_free(normal_indices);

    return mesh;
}

mesh_t *mesh_load(const char *filename) {
    const char *extension = private_get_extension(filename);
    if (strcmp(extension, "obj") == 0) {
        return load_obj(filename);
    } else {
        assert(0);
        return NULL;
    }
}

void mesh_release(mesh_t *mesh) {
    free(mesh->vertices);
    free(mesh);
}

/* vertex retrieving */

int mesh_get_num_faces(mesh_t *mesh) {
    return mesh->num_faces;
}

vertex_t *mesh_get_vertices(mesh_t *mesh) {
    return mesh->vertices;
}

vec3_t mesh_get_center(mesh_t *mesh) {
    return mesh->center;
}
