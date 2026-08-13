//                        MIT License
//
//                  Copyright (c) 2026 Toby

#include <Moss/Moss_Physics.h>

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace {

const char* moss_physics_limit_notes =
    "Soft body vs soft body, high-level ragdoll presets, compound mass/inertia precision, "
    "and sensor callback normalization are partially supported and should be validated per game.";

size_t moss_append(char* buffer, size_t buffer_size, size_t offset, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    int written = std::vsnprintf(buffer && offset < buffer_size ? buffer + offset : nullptr,
                                 buffer && offset < buffer_size ? buffer_size - offset : 0,
                                 fmt,
                                 args);
    va_end(args);

    if (written < 0) {
        return offset;
    }
    return offset + static_cast<size_t>(written);
}

uint32_t moss_read_u32_after(const char* text, const char* key) {
    const char* found = std::strstr(text, key);
    if (!found) {
        return 0;
    }
    found += std::strlen(key);
    while (*found == ' ' || *found == '\t' || *found == ':') {
        ++found;
    }
    unsigned value = 0;
    std::sscanf(found, "%u", &value);
    return static_cast<uint32_t>(value);
}

} // namespace

void Moss_PhysicsGetLimitations(Moss_PhysicsLimitations* limitations) {
    if (!limitations) {
        return;
    }
    *limitations = {};
    limitations->soft_body_vs_soft_body = MOSS_PHYSICS_FEATURE_PARTIAL;
    limitations->ragdoll_constraint_presets = MOSS_PHYSICS_FEATURE_PARTIAL;
    limitations->collision_mass_inertia_precision = MOSS_PHYSICS_FEATURE_PARTIAL;
    limitations->sensor_update_callbacks = MOSS_PHYSICS_FEATURE_PARTIAL;
    limitations->notes = moss_physics_limit_notes;
}

uint32_t Moss_PhysicsDebugDrawBodies(Moss_DebugDrawList* list, const Moss_PhysicsDebugBodyDesc* bodies, uint32_t body_count, uint32_t flags) {
    if (!list || !bodies || body_count == 0 || (flags & MOSS_PHYSICS_DEBUG_DRAW_BODIES) == 0) {
        return 0;
    }

    uint32_t added = 0;
    for (uint32_t i = 0; i < body_count; ++i) {
        if (bodies[i].is_sensor && (flags & MOSS_PHYSICS_DEBUG_DRAW_SENSORS) == 0) {
            continue;
        }

        Moss_DebugBoxDesc box{};
        box.center = bodies[i].center;
        box.size = bodies[i].size;
        box.color = bodies[i].is_sensor ? Color(0.3f, 1.0f, 0.5f, 1.0f) : bodies[i].color;
        added += Moss_DebugDrawBox(list, &box) ? 1u : 0u;
    }
    return added;
}

uint32_t Moss_PhysicsDebugDrawContacts(Moss_DebugDrawList* list, const Moss_PhysicsDebugContactDesc* contacts, uint32_t contact_count) {
    if (!list || !contacts || contact_count == 0) {
        return 0;
    }

    uint32_t added = 0;
    for (uint32_t i = 0; i < contact_count; ++i) {
        Moss_DebugPhysicsContactDesc contact{};
        contact.position = contacts[i].position;
        contact.normal = contacts[i].normal;
        contact.impulse = contacts[i].impulse;
        contact.color = contacts[i].color;
        added += Moss_DebugDrawPhysicsContact(list, &contact) ? 1u : 0u;
    }
    return added;
}

size_t Moss_PhysicsShapeSerialize(const Moss_PhysicsShapeSerializeDesc* shape, char* buffer, size_t buffer_size) {
    if (!shape) {
        return 0;
    }

    const char* type = shape->type ? shape->type : "unknown";
    size_t offset = 0;
    offset = moss_append(buffer, buffer_size, offset,
                         "{\"type\":\"%s\",\"center\":[%.6g,%.6g,%.6g],\"size\":[%.6g,%.6g,%.6g],"
                         "\"radius\":%.6g,\"height\":%.6g,\"mass\":%.6g,\"sensor\":%s}",
                         type,
                         shape->center.x, shape->center.y, shape->center.z,
                         shape->size.x, shape->size.y, shape->size.z,
                         shape->radius, shape->height, shape->mass,
                         shape->sensor ? "true" : "false");
    if (buffer && buffer_size > 0) {
        buffer[buffer_size - 1] = '\0';
    }
    return offset;
}

size_t Moss_PhysicsSceneSerialize(const Moss_PhysicsSceneSerializeDesc* scene, char* buffer, size_t buffer_size) {
    if (!scene) {
        return 0;
    }

    size_t offset = 0;
    offset = moss_append(buffer, buffer_size, offset,
                         "{\"shape_count\":%u,\"contact_count\":%u,\"shapes\":[",
                         scene->shape_count,
                         scene->contact_count);
    for (uint32_t i = 0; scene->shapes && i < scene->shape_count; ++i) {
        if (i > 0) {
            offset = moss_append(buffer, buffer_size, offset, ",");
        }
        char shape_buffer[512];
        Moss_PhysicsShapeSerialize(&scene->shapes[i], shape_buffer, sizeof(shape_buffer));
        offset = moss_append(buffer, buffer_size, offset, "%s", shape_buffer);
    }
    offset = moss_append(buffer, buffer_size, offset, "],\"contacts\":[");
    for (uint32_t i = 0; scene->contacts && i < scene->contact_count; ++i) {
        if (i > 0) {
            offset = moss_append(buffer, buffer_size, offset, ",");
        }
        const Moss_PhysicsDebugContactDesc& contact = scene->contacts[i];
        offset = moss_append(buffer, buffer_size, offset,
                             "{\"position\":[%.6g,%.6g,%.6g],\"normal\":[%.6g,%.6g,%.6g],\"impulse\":%.6g}",
                             contact.position.x, contact.position.y, contact.position.z,
                             contact.normal.x, contact.normal.y, contact.normal.z,
                             contact.impulse);
    }
    offset = moss_append(buffer, buffer_size, offset, "]}");
    if (buffer && buffer_size > 0) {
        buffer[buffer_size - 1] = '\0';
    }
    return offset;
}

bool Moss_PhysicsSceneDeserializeInfo(const char* serialized_scene, Moss_PhysicsSerializedSceneInfo* info) {
    if (!serialized_scene || !info) {
        return false;
    }

    info->shape_count = moss_read_u32_after(serialized_scene, "\"shape_count\"");
    info->contact_count = moss_read_u32_after(serialized_scene, "\"contact_count\"");
    return std::strstr(serialized_scene, "\"shapes\"") != nullptr;
}