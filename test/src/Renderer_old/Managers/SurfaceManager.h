#ifndef MOSS_MESH_MANAGER3D_H
#define MOSS_MESH_MANAGER3D_H

#include <vector>

// Holds mesh data
// TODO Incorperate Mesh optimizer into this
class [[nodiscard]] MeshManager3D {

private:
    struct MeshEntry {
        std::string name;
        std::vector<Mesh> lods;
    };

    std::unordered_map<std::string, MeshEntry> m_meshes;
}
#endif // MOSS_MESH_MANAGER3D_H