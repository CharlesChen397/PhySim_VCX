#pragma once

#include <cstdint>
#include <vector>

#include <Eigen/Dense>

#include "Labs/1-RigidBody/RigidBodySystem.h"

namespace VCX::Labs::RigidBody {

    struct MeshData {
        std::vector<Eigen::Vector3f> Vertices;
        std::vector<std::uint32_t>   Triangles;
        std::vector<std::uint32_t>   Lines;
    };

    MeshData BuildBodyMesh(RigidBody const & body);

} // namespace VCX::Labs::RigidBody
