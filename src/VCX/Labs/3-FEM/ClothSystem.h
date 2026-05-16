#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

namespace VCX::Labs::FEM {

    class ClothSystem {
    public:
        struct Triangle {
            std::array<std::size_t, 3> Idx {};
            glm::mat2                  InvRest { 1.f };
            float                      RestArea { 0.f };
        };

        struct BendEdge {
            std::size_t I0 { 0 };
            std::size_t I1 { 0 };
            float       RestLength { 0.f };
        };

        struct GridResolution {
            std::size_t X { 24 };
            std::size_t Y { 24 };
        };

        float YoungModulus { 50.f };
        float PoissonRatio { 0.3f };
        float AreaDensity { 0.5f };
        float Gravity { 9.8f };
        float VelocityDamping { 0.18f };
        float BendingStiffness { 0.04f };
        float MaxVelocity { 20.f };

        std::vector<glm::vec3> Positions;
        std::vector<glm::vec3> RestPositions;
        std::vector<glm::vec2> RestUV;
        std::vector<glm::vec3> Velocities;
        std::vector<glm::vec3> Forces;
        std::vector<float>     Masses;
        std::vector<bool>      Fixed;
        std::vector<Triangle>  Triangles;
        std::vector<BendEdge>  BendEdges;

        std::vector<std::uint32_t> SurfaceTriangles;
        std::vector<std::uint32_t> SurfaceLines;

        void ResetSheet(GridResolution resolution, glm::vec2 size);
        void RecomputeMasses();
        void Step(float dt, std::size_t controlParticle, glm::vec3 const & controlForce);

        std::size_t GetID(std::size_t i, std::size_t j) const;
        std::size_t FindNearestRestParticle(glm::vec3 const & p) const;

    private:
        GridResolution _resolution {};

        void AddParticle(glm::vec3 const & position, glm::vec2 const & uv, bool fixed);
        void AddTriangle(std::size_t i0, std::size_t i1, std::size_t i2);
        void BuildLinesAndBending();
    };

} // namespace VCX::Labs::FEM
