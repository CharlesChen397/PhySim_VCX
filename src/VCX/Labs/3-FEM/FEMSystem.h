#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

namespace VCX::Labs::FEM {

    class FEMSystem {
    public:
        struct Tet {
            std::array<std::size_t, 4> Idx {};
            glm::mat3                  InvRest { 1.f };
            float                      RestVolume { 0.f };
        };

        struct GridResolution {
            std::size_t X { 8 };
            std::size_t Y { 2 };
            std::size_t Z { 2 };
        };

        float YoungModulus { 20000.f };
        float PoissonRatio { 0.2f };
        float Density { 400.f };
        float Gravity { 0.05f };
        float VelocityDamping { 1.2f };
        float MaxVelocity { 25.f };

        std::vector<glm::vec3> Positions;
        std::vector<glm::vec3> RestPositions;
        std::vector<glm::vec3> Velocities;
        std::vector<glm::vec3> Forces;
        std::vector<float>     Masses;
        std::vector<bool>      Fixed;
        std::vector<Tet>       Tets;

        std::vector<std::uint32_t> SurfaceTriangles;
        std::vector<std::uint32_t> SurfaceLines;

        void ResetBlock(GridResolution resolution, glm::vec3 size);
        void RecomputeMasses();
        void Step(float dt, std::size_t controlParticle, glm::vec3 const & controlForce);

        std::size_t GetID(std::size_t i, std::size_t j, std::size_t k) const;
        std::size_t FindNearestRestParticle(glm::vec3 const & p) const;

    private:
        GridResolution _resolution {};

        void AddParticle(glm::vec3 const & position, bool fixed);
        void AddTet(std::size_t i0, std::size_t i1, std::size_t i2, std::size_t i3);
        void BuildBoundarySurface();
    };

} // namespace VCX::Labs::FEM
