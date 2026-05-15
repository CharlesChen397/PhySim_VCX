#include "Labs/3-FEM/FEMSystem.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <utility>

#include <glm/gtc/matrix_inverse.hpp>

namespace VCX::Labs::FEM {

    namespace {
        struct FaceKey {
            std::array<std::uint32_t, 3> V {};

            bool operator==(FaceKey const & other) const { return V == other.V; }
        };

        struct FaceKeyHash {
            std::size_t operator()(FaceKey const & key) const noexcept {
                std::size_t h = 0;
                for (auto v : key.V) {
                    h ^= std::hash<std::uint32_t> {}(v + 0x9e3779b9u + (h << 6) + (h >> 2));
                }
                return h;
            }
        };

        FaceKey makeFace(std::uint32_t a, std::uint32_t b, std::uint32_t c) {
            std::array<std::uint32_t, 3> v { a, b, c };
            std::sort(v.begin(), v.end());
            return FaceKey { v };
        }

        glm::mat3 makeColumnMat(glm::vec3 const & c0, glm::vec3 const & c1, glm::vec3 const & c2) {
            return glm::mat3(c0, c1, c2);
        }

        glm::vec3 col(glm::mat3 const & m, int i) {
            return glm::vec3(m[i]);
        }

        float length2(glm::vec3 const & v) {
            return glm::dot(v, v);
        }

        float traceMat(glm::mat3 const & m) {
            return m[0][0] + m[1][1] + m[2][2];
        }

        glm::vec3 safeNormalize(glm::vec3 const & v) {
            float const len2 = length2(v);
            if (len2 <= 1e-20f) return glm::vec3(0.f);
            return v * (1.f / std::sqrt(len2));
        }
    } // namespace

    std::size_t FEMSystem::GetID(std::size_t i, std::size_t j, std::size_t k) const {
        return i * (_resolution.Y + 1) * (_resolution.Z + 1) + j * (_resolution.Z + 1) + k;
    }

    void FEMSystem::AddParticle(glm::vec3 const & position, bool fixed) {
        Positions.push_back(position);
        RestPositions.push_back(position);
        Velocities.emplace_back(0.f);
        Forces.emplace_back(0.f);
        Masses.push_back(0.f);
        Fixed.push_back(fixed);
    }

    void FEMSystem::AddTet(std::size_t i0, std::size_t i1, std::size_t i2, std::size_t i3) {
        glm::mat3 Dm = makeColumnMat(
            RestPositions[i1] - RestPositions[i0],
            RestPositions[i2] - RestPositions[i0],
            RestPositions[i3] - RestPositions[i0]);
        float     det = glm::determinant(Dm);
        if (det < 0.f) {
            std::swap(i1, i2);
            Dm = makeColumnMat(
                RestPositions[i1] - RestPositions[i0],
                RestPositions[i2] - RestPositions[i0],
                RestPositions[i3] - RestPositions[i0]);
            det = glm::determinant(Dm);
        }

        float const volume = std::abs(det) / 6.f;
        glm::mat3 const invRest = glm::inverse(Dm);

        Tets.push_back(Tet {
            .Idx        = { i0, i1, i2, i3 },
            .InvRest    = invRest,
            .RestVolume = volume,
        });
    }

    void FEMSystem::BuildBoundarySurface() {
        std::unordered_map<FaceKey, std::array<std::uint32_t, 3>, FaceKeyHash> faces;
        std::unordered_map<FaceKey, std::size_t, FaceKeyHash>                  counts;

        auto addFace = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c) {
            FaceKey const key = makeFace(a, b, c);
            counts[key] += 1;
            faces.try_emplace(key, std::array<std::uint32_t, 3> { a, b, c });
        };

        for (auto const & tet : Tets) {
            auto const a = static_cast<std::uint32_t>(tet.Idx[0]);
            auto const b = static_cast<std::uint32_t>(tet.Idx[1]);
            auto const c = static_cast<std::uint32_t>(tet.Idx[2]);
            auto const d = static_cast<std::uint32_t>(tet.Idx[3]);
            addFace(a, b, c);
            addFace(a, c, d);
            addFace(a, d, b);
            addFace(b, d, c);
        }

        SurfaceTriangles.clear();
        SurfaceLines.clear();

        std::unordered_map<std::uint64_t, bool> lineSeen;
        auto lineKey = [](std::uint32_t a, std::uint32_t b) {
            if (a > b) std::swap(a, b);
            return (std::uint64_t(a) << 32) | std::uint64_t(b);
        };

        for (auto const & [key, count] : counts) {
            if (count != 1) continue;
            auto tri = faces.at(key);
            SurfaceTriangles.push_back(tri[0]);
            SurfaceTriangles.push_back(tri[1]);
            SurfaceTriangles.push_back(tri[2]);

            auto addLine = [&](std::uint32_t a, std::uint32_t b) {
                auto const lk = lineKey(a, b);
                if (lineSeen.emplace(lk, true).second) {
                    SurfaceLines.push_back(a);
                    SurfaceLines.push_back(b);
                }
            };
            addLine(tri[0], tri[1]);
            addLine(tri[1], tri[2]);
            addLine(tri[2], tri[0]);
        }
    }

    void FEMSystem::RecomputeMasses() {
        std::fill(Masses.begin(), Masses.end(), 0.f);
        for (auto const & tet : Tets) {
            float const m = tet.RestVolume * Density;
            float const share = m / 4.f;
            for (auto idx : tet.Idx) {
                Masses[idx] += share;
            }
        }
        for (std::size_t i = 0; i < Masses.size(); ++i) {
            if (Fixed[i]) {
                Masses[i] = std::numeric_limits<float>::infinity();
                Velocities[i] = glm::vec3(0.f);
            } else {
                Masses[i] = std::max(Masses[i], 1e-6f);
            }
        }
    }

    void FEMSystem::ResetBlock(GridResolution resolution, glm::vec3 size) {
        _resolution = resolution;

        Positions.clear();
        RestPositions.clear();
        Velocities.clear();
        Forces.clear();
        Masses.clear();
        Fixed.clear();
        Tets.clear();
        SurfaceTriangles.clear();
        SurfaceLines.clear();

        glm::vec3 const origin = -0.5f * size;
        glm::vec3 const delta {
            size.x / float(_resolution.X),
            size.y / float(_resolution.Y),
            size.z / float(_resolution.Z),
        };

        for (std::size_t i = 0; i <= _resolution.X; ++i) {
            for (std::size_t j = 0; j <= _resolution.Y; ++j) {
                for (std::size_t k = 0; k <= _resolution.Z; ++k) {
                    glm::vec3 const p = origin + glm::vec3(float(i) * delta.x, float(j) * delta.y, float(k) * delta.z);
                    bool const fixed = i == 0;
                    AddParticle(p, fixed);
                }
            }
        }

        for (std::size_t i = 0; i < _resolution.X; ++i) {
            for (std::size_t j = 0; j < _resolution.Y; ++j) {
                for (std::size_t k = 0; k < _resolution.Z; ++k) {
                    AddTet(GetID(i, j, k), GetID(i, j, k + 1), GetID(i, j + 1, k + 1), GetID(i + 1, j + 1, k + 1));
                    AddTet(GetID(i, j, k), GetID(i, j + 1, k), GetID(i, j + 1, k + 1), GetID(i + 1, j + 1, k + 1));
                    AddTet(GetID(i, j, k), GetID(i, j, k + 1), GetID(i + 1, j, k + 1), GetID(i + 1, j + 1, k + 1));
                    AddTet(GetID(i, j, k), GetID(i + 1, j, k), GetID(i + 1, j, k + 1), GetID(i + 1, j + 1, k + 1));
                    AddTet(GetID(i, j, k), GetID(i, j + 1, k), GetID(i + 1, j + 1, k), GetID(i + 1, j + 1, k + 1));
                    AddTet(GetID(i, j, k), GetID(i + 1, j, k), GetID(i + 1, j + 1, k), GetID(i + 1, j + 1, k + 1));
                }
            }
        }

        BuildBoundarySurface();
        RecomputeMasses();
    }

    std::size_t FEMSystem::FindNearestRestParticle(glm::vec3 const & p) const {
        std::size_t best = 0;
        float       bestDist2 = std::numeric_limits<float>::infinity();
        for (std::size_t i = 0; i < RestPositions.size(); ++i) {
            float const d2 = length2(RestPositions[i] - p);
            if (d2 < bestDist2) {
                best = i;
                bestDist2 = d2;
            }
        }
        return best;
    }

    void FEMSystem::Step(float dt, std::size_t controlParticle, glm::vec3 const & controlForce) {
        if (Positions.empty() || dt <= 0.f) return;

        if (controlParticle >= Forces.size()) {
            controlParticle = std::numeric_limits<std::size_t>::max();
        }

        std::fill(Forces.begin(), Forces.end(), glm::vec3(0.f));

        glm::vec3 const gravityForce(0.f, -Gravity, 0.f);
        float const     mu = YoungModulus / (2.f * (1.f + PoissonRatio));
        float const     lambda = YoungModulus * PoissonRatio / ((1.f + PoissonRatio) * (1.f - 2.f * PoissonRatio));

        for (std::size_t i = 0; i < Positions.size(); ++i) {
            if (Fixed[i]) continue;
            Forces[i] += Masses[i] * gravityForce;
            Forces[i] -= VelocityDamping * Masses[i] * Velocities[i];
        }

        for (auto const & tet : Tets) {
            std::size_t const i0 = tet.Idx[0];
            std::size_t const i1 = tet.Idx[1];
            std::size_t const i2 = tet.Idx[2];
            std::size_t const i3 = tet.Idx[3];

            glm::mat3 const Ds = makeColumnMat(
                Positions[i1] - Positions[i0],
                Positions[i2] - Positions[i0],
                Positions[i3] - Positions[i0]);
            glm::mat3 const F = Ds * tet.InvRest;
            glm::mat3 const C = glm::transpose(F) * F;
            glm::mat3 const G = 0.5f * (C - glm::mat3(1.f));
            glm::mat3 const S = 2.f * mu * G + lambda * traceMat(G) * glm::mat3(1.f);
            glm::mat3 const P = F * S;
            glm::mat3 const H = -tet.RestVolume * P * glm::transpose(tet.InvRest);

            Forces[i1] += col(H, 0);
            Forces[i2] += col(H, 1);
            Forces[i3] += col(H, 2);
            Forces[i0] -= col(H, 0) + col(H, 1) + col(H, 2);
        }

        if (controlParticle < Forces.size() && !Fixed[controlParticle]) {
            Forces[controlParticle] += controlForce;
        }

        for (std::size_t i = 0; i < Positions.size(); ++i) {
            if (Fixed[i]) {
                Velocities[i] = glm::vec3(0.f);
                Positions[i] = RestPositions[i];
                continue;
            }

            glm::vec3 const accel = Forces[i] / Masses[i];
            Velocities[i] += accel * dt;
            float const speed = glm::length(Velocities[i]);
            if (speed > MaxVelocity) {
                Velocities[i] *= MaxVelocity / speed;
            }
            Positions[i] += Velocities[i] * dt;
        }
    }

} // namespace VCX::Labs::FEM
