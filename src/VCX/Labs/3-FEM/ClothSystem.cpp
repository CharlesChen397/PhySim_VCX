#include "Labs/3-FEM/ClothSystem.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace VCX::Labs::FEM {

    namespace {
        std::uint64_t edgeKey(std::uint32_t a, std::uint32_t b) {
            if (a > b) std::swap(a, b);
            return (std::uint64_t(a) << 32) | std::uint64_t(b);
        }

        float length2(glm::vec3 const & v) {
            return glm::dot(v, v);
        }

        float traceMat(glm::mat2 const & m) {
            return m[0][0] + m[1][1];
        }

        glm::vec3 mat2x3Col0(glm::mat2x3 const & m) {
            return glm::vec3(m[0]);
        }

        glm::vec3 mat2x3Col1(glm::mat2x3 const & m) {
            return glm::vec3(m[1]);
        }
    } // namespace

    std::size_t ClothSystem::GetID(std::size_t i, std::size_t j) const {
        return i * (_resolution.Y + 1) + j;
    }

    void ClothSystem::AddParticle(glm::vec3 const & position, glm::vec2 const & uv, bool fixed) {
        Positions.push_back(position);
        RestPositions.push_back(position);
        RestUV.push_back(uv);
        Velocities.emplace_back(0.f);
        Forces.emplace_back(0.f);
        Masses.push_back(0.f);
        Fixed.push_back(fixed);
    }

    void ClothSystem::AddTriangle(std::size_t i0, std::size_t i1, std::size_t i2) {
        glm::mat2 Dm(RestUV[i1] - RestUV[i0], RestUV[i2] - RestUV[i0]);
        float det = glm::determinant(Dm);
        if (det < 0.f) {
            std::swap(i1, i2);
            Dm = glm::mat2(RestUV[i1] - RestUV[i0], RestUV[i2] - RestUV[i0]);
            det = glm::determinant(Dm);
        }
        float const area = std::abs(det) * 0.5f;
        Triangles.push_back(Triangle {
            .Idx      = { i0, i1, i2 },
            .InvRest  = glm::inverse(Dm),
            .RestArea = area,
        });
    }

    void ClothSystem::BuildLinesAndBending() {
        SurfaceTriangles.clear();
        SurfaceLines.clear();
        BendEdges.clear();

        std::unordered_map<std::uint64_t, bool> seen;
        auto addLine = [&](std::uint32_t a, std::uint32_t b) {
            auto const key = edgeKey(a, b);
            if (seen.emplace(key, true).second) {
                SurfaceLines.push_back(a);
                SurfaceLines.push_back(b);
            }
        };

        for (auto const & tri : Triangles) {
            auto const a = static_cast<std::uint32_t>(tri.Idx[0]);
            auto const b = static_cast<std::uint32_t>(tri.Idx[1]);
            auto const c = static_cast<std::uint32_t>(tri.Idx[2]);
            SurfaceTriangles.insert(SurfaceTriangles.end(), { a, b, c });
            addLine(a, b);
            addLine(b, c);
            addLine(c, a);
        }

        for (std::size_t i = 0; i <= _resolution.X; ++i) {
            for (std::size_t j = 0; j <= _resolution.Y; ++j) {
                if (i + 2 <= _resolution.X) {
                    std::size_t const a = GetID(i, j);
                    std::size_t const b = GetID(i + 2, j);
                    BendEdges.push_back({ a, b, glm::length(RestPositions[a] - RestPositions[b]) });
                }
                if (j + 2 <= _resolution.Y) {
                    std::size_t const a = GetID(i, j);
                    std::size_t const b = GetID(i, j + 2);
                    BendEdges.push_back({ a, b, glm::length(RestPositions[a] - RestPositions[b]) });
                }
            }
        }
    }

    void ClothSystem::RecomputeMasses() {
        std::fill(Masses.begin(), Masses.end(), 0.f);
        for (auto const & tri : Triangles) {
            float const share = tri.RestArea * AreaDensity / 3.f;
            for (auto idx : tri.Idx) {
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

    void ClothSystem::ResetSheet(GridResolution resolution, glm::vec2 size) {
        _resolution = resolution;
        Positions.clear();
        RestPositions.clear();
        RestUV.clear();
        Velocities.clear();
        Forces.clear();
        Masses.clear();
        Fixed.clear();
        Triangles.clear();
        BendEdges.clear();
        SurfaceTriangles.clear();
        SurfaceLines.clear();

        glm::vec2 const origin = -0.5f * size;
        glm::vec2 const delta(size.x / float(_resolution.X), size.y / float(_resolution.Y));

        for (std::size_t i = 0; i <= _resolution.X; ++i) {
            for (std::size_t j = 0; j <= _resolution.Y; ++j) {
                glm::vec2 const uv = origin + glm::vec2(float(i) * delta.x, float(j) * delta.y);
                glm::vec3 const p(uv.x, 1.2f, uv.y);
                bool const fixed = (i == 0 && (j == 0 || j == _resolution.Y));
                AddParticle(p, uv, fixed);
            }
        }

        for (std::size_t i = 0; i < _resolution.X; ++i) {
            for (std::size_t j = 0; j < _resolution.Y; ++j) {
                AddTriangle(GetID(i, j), GetID(i + 1, j), GetID(i + 1, j + 1));
                AddTriangle(GetID(i, j), GetID(i + 1, j + 1), GetID(i, j + 1));
            }
        }

        BuildLinesAndBending();
        RecomputeMasses();
    }

    std::size_t ClothSystem::FindNearestRestParticle(glm::vec3 const & p) const {
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

    void ClothSystem::Step(float dt, std::size_t controlParticle, glm::vec3 const & controlForce) {
        if (Positions.empty() || dt <= 0.f) return;
        std::fill(Forces.begin(), Forces.end(), glm::vec3(0.f));

        float const mu = YoungModulus / (2.f * (1.f + PoissonRatio));
        float const lambda = YoungModulus * PoissonRatio / ((1.f + PoissonRatio) * (1.f - PoissonRatio));

        for (std::size_t i = 0; i < Positions.size(); ++i) {
            if (Fixed[i]) continue;
            Forces[i] += Masses[i] * glm::vec3(0.f, -Gravity, 0.f);
            Forces[i] -= VelocityDamping * Masses[i] * Velocities[i];
        }

        for (auto const & tri : Triangles) {
            std::size_t const i0 = tri.Idx[0];
            std::size_t const i1 = tri.Idx[1];
            std::size_t const i2 = tri.Idx[2];

            glm::mat2x3 const Ds(Positions[i1] - Positions[i0], Positions[i2] - Positions[i0]);
            glm::mat2x3 const F = Ds * tri.InvRest;
            glm::mat2 const C = glm::transpose(F) * F;
            glm::mat2 const G = 0.5f * (C - glm::mat2(1.f));
            glm::mat2 const S = 2.f * mu * G + lambda * traceMat(G) * glm::mat2(1.f);
            glm::mat2x3 const P = F * S;
            glm::mat2x3 const H = -tri.RestArea * P * glm::transpose(tri.InvRest);

            glm::vec3 const f1 = mat2x3Col0(H);
            glm::vec3 const f2 = mat2x3Col1(H);
            Forces[i1] += f1;
            Forces[i2] += f2;
            Forces[i0] -= f1 + f2;
        }

        for (auto const & edge : BendEdges) {
            glm::vec3 const x = Positions[edge.I1] - Positions[edge.I0];
            float const     len = glm::length(x);
            if (len <= 1e-7f) continue;
            glm::vec3 const dir = x / len;
            glm::vec3 const f = BendingStiffness * (len - edge.RestLength) * dir;
            if (!Fixed[edge.I0]) Forces[edge.I0] += f;
            if (!Fixed[edge.I1]) Forces[edge.I1] -= f;
        }

        if (controlParticle < Forces.size() && !Fixed[controlParticle]) {
            Forces[controlParticle] += controlForce;
        }

        for (std::size_t i = 0; i < Positions.size(); ++i) {
            if (Fixed[i]) {
                Positions[i] = RestPositions[i];
                Velocities[i] = glm::vec3(0.f);
                continue;
            }
            Velocities[i] += (Forces[i] / Masses[i]) * dt;
            float const speed = glm::length(Velocities[i]);
            if (speed > MaxVelocity) Velocities[i] *= MaxVelocity / speed;
            Positions[i] += Velocities[i] * dt;
        }
    }

} // namespace VCX::Labs::FEM
