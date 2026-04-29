#include "FluidSimulator.h"

#include <array>
#include <cmath>
#include <limits>

namespace VCX::Labs::Fluid {

    namespace {
        constexpr int   EMPTY_CELL = 0;
        constexpr int   FLUID_CELL = 1;
        constexpr int   SOLID_CELL = 2;
        constexpr float kEps       = 1e-6f;

        float & component(glm::vec3 & value, int dir) {
            return dir == 0 ? value.x : (dir == 1 ? value.y : value.z);
        }

        float component(glm::vec3 const & value, int dir) {
            return dir == 0 ? value.x : (dir == 1 ? value.y : value.z);
        }

        glm::vec3 obstacleHalfExtent(Simulator const & sim, float padding = 0.0f) {
            if (sim.m_obstacleShape == ObstacleShape::Sphere) {
                return glm::vec3(sim.m_obstacleRadius + padding);
            }
            return glm::vec3(sim.m_obstacleRadius + padding);
        }

        bool containsObstacle(Simulator const & sim, glm::vec3 const & point, float padding = 0.0f) {
            glm::vec3 const delta = point - sim.m_obstaclePos;
            if (sim.m_obstacleShape == ObstacleShape::Sphere) {
                float const radius = sim.m_obstacleRadius + padding;
                return glm::dot(delta, delta) <= radius * radius;
            }

            glm::vec3 const halfExtent = obstacleHalfExtent(sim, padding);
            return std::abs(delta.x) <= halfExtent.x && std::abs(delta.y) <= halfExtent.y && std::abs(delta.z) <= halfExtent.z;
        }
    } // namespace

    namespace {
        glm::vec3 cellCenter(Simulator const & sim, glm::ivec3 index) {
            return glm::vec3(index) * sim.m_h + glm::vec3(0.5f * sim.m_h - 0.5f);
        }

        glm::vec3 faceCenter(Simulator const & sim, glm::ivec3 index, int dir) {
            glm::vec3 center = glm::vec3(index) * sim.m_h + glm::vec3(-0.5f);
            center[(dir + 1) % 3] += 0.5f * sim.m_h;
            center[(dir + 2) % 3] += 0.5f * sim.m_h;
            return center;
        }

        template<typename Fn>
        void forEachCellWeight(Simulator const & sim, glm::vec3 const & pos, Fn && fn) {
            glm::vec3 const coord = (pos + glm::vec3(0.5f)) / sim.m_h - glm::vec3(0.5f);
            glm::ivec3 const base = glm::ivec3(glm::floor(coord));
            glm::vec3 const frac  = coord - glm::vec3(base);

            std::array<std::pair<int, float>, 8> entries {};
            int                                  count = 0;
            float                                wsum  = 0.0f;

            for (int dx = 0; dx <= 1; ++dx) {
                for (int dy = 0; dy <= 1; ++dy) {
                    for (int dz = 0; dz <= 1; ++dz) {
                        glm::ivec3 const index = base + glm::ivec3(dx, dy, dz);
                        if (! sim.isValidCell(index)) {
                            continue;
                        }
                        float const wx = dx ? frac.x : (1.0f - frac.x);
                        float const wy = dy ? frac.y : (1.0f - frac.y);
                        float const wz = dz ? frac.z : (1.0f - frac.z);
                        float const w  = wx * wy * wz;
                        if (w <= 0.0f) {
                            continue;
                        }
                        entries[count++] = { sim.index2GridOffset(index), w };
                        wsum += w;
                    }
                }
            }

            if (wsum <= kEps) {
                return;
            }
            for (int i = 0; i < count; ++i) {
                fn(entries[i].first, entries[i].second / wsum);
            }
        }

        template<typename Fn>
        void forEachFaceWeight(Simulator const & sim, glm::vec3 const & pos, int dir, Fn && fn) {
            glm::vec3 coord = (pos + glm::vec3(0.5f)) / sim.m_h;
            if (dir != 0) coord.x -= 0.5f;
            if (dir != 1) coord.y -= 0.5f;
            if (dir != 2) coord.z -= 0.5f;

            glm::ivec3 const base = glm::ivec3(glm::floor(coord));
            glm::vec3 const frac  = coord - glm::vec3(base);

            std::array<std::pair<int, float>, 8> entries {};
            int                                  count = 0;
            float                                wsum  = 0.0f;

            for (int dx = 0; dx <= 1; ++dx) {
                for (int dy = 0; dy <= 1; ++dy) {
                    for (int dz = 0; dz <= 1; ++dz) {
                        glm::ivec3 const index = base + glm::ivec3(dx, dy, dz);
                        if (! sim.isValidVelocity(index.x, index.y, index.z, dir)) {
                            continue;
                        }
                        float const wx = dx ? frac.x : (1.0f - frac.x);
                        float const wy = dy ? frac.y : (1.0f - frac.y);
                        float const wz = dz ? frac.z : (1.0f - frac.z);
                        float const w  = wx * wy * wz;
                        if (w <= 0.0f) {
                            continue;
                        }
                        entries[count++] = { sim.index2GridOffset(index), w };
                        wsum += w;
                    }
                }
            }

            if (wsum <= kEps) {
                return;
            }
            for (int i = 0; i < count; ++i) {
                fn(entries[i].first, entries[i].second / wsum);
            }
        }

        void rebuildSolidMask(Simulator & sim) {
            std::fill(sim.m_s.begin(), sim.m_s.end(), 0.0f);
            for (int i = 0; i < sim.cellCountX(); ++i) {
                for (int j = 0; j < sim.cellCountY(); ++j) {
                    for (int k = 0; k < sim.cellCountZ(); ++k) {
                        glm::ivec3 const index(i, j, k);
                        bool const boundary =
                            i == 0 || j == 0 || k == 0 ||
                            i == sim.cellCountX() - 1 ||
                            j == sim.cellCountY() - 1 ||
                            k == sim.cellCountZ() - 1;

                        bool obstacleSolid = false;
                        if (sim.m_enableObstacle) {
                            glm::vec3 const center = cellCenter(sim, index);
                            obstacleSolid = containsObstacle(sim, center, 0.5f * sim.m_h);
                        }

                        sim.m_s[sim.index2GridOffset(index)] = (boundary || obstacleSolid) ? 0.0f : 1.0f;
                    }
                }
            }
        }

        bool isObstacleFace(Simulator const & sim, glm::ivec3 faceIndex, int dir) {
            if (! sim.m_enableObstacle) {
                return false;
            }
            glm::vec3 const center = faceCenter(sim, faceIndex, dir);
            return containsObstacle(sim, center, 0.35f * sim.m_h);
        }

        void applyBoundaryVelocities(Simulator & sim) {
            for (int dir = 0; dir < 3; ++dir) {
                for (int i = 0; i < sim.m_iCellX; ++i) {
                    for (int j = 0; j < sim.m_iCellY; ++j) {
                        for (int k = 0; k < sim.m_iCellZ; ++k) {
                            if (! sim.isValidVelocity(i, j, k, dir)) {
                                continue;
                            }

                            glm::ivec3 const face(i, j, k);
                            glm::ivec3 left  = face;
                            glm::ivec3 right = face;
                            left[dir] -= 1;

                            bool const leftSolid  = ! sim.isValidCell(left) || sim.m_s[sim.index2GridOffset(left)] == 0.0f;
                            bool const rightSolid = ! sim.isValidCell(right) || sim.m_s[sim.index2GridOffset(right)] == 0.0f;
                            bool const wallFace   = leftSolid || rightSolid;

                            if (! wallFace) {
                                continue;
                            }

                            float target = 0.0f;
                            if (isObstacleFace(sim, face, dir)) {
                                target = component(sim.m_obstacleVel, dir);
                            }
                            component(sim.m_vel[sim.index2GridOffset(face)], dir) = target;
                        }
                    }
                }
            }
        }

        void rebuildParticleBins(Simulator & sim) {
            std::fill(sim.m_hashtableindex.begin(), sim.m_hashtableindex.end(), 0);

            auto particleCell = [&](glm::vec3 const & pos) {
                glm::vec3 coord = (pos + glm::vec3(0.5f)) / sim.m_h;
                glm::ivec3 cell = glm::ivec3(glm::floor(coord));
                cell.x          = std::clamp(cell.x, 0, sim.cellCountX() - 1);
                cell.y          = std::clamp(cell.y, 0, sim.cellCountY() - 1);
                cell.z          = std::clamp(cell.z, 0, sim.cellCountZ() - 1);
                return cell;
            };

            for (glm::vec3 const & pos : sim.m_particlePos) {
                glm::ivec3 const cell = particleCell(pos);
                sim.m_hashtableindex[sim.index2GridOffset(cell) + 1]++;
            }

            for (int i = 1; i <= sim.m_iNumCells; ++i) {
                sim.m_hashtableindex[i] += sim.m_hashtableindex[i - 1];
            }

            std::vector<int> next = sim.m_hashtableindex;
            for (int p = 0; p < sim.m_iNumSpheres; ++p) {
                glm::ivec3 const cell = particleCell(sim.m_particlePos[p]);
                sim.m_hashtable[next[sim.index2GridOffset(cell)]++] = p;
            }
        }
    } // namespace

    void Simulator::setObstacle(ObstacleShape shape, glm::vec3 const & position, glm::vec3 const & velocity, float radius, bool enabled) {
        m_obstacleShape  = shape;
        m_obstaclePos    = position;
        m_obstacleVel    = velocity;
        m_obstacleRadius = radius;
        m_enableObstacle = enabled;
    }

    void Simulator::integrateParticles(float timeStep) {
        float const maxVelocity = 3.0f / std::max(m_h, kEps);

        for (int i = 0; i < m_iNumSpheres; ++i) {
            m_particleVel[i] += gravity * timeStep;

            float const speed = glm::length(m_particleVel[i]);
            if (speed > maxVelocity) {
                m_particleVel[i] *= maxVelocity / speed;
            }

            m_particlePos[i] += m_particleVel[i] * timeStep;
        }
    }

    void Simulator::handleParticleCollisions(glm::vec3 obstaclePos, float obstacleRadius, glm::vec3 obstacleVel) {
        float const minBound = -0.5f + m_h + m_particleRadius;
        float const maxBound = 0.5f - m_h - m_particleRadius;

        for (int i = 0; i < m_iNumSpheres; ++i) {
            glm::vec3 & pos = m_particlePos[i];
            glm::vec3 & vel = m_particleVel[i];

            for (int dir = 0; dir < 3; ++dir) {
                if (pos[dir] < minBound) {
                    pos[dir] = minBound;
                    vel[dir] = 0.0f;
                } else if (pos[dir] > maxBound) {
                    pos[dir] = maxBound;
                    vel[dir] = 0.0f;
                }
            }

            if (obstacleRadius <= 0.0f) {
                continue;
            }

            if (m_obstacleShape == ObstacleShape::Sphere) {
                glm::vec3 delta = pos - obstaclePos;
                float     dist2 = glm::dot(delta, delta);
                float     minDist = obstacleRadius + m_particleRadius;
                if (dist2 >= minDist * minDist) {
                    continue;
                }

                float dist = std::sqrt(std::max(dist2, kEps));
                glm::vec3 normal = dist > kEps ? delta / dist : glm::vec3(0.0f, 1.0f, 0.0f);
                pos              = obstaclePos + normal * minDist;

                glm::vec3 relVel = vel - obstacleVel;
                float     vn     = glm::dot(relVel, normal);
                if (vn < 0.0f) {
                    relVel -= vn * normal;
                }
                vel = relVel + obstacleVel;
                continue;
            }

            glm::vec3 const halfExtent = glm::vec3(obstacleRadius + m_particleRadius);
            glm::vec3 const local = pos - obstaclePos;
            if (std::abs(local.x) > halfExtent.x || std::abs(local.y) > halfExtent.y || std::abs(local.z) > halfExtent.z) {
                continue;
            }

            glm::vec3 penetration = halfExtent - glm::abs(local);
            int       axis = 0;
            if (penetration.y < penetration.x) axis = 1;
            if (penetration.z < penetration[axis]) axis = 2;

            float sign = local[axis] >= 0.0f ? 1.0f : -1.0f;
            if (std::abs(local[axis]) <= kEps) {
                sign = vel[axis] >= obstacleVel[axis] ? 1.0f : -1.0f;
            }

            glm::vec3 normal(0.0f);
            normal[axis] = sign;
            pos[axis] = obstaclePos[axis] + sign * halfExtent[axis];

            glm::vec3 relVel = vel - obstacleVel;
            float     vn     = glm::dot(relVel, normal);
            if (vn < 0.0f) {
                relVel -= vn * normal;
            }
            vel = relVel + obstacleVel;
        }
    }

    void Simulator::pushParticlesApart(int numIters) {
        rebuildParticleBins(*this);

        float const minDist = 2.0f * m_particleRadius;
        float const minDist2 = minDist * minDist;

        for (int iter = 0; iter < numIters; ++iter) {
            for (int p = 0; p < m_iNumSpheres; ++p) {
                glm::vec3 const & pos = m_particlePos[p];
                glm::vec3 coord       = (pos + glm::vec3(0.5f)) / m_h;
                glm::ivec3 cell       = glm::ivec3(glm::floor(coord));
                cell.x                = std::clamp(cell.x, 0, cellCountX() - 1);
                cell.y                = std::clamp(cell.y, 0, cellCountY() - 1);
                cell.z                = std::clamp(cell.z, 0, cellCountZ() - 1);

                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dz = -1; dz <= 1; ++dz) {
                            glm::ivec3 const neighbor = cell + glm::ivec3(dx, dy, dz);
                            if (! isValidCell(neighbor)) {
                                continue;
                            }

                            int const begin = m_hashtableindex[index2GridOffset(neighbor)];
                            int const end   = m_hashtableindex[index2GridOffset(neighbor) + 1];
                            for (int cursor = begin; cursor < end; ++cursor) {
                                int const q = m_hashtable[cursor];
                                if (q <= p) {
                                    continue;
                                }

                                glm::vec3 delta = m_particlePos[p] - m_particlePos[q];
                                float     dist2 = glm::dot(delta, delta);
                                if (dist2 >= minDist2 || dist2 <= kEps) {
                                    continue;
                                }

                                float     dist = std::sqrt(dist2);
                                glm::vec3 push = 0.5f * (minDist - dist) * delta / dist;
                                m_particlePos[p] += push;
                                m_particlePos[q] -= push;
                            }
                        }
                    }
                }
            }

            handleParticleCollisions(m_obstaclePos, m_enableObstacle ? m_obstacleRadius : 0.0f, m_obstacleVel);
            rebuildParticleBins(*this);
        }
    }

    void Simulator::updateParticleDensity() {
        std::fill(m_particleDensity.begin(), m_particleDensity.end(), 0.0f);

        for (glm::vec3 const & pos : m_particlePos) {
            forEachCellWeight(*this, pos, [&](int gridOffset, float weight) {
                m_particleDensity[gridOffset] += weight;
            });
        }

        if (m_particleRestDensity > 0.0f) {
            return;
        }

        float sum   = 0.0f;
        int   count = 0;
        for (int i = 0; i < cellCountX(); ++i) {
            for (int j = 0; j < cellCountY(); ++j) {
                for (int k = 0; k < cellCountZ(); ++k) {
                    int const idx = index2GridOffset(glm::ivec3(i, j, k));
                    if (m_type[idx] != FLUID_CELL || m_particleDensity[idx] <= 0.0f) {
                        continue;
                    }
                    sum += m_particleDensity[idx];
                    ++count;
                }
            }
        }
        if (count > 0) {
            m_particleRestDensity = sum / static_cast<float>(count);
        }
    }

    void Simulator::transferVelocities(bool toGrid, float flipRatio) {
        if (toGrid) {
            rebuildSolidMask(*this);

            std::fill(m_vel.begin(), m_vel.end(), glm::vec3(0.0f));
            for (int dir = 0; dir < 3; ++dir) {
                std::fill(m_near_num[dir].begin(), m_near_num[dir].end(), 0.0f);
            }

            std::fill(m_type.begin(), m_type.end(), SOLID_CELL);
            for (int i = 0; i < cellCountX(); ++i) {
                for (int j = 0; j < cellCountY(); ++j) {
                    for (int k = 0; k < cellCountZ(); ++k) {
                        int const idx = index2GridOffset(glm::ivec3(i, j, k));
                        m_type[idx]   = (m_s[idx] == 0.0f) ? SOLID_CELL : EMPTY_CELL;
                    }
                }
            }

            for (glm::vec3 const & pos : m_particlePos) {
                glm::ivec3 cell = glm::ivec3(glm::floor((pos + glm::vec3(0.5f)) / m_h));
                cell.x          = std::clamp(cell.x, 0, cellCountX() - 1);
                cell.y          = std::clamp(cell.y, 0, cellCountY() - 1);
                cell.z          = std::clamp(cell.z, 0, cellCountZ() - 1);

                int const idx = index2GridOffset(cell);
                if (m_type[idx] == EMPTY_CELL) {
                    m_type[idx] = FLUID_CELL;
                }
            }

            for (int p = 0; p < m_iNumSpheres; ++p) {
                for (int dir = 0; dir < 3; ++dir) {
                    float const particleComponent = component(m_particleVel[p], dir);
                    forEachFaceWeight(*this, m_particlePos[p], dir, [&](int gridOffset, float weight) {
                        component(m_vel[gridOffset], dir) += particleComponent * weight;
                        m_near_num[dir][gridOffset] += weight;
                    });
                }
            }

            for (int dir = 0; dir < 3; ++dir) {
                for (int i = 0; i < m_iNumCells; ++i) {
                    if (m_near_num[dir][i] > 0.0f) {
                        component(m_vel[i], dir) /= m_near_num[dir][i];
                    } else {
                        component(m_vel[i], dir) = 0.0f;
                    }
                }
            }

            applyBoundaryVelocities(*this);
            return;
        }

        float const maxVelocity = 3.0f / std::max(m_h, kEps);
        for (int p = 0; p < m_iNumSpheres; ++p) {
            for (int dir = 0; dir < 3; ++dir) {
                float pic = 0.0f;
                float flip = component(m_particleVel[p], dir);
                float wsum = 0.0f;

                forEachFaceWeight(*this, m_particlePos[p], dir, [&](int gridOffset, float weight) {
                    pic += component(m_vel[gridOffset], dir) * weight;
                    flip += (component(m_vel[gridOffset], dir) - component(m_pre_vel[gridOffset], dir)) * weight;
                    wsum += weight;
                });

                if (wsum > 0.0f) {
                    component(m_particleVel[p], dir) = (1.0f - flipRatio) * pic + flipRatio * flip;
                }
            }

            float const speed = glm::length(m_particleVel[p]);
            if (speed > maxVelocity) {
                m_particleVel[p] *= maxVelocity / speed;
            }
        }
    }

    void Simulator::solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift) {
        m_pre_vel = m_vel;
        std::fill(m_p.begin(), m_p.end(), 0.0f);

        float const pressureScale = m_h / std::max(dt, kEps);

        for (int iter = 0; iter < numIters; ++iter) {
            for (int i = 1; i < cellCountX() - 1; ++i) {
                for (int j = 1; j < cellCountY() - 1; ++j) {
                    for (int k = 1; k < cellCountZ() - 1; ++k) {
                        glm::ivec3 const cell(i, j, k);
                        int const         idx = index2GridOffset(cell);
                        if (m_type[idx] != FLUID_CELL) {
                            continue;
                        }

                        float div =
                            m_vel[index2GridOffset(glm::ivec3(i + 1, j, k))].x - m_vel[index2GridOffset(glm::ivec3(i, j, k))].x +
                            m_vel[index2GridOffset(glm::ivec3(i, j + 1, k))].y - m_vel[index2GridOffset(glm::ivec3(i, j, k))].y +
                            m_vel[index2GridOffset(glm::ivec3(i, j, k + 1))].z - m_vel[index2GridOffset(glm::ivec3(i, j, k))].z;

                        if (compensateDrift && m_particleRestDensity > 0.0f) {
                            float const compression = m_particleDensity[idx] - m_particleRestDensity;
                            if (compression > 0.0f) {
                                div -= 0.8f * compression;
                            }
                        }

                        float const sx0 = m_s[index2GridOffset(glm::ivec3(i - 1, j, k))];
                        float const sx1 = m_s[index2GridOffset(glm::ivec3(i + 1, j, k))];
                        float const sy0 = m_s[index2GridOffset(glm::ivec3(i, j - 1, k))];
                        float const sy1 = m_s[index2GridOffset(glm::ivec3(i, j + 1, k))];
                        float const sz0 = m_s[index2GridOffset(glm::ivec3(i, j, k - 1))];
                        float const sz1 = m_s[index2GridOffset(glm::ivec3(i, j, k + 1))];
                        float const s   = sx0 + sx1 + sy0 + sy1 + sz0 + sz1;
                        if (s <= kEps) {
                            continue;
                        }

                        float const pressureDelta = -overRelaxation * div / s;
                        m_p[idx] += pressureScale * pressureDelta;

                        m_vel[index2GridOffset(glm::ivec3(i, j, k))].x -= sx0 * pressureDelta;
                        m_vel[index2GridOffset(glm::ivec3(i + 1, j, k))].x += sx1 * pressureDelta;
                        m_vel[index2GridOffset(glm::ivec3(i, j, k))].y -= sy0 * pressureDelta;
                        m_vel[index2GridOffset(glm::ivec3(i, j + 1, k))].y += sy1 * pressureDelta;
                        m_vel[index2GridOffset(glm::ivec3(i, j, k))].z -= sz0 * pressureDelta;
                        m_vel[index2GridOffset(glm::ivec3(i, j, k + 1))].z += sz1 * pressureDelta;
                    }
                }
            }
            applyBoundaryVelocities(*this);
        }
    }

    void Simulator::updateParticleColors() {
        float maxSpeed = 1e-4f;
        for (int i = 0; i < m_iNumSpheres; ++i) {
            maxSpeed = std::max(maxSpeed, glm::length(m_particleVel[i]));
        }

        for (int i = 0; i < m_iNumSpheres; ++i) {
            float const speed01 = std::clamp(1.35f * glm::length(m_particleVel[i]) / maxSpeed, 0.0f, 1.0f);
            float const tint    = 0.08f + 0.72f * std::pow(speed01, 0.65f);
            glm::vec3 const deepBlue(0.02f, 0.08f, 0.38f);
            glm::vec3 const lightBlue(0.32f, 0.56f, 0.92f);
            m_particleColor[i] = glm::mix(deepBlue, lightBlue, tint);
        }
    }

} // namespace VCX::Labs::Fluid
