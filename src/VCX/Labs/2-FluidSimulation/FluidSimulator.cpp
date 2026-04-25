#include "FluidSimulator.h"

namespace VCX::Labs::Fluid {

    void Simulator::integrateParticles(float timeStep) {
        const float maxVelocity = 10.0f; // 限制最大速度

        for (int i = 0; i < m_iNumSpheres; i++) {
            m_particleVel[i] += gravity * timeStep;

            // 限制速度大小防止爆炸
            float speed = glm::length(m_particleVel[i]);
            if (speed > maxVelocity) {
                m_particleVel[i] *= maxVelocity / speed;
            }

            m_particlePos[i] += m_particleVel[i] * timeStep;
        }
    }

    void Simulator::handleParticleCollisions(glm::vec3 obstaclePos, float obstacleRadius, glm::vec3 obstacleVel) {
        for (int i = 0; i < m_iNumSpheres; i++) {
            glm::vec3& pos = m_particlePos[i];
            glm::vec3& vel = m_particleVel[i];

            // 边界碰撞 - 使用正确的坐标计算
            if (pos.x < 0.5f * m_h + m_particleRadius - 0.5f) {
                pos.x = 0.5f * m_h + m_particleRadius - 0.5f;
                vel.x = 0.0f;
            }
            if (pos.x > 0.5f - 0.5f * m_h - m_particleRadius) {
                pos.x = 0.5f - 0.5f * m_h - m_particleRadius;
                vel.x = 0.0f;
            }

            if (pos.y < 0.5f * m_h + m_particleRadius - 0.5f) {
                pos.y = 0.5f * m_h + m_particleRadius - 0.5f;
                vel.y = 0.0f;
            }
            if (pos.y > 0.5f - 0.5f * m_h - m_particleRadius) {
                pos.y = 0.5f - 0.5f * m_h - m_particleRadius;
                vel.y = 0.0f;
            }

            if (pos.z < 0.5f * m_h + m_particleRadius - 0.5f) {
                pos.z = 0.5f * m_h + m_particleRadius - 0.5f;
                vel.z = 0.0f;
            }
            if (pos.z > 0.5f - 0.5f * m_h - m_particleRadius) {
                pos.z = 0.5f - 0.5f * m_h - m_particleRadius;
                vel.z = 0.0f;
            }

            // 障碍物碰撞
            if (obstacleRadius > 0.0f) {
                glm::vec3 delta = pos - obstaclePos;
                float dist = glm::length(delta);
                if (dist < m_particleRadius + obstacleRadius) {
                    glm::vec3 normal = delta / dist;
                    pos = obstaclePos + (m_particleRadius + obstacleRadius) * normal;
                    glm::vec3 deltaVel = vel - obstacleVel;
                    float deltaVelN = glm::dot(deltaVel, normal);
                    if (deltaVelN < 0) {
                        vel -= deltaVelN * normal;
                    }
                }
            }
        }
    }

    void Simulator::pushParticlesApart(int numIters) {
        // 使用简化的哈希表实现
        for (int iter = 0; iter < numIters; iter++) {
            for (int i = 0; i < m_iNumSpheres; i++) {
                for (int j = i + 1; j < m_iNumSpheres; j++) {
                    glm::vec3 d = m_particlePos[i] - m_particlePos[j];
                    float dist = glm::length(d + 1e-6f);
                    if (dist < 2.0f * m_particleRadius) {
                        glm::vec3 s = 0.5f * (2.0f * m_particleRadius - dist) / dist * d;
                        m_particlePos[i] += s;
                        m_particlePos[j] -= s;
                    }
                }
            }
        }
    }

    void Simulator::updateParticleDensity() {
        std::fill(m_particleDensity.begin(), m_particleDensity.end(), 0.0f);

        for (int i = 0; i < m_iNumSpheres; i++) {
            glm::vec3 pos = m_particlePos[i];
            // 关键：使用 0.5f + 0.5f * m_h 作为偏移
            glm::vec3 rel_pos = pos + (0.5f + 0.5f * m_h);
            glm::ivec3 index = glm::ivec3(rel_pos / m_h);
            glm::vec3 delta = (rel_pos - glm::vec3(index) * m_h) / m_h;
            glm::vec3 deltaComp = glm::vec3(1.0f) - delta;

            // 三线性插值到8个邻居
            m_particleDensity[index2GridOffset(index + glm::ivec3(0, 0, 0))] += deltaComp.x * deltaComp.y * deltaComp.z;
            m_particleDensity[index2GridOffset(index + glm::ivec3(1, 0, 0))] += delta.x * deltaComp.y * deltaComp.z;
            m_particleDensity[index2GridOffset(index + glm::ivec3(0, 1, 0))] += deltaComp.x * delta.y * deltaComp.z;
            m_particleDensity[index2GridOffset(index + glm::ivec3(1, 1, 0))] += delta.x * delta.y * deltaComp.z;
            m_particleDensity[index2GridOffset(index + glm::ivec3(0, 0, 1))] += deltaComp.x * deltaComp.y * delta.z;
            m_particleDensity[index2GridOffset(index + glm::ivec3(1, 0, 1))] += delta.x * deltaComp.y * delta.z;
            m_particleDensity[index2GridOffset(index + glm::ivec3(0, 1, 1))] += deltaComp.x * delta.y * delta.z;
            m_particleDensity[index2GridOffset(index + glm::ivec3(1, 1, 1))] += delta.x * delta.y * delta.z;
        }

        // 计算静止密度
        if (m_particleRestDensity == 0.0f) {
            float sum = 0.0f;
            int count = 0;
            for (int i = 0; i < m_iNumCells; i++) {
                if (m_type[i] == 1 && m_particleDensity[i] > 0.0f) {
                    sum += m_particleDensity[i];
                    count++;
                }
            }
            if (count > 0) {
                m_particleRestDensity = sum / count;
            }
        }
    }

    void Simulator::transferVelocities(bool toGrid, float flipRatio) {
        if (toGrid) {
            // 清空网格速度
            std::fill(m_vel.begin(), m_vel.end(), glm::vec3(0.0f));
            for (int i = 0; i < 3; i++) {
                std::fill(m_near_num[i].begin(), m_near_num[i].end(), 0.0f);
            }

            // 标记cell类型
            int n = m_iCellY * m_iCellZ;
            int m_idx = m_iCellZ;
            for (int i = 0; i < m_iCellX; i++) {
                for (int j = 0; j < m_iCellY; j++) {
                    for (int k = 0; k < m_iCellZ; k++) {
                        int index = i * n + j * m_idx + k;
                        if (m_s[index] == 0.0f) {
                            m_type[index] = 2; // SOLID
                        } else {
                            m_type[index] = 0; // EMPTY
                        }
                    }
                }
            }
        }

        // P2G 和 G2P
        for (int p = 0; p < m_iNumSpheres; p++) {
            glm::vec3 pos = m_particlePos[p];

            // 标记粒子所在cell为流体
            if (toGrid) {
                glm::ivec3 index = glm::ivec3((pos + (0.5f + 0.5f * m_h)) / m_h);
                if (m_type[index2GridOffset(index)] == 0) {
                    m_type[index2GridOffset(index)] = 1; // FLUID
                }
            }

            // 对每个速度分量
            for (int dir = 0; dir < 3; dir++) {
                // 关键：offset计算
                glm::vec3 offset = glm::vec3(-0.5f);
                offset[dir] -= m_h * 0.5f;  // 注意是减去！

                glm::vec3 rel_pos = pos - offset;
                glm::ivec3 index = glm::ivec3(rel_pos / m_h);
                glm::vec3 delta = (rel_pos - glm::vec3(index) * m_h) / m_h;
                glm::vec3 deltaComp = glm::vec3(1.0f) - delta;

                if (toGrid) {
                    // P2G
                    float vel = m_particleVel[p][dir];
                    m_vel[index2GridOffset(index + glm::ivec3(0, 0, 0))][dir] += vel * deltaComp.x * deltaComp.y * deltaComp.z;
                    m_vel[index2GridOffset(index + glm::ivec3(1, 0, 0))][dir] += vel * delta.x * deltaComp.y * deltaComp.z;
                    m_vel[index2GridOffset(index + glm::ivec3(0, 1, 0))][dir] += vel * deltaComp.x * delta.y * deltaComp.z;
                    m_vel[index2GridOffset(index + glm::ivec3(1, 1, 0))][dir] += vel * delta.x * delta.y * deltaComp.z;
                    m_vel[index2GridOffset(index + glm::ivec3(0, 0, 1))][dir] += vel * deltaComp.x * deltaComp.y * delta.z;
                    m_vel[index2GridOffset(index + glm::ivec3(1, 0, 1))][dir] += vel * delta.x * deltaComp.y * delta.z;
                    m_vel[index2GridOffset(index + glm::ivec3(0, 1, 1))][dir] += vel * deltaComp.x * delta.y * delta.z;
                    m_vel[index2GridOffset(index + glm::ivec3(1, 1, 1))][dir] += vel * delta.x * delta.y * delta.z;

                    m_near_num[dir][index2GridOffset(index + glm::ivec3(0, 0, 0))] += deltaComp.x * deltaComp.y * deltaComp.z;
                    m_near_num[dir][index2GridOffset(index + glm::ivec3(1, 0, 0))] += delta.x * deltaComp.y * deltaComp.z;
                    m_near_num[dir][index2GridOffset(index + glm::ivec3(0, 1, 0))] += deltaComp.x * delta.y * deltaComp.z;
                    m_near_num[dir][index2GridOffset(index + glm::ivec3(1, 1, 0))] += delta.x * delta.y * deltaComp.z;
                    m_near_num[dir][index2GridOffset(index + glm::ivec3(0, 0, 1))] += deltaComp.x * deltaComp.y * delta.z;
                    m_near_num[dir][index2GridOffset(index + glm::ivec3(1, 0, 1))] += delta.x * deltaComp.y * delta.z;
                    m_near_num[dir][index2GridOffset(index + glm::ivec3(0, 1, 1))] += deltaComp.x * delta.y * delta.z;
                    m_near_num[dir][index2GridOffset(index + glm::ivec3(1, 1, 1))] += delta.x * delta.y * delta.z;
                } else {
                    // G2P
                    float vel = 0;
                    vel += m_vel[index2GridOffset(index + glm::ivec3(0, 0, 0))][dir] * deltaComp.x * deltaComp.y * deltaComp.z;
                    vel += m_vel[index2GridOffset(index + glm::ivec3(1, 0, 0))][dir] * delta.x * deltaComp.y * deltaComp.z;
                    vel += m_vel[index2GridOffset(index + glm::ivec3(0, 1, 0))][dir] * deltaComp.x * delta.y * deltaComp.z;
                    vel += m_vel[index2GridOffset(index + glm::ivec3(1, 1, 0))][dir] * delta.x * delta.y * deltaComp.z;
                    vel += m_vel[index2GridOffset(index + glm::ivec3(0, 0, 1))][dir] * deltaComp.x * deltaComp.y * delta.z;
                    vel += m_vel[index2GridOffset(index + glm::ivec3(1, 0, 1))][dir] * delta.x * deltaComp.y * delta.z;
                    vel += m_vel[index2GridOffset(index + glm::ivec3(0, 1, 1))][dir] * deltaComp.x * delta.y * delta.z;
                    vel += m_vel[index2GridOffset(index + glm::ivec3(1, 1, 1))][dir] * delta.x * delta.y * delta.z;

                    float deltaVel = 0;
                    deltaVel += (m_vel[index2GridOffset(index + glm::ivec3(0, 0, 0))][dir] - m_pre_vel[index2GridOffset(index + glm::ivec3(0, 0, 0))][dir]) * deltaComp.x * deltaComp.y * deltaComp.z;
                    deltaVel += (m_vel[index2GridOffset(index + glm::ivec3(1, 0, 0))][dir] - m_pre_vel[index2GridOffset(index + glm::ivec3(1, 0, 0))][dir]) * delta.x * deltaComp.y * deltaComp.z;
                    deltaVel += (m_vel[index2GridOffset(index + glm::ivec3(0, 1, 0))][dir] - m_pre_vel[index2GridOffset(index + glm::ivec3(0, 1, 0))][dir]) * deltaComp.x * delta.y * deltaComp.z;
                    deltaVel += (m_vel[index2GridOffset(index + glm::ivec3(1, 1, 0))][dir] - m_pre_vel[index2GridOffset(index + glm::ivec3(1, 1, 0))][dir]) * delta.x * delta.y * deltaComp.z;
                    deltaVel += (m_vel[index2GridOffset(index + glm::ivec3(0, 0, 1))][dir] - m_pre_vel[index2GridOffset(index + glm::ivec3(0, 0, 1))][dir]) * deltaComp.x * deltaComp.y * delta.z;
                    deltaVel += (m_vel[index2GridOffset(index + glm::ivec3(1, 0, 1))][dir] - m_pre_vel[index2GridOffset(index + glm::ivec3(1, 0, 1))][dir]) * delta.x * deltaComp.y * delta.z;
                    deltaVel += (m_vel[index2GridOffset(index + glm::ivec3(0, 1, 1))][dir] - m_pre_vel[index2GridOffset(index + glm::ivec3(0, 1, 1))][dir]) * deltaComp.x * delta.y * delta.z;
                    deltaVel += (m_vel[index2GridOffset(index + glm::ivec3(1, 1, 1))][dir] - m_pre_vel[index2GridOffset(index + glm::ivec3(1, 1, 1))][dir]) * delta.x * delta.y * delta.z;

                    // FLIP95混合
                    m_particleVel[p][dir] = (deltaVel + m_particleVel[p][dir]) * flipRatio + (1 - flipRatio) * vel;
                }
            }
        }

        // 归一化网格速度
        if (toGrid) {
            for (int i = 0; i < m_iNumCells; i++) {
                for (int dir = 0; dir < 3; dir++) {
                    if (m_near_num[dir][i] > 0.0f) {
                        m_vel[i][dir] /= m_near_num[dir][i];
                    }
                }
            }
        }
    }

    void Simulator::solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift) {
        // 保存旧速度用于FLIP
        m_pre_vel = m_vel;

        for (int iter = 0; iter < numIters; iter++) {
            for (int i = 1; i < m_iCellX - 1; i++) {
                for (int j = 1; j < m_iCellY - 1; j++) {
                    for (int k = 1; k < m_iCellZ - 1; k++) {
                        int idx = index2GridOffset(glm::ivec3(i, j, k));

                        if (m_type[idx] != 1) continue; // 只处理流体cell

                        // 计算散度
                        float div =
                            m_vel[index2GridOffset(glm::ivec3(i + 1, j, k))].x - m_vel[idx].x +
                            m_vel[index2GridOffset(glm::ivec3(i, j + 1, k))].y - m_vel[idx].y +
                            m_vel[index2GridOffset(glm::ivec3(i, j, k + 1))].z - m_vel[idx].z;

                        // 漂移补偿 - 使用更小的系数
                        if (compensateDrift && m_particleRestDensity > 0.0f) {
                            float compression = m_particleDensity[idx] - m_particleRestDensity;
                            if (compression > 0.0f) {
                                div -= 0.5f * compression; // 降低补偿强度
                            }
                        }

                        // 计算邻居的固体标记
                        float s =
                            m_s[index2GridOffset(glm::ivec3(i - 1, j, k))] +
                            m_s[index2GridOffset(glm::ivec3(i + 1, j, k))] +
                            m_s[index2GridOffset(glm::ivec3(i, j - 1, k))] +
                            m_s[index2GridOffset(glm::ivec3(i, j + 1, k))] +
                            m_s[index2GridOffset(glm::ivec3(i, j, k - 1))] +
                            m_s[index2GridOffset(glm::ivec3(i, j, k + 1))];

                        if (s < 0.01f) continue;

                        // 压力修正
                        float p = -div / s * overRelaxation;

                        // 更新速度
                        m_vel[idx].x -= p * m_s[index2GridOffset(glm::ivec3(i - 1, j, k))];
                        m_vel[index2GridOffset(glm::ivec3(i + 1, j, k))].x += p * m_s[index2GridOffset(glm::ivec3(i + 1, j, k))];
                        m_vel[idx].y -= p * m_s[index2GridOffset(glm::ivec3(i, j - 1, k))];
                        m_vel[index2GridOffset(glm::ivec3(i, j + 1, k))].y += p * m_s[index2GridOffset(glm::ivec3(i, j + 1, k))];
                        m_vel[idx].z -= p * m_s[index2GridOffset(glm::ivec3(i, j, k - 1))];
                        m_vel[index2GridOffset(glm::ivec3(i, j, k + 1))].z += p * m_s[index2GridOffset(glm::ivec3(i, j, k + 1))];
                    }
                }
            }
        }
    }

    void Simulator::updateParticleColors() {
        // 计算最大速度用于归一化
        float maxSpeed = 0.001f;
        for (int i = 0; i < m_iNumSpheres; i++) {
            float speed = glm::length(m_particleVel[i]);
            if (speed > maxSpeed) maxSpeed = speed;
        }

        // 按速度染色：蓝色(慢) -> 青色 -> 绿色 -> 黄色 -> 红色(快)
        for (int i = 0; i < m_iNumSpheres; i++) {
            float speed = glm::length(m_particleVel[i]);
            float t = speed / maxSpeed;

            // 使用HSV到RGB的转换，色相从蓝色(240度)到红色(0度)
            float hue = (1.0f - t) * 240.0f;
            float h = hue / 60.0f;
            int hi = int(h) % 6;
            float f = h - int(h);

            float v = 1.0f;
            float s = 0.8f;
            float p = v * (1.0f - s);
            float q = v * (1.0f - f * s);
            float r = v * (1.0f - (1.0f - f) * s);

            switch(hi) {
                case 0: m_particleColor[i] = glm::vec3(v, r, p); break;
                case 1: m_particleColor[i] = glm::vec3(q, v, p); break;
                case 2: m_particleColor[i] = glm::vec3(p, v, r); break;
                case 3: m_particleColor[i] = glm::vec3(p, q, v); break;
                case 4: m_particleColor[i] = glm::vec3(r, p, v); break;
                case 5: m_particleColor[i] = glm::vec3(v, p, q); break;
            }
        }
    }

    bool Simulator::isValidVelocity(int i, int j, int k, int dir) {
        glm::ivec3 index(i, j, k);
        if (m_type[index2GridOffset(index)] == 2) return false; // SOLID
        index[dir] += 1;
        if (m_type[index2GridOffset(index)] == 2) return false; // SOLID
        return true;
    }

    int Simulator::index2GridOffset(glm::ivec3 index) {
        return index.x * m_iCellY * m_iCellZ + index.y * m_iCellZ + index.z;
    }

} // namespace VCX::Labs::Fluid
