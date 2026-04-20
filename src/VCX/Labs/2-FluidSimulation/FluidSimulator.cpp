#include "FluidSimulator.h"

namespace VCX::Labs::Fluid {

    // 粒子积分：更新速度和位置
    void Simulator::integrateParticles(float timeStep) {
        for (int i = 0; i < m_iNumSpheres; i++) {
            m_particleVel[i] += timeStep * gravity;
            m_particlePos[i] += timeStep * m_particleVel[i];
        }
    }

    // 处理粒子与边界的碰撞
    void Simulator::handleParticleCollisions(glm::vec3 obstaclePos, float obstacleRadius, glm::vec3 obstacleVel) {
        float minX = m_h + m_particleRadius;
        float maxX = (m_iCellX - 2) * m_h - m_particleRadius;
        float minY = m_h + m_particleRadius;
        float maxY = (m_iCellY - 2) * m_h - m_particleRadius;
        float minZ = m_h + m_particleRadius;
        float maxZ = (m_iCellZ - 2) * m_h - m_particleRadius;

        for (int i = 0; i < m_iNumSpheres; i++) {
            glm::vec3& pos = m_particlePos[i];
            glm::vec3& vel = m_particleVel[i];

            // X边界
            if (pos.x < minX) {
                pos.x = minX;
                vel.x = 0.0f;
            }
            if (pos.x > maxX) {
                pos.x = maxX;
                vel.x = 0.0f;
            }

            // Y边界
            if (pos.y < minY) {
                pos.y = minY;
                vel.y = 0.0f;
            }
            if (pos.y > maxY) {
                pos.y = maxY;
                vel.y = 0.0f;
            }

            // Z边界
            if (pos.z < minZ) {
                pos.z = minZ;
                vel.z = 0.0f;
            }
            if (pos.z > maxZ) {
                pos.z = maxZ;
                vel.z = 0.0f;
            }

            // 球形障碍物碰撞（如果半径>0）
            if (obstacleRadius > 0.0f) {
                glm::vec3 dir = pos - obstaclePos;
                float dist = glm::length(dir);
                float minDist = obstacleRadius + m_particleRadius;

                if (dist < minDist && dist > 0.0f) {
                    dir /= dist;
                    pos = obstaclePos + dir * minDist;
                    // 反射速度
                    glm::vec3 relVel = vel - obstacleVel;
                    float normalVel = glm::dot(relVel, dir);
                    if (normalVel < 0.0f) {
                        vel -= dir * normalVel;
                        vel += obstacleVel;
                    }
                }
            }
        }
    }

    // 粒子分离：防止粒子重叠
    void Simulator::pushParticlesApart(int numIters) {
        float minDist = 2.0f * m_particleRadius;
        float minDist2 = minDist * minDist;

        // 构建空间哈希表
        float cellSize = 2.2f * m_particleRadius;
        float invCellSize = 1.0f / cellSize;

        // 清空哈希表
        std::fill(m_hashtableindex.begin(), m_hashtableindex.end(), 0);

        // 第一遍：计算每个cell的粒子数量
        for (int i = 0; i < m_iNumSpheres; i++) {
            glm::vec3 pos = m_particlePos[i];
            int xi = (int)std::floor(pos.x * invCellSize);
            int yi = (int)std::floor(pos.y * invCellSize);
            int zi = (int)std::floor(pos.z * invCellSize);

            // 映射到网格索引
            int hashIdx = ((xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481)) % m_iNumCells;
            if (hashIdx < 0) hashIdx += m_iNumCells;

            m_hashtableindex[hashIdx]++;
        }

        // 计算前缀和
        int sum = 0;
        for (int i = 0; i < m_iNumCells; i++) {
            int count = m_hashtableindex[i];
            m_hashtableindex[i] = sum;
            sum += count;
        }
        m_hashtableindex[m_iNumCells] = sum;

        // 第二遍：填充哈希表
        for (int i = 0; i < m_iNumSpheres; i++) {
            glm::vec3 pos = m_particlePos[i];
            int xi = (int)std::floor(pos.x * invCellSize);
            int yi = (int)std::floor(pos.y * invCellSize);
            int zi = (int)std::floor(pos.z * invCellSize);

            int hashIdx = ((xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481)) % m_iNumCells;
            if (hashIdx < 0) hashIdx += m_iNumCells;

            m_hashtable[m_hashtableindex[hashIdx]++] = i;
        }

        // 恢复索引
        for (int i = m_iNumCells; i > 0; i--) {
            m_hashtableindex[i] = m_hashtableindex[i - 1];
        }
        m_hashtableindex[0] = 0;

        // Gauss-Seidel迭代分离粒子
        for (int iter = 0; iter < numIters; iter++) {
            for (int i = 0; i < m_iNumSpheres; i++) {
                glm::vec3 pos = m_particlePos[i];

                int xi = (int)std::floor(pos.x * invCellSize);
                int yi = (int)std::floor(pos.y * invCellSize);
                int zi = (int)std::floor(pos.z * invCellSize);

                // 检查周围27个cell
                for (int dx = -1; dx <= 1; dx++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dz = -1; dz <= 1; dz++) {
                            int nxi = xi + dx;
                            int nyi = yi + dy;
                            int nzi = zi + dz;

                            int hashIdx = ((nxi * 92837111) ^ (nyi * 689287499) ^ (nzi * 283923481)) % m_iNumCells;
                            if (hashIdx < 0) hashIdx += m_iNumCells;

                            int start = m_hashtableindex[hashIdx];
                            int end = m_hashtableindex[hashIdx + 1];

                            for (int idx = start; idx < end; idx++) {
                                int j = m_hashtable[idx];
                                if (j <= i) continue;

                                glm::vec3 dir = pos - m_particlePos[j];
                                float dist2 = glm::dot(dir, dir);

                                if (dist2 > 0.0f && dist2 < minDist2) {
                                    float dist = std::sqrt(dist2);
                                    glm::vec3 correction = dir * (0.5f * (minDist - dist) / dist);
                                    m_particlePos[i] += correction;
                                    m_particlePos[j] -= correction;
                                    pos = m_particlePos[i]; // 更新当前粒子位置
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // 更新粒子密度
    void Simulator::updateParticleDensity() {
        // 清空密度
        std::fill(m_particleDensity.begin(), m_particleDensity.end(), 0.0f);

        // 计算每个cell中心的粒子数量
        for (int p = 0; p < m_iNumSpheres; p++) {
            glm::vec3 pos = m_particlePos[p];

            // cell中心偏移半格
            glm::vec3 gridPos = (pos - glm::vec3(m_h * 0.5f)) * m_fInvSpacing;

            int x0 = (int)std::floor(gridPos.x);
            int y0 = (int)std::floor(gridPos.y);
            int z0 = (int)std::floor(gridPos.z);

            float tx = gridPos.x - x0;
            float ty = gridPos.y - y0;
            float tz = gridPos.z - z0;

            // 对周围8个cell进行插值
            for (int dx = 0; dx <= 1; dx++) {
                for (int dy = 0; dy <= 1; dy++) {
                    for (int dz = 0; dz <= 1; dz++) {
                        int xi = x0 + dx;
                        int yi = y0 + dy;
                        int zi = z0 + dz;

                        if (xi < 0 || xi >= m_iCellX || yi < 0 || yi >= m_iCellY || zi < 0 || zi >= m_iCellZ)
                            continue;

                        float wx = (dx == 0) ? (1.0f - tx) : tx;
                        float wy = (dy == 0) ? (1.0f - ty) : ty;
                        float wz = (dz == 0) ? (1.0f - tz) : tz;
                        float weight = wx * wy * wz;

                        int idx = index2GridOffset(glm::ivec3(xi, yi, zi));
                        m_particleDensity[idx] += weight;
                    }
                }
            }
        }

        // 如果还没有设置静止密度，计算平均密度
        if (m_particleRestDensity == 0.0f) {
            float sum = 0.0f;
            int count = 0;
            for (int i = 0; i < m_iNumCells; i++) {
                if (m_type[i] == 1 && m_particleDensity[i] > 0.0f) { // FLUID_CELL
                    sum += m_particleDensity[i];
                    count++;
                }
            }
            if (count > 0) {
                m_particleRestDensity = sum / count;
            }
        }
    }

    // 速度传递：粒子<->网格
    void Simulator::transferVelocities(bool toGrid, float flipRatio) {
        if (toGrid) {
            // Particle to Grid (P2G)
            // 保存旧速度用于FLIP
            m_pre_vel = m_vel;

            // 清空网格速度和权重
            std::fill(m_vel.begin(), m_vel.end(), glm::vec3(0.0f));
            for (int i = 0; i < 3; i++) {
                std::fill(m_near_num[i].begin(), m_near_num[i].end(), 0.0f);
            }

            // 标记cell类型
            std::fill(m_type.begin(), m_type.end(), 0); // EMPTY_CELL
            for (int i = 0; i < m_iNumCells; i++) {
                if (m_s[i] == 0.0f) {
                    m_type[i] = 2; // SOLID_CELL
                }
            }

            // 遍历所有粒子，将速度传递到网格
            for (int p = 0; p < m_iNumSpheres; p++) {
                glm::vec3 pos = m_particlePos[p];
                glm::vec3 vel = m_particleVel[p];

                // 对于每个速度分量（u, v, w），使用交错网格
                for (int comp = 0; comp < 3; comp++) {
                    // 计算粒子在该速度分量网格上的位置
                    glm::vec3 offset(0.0f);
                    if (comp == 0) offset.x = 0.0f;      // u在x面上
                    else if (comp == 1) offset.y = 0.0f; // v在y面上
                    else offset.z = 0.0f;                // w在z面上

                    // 其他两个方向偏移半格
                    if (comp != 0) offset.x = m_h * 0.5f;
                    if (comp != 1) offset.y = m_h * 0.5f;
                    if (comp != 2) offset.z = m_h * 0.5f;

                    glm::vec3 gridPos = (pos - offset) * m_fInvSpacing;

                    // 找到左下角网格索引
                    int x0 = (int)std::floor(gridPos.x);
                    int y0 = (int)std::floor(gridPos.y);
                    int z0 = (int)std::floor(gridPos.z);

                    // 计算插值权重（线性插值）
                    float tx = gridPos.x - x0;
                    float ty = gridPos.y - y0;
                    float tz = gridPos.z - z0;

                    // 对周围8个网格点进行插值
                    for (int dx = 0; dx <= 1; dx++) {
                        for (int dy = 0; dy <= 1; dy++) {
                            for (int dz = 0; dz <= 1; dz++) {
                                int xi = x0 + dx;
                                int yi = y0 + dy;
                                int zi = z0 + dz;

                                if (xi < 0 || xi >= m_iCellX || yi < 0 || yi >= m_iCellY || zi < 0 || zi >= m_iCellZ)
                                    continue;

                                // 线性插值权重
                                float wx = (dx == 0) ? (1.0f - tx) : tx;
                                float wy = (dy == 0) ? (1.0f - ty) : ty;
                                float wz = (dz == 0) ? (1.0f - tz) : tz;
                                float weight = wx * wy * wz;

                                int idx = index2GridOffset(glm::ivec3(xi, yi, zi));

                                if (comp == 0) m_vel[idx].x += weight * vel.x;
                                else if (comp == 1) m_vel[idx].y += weight * vel.y;
                                else m_vel[idx].z += weight * vel.z;

                                m_near_num[comp][idx] += weight;

                                // 标记为流体cell
                                if (m_s[idx] == 1.0f) {
                                    m_type[idx] = 1; // FLUID_CELL
                                }
                            }
                        }
                    }
                }
            }

            // 归一化网格速度
            for (int i = 0; i < m_iNumCells; i++) {
                for (int comp = 0; comp < 3; comp++) {
                    if (m_near_num[comp][i] > 0.0f) {
                        if (comp == 0) m_vel[i].x /= m_near_num[comp][i];
                        else if (comp == 1) m_vel[i].y /= m_near_num[comp][i];
                        else m_vel[i].z /= m_near_num[comp][i];
                    }
                }
            }

        } else {
            // Grid to Particle (G2P)
            for (int p = 0; p < m_iNumSpheres; p++) {
                glm::vec3 pos = m_particlePos[p];
                glm::vec3 picVel(0.0f);
                glm::vec3 flipVel = m_particleVel[p];

                for (int comp = 0; comp < 3; comp++) {
                    // 计算粒子在该速度分量网格上的位置
                    glm::vec3 offset(0.0f);
                    if (comp == 0) offset.x = 0.0f;
                    else if (comp == 1) offset.y = 0.0f;
                    else offset.z = 0.0f;

                    if (comp != 0) offset.x = m_h * 0.5f;
                    if (comp != 1) offset.y = m_h * 0.5f;
                    if (comp != 2) offset.z = m_h * 0.5f;

                    glm::vec3 gridPos = (pos - offset) * m_fInvSpacing;

                    int x0 = (int)std::floor(gridPos.x);
                    int y0 = (int)std::floor(gridPos.y);
                    int z0 = (int)std::floor(gridPos.z);

                    float tx = gridPos.x - x0;
                    float ty = gridPos.y - y0;
                    float tz = gridPos.z - z0;

                    float picComponent = 0.0f;
                    float flipDelta = 0.0f;
                    float totalWeight = 0.0f;

                    for (int dx = 0; dx <= 1; dx++) {
                        for (int dy = 0; dy <= 1; dy++) {
                            for (int dz = 0; dz <= 1; dz++) {
                                int xi = x0 + dx;
                                int yi = y0 + dy;
                                int zi = z0 + dz;

                                if (xi < 0 || xi >= m_iCellX || yi < 0 || yi >= m_iCellY || zi < 0 || zi >= m_iCellZ)
                                    continue;

                                float wx = (dx == 0) ? (1.0f - tx) : tx;
                                float wy = (dy == 0) ? (1.0f - ty) : ty;
                                float wz = (dz == 0) ? (1.0f - tz) : tz;
                                float weight = wx * wy * wz;

                                int idx = index2GridOffset(glm::ivec3(xi, yi, zi));

                                float newVel, oldVel;
                                if (comp == 0) {
                                    newVel = m_vel[idx].x;
                                    oldVel = m_pre_vel[idx].x;
                                } else if (comp == 1) {
                                    newVel = m_vel[idx].y;
                                    oldVel = m_pre_vel[idx].y;
                                } else {
                                    newVel = m_vel[idx].z;
                                    oldVel = m_pre_vel[idx].z;
                                }

                                picComponent += weight * newVel;
                                flipDelta += weight * (newVel - oldVel);
                                totalWeight += weight;
                            }
                        }
                    }

                    if (totalWeight > 0.0f) {
                        picComponent /= totalWeight;
                        flipDelta /= totalWeight;
                    }

                    // FLIP95混合
                    float oldComponent = (comp == 0) ? m_particleVel[p].x :
                                        (comp == 1) ? m_particleVel[p].y : m_particleVel[p].z;
                    float newComponent = (1.0f - flipRatio) * picComponent + flipRatio * (oldComponent + flipDelta);

                    if (comp == 0) m_particleVel[p].x = newComponent;
                    else if (comp == 1) m_particleVel[p].y = newComponent;
                    else m_particleVel[p].z = newComponent;
                }
            }
        }
    }

    // 求解不可压缩性
    void Simulator::solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift) {
        // 清空压力
        std::fill(m_p.begin(), m_p.end(), 0.0f);

        // Gauss-Seidel迭代
        for (int iter = 0; iter < numIters; iter++) {
            for (int i = 1; i < m_iCellX - 1; i++) {
                for (int j = 1; j < m_iCellY - 1; j++) {
                    for (int k = 1; k < m_iCellZ - 1; k++) {
                        int idx = index2GridOffset(glm::ivec3(i, j, k));

                        if (m_type[idx] != 1) continue; // 只处理流体cell

                        // 计算散度
                        int idx_xp = index2GridOffset(glm::ivec3(i + 1, j, k));
                        int idx_xm = index2GridOffset(glm::ivec3(i - 1, j, k));
                        int idx_yp = index2GridOffset(glm::ivec3(i, j + 1, k));
                        int idx_ym = index2GridOffset(glm::ivec3(i, j - 1, k));
                        int idx_zp = index2GridOffset(glm::ivec3(i, j, k + 1));
                        int idx_zm = index2GridOffset(glm::ivec3(i, j, k - 1));

                        float sx0 = m_s[idx_xm];
                        float sx1 = m_s[idx];
                        float sy0 = m_s[idx_ym];
                        float sy1 = m_s[idx];
                        float sz0 = m_s[idx_zm];
                        float sz1 = m_s[idx];

                        float s = sx0 + sx1 + sy0 + sy1 + sz0 + sz1;
                        if (s == 0.0f) continue;

                        // 计算散度
                        float div = m_vel[idx_xp].x - m_vel[idx].x +
                                   m_vel[idx_yp].y - m_vel[idx].y +
                                   m_vel[idx_zp].z - m_vel[idx].z;

                        // 漂移补偿
                        if (compensateDrift && m_particleRestDensity > 0.0f) {
                            float compression = m_particleDensity[idx] - m_particleRestDensity;
                            if (compression > 0.0f) {
                                div -= 1.0f * compression;
                            }
                        }

                        // 压力修正
                        float p = -div / s * overRelaxation;
                        m_p[idx] += p;

                        // 更新速度
                        m_vel[idx].x -= sx0 * p;
                        m_vel[idx_xp].x += sx1 * p;
                        m_vel[idx].y -= sy0 * p;
                        m_vel[idx_yp].y += sy1 * p;
                        m_vel[idx].z -= sz0 * p;
                        m_vel[idx_zp].z += sz1 * p;
                    }
                }
            }
        }
    }

    // 更新粒子颜色
    void Simulator::updateParticleColors() {
        for (int i = 0; i < m_iNumSpheres; i++) {
            m_particleColor[i] = glm::vec3(0.0f, 0.5f, 1.0f); // 蓝色
        }
    }

    // 检查速度是否有效
    bool Simulator::isValidVelocity(int i, int j, int k, int dir) {
        if (i < 0 || i >= m_iCellX || j < 0 || j >= m_iCellY || k < 0 || k >= m_iCellZ)
            return false;
        return true;
    }

    // 将网格索引转换为偏移量
    int Simulator::index2GridOffset(glm::ivec3 index) {
        return index.x * m_iCellY * m_iCellZ + index.y * m_iCellZ + index.z;
    }

} // namespace VCX::Labs::Fluid
