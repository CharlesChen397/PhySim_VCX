# Lab 2 FLIP流体仿真实现说明

## 已实现的核心功能

### 1. 粒子积分 (integrateParticles)
- 更新粒子速度：`v += dt * g`
- 更新粒子位置：`x += dt * v`

### 2. 边界碰撞处理 (handleParticleCollisions)
- 处理6个边界面的碰撞
- 限制粒子位置在边界内
- 重置法向速度为0
- 支持球形障碍物碰撞（可扩展）

### 3. 速度传递 (transferVelocities)
#### Particle to Grid (P2G)
- 使用交错网格（MAC Grid）
- u速度在x面，v速度在y面，w速度在z面
- 线性插值权重计算
- 自动标记流体/空/固体cell

#### Grid to Particle (G2P)
- 支持PIC和FLIP混合
- FLIP95: `v_new = 0.05 * PIC + 0.95 * FLIP`
- PIC: 直接插值网格速度
- FLIP: 只传递速度变化量

### 4. 不可压缩求解 (solveIncompressibility)
- Gauss-Seidel迭代法
- 支持Over-Relaxation加速收敛
- 计算速度散度并消除
- 支持漂移补偿（Drift Compensation）
- 只处理流体cell，跳过固体和空cell

### 5. 粒子密度更新 (updateParticleDensity)
- 在cell中心计算粒子密度
- 使用线性插值
- 自动计算静止密度（首次调用时）
- 用于漂移补偿

### 6. 粒子分离 (pushParticlesApart)
- 使用空间哈希表加速
- Gauss-Seidel迭代分离重叠粒子
- 哈希表cell大小：2.2 * particleRadius
- 检查27个邻居cell

### 7. 可视化和交互
- 实时渲染粒子
- 可调节时间步长
- 可调节FLIP比例（0.0=PIC, 1.0=FLIP, 0.95=FLIP95）
- 显示粒子数量和网格分辨率
- 支持暂停/继续/重置

## 实现细节

### 交错网格（MAC Grid）
```
u速度: offset = (0, h/2, h/2)
v速度: offset = (h/2, 0, h/2)
w速度: offset = (h/2, h/2, 0)
```

### 线性插值
对周围8个网格点进行三线性插值：
```
weight = wx * wy * wz
wx = (1-tx) or tx
wy = (1-ty) or ty
wz = (1-tz) or tz
```

### 散度计算
```
div = (u[i+1,j,k] - u[i,j,k]) + 
      (v[i,j+1,k] - v[i,j,k]) + 
      (w[i,j,k+1] - w[i,j,k])
```

### 压力投影
```
p = -div / s * omega
更新速度：u -= s * p
```

## 参数设置

- 网格分辨率：24x24x24（推荐）
- 时间步长：0.016s（默认）
- FLIP比例：0.95（FLIP95）
- 粒子分离迭代：5次
- 压力求解迭代：30次
- Over-Relaxation：0.5

## 编译和运行

```bash
xmake build lab2
xmake run lab2
```

## 参考资料

- PPT: src/VCX/Labs/2-FluidSimulation/ppt.md
- 作业要求: src/VCX/Labs/2-FluidSimulation/lab2-Flip.md
