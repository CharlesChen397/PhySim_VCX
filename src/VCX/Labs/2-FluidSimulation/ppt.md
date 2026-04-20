# Physics Simulation for Computer Graphics
## 07 FLIP: Fluid Implicit Particle

---

## 1. Physics Model — Navier-Stokes Equations

### Momentum:
$$\rho \frac{D\mathbf{u}}{Dt} = -\nabla p + \rho \mathbf{g} + \mu \nabla^2 \mathbf{u}$$

### Mass Conservation & Incompressibility:
$$\frac{\partial \rho}{\partial t} = -\nabla \cdot \rho \mathbf{u}$$

当流体不可压缩时 $$\frac{D\rho}{Dt} = 0$$，则：

$$\nabla \cdot \mathbf{u} = 0$$

> **Note:** 左侧为 Lagrangian 视角，右侧为 Eulerian 视角。

---

## 2. PIC Method（Particle in Cell）

> **核心思想：** 粒子携带速度 → 可以跳过网格平流步骤！

### 模拟流程图

```
Simulate Particles          Particle Domain
(simply move with velocity)
        ↓
Velocity Transfer (Particle2Grid)
        ↓
Make the grid velocities incompressible (Pressure Solve)
        ↓
Velocity Transfer (Grid2Particle)
                            Grid Domain
```

### 分步方程

| 步骤 | 方程 |
|------|------|
| ① 平流 | $$\frac{\partial \mathbf{u}}{\partial t} = -(\mathbf{u} \cdot \nabla)\mathbf{u}$$ |
| ② 外力 | $$\frac{\partial \mathbf{u}}{\partial t} = \mathbf{g}$$ |
| ③ 粘性 | $$\frac{\partial \mathbf{u}}{\partial t} = \frac{\mu}{\rho} \Delta \mathbf{u}$$ |
| ④ 压力 | $$\frac{\partial \mathbf{u}}{\partial t} = -\frac{1}{\rho} \nabla p$$ |

### 伪代码

```cpp
for (int step = 0; step < numSubSteps; step++) {
    integrateParticles(sdt);
    handleParticleCollisions(/* Boundary Conditions */);
    transferVelocities(toGrid = true);
    solveIncompressibility();
    transferVelocities(toGrid = false);
}
```

### PIC 的问题：数值耗散

- **问题：** 粒子与网格之间插值导致耗散（dissipation）。
- 大量单个粒子的运动信息在 Grid → Particle 过程中丢失。

#### B-Spline 核函数 $$N(x)$$（插值权重）

| 类型 | 精度 |
|------|------|
| Linear（线性） | 低 |
| Quadratic（二次） | 中 |
| Cubic（三次） | 高 |

> 距离越近，权重越大。

---

## 3. FLIP Method（Fluid Implicit Particles）

> **核心思想：** 粒子保留自身速度 → 只更新速度的**变化量**（如 $$-\frac{\Delta t}{\rho} \nabla p$$）！

### 流程图（与 PIC 对比）

```
Simulate Particles
        ↓
Velocity Transfer (Particle2Grid)  →  Make a Copy (u*)
        ↓
Make the grid velocities incompressible (Pressure Solve)
        ↓
Velocity Transfer (Grid2Particle)  ←  Add Changes (∂u/∂t)
```

---

## 4. Mix PIC and FLIP — FLIP95

$$\mathbf{u}_{particle} = 0.05 \times \text{PIC} + 0.95 \times \text{FLIP}$$

| 方法 | 稳定性 | 无耗散 |
|------|--------|--------|
| PIC（网格插值） | ✅ 稳定 | ❌ 有耗散 |
| FLIP（粒子+网格修正） | ❌ 不稳定 | ✅ 无耗散 |
| **FLIP95（混合）** | ✅ | ✅ |

### FLIP95 伪代码

```cpp
for (int step = 0; step < numSubSteps; step++) {
    integrateParticles(sdt);
    handleParticleCollisions(/* Boundary Conditions */);
    transferVelocities(toGrid = true);
    solveIncompressibility();
    transferVelocities(toGrid = false, FlipRatio = 0.95);
}
```

---

## 5. FLIP95 实现细节

### 5.1 粒子积分 — `integrateParticles(sdt)`

粒子存储位置 $$\mathbf{X}$$ 和速度 $$\mathbf{v}$$：

```
for all particles p:
    v_p ← v_p + Δt · g
    X_p ← X_p + Δt · v_p
```

### 5.2 碰撞处理 — `handleParticleCollisions(...)`

边界参数：`Vec3 obstaclePos, float obstacleRadius, Vec3 obstacleVel`

```
// 左边界（x_min）
if X_p.x < x_min + h + r:
    X_p.x = h + r
    v_p.x = 0

// 右边界（x_min + res·h）
if X_p.x > x_min + (res - 1)h - r:
    X_p.x = (res - 1)h - r
    v_p.x = 0
```

> **"No-stick" 边界：** 仅替换法线方向的 $$u_{i,j}$$ 为 $$u_{solid}$$

---

### 5.3 速度传递 — `transferVelocities(toGrid=true/false)`

首先将所有 cell 标记为 **solid / empty / fluid**。

#### Particle → Grid（P2G）

坐标偏移计算：

$$x_{cell} = \left\lfloor \frac{x_{p0}}{h} \right\rfloor, \quad \Delta x = x_{p0} - x_{cell} \cdot h$$

$$y_{cell} = \left\lfloor \frac{y_{p0.5}}{h} \right\rfloor, \quad \Delta y = y_{p0.5} - y_{cell} \cdot h$$

其中：

$$x_{p0} = x_p - x_{min}, \quad y_{p0.5} = y_p - y_{min} - \frac{h}{2}, \quad z_{p0.5} = z_p - z_{min} - \frac{h}{2}$$

网格速度更新：

$$u_i = \frac{\sum w_p \, u_p}{\sum w_p}$$

#### Grid → Particle（G2P）

- **PIC：**

$$u_p = \frac{\sum w_i \, u_i}{\sum w_i}$$

- **FLIP：**

$$u_p \mathrel{+}= \frac{\sum w_i \, \Delta u_i}{\sum w_i}$$

- **FLIP95 混合：**

$$u_p = 0.05 \cdot \frac{\sum w_i \, u_i}{\sum w_i} + 0.95 \cdot \left(u_{p,old} + \Delta u_p\right)$$

其中：

$$\Delta u_p = \frac{\sum w_i (u_i - u_{i,old})}{\sum w_i}$$

---

### 5.4 不可压缩求解 — `solveIncompressibility(...)`

#### 散度计算

$$d = \omega \cdot (u_{i+1,j} + v_{i,j+1} - u_{i,j} - v_{i,j})$$

（solid cell: $$s=0$$；其他 cell: $$s=1$$ ）

#### 压力泊松方程

$$\nabla \cdot \nabla p = \frac{\rho}{\Delta t} \nabla \cdot \mathbf{u}^*$$

离散形式：

$$\frac{1}{h^2}\left(4p_{i,j} - p_{i-1,j} - p_{i+1,j} - p_{i,j-1} - p_{i,j+1}\right) = \frac{\rho}{\Delta t} \cdot \frac{1}{h}\left(-u_{i+1,j} - v_{i,j+1} + u_{i,j} + v_{i,j}\right)$$

#### Gauss-Seidel 迭代求解（Over-Relaxation）

超松弛因子 $$1 < \omega < 2$$，例如 $$\omega = 1.9$$：

$$d = \omega \cdot (u_{i+1,j} + v_{i,j+1} - u_{i,j} - v_{i,j})$$

---

## 6. FLIP95 + Over Relaxation + Drift Compensation

### 完整伪代码

```cpp
for (int step = 0; step < numSubSteps; step++) {
    integrateParticles(sdt);

    if (separateParticles)
        pushParticlesApart(numParticleIters);

    handleParticleCollisions(/* Boundary Conditions */);
    transferVelocities(toGrid = true);

    updateParticleDensity();
    solveIncompressibility(overRelaxation, compensateDrift);

    transferVelocities(toGrid = false);
}
```

---

### 6.1 粒子分离 — `pushParticlesApart(numParticleIters)`

#### Gauss-Seidel 方法

```
for n iterations:
    for all particle pi:
        for all particle pj:
            d = |X_pi - X_pj|
            if d < 2r:
                g = 0.5 * (2r - d) * (X_pi - X_pj) / d
                X_pi += g
                X_pj -= g
```

#### Hash Table 加速

- 哈希格子大小：$$h_{cell} = 2.2 \cdot r_{radius}$$（略大于 $$2r$$）
- 对粒子 $$p_i$$ 所在格子 $$c_{ij}$$，检查周围 9 个邻居格子：

$$\{c_{i-1,j-1},\ c_{i-1,j},\ c_{i-1,j+1},\ c_{i,j-1},\ c_{i,j},\ c_{i,j+1},\ c_{i+1,j-1},\ c_{i+1,j},\ c_{i+1,j+1}\}$$

---

### 6.2 粒子密度更新 — `updateParticleDensity()`

- 在每个 cell 中心计算粒子密度 $$\rho$$
- 网格偏移 $$\frac{h}{2}$$（两个方向均偏移）

---

### 6.3 漂移补偿 — Drift Compensation

$$\rho_0$$：场景初始化后所有流体 cell 的平均粒子密度。

当 $$\rho > \rho_0$$ 时，在散度中加入补偿项（$$k=1$$ 为例）：

$$d = \omega \cdot (u_{i+1,j} + v_{i,j+1} - u_{i,j} - v_{i,j}) - k(\rho - \rho_0)$$

调用方式：

```cpp
solveIncompressibility(overRelaxation, compensateDrift, ρ - ρ₀);
```

---

## 7. 减少数值扩散（Reduce Numerical Diffusion）

### 7.1 方法概览

| 方法 | 特点 |
|------|------|
| MacCormack Advection | 二阶精度，前向+后向误差修正 |
| Flow Map | 减少插值次数 |
| Flow Map + Covector Advection | Impulse Fluid |
| Advection-Reflection | 修正 splitting 误差，保持动能 |
| Vorticity Confinement | 增强涡量细节 |
| Vortex Method | 基于涡量方程求解 |

---

### 7.2 Advection-Reflection Method

- **左图：** 投影到无散度子空间导致动能损失（红色）
- **中图：** Reflection solver 使用能量守恒反射（黄色）
- **右图：** 不使用反射，半步长投影也会损失更多能量

$$\mathbf{X}^* = \mathbf{X}_0 + \mathbf{v}_0 \Delta t, \quad \Delta \mathbf{X} = \mathbf{X}_1 - \mathbf{X}^*$$

---

### 7.3 Vorticity Confinement

步骤：
1. 从速度场出发
2. 计算涡量（vorticity）
3. 计算指向涡量局部极大值的向量
4. 计算并施加力

> 参考：[Steinhoff & Underhill '94; Fedkiw et al. '01]

---

### 7.4 MacCormack Advection

- **前向+后向误差修正**（back-and-forth error correction）
- **二阶精度**
- 保留更精细的流动细节

> 代表工作：**BiMocq2** [Qu et al. SIGGRAPH 2019]
> 通过多级映射的前向/后向平流来补偿误差，保持长时间平流精度。

---

### 7.5 Flow Map + Covector Advection

> 代表工作：**Neural Flow Maps**
> *FLUID SIMULATION ON NEURAL FLOW MAPS* — **SIGGRAPH 2023 Best Paper Award**
> 🔗 https://yitongdeng-projects.github.io/neural_flow_maps_webpage/

---

## 8. 参考文献

- Jiang et al. *The Affine Particle-in-Cell Method*, SIGGRAPH 2015
- Kim et al., BFECC, 2005
- Selle et al., MacCormack, 2008
- Qu et al., BiMocq2, SIGGRAPH 2019
- Deng et al., Neural Flow Maps, SIGGRAPH 2023 (Best Paper)
- Steinhoff & Underhill, Vorticity Confinement, 1994
- Fedkiw et al., Vorticity Confinement, 2001
