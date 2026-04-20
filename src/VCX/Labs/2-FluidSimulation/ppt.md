# Physics Simulation for Computer Graphics
## 完整讲义合集：流体仿真（Lecture 06 + 07 + 08）

---

# Lecture 06 — Fluid Simulation（流体仿真基础）

---

## 1. 数学基础

### 梯度（Gradient）

$$\nabla f(\mathbf{X}) = \frac{\partial f}{\partial \mathbf{X}} = \left(\frac{\partial f}{\partial x},\ \frac{\partial f}{\partial y},\ \frac{\partial f}{\partial z}\right)$$

### 散度（Divergence）

$$\nabla \cdot \mathbf{F}(\mathbf{X}) = \frac{d}{d\mathbf{X}} \cdot \mathbf{F} = \frac{\partial f}{\partial x} + \frac{\partial g}{\partial y} + \frac{\partial h}{\partial z}, \quad \mathbf{F}(\mathbf{X}) = (f, g, h) \in \mathbb{R}^3$$

### 旋度（Curl）

$$\nabla \times \mathbf{F}(\mathbf{X}) = \begin{pmatrix} \frac{\partial h}{\partial y} - \frac{\partial g}{\partial z} \\ \frac{\partial f}{\partial z} - \frac{\partial h}{\partial x} \\ \frac{\partial g}{\partial x} - \frac{\partial f}{\partial y} \end{pmatrix}$$

### 拉普拉斯算子（Laplacian）

$$\Delta f = \nabla^2 f = \nabla \cdot \nabla f = \frac{\partial^2 f}{\partial x^2} + \frac{\partial^2 f}{\partial y^2} + \frac{\partial^2 f}{\partial z^2} = \text{trace}(H)$$

其中 Jacobian 和 Hessian 定义为：

$$J(f) = \nabla^T f(\mathbf{X}), \quad H(f) = J(\nabla f) = \nabla^T \nabla f = \nabla \nabla^T f = \nabla \otimes \nabla f$$

### 离散形式（3D 网格）

$$\nabla s \approx \frac{1}{h} \begin{pmatrix} s_{i+1,j,k} - s_{i,j,k} \\ s_{i,j+1,k} - s_{i,j,k} \\ s_{i,j,k+1} - s_{i,j,k} \end{pmatrix}$$

$$\nabla \cdot \mathbf{v} \approx \frac{1}{h}\left[(v_x)_{i+1,j,k} - (v_x)_{i,j,k} + (v_y)_{i,j+1,k} - (v_y)_{i,j,k} + (v_z)_{i,j,k+1} - (v_z)_{i,j,k}\right]$$

$$\nabla^2 s \approx \frac{1}{h^2}\left[s_{i+1,j,k} + s_{i-1,j,k} + s_{i,j+1,k} + s_{i,j-1,k} + s_{i,j,k+1} + s_{i,j,k-1} - 6s_{i,j,k}\right]$$

---

## 2. 物理模型

### Navier-Stokes 动量方程

$$\rho \frac{D\mathbf{u}}{Dt} = \underbrace{-\nabla p}_{\text{Pressure}} + \underbrace{\rho \mathbf{g}}_{\text{Extra Force}} + \underbrace{\mu \nabla^2 \mathbf{u}}_{\text{Viscosity}}$$

展开物质导数：

$$\rho \left(\frac{\partial \mathbf{u}}{\partial t} + \mathbf{u} \cdot \nabla \mathbf{u}\right) = -\nabla p + \rho \mathbf{g} + \mu \nabla^2 \mathbf{u}$$

### Cauchy 动量方程（任意连续介质）

$$\rho \frac{D\mathbf{u}}{Dt} = \nabla \cdot \boldsymbol{\sigma} + \mathbf{F}_{body}$$

---

## 3. Lagrangian 与 Eulerian 视角

### 物质导数（Material Derivative）

$$\frac{D\mathbf{u}}{Dt} = \frac{\partial \mathbf{u}}{\partial \mathbf{X}} \frac{\partial \mathbf{X}}{\partial t} + \frac{\partial \mathbf{u}}{\partial t} = \frac{\partial \mathbf{u}}{\partial t} + \mathbf{u} \cdot \nabla \mathbf{u}$$

对任意物理量 $$Q$$：

$$\frac{DQ}{Dt} = \frac{\partial Q}{\partial t} + \mathbf{u} \cdot \nabla Q$$

| 术语 | 含义 |
|------|------|
| $$\frac{D}{Dt}$$ | 全导数 / 物质导数（Lagrangian） |
| $$\frac{\partial}{\partial t}$$ | 偏导数 / 局部导数（Eulerian） |
| $$\mathbf{u} \cdot \nabla Q$$ | 对流项（Advective term） |

---

## 4. 质量守恒与不可压缩性

### 连续性方程

$$\frac{d}{dt}\int_\Omega \rho \, dV = -\oint_{\partial\Omega} \rho \mathbf{u} \cdot \hat{n} \, dS = -\int_\Omega \nabla \cdot (\rho \mathbf{u}) \, dV$$

$$\Rightarrow \quad \frac{\partial \rho}{\partial t} = -\nabla \cdot (\rho \mathbf{u})$$

### 不可压缩条件

$$\frac{D\rho}{Dt} = 0 \Rightarrow \frac{\partial \rho}{\partial t} + \mathbf{u} \cdot \nabla \rho = 0 \Rightarrow \rho \nabla \cdot \mathbf{u} = 0$$

$$\boxed{\nabla \cdot \mathbf{u} = 0}$$

---

## 5. 边界条件

| 类型 | 定义 |
|------|------|
| Dirichlet | $$f\big\|_{\partial\Omega}(\mathbf{X}, t) = C_{Dirichlet}(\mathbf{X}, t)$$ |
| Neumann | $$\frac{\partial f}{\partial \hat{n}}\big\|_{\partial\Omega} = \hat{n} \cdot \nabla f\big\|_{\partial\Omega} = C_{Neumann}(\mathbf{X}, t)$$ |
| Robin | $$a \cdot f\big\|_{\partial\Omega} + b \cdot \frac{\partial f}{\partial \hat{n}}\big\|_{\partial\Omega} = C_{Robin}(\mathbf{X}, t)$$ |

---

## 6. 网格表示与有限差分

### 中心差分（Central Differencing）

$$\frac{f(t_0 + \Delta t) - f(t_0 - \Delta t)}{2\Delta t} \approx \frac{df(t_0)}{dt} + O(\Delta t^2)$$

### 网格上的有限差分

$$\frac{\partial f_{i+0.5,j}}{\partial x} \approx \frac{f_{i+1,j} - f_{i,j}}{h}$$

$$\frac{\partial f_{i-0.5,j}}{\partial x} \approx \frac{f_{i,j} - f_{i-1,j}}{h}$$

$$\frac{\partial^2 f_{i,j}}{\partial x^2} \approx \frac{f_{i-1,j} + f_{i+1,j} - 2f_{i,j}}{h^2}$$

$$\frac{\partial^2 f_{i,j}}{\partial y^2} \approx \frac{f_{i,j-1} + f_{i,j+1} - 2f_{i,j}}{h^2}$$

### 离散 Laplacian

$$\Delta f_{i,j} = \frac{\partial^2 f_{i,j}}{\partial x^2} + \frac{\partial^2 f_{i,j}}{\partial y^2} \approx \frac{f_{i-1,j} + f_{i+1,j} + f_{i,j-1} + f_{i,j+1} - 4f_{i,j}}{h^2}$$

### 边界条件对 Laplacian 的影响

- **Dirichlet 边界**（$$f_{i-1,j} = C$$）：

$$\Delta f_{i,j} \approx \frac{C + f_{i+1,j} + f_{i,j-1} + f_{i,j+1} - 4f_{i,j}}{h^2}$$

- **Neumann 边界**（$$f_{i-1,j} = f_{i,j}$$）：

$$\Delta f_{i,j} \approx \frac{f_{i+1,j} + f_{i,j-1} + f_{i,j+1} - 3f_{i,j}}{h^2}$$

---

## 7. Laplace 方程求解

$$\Delta f = 0 \Rightarrow f_{i-1,j} + f_{i+1,j} + f_{i,j-1} + f_{i,j+1} - 4f_{i,j} = 0$$

线性方程组形式：$$L_\Delta \mathbf{F} = \mathbf{b}$$

### Jacobi 迭代法

```
Initialize f
For k = 0 ... K:
    For i, j:
        f_ij_new ← f_ij + α * (4*f_ij - f_{i-1,j} - f_{i+1,j} - f_{i,j-1} - f_{i,j+1})
    For i, j:
        f_ij ← f_ij_new
```

---

## 8. Staggered Grid（交错网格）

速度分量定义在面上（而非格心）：

- $$u_{i,j}$$：定义在竖直面（x 方向速度）
- $$v_{i,j}$$：定义在水平面（y 方向速度）

散度离散：

$$\nabla \cdot \mathbf{u}_{i,j} = \frac{u_{i+1,j} - u_{i,j}}{h} + \frac{v_{i,j+1} - v_{i,j}}{h} = 0$$

---

## 9. 双线性插值（Bilinear Interpolation）

$$i \leftarrow \lfloor x \rfloor, \quad j \leftarrow \lfloor y \rfloor$$

$$f(\mathbf{X}) = (i+1-x)(j+1-y) f_{i,j} + (x-i)(j+1-y) f_{i+1,j} + (i+1-x)(y-j) f_{i,j+1} + (x-i)(y-j) f_{i+1,j+1}$$

对交错速度 $$u$$，插值前需偏移：$$x \leftarrow x - 0.5$$

---

## 10. 不可压缩粘性流体 Eulerian 方法

### 方程分裂（Operator Splitting）

$$\nabla \cdot \mathbf{u} = 0, \quad \frac{\partial \mathbf{u}}{\partial t} = \underbrace{-\mathbf{u} \cdot \nabla \mathbf{u}}_{\text{①平流}} + \underbrace{\mathbf{g}}_{\text{②外力}} + \underbrace{\frac{\mu}{\rho} \Delta \mathbf{u}}_{\text{③粘性}} \underbrace{- \frac{1}{\rho} \nabla p}_{\text{④压力}}$$

---

### Step 1：平流（Advection）—— Semi-Lagrangian 方法

求解 $$\frac{\partial \mathbf{u}}{\partial t} = -\mathbf{u} \cdot \nabla \mathbf{u}$$

**算法（以 $$u$$ 分量为例）：**

```
1. 定义 X0 ← (i - 0.5, j)
2. 计算 u(X0)
3. X1 ← X0 - Δt · u(X0)
4. 计算 u(X1)
5. u_ij_new ← u(X1)
```

**子步细化（提高精度）：**

```
1. 定义 X0 ← (i - 0.5, j)
2. X1 ← X0 - (1/3)Δt · u(X0)
3. X2 ← X1 - (1/3)Δt · u(X1)
4. X3 ← X2 - (1/3)Δt · u(X2)
5. u_ij_new ← u(X3)
```

> Semi-Lagrangian 方法是**无条件稳定**的。

**数值粘性分析（1D）：**

$$q_i^{n+1} = \frac{\Delta t \, u}{\Delta x} q_{i-1}^n + \left(1 - \frac{\Delta t \, u}{\Delta x}\right) q_i^n$$

展开后得到：

$$\frac{\partial q}{\partial t} + u \frac{\partial q}{\partial x} = u \frac{\Delta x}{2} \frac{\partial^2 q}{\partial x^2}$$

> 右侧即为**数值扩散（Numerical Diffusion）**，CFL 条件：$$\Delta t \leq C \Delta x / |u_{max}|$$

---

### Step 2：外力（External Acceleration）

求解 $$\frac{\partial \mathbf{u}}{\partial t} = \mathbf{g}$$，以重力为例：

$$v_{i,j}^{new} \leftarrow v_{i,j} + \Delta t \cdot g$$

---

### Step 3：粘性（Viscosity / Diffusion）

求解 $$\frac{\partial \mathbf{u}}{\partial t} = \frac{\mu}{\rho} \Delta \mathbf{u}$$

**显式（可能不稳定）：**

$$u_{i,j}^{new} \leftarrow u_{i,j} + \frac{\mu}{\rho} \Delta t \cdot \frac{u_{i-1,j} + u_{i+1,j} + u_{i,j-1} + u_{i,j+1} - 4u_{i,j}}{h^2}$$

$$v_{i,j}^{new} \leftarrow v_{i,j} + \frac{\mu}{\rho} \Delta t \cdot \frac{v_{i-1,j} + v_{i+1,j} + v_{i,j-1} + v_{i,j+1} - 4v_{i,j}}{h^2}$$

**半隐式（两步）：**

$$u_{i,j}^{temp} \leftarrow u_{i,j} + \frac{\mu}{\rho} \frac{\Delta t}{2} \cdot \frac{u_{i-1,j} + u_{i+1,j} + u_{i,j-1} + u_{i,j+1} - 4u_{i,j}}{h^2}$$

$$u_{i,j}^{new} \leftarrow u_{i,j}^{temp} + \frac{\mu}{\rho} \frac{\Delta t}{2} \cdot \frac{u_{i-1,j}^{temp} + u_{i+1,j}^{temp} + u_{i,j-1}^{temp} + u_{i,j+1}^{temp} - 4u_{i,j}^{temp}}{h^2}$$

---

### Step 4：压力投影（Pressure Projection）

求解 $$\frac{\partial \mathbf{u}}{\partial t} = -\frac{1}{\rho} \nabla p$$，使 $$\nabla \cdot \mathbf{u}^{new} = 0$$

由 $$\mathbf{u}^{new} = \mathbf{u}^* - \frac{\Delta t}{\rho} \nabla p$$，得泊松方程：

$$\nabla \cdot \nabla p = \frac{\rho}{\Delta t} \nabla \cdot \mathbf{u}^*$$

离散形式：

$$4p_{i,j} - p_{i-1,j} - p_{i+1,j} - p_{i,j-1} - p_{i,j+1} = \frac{\rho h}{\Delta t}\left(-u_{i+1,j} - v_{i,j+1} + u_{i,j} + v_{i,j}\right)$$

线性系统：$$A\mathbf{p} = \mathbf{b}$$

速度更新：

$$u_{i,j}^{new} \leftarrow u_{i,j} - \frac{\Delta t}{\rho h}(p_{i,j} - p_{i-1,j})$$

$$v_{i,j}^{new} \leftarrow v_{i,j} - \frac{\Delta t}{\rho h}(p_{i,j} - p_{i,j-1})$$

**边界条件处理：**

- **Dirichlet（自由表面）**：$$p_{i-1,j} = 0$$，去掉该项
- **Neumann（固体壁面）**：
  - "No-stick"：仅替换法线方向速度 $$u_{i,j}$$ 为 $$u_{solid}$$
  - "No-slip"：同时替换切线方向速度 $$v_{i,j}$$ 为 $$v_{solid}$$
  - 去掉 $$p_{i-1,j}$$，将 $$p_{i,j}$$ 系数从 4 减为 3

> 矩阵 $$A$$ 是**对称正（半）定矩阵**，使用**预条件共轭梯度法（PCG）** + 不完全 Cholesky 预条件求解。

---

## 11. 烟雾与染料（Dye and Smoke）

$$\frac{\partial \mathbf{u}}{\partial t} = -\mathbf{u} \cdot \nabla \mathbf{u} + \mathbf{g} + \mathbf{F}_{buoyancy} + \frac{\mu}{\rho} \Delta \mathbf{u} - \frac{1}{\rho} \nabla p'$$

$$\frac{\partial c}{\partial t} = -\mathbf{u} \cdot \nabla c$$

浮力模型（$$p = p' + \rho g H$$）：

$$\mathbf{F}_b = -\alpha c + \beta(T - T_{amb})$$

其中 $$H = (n - j)h$$ 为格子高度。

---

## 12. 水面追踪

### Marker Particles

$$\frac{d\mathbf{X}_p}{dt} = \mathbf{u}(\mathbf{X}_p)$$（可用 Runge-Kutta 2 更新）

### Level Set 方法

1. **初始化**：$$\phi(\mathbf{X})$$ 为有符号距离函数（Fast Sweeping Method）
2. **平流**：$$\frac{\partial \phi}{\partial t} = -\mathbf{u} \cdot \nabla c$$
3. **重初始化**：$$|\nabla \phi(\mathbf{X})| = 1$$

$$\phi(\mathbf{X}) \begin{cases} > 0 & \text{外部} \\ = 0 & \text{表面} \\ < 0 & \text{内部} \end{cases}$$

> Level Set 对数值扩散非常敏感，细节特征会随时间平滑消失。

---

## 13. 减少数值扩散（Reduce Numerical Diffusion）

| 方法 | 特点 |
|------|------|
| MacCormack Advection | 二阶精度，前向+后向误差修正 |
| Flow Map | 减少插值次数 |
| Flow Map + Covector Advection | Impulse Fluid |
| Advection-Reflection | 修正 splitting 误差，保持动能 |
| Vorticity Confinement | 增强涡量细节 [Steinhoff & Underhill '94; Fedkiw et al. '01] |
| Vortex Method | 基于涡量方程求解 |

**MacCormack 方法：** 前向+后向误差修正，二阶精度。

**BiMocq2** [Qu et al. SIGGRAPH 2019]：多级映射前向/后向平流，保持长时间精度。

**Neural Flow Maps** [SIGGRAPH 2023 Best Paper]：
🔗 https://yitongdeng-projects.github.io/neural_flow_maps_webpage/

---

---

# Lecture 07 — FLIP: Fluid Implicit Particle

---

## 1. 物理模型回顾

$$\rho \frac{D\mathbf{u}}{Dt} = -\nabla p + \rho \mathbf{g} + \mu \nabla^2 \mathbf{u}$$

质量守恒：$$\frac{\partial \rho}{\partial t} = -\nabla \cdot (\rho \mathbf{u})$$

不可压缩：$$\nabla \cdot \mathbf{u} = 0$$

---

## 2. PIC 方法（Particle in Cell）

> **核心思想：** 粒子携带速度 → 可跳过网格平流！

### 分步方程

| 步骤 | 方程 |
|------|------|
| ① 平流 | $$\frac{\partial \mathbf{u}}{\partial t} = -(\mathbf{u} \cdot \nabla)\mathbf{u}$$ |
| ② 外力 | $$\frac{\partial \mathbf{u}}{\partial t} = \mathbf{g}$$ |
| ③ 粘性 | $$\frac{\partial \mathbf{u}}{\partial t} = \frac{\mu}{\rho} \Delta \mathbf{u}$$ |
| ④ 压力 | $$\frac{\partial \mathbf{u}}{\partial t} = -\frac{1}{\rho} \nabla p$$ |

### PIC 伪代码

```cpp
for (int step = 0; step < numSubSteps; step++) {
    integrateParticles(sdt);
    handleParticleCollisions(/* Boundary Conditions */);
    transferVelocities(toGrid = true);
    solveIncompressibility();
    transferVelocities(toGrid = false);
}
```

### PIC 的问题

- **数值耗散**：粒子与网格插值导致大量单粒子运动信息丢失。

#### B-Spline 核函数 $$N(x)$$

| 类型 | 精度 |
|------|------|
| Linear | 低 |
| Quadratic | 中 |
| Cubic | 高 |

---

## 3. FLIP 方法（Fluid Implicit Particles）

> **核心思想：** 粒子保留自身速度 → 只更新速度**变化量** $$-\frac{\Delta t}{\rho}\nabla p$$

### 流程

```
Simulate Particles
        ↓
P2G：Velocity Transfer (Particle2Grid)  →  Make a Copy (u*)
        ↓
Pressure Solve（Make grid velocities incompressible）
        ↓
G2P：Velocity Transfer (Grid2Particle)  ←  Add Changes (∂u/∂t)
```

---

## 4. FLIP95 混合方法

$$\mathbf{u}_{particle} = 0.05 \times \text{PIC} + 0.95 \times \text{FLIP}$$

| 方法 | 稳定性 | 无耗散 |
|------|--------|--------|
| PIC（网格插值） | ✅ | ❌ |
| FLIP（粒子+网格修正） | ❌ | ✅ |
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

```
for all particles p:
    v_p ← v_p + Δt · g
    X_p ← X_p + Δt · v_p
```

### 5.2 碰撞处理 — `handleParticleCollisions(...)`

参数：`Vec3 obstaclePos, float obstacleRadius, Vec3 obstacleVel`

```
// 左边界
if X_p.x < x_min + h + r:
    X_p.x = h + r
    v_p.x = 0

// 右边界
if X_p.x > x_min + (res - 1)*h - r:
    X_p.x = (res - 1)*h - r
    v_p.x = 0
```

> **"No-stick" 边界：** 仅替换法线方向的 $$u_{i,j}$$ 为 $$u_{solid}$$

---

### 5.3 速度传递 — `transferVelocities(toGrid=true/false)`

首先将所有 cell 标记为 **solid / empty / fluid**。

**坐标偏移：**

$$x_{p0} = x_p - x_{min}, \quad y_{p0.5} = y_p - y_{min} - \frac{h}{2}, \quad z_{p0.5} = z_p - z_{min} - \frac{h}{2}$$

$$x_{cell} = \left\lfloor \frac{x_{p0}}{h} \right\rfloor, \quad \Delta x = x_{p0} - x_{cell} \cdot h$$

$$y_{cell} = \left\lfloor \frac{y_{p0.5}}{h} \right\rfloor, \quad \Delta y = y_{p0.5} - y_{cell} \cdot h$$

**P2G（粒子 → 网格）：**

$$u_i = \frac{\sum w_p \, u_p}{\sum w_p}$$

**G2P（网格 → 粒子）：**

- PIC：$$u_p = \dfrac{\sum w_i \, u_i}{\sum w_i}$$

- FLIP：$$u_p \mathrel{+}= \dfrac{\sum w_i \, \Delta u_i}{\sum w_i}$$

- FLIP95 混合：

$$u_p = 0.05 \cdot \frac{\sum w_i \, u_i}{\sum w_i} + 0.95 \cdot \left(u_{p,old} + \Delta u_p\right)$$

其中 $$\Delta u_p = \dfrac{\sum w_i (u_i - u_{i,old})}{\sum w_i}$$

---

### 5.4 不可压缩求解 — `solveIncompressibility(...)`

**散度（含 Over-Relaxation）：**

$$d = \omega \cdot (u_{i+1,j} + v_{i,j+1} - u_{i,j} - v_{i,j}), \quad 1 < \omega < 2 \text{（如 } \omega = 1.9\text{）}$$

**压力泊松方程：**

$$\nabla \cdot \nabla p = \frac{\rho}{\Delta t} \nabla \cdot \mathbf{u}^*$$

离散形式：

$$\frac{1}{h^2}\left(4p_{i,j} - p_{i-1,j} - p_{i+1,j} - p_{i,j-1} - p_{i,j+1}\right) = \frac{\rho}{\Delta t} \cdot \frac{1}{h}\left(-u_{i+1,j} - v_{i,j+1} + u_{i,j} + v_{i,j}\right)$$

---

## 6. FLIP95 + Over-Relaxation + Drift Compensation

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

**Gauss-Seidel 方法：**

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

**Hash Table 加速：**

- 哈希格子大小：$$h_{cell} = 2.2 \cdot r_{radius}$$
- 对粒子 $$p_i$$ 所在格子 $$c_{ij}$$，检查周围 9 个邻居格子：

$$\{c_{i-1,j-1},\ c_{i-1,j},\ c_{i-1,j+1},\ c_{i,j-1},\ c_{i,j},\ c_{i,j+1},\ c_{i+1,j-1},\ c_{i+1,j},\ c_{i+1,j+1}\}$$

---

### 6.2 漂移补偿 — Drift Compensation

$$\rho_0$$：场景初始化后所有流体 cell 的平均粒子密度。

当 $$\rho > \rho_0$$ 时（$$k = 1$$ 为例）：

$$d = \omega \cdot (u_{i+1,j} + v_{i,j+1} - u_{i,j} - v_{i,j}) - k(\rho - \rho_0)$$

调用：

```cpp
solveIncompressibility(overRelaxation, compensateDrift, ρ - ρ₀);
```

---

---

# Lecture 08 — SPH and APIC

---

## 1. 粒子表示与核函数

### 1.1 平滑插值（Smoothed Interpolation）

**简单模型（等权重）：**

$$A_i^{SMOOTH} = \frac{1}{n} \sum_j A_j \quad \text{（当 } |\mathbf{X}_i - \mathbf{X}_j| < R\text{）}$$

**考虑体积：**

$$A_i^{SMOOTH} = \sum_j V_j A_j W_{ij}$$

**最终方案（含密度归一化）：**

$$A_i^{SMOOTH} = \sum_j \frac{m_j}{\sum_k m_k W_{jk}} A_j W_{ij}$$

粒子体积估计：

$$V_i = \frac{m_i}{\rho_i}, \quad \rho_i^{SMOOTH} = \sum_j m_j W_{ij}$$

$$V_i = \frac{m_i}{\sum_j m_j W_{ij}}$$

---

### 1.2 核函数 $$W_{ij}$$

**B-Spline 核函数（$$q = |\mathbf{X}_i - \mathbf{X}_j| / h$$）：**

$$W_{ij} = \frac{3}{2\pi h^3} \begin{cases} \frac{2}{3} - q^2 + \frac{1}{2}q^3 & 0 \leq q < 1 \\ \frac{1}{6}(2-q)^3 & 1 \leq q < 2 \\ 0 & 2 \leq q \end{cases}$$

**Poly6 核函数：**

$$W_{poly6}(q) = \frac{315}{64\pi h^9}(h^2 - q^2)^3 \quad \text{if } 0 \leq q \leq h, \text{ else } 0$$

---

### 1.3 核函数导数

**梯度（Gradient）：**

$$\nabla_i W_{ij} = \frac{\partial W_{ij}}{\partial q} \cdot \frac{\mathbf{X}_i - \mathbf{X}_j}{|\mathbf{X}_i - \mathbf{X}_j| \cdot h}$$

$$\nabla_j W_{ji} = -\nabla_i W_{ij}$$

其中：

$$\frac{\partial W_{ij}}{\partial q} = \frac{3}{2\pi h^3} \begin{cases} -2q + \frac{3}{2}q^2 & 0 \leq q < 1 \\ -\frac{1}{2}(2-q)^2 & 1 \leq q < 2 \\ 0 & 2 \leq q \end{cases}$$

**Laplacian（标量）：**

$$\nabla_i \cdot \nabla_i W_{ij} = \frac{\partial^2 W_{ij}}{\partial q^2} \cdot \frac{1}{h^2} + \frac{\partial W_{ij}}{\partial q} \cdot \frac{2}{h^2 q}$$

$$\frac{\partial^2 W_{ij}}{\partial q^2} = \frac{3}{2\pi h^3} \begin{cases} -2 + 3q & 0 \leq q < 1 \\ 2 - q & 1 \leq q < 2 \\ 0 & 2 \leq q \end{cases}$$

$$\Delta_j W_{ji} = \Delta_i W_{ij}$$

---

### 1.4 核函数汇总

$$A_i^{SMOOTH} = \sum_j \frac{m_j}{\sum_k m_k W_{jk}} A_j W_{ij}$$

$$\nabla_i W_{ij} = \frac{\partial W_{ij}}{\partial q} \cdot \frac{\mathbf{X}_i - \mathbf{X}_j}{|\mathbf{X}_i - \mathbf{X}_j| \cdot h}; \quad \nabla_j W_{ji} = -\nabla_i W_{ij}$$

$$\nabla_i \cdot \nabla_i W_{ij} = \frac{\partial^2 W_{ij}}{\partial q^2} \cdot \frac{1}{h^2} + \frac{\partial W_{ij}}{\partial q} \cdot \frac{2}{h^2 q}; \quad \Delta_j W_{ji} = \Delta_i W_{ij}$$

---

## 2. SPH 流体仿真

### 2.1 Lagrangian 方程分裂

| 步骤 | 方程 |
|------|------|
| ① 平流 | $$\frac{d\mathbf{u}}{dt} = 0,\quad \mathbf{X}^{t+1} = \mathbf{X}^t + \mathbf{u}^t \Delta t$$ |
| ② 外力 | $$\frac{d\mathbf{u}}{dt} = \mathbf{g}$$ |
| ③ 粘性 | $$\frac{d\mathbf{u}}{dt} = \frac{\mu}{\rho} \Delta \mathbf{u}$$ |
| ④ 压力 | $$\frac{d\mathbf{u}}{dt} = -\frac{1}{\rho} \nabla p$$ |

---

### 2.2 密度计算

$$\rho(\mathbf{x}_i) = \sum_j m_j W(\mathbf{X}_j - \mathbf{X}_i)$$

---

### 2.3 压力计算

$$p_i = k(\rho_i - \rho_0)^\gamma, \quad \text{（如 } \gamma = 7\text{）}$$

**压力梯度插值：**

$$p(\mathbf{x}) = \sum_i p_i \frac{m_i}{\rho_i} W(\mathbf{X} - \mathbf{X}_i)$$

$$\nabla p(\mathbf{x}) = \sum_i p_i \frac{m_i}{\rho_i} \nabla W(\mathbf{X} - \mathbf{X}_i)$$

**压力力：**

$$\mathbf{F}_i^p = -\frac{m_i}{\rho_i} \sum_j p_j \frac{m_j}{\rho_j} \nabla_i W(\mathbf{X}_j - \mathbf{X}_i)$$

**动量守恒形式：**

$$\mathbf{f}_i^p = -m_i \sum_j m_j \left(\frac{p_j}{\rho_j^2} + \frac{p_i}{\rho_i^2}\right) \nabla_i W(\mathbf{X}_j - \mathbf{X}_i)$$

---

### 2.4 粘性力计算

**基础形式：**

$$\mathbf{F}_i^v = \nu m_i \sum_j m_j \frac{\mathbf{u}_j}{\rho_j} \Delta_i W(\mathbf{X}_j - \mathbf{X}_i)$$

**动量守恒形式：**

$$\mathbf{F}_i^v = \nu m_i \sum_j m_j \left(\frac{\mathbf{u}_j}{\rho_j} - \frac{\mathbf{u}_i}{\rho_i}\right) \Delta_i W(\mathbf{X}_j - \mathbf{X}_i)$$

---

### 2.5 完整 SPH 算法

```
每个时间步：
1. 更新位置和速度（重力）
2. 计算每个粒子密度：ρ(x_i) = Σ_j m_j W(X_j - X_i)
3. 更新粘性力：F_i^v = ν m_i Σ_j m_j (u_j/ρ_j - u_i/ρ_i) Δ_i W(X_j - X_i)
4. 计算每个粒子压力：p_i = k(ρ_i - ρ_0)^γ
5. 计算压力力：f_i^p = -m_i Σ_j m_j (p_j/ρ_j² + p_i/ρ_i²) ∇_i W(X_j - X_i)
6. 用压力力更新速度
```

**动量守恒验证：**

$$\mathbf{f}_i^p + \mathbf{f}_j^p = 0 \quad (\text{由 } \nabla_j W_{ji} = -\nabla_i W_{ij})$$

$$\mathbf{F}_i^v + \mathbf{F}_j^v = 0 \quad (\text{由 } \Delta_j W_{ji} = \Delta_i W_{ij})$$

---

### 2.6 SPH 方法优缺点

| 优点 | 缺点 |
|------|------|
| 速度快，适合 GPU | 需要稠密邻域才稳定 |
| 粒子表示便于处理障碍物、软体 | 压力局部近似，不保证不可压缩 |

**改进方法：** IISPH、PCISPH、Position-Based Fluids 等

**邻域搜索加速：** Hash Table 或 Octree [SIGGRAPH Asia 2022]

---

## 3. Eulerian vs. Lagrangian 对比

| | Eulerian（静态网格） | Lagrangian（粒子） |
|---|---|---|
| 更新量 | $$\mathbf{u}^{t+1} = \mathbf{u}^t + \frac{\partial \mathbf{u}}{\partial t} \Delta t$$ | $$\mathbf{u}^{t+1} = \mathbf{u}^t + \frac{d\mathbf{u}}{dt} \Delta t$$ |
| 平流 | Semi-Lagrangian | 粒子直接移动 |
| 压力求解 | 全局泊松方程 | 局部 SPH 核近似 |
| 代表方法 | Stable Fluids | SPH |
| 混合方法 | — | FLIP, APIC |

**NS 方程统一形式：**

$$\frac{d\mathbf{u}}{dt} = \frac{\partial \mathbf{u}}{\partial t} + \mathbf{u} \cdot \nabla \mathbf{u} = -\frac{1}{\rho}\nabla p + \nu \nabla \cdot \nabla \mathbf{u} + \mathbf{g}, \quad \nabla \cdot \mathbf{u} = 0$$

---

## 4. APIC 方法（Affine Particle-In-Cell）

### 4.1 PIC 耗散问题回顾

PIC 在 G2P 后，大量单粒子运动信息丢失，只剩平移运动。

### 4.2 RPIC（旋转 PIC）

每个粒子携带平移速度 + 旋转速度。

**惯性张量：**

$$\mathbf{I} = \sum_i m_i(\mathbf{X}_i^T \mathbf{X}_i \mathbf{I}_{ID} - \mathbf{X}_i \mathbf{X}_i^T)$$

**角动量：**

$$\mathbf{L} = \sum_i \mathbf{X}_i \times m_i (\boldsymbol{\omega} \times \mathbf{X}_i) = \mathbf{I}\boldsymbol{\omega}$$

**动量矩矩阵（RPIC/APIC）：**

$$\mathbf{B}_p^n = \sum_j w_{jp}^n m_p \mathbf{r}_j^T \mathbf{r}_j^{IB} - \mathbf{r}_j^{v_j^T}, \quad \mathbf{r}_j = \mathbf{X}_j - \mathbf{X}_p^n$$

---

### 4.3 APIC（仿射 PIC）

每个粒子携带仿射运动矩阵 $$\mathbf{C}_p$$：

$$\mathbf{u}(\mathbf{X}) \approx \mathbf{u}_p + \mathbf{C}_p (\mathbf{X} - \mathbf{X}_p)$$

**B 矩阵（Momentum Moment Matrix，动量矩矩阵）：**

$$\mathbf{B}_p^n = \sum_j w_{jp}^n m_p \mathbf{r}_j^T \mathbf{r}_j^{IB} - \mathbf{r}_j^{v_j^T}$$

**D 矩阵（Affine Motion Inertia Tensor）：**

$$\mathbf{C}_p^{n+1} = \mathbf{B}_p^{n+1} (\mathbf{D}_p^n)^{-1}$$

通过保持角动量守恒推导：

$$L_{tot}^{G,n} = L_{tot}^{G,n+1} = L_{tot}^{P,n} = L_{tot}^{P,n+1}$$

---

### 4.4 Quadratic APIC on Centered Grids

对于中心网格 + 二次 B-Spline 核函数：

$$\mathbf{D}_p^n = \frac{1}{4}\Delta x^2 \mathbf{I}$$（均匀粒子分布下为常数）

对于三次 B-Spline：

$$\mathbf{D}_p^n = \frac{1}{3}\Delta x^2 \mathbf{I}$$

**Taichi 实现代码（二次 APIC，中心网格）：**

```python
def APIC_P2G():
    for p in x:
        base = (x[p] * inv_dx - 0.5).cast(int)
        fx = x[p] * inv_dx - base.cast(float)
        # Quadratic B-spline
        w = [0.5 * (1.5 - fx) ** 2,
             0.75 - (fx - 1) ** 2,
             0.5 * (fx - 0.5) ** 2]
        affine = C[p]
        for i in ti.static(range(3)):
            for j in ti.static(range(3)):
                offset = ti.Vector([i, j])
                dpos = (offset.cast(float) - fx) * dx
                weight = w[i][0] * w[j][1]
                grid_v[base + offset] += weight * (v[p] + affine @ dpos)
                grid_m[base + offset] += weight

def APIC_G2P():
    for p in x:
        base = (x[p] * inv_dx - 0.5).cast(int)
        fx = x[p] * inv_dx - base.cast(float)
        # Quadratic B-spline
        w = [0.5 * (1.5 - fx) ** 2,
             0.75 - (fx - 1) ** 2,
             0.5 * (fx - 0.5) ** 2]
        new_v = ti.Vector.zero(ti.f32, 2)
        new_C = ti.Matrix.zero(ti.f32, 2, 2)
        for i in ti.static(range(3)):
            for j in ti.static(range(3)):
                dpos = ti.Vector([i, j]).cast(float) - fx
                g_v = grid_v[base + ti.Vector([i, j])]
                weight = w[i][0] * w[j][1]
                new_v += weight * g_v
                new_C += 4 * weight * g_v.outer_product(dpos) * inv_dx
        x[p] = clamp_pos(x[p] + new_v * dt)
        v[p] = new_v
        C[p] = new_C
```

---

### 4.5 Trilinear APIC on MAC Grid

对于 MAC 网格 + 三线性 B-Spline 核函数，利用：

$$w_{ip}^n \mathbf{D}_p^{n-1} (\mathbf{X}_i - \mathbf{X}_p^n) = \nabla w_{ip}^n$$

$$\mathbf{C}_p^n = \begin{pmatrix} (\mathbf{c}_p^x)^n{}^T \\ (\mathbf{c}_p^y)^n{}^T \\ (\mathbf{c}_p^z)^n{}^T \end{pmatrix}$$

$$\nabla w_{ip}^n = \nabla(N(x)N(y)N(z)) = \begin{pmatrix} N(y)N(z)\nabla N(x) \\ N(x)N(z)\nabla N(y) \\ N(x)N(y)\nabla N(z) \end{pmatrix}$$

> 参考实现：Blender/Mantaflow
> 🔗 https://github.com/blender/blender/blob/main/extern/mantaflow/preprocessed/plugin/apic.cpp

---

### 4.6 APIC 完整流程

```
Simulate Particles
        ↓
Affine Transfer (Particle2Grid)
        ↓
Make the grid velocities incompressible (Pressure Solve)
        ↓
Affine Transfer (Grid2Particle)
```

| 特性 | PIC | FLIP | APIC |
|------|-----|------|------|
| 稳定性 | ✅ | ❌ | ✅ |
| 无耗散 | ❌ | ✅ | ✅ |
| 角动量守恒 | ❌ | ❌ | ✅（仅 P2G+G2P） |

---

### 4.7 PIC vs APIC 速度贡献对比

**PIC：**

$$\mathbf{v}_i^n = \sum_p w_{ip}^n \mathbf{v}_p^n$$

**APIC：**

$$\mathbf{v}_i^n = \sum_p w_{ip}^n \left(\mathbf{v}_p^n + \mathbf{C}_p^n (\mathbf{X}_i - \mathbf{X}_p^n)\right)$$

$$\mathbf{C}_p = \begin{pmatrix} C_{00} & C_{01} \\ C_{10} & C_{11} \end{pmatrix}$$

- $$C_{00}$$：水平速度随 $$(\mathbf{X}_i - \mathbf{X}_p)$$ 的 x 分量变化
- $$C_{01}$$：水平速度随 $$(\mathbf{X}_i - \mathbf{X}_p)$$ 的 y 分量变化
- $$C_{10}$$：竖直速度随 $$(\mathbf{X}_i - \mathbf{X}_p)$$ 的 x 分量变化
- $$C_{11}$$：竖直速度随 $$(\mathbf{X}_i - \mathbf{X}_p)$$ 的 y 分量变化

---

## 5. 连续介质守恒定律（附录）

### 5.1 质量守恒

$$\frac{\partial \rho}{\partial t} = -\nabla \cdot (\rho \mathbf{u}), \quad \nabla \cdot \mathbf{u} = 0 \text{（不可压缩）}$$

### 5.2 线动量守恒

$$\int_\Omega \rho \frac{d\mathbf{u}}{dt} dV = \oint_{\partial\Omega} \mathbf{T} \, dS + \int_\Omega \mathbf{F}_{body} \, dV$$

**牵引力（Traction）：** $$\mathbf{T} = \boldsymbol{\sigma} \hat{n}$$，$$\boldsymbol{\sigma}$$ 为 Cauchy 应力张量

**Cauchy 运动方程：**

$$\rho \frac{d\mathbf{u}}{dt} = \nabla \cdot \boldsymbol{\sigma} + \mathbf{F}_{body}$$

### 5.3 角动量守恒

牵引力不对体积元产生力矩，因此：

$$\boldsymbol{\sigma} = \boldsymbol{\sigma}^T \quad \text{（应力张量对称）}$$

### 5.4 Newtonian 流体本构关系

$$\boldsymbol{\sigma} = -p\mathbf{I} + \mu(\nabla \mathbf{u} + \nabla \mathbf{u}^T)$$

代入 Cauchy 方程即得 Navier-Stokes 方程。

---

## 6. 参考文献

| 文献 | 内容 |
|------|------|
| Bridson et al. 2015 | *Fluid Simulation for Computer Graphics* (2nd ed.) |
| SIGGRAPH 2006/2007 Course | Fluid Simulation |
| Jiang et al. SIGGRAPH 2015 | The Affine Particle-In-Cell Method (APIC) |
| Fu et al. SIGGRAPH Asia 2017 | A Polynomial Particle-In-Cell Method (PolyPIC) |
| Müller et al. SCA 2003 | SPH 流体仿真 |
| Macklin et al. SIGGRAPH 2013 | Position-Based Fluids |
| Qu et al. SIGGRAPH 2019 | BiMocq2 |
| Deng et al. SIGGRAPH 2023 | Neural Flow Maps（Best Paper） |
| Steinhoff & Underhill 1994 | Vorticity Confinement |
| Fedkiw et al. 2001 | Vorticity Confinement |
