# 10 Spring-Mass System

Physics Simulation for Computer Graphics

- Source PDF: `/Users/charleschen/Library/Containers/com.apple.Safari/Data/tmp/WebKitPDFs-WGKAxE/10_MassSpring.pdf`
- Slides: 18
- 说明：下面先整理与仿真实现直接相关的公式、推导和伪代码；后半部分按页保留课件转写文本，并嵌入每页原始截图，便于对照 PDF 中的图示、布局和符号。
## 仿真实现重点

### 粒子系统

每个 particle 通常包含：

- position $\mathbf{x}$
- velocity $\mathbf{v}$
- force accumulator $\mathbf{f}$
- mass $m$

整个系统在时间步中维护所有粒子的 $\mathbf{x},\mathbf{v},\mathbf{f},m$。

### Spring energy 和力

一根弹簧连接粒子 $i,j$：

$$
\mathbf{x}_{ij}=\mathbf{x}_j-\mathbf{x}_i
$$

弹簧能量：

$$
E_{ij}=\frac12 k(\|\mathbf{x}_{ij}\|-l_0)^2
$$

粒子 $j$ 对粒子 $i$ 的力：

$$
\mathbf{f}_{ij}=k(\|\mathbf{x}_{ij}\|-l_0)\frac{\mathbf{x}_{ij}}{\|\mathbf{x}_{ij}\|}
=-\mathbf{f}_{ji}
$$

粒子 $i$ 的总力：

$$
\mathbf{f}_{i}=\sum_{j\in N(i)}\mathbf{f}_{ij}+\mathbf{f}_{i}^{ext}
$$

显式力计算伪代码：

```cpp
for (auto& p : particles) {
    p.force = p.mass * gravity;
}

for (auto& spring : springs) {
    int i = spring.i;
    int j = spring.j;
    Vec3 xij = particles[j].x - particles[i].x;
    Real len = norm(xij);
    Vec3 dir = xij / max(len, eps);
    Vec3 fij = spring.k * (len - spring.restLength) * dir;
    particles[i].force += fij;
    particles[j].force -= fij;
}
```

### Explicit Euler

积分近似：

$$
\mathbf{x}(t_n)-\mathbf{x}(t_{n-1})
=\int_{t_{n-1}}^{t_n}\mathbf{v}(t)dt
\approx \mathbf{v}(t_{n-1})\Delta t
$$

$$
\mathbf{v}(t_n)-\mathbf{v}(t_{n-1})
=\frac1m\int_{t_{n-1}}^{t_n}\mathbf{f}(t)dt
\approx \frac1m\mathbf{f}(t_{n-1})\Delta t
$$

显式 Euler simulator：

```cpp
for each timestep tn -> tn+1:
    for each particle i:
        compute force f_i(tn) from current positions
        x_i(tn+1) = x_i(tn) + v_i(tn) * dt
        v_i(tn+1) = v_i(tn) + (f_i(tn) / m_i) * dt
```

课件指出：显式 Euler 不稳定且误差较大。

### Implicit Euler

显式 Euler：

$$
\mathbf{x}^{n+1}=\mathbf{x}^{n}+\mathbf{v}^{n}\Delta t,
\qquad
\mathbf{v}^{n+1}=\mathbf{v}^{n}+\frac1m\mathbf{f}^{n}\Delta t
$$

隐式 Euler：

$$
\mathbf{x}^{n+1}=\mathbf{x}^{n}+\mathbf{v}^{n+1}\Delta t,
\qquad
\mathbf{v}^{n+1}=\mathbf{v}^{n}+\frac1m\mathbf{f}^{n+1}\Delta t
$$

令 $h=\Delta t$，把所有粒子堆成向量 $\mathbf{x},\mathbf{v},\mathbf{f}\in\mathbb{R}^{3n}$：

$$
\mathbf{x}^{n+1}=\mathbf{x}^{n}+h\mathbf{v}^{n+1}
$$

$$
\mathbf{v}^{n+1}=\mathbf{v}^{n}+h\mathbf{M}^{-1}\mathbf{f}(\mathbf{x}^{n+1})
$$

代入得到：

$$
\mathbf{x}^{n+1}=\mathbf{x}^{n}+h\mathbf{v}^{n}+h^2\mathbf{M}^{-1}\mathbf{f}(\mathbf{x}^{n+1})
$$

把力分成外力和内力：

$$
\mathbf{x}^{n+1}=\mathbf{x}^{n}+h(\mathbf{v}^{n}+h\mathbf{M}^{-1}\mathbf{f}^{ext})+h^2\mathbf{M}^{-1}\mathbf{f}^{int}(\mathbf{x}^{n+1})
$$

记独立于 $\mathbf{x}^{n+1}$ 的项为 $\mathbf{y}$：

$$
\mathbf{x}^{n+1}=\mathbf{y}+h^2\mathbf{M}^{-1}\mathbf{f}^{int}(\mathbf{x}^{n+1})
$$

### Implicit Euler = energy minimization

若内力来自势能：

$$
\mathbf{f}^{int}(\mathbf{x})=-\frac{dE(\mathbf{x})}{d\mathbf{x}}
$$

则隐式步等价于：

$$
\mathbf{x}^{n+1}=\arg\min_{\mathbf{x}} g(\mathbf{x}),
\qquad
 g(\mathbf{x})=\frac{1}{2h^2}\|\mathbf{x}-\mathbf{y}\|_{\mathbf{M}}^2+E(\mathbf{x})
$$

其中

$$
\|\mathbf{x}\|_{\mathbf{M}}^2=\mathbf{x}^{T}\mathbf{M}\mathbf{x}
$$

一阶最优条件：

$$
\nabla g(\mathbf{x})=\frac{1}{h^2}\mathbf{M}(\mathbf{x}-\mathbf{y})-\mathbf{f}^{int}(\mathbf{x})=0
$$

即：

$$
\mathbf{x}-\mathbf{y}-h^2\mathbf{M}^{-1}\mathbf{f}^{int}(\mathbf{x})=0
$$

这个形式适用于所有 conservative system，不只 mass-spring。

### Newton solver

目标：

$$
\mathbf{x}^{n+1}=\arg\min_{\mathbf{x}} g(\mathbf{x})
$$

Newton 更新：

$$
\mathbf{x}^{k+1}=\mathbf{x}^{k}-\mathbf{H}(g)^{-1}\nabla g
$$

实际实现通常解线性方程：

$$
\mathbf{H}(g)\Delta\mathbf{x}=-\nabla g,
\qquad
\mathbf{x}^{k+1}=\mathbf{x}^{k}+\Delta\mathbf{x}
$$

实现要点：每步计算 Hessian，求解矩阵方程，这是主要瓶颈。可使用 line search 防止 overshoot。线性/二次系统可用 Jacobi、Gauss-Seidel、Conjugate Gradient、Multigrid 等。

### Spring 的一阶和二阶导

设

$$
\mathbf{x}_{01}=\mathbf{x}_0-\mathbf{x}_1,
\qquad
E(\mathbf{x})=\frac{k}{2}(\|\mathbf{x}_{01}\|-L)^2
$$

力：

$$
\mathbf{f}(\mathbf{x})=-\nabla E(\mathbf{x})=
\begin{bmatrix}
\mathbf{f}_e\\
-\mathbf{f}_e
\end{bmatrix}
$$

$$
\mathbf{f}_e=-k(\|\mathbf{x}_{01}\|-L)\frac{\mathbf{x}_{01}}{\|\mathbf{x}_{01}\|}
$$

切线刚度：

$$
\mathbf{H}(\mathbf{x})=
\begin{bmatrix}
\frac{\partial^2E}{\partial\mathbf{x}_0^2} & \frac{\partial^2E}{\partial\mathbf{x}_0\partial\mathbf{x}_1}\\
\frac{\partial^2E}{\partial\mathbf{x}_0\partial\mathbf{x}_1} & \frac{\partial^2E}{\partial\mathbf{x}_1^2}
\end{bmatrix}
=
\begin{bmatrix}
\mathbf{H}_e & -\mathbf{H}_e\\
-\mathbf{H}_e & \mathbf{H}_e
\end{bmatrix}
$$

$$
\mathbf{H}_e=
 k\frac{\mathbf{x}_{01}\mathbf{x}_{01}^{T}}{\|\mathbf{x}_{01}\|^2}
+k\left(1-\frac{L}{\|\mathbf{x}_{01}\|}\right)
\left(\mathbf{I}-\frac{\mathbf{x}_{01}\mathbf{x}_{01}^{T}}{\|\mathbf{x}_{01}\|^2}\right)
$$

范数导数：

$$
\frac{\partial \|\mathbf{x}\|}{\partial\mathbf{x}}
=\frac{\partial(\mathbf{x}^T\mathbf{x})^{1/2}}{\partial\mathbf{x}}
=\frac{\mathbf{x}^{T}}{\|\mathbf{x}\|}
$$

### Implicit spring-mass 的梯度和 Hessian

$$
g(\mathbf{x})=\frac{1}{2h^2}\|\mathbf{x}-\mathbf{y}\|_{\mathbf{M}}^2+E(\mathbf{x})
$$

课件公式按仿真实现可写为：

$$
\nabla g(\mathbf{x}^{(k)})=\frac{1}{h^2}\mathbf{M}(\mathbf{x}^{(k)}-\mathbf{y})-\mathbf{f}(\mathbf{x}^{(k)})
$$

$$
\frac{\partial^2 g(\mathbf{x}^{(k)})}{\partial\mathbf{x}^2}=\frac{1}{h^2}\mathbf{M}+\mathbf{H}(\mathbf{x}^{(k)})
$$

总 stiffness 从每条边的局部矩阵装配：

$$
\mathbf{H}(\mathbf{x})=\sum_{e=\{i,j\}}
\begin{bmatrix}
\mathbf{H}_e & -\mathbf{H}_e\\
-\mathbf{H}_e & \mathbf{H}_e
\end{bmatrix}
$$

```cpp
for each Newton iteration:
    compute spring forces f(x)
    assemble H = M / (h*h) + springStiffness(x)
    grad = M * (x - y) / (h*h) - f(x)
    solve H * dx = -grad
    line_search_optional(x, dx)
    x += dx
```

## 逐页转写

### Slide 01

![Slide 01](assets/slides/10_MassSpring/10_MassSpring_p01.png)

```text
Physics Simulation for
Computer Graphics
10 Spring-Mass System
```

### Slide 02

![Slide 02](assets/slides/10_MassSpring/10_MassSpring_p02.png)

```text
Spatial Discretization: 1. Particle System
𝐱
𝐯
𝐟
𝑚
𝐱
𝐯
𝐟
𝑚
position
velocity
force calculator
mass
Particle Structure
2
particles n time
𝐱
𝐯
𝐟
𝑚
𝐱
𝐯
𝐟
𝑚
𝐱
𝐯
𝐟
𝑚
𝐱
𝐯
𝐟
𝑚…
Particle System
```

### Slide 03

![Slide 03](assets/slides/10_MassSpring/10_MassSpring_p03.png)

```text
Spatial Discretization: 2. Spring Energy
• Use springs to mimic elasticity energy
• For one spring:
𝐱𝑖𝑗 = 𝐱𝑗 − 𝐱𝑖, 𝐸𝑖𝑗 = 1
2 𝑘( 𝐱𝑖𝑗 − 𝑙0)2
• Force from particle 𝑗 to particle 𝑖:
𝐟𝑖𝑗 = 𝑘 𝐱𝑖𝑗 − 𝑙0
𝐱𝑖𝑗
𝐱𝑖𝑗
= −𝐟𝑗𝑖
• The total force on particle 𝑖:
𝐟𝑖 = ෍
𝑗∈𝑁(𝑖)
𝐟𝑖𝑗 + 𝐟𝑖
𝑒𝑥𝑡
𝑖
𝑗
3
𝐱𝑖𝑗
```

### Slide 04

![Slide 04](assets/slides/10_MassSpring/10_MassSpring_p04.png)

```text
Temporal Discretization: Explicit Euler
• Explicit Euler
• 𝐱 𝑡𝑛 − 𝐱 𝑡𝑛−1 =׬𝑡𝑛−1
𝑡𝑛 𝐯 𝑡 𝑑𝑡 ≈ 𝐯 𝑡𝑛−1 Δ𝑡
• 𝐯 𝑡𝑛 − 𝐯 𝑡𝑛−1 =
1
𝑚׬𝑡𝑛−1
𝑡𝑛 𝐟 𝑡 𝑑𝑡 ≈
1
𝑚 𝐟 𝑡𝑛−1 Δt
• We know this is very inaccurate…
4
∆𝑡
𝑡[n]𝑡[n−1]
න
𝑡[𝑛−1]
𝑡[𝑛]
𝐯 𝑡 𝑑𝑡
𝐯 𝑡
𝑣 𝑡𝑛−1 Δ𝑡
```

### Slide 05

![Slide 05](assets/slides/10_MassSpring/10_MassSpring_p05.png)

```text
Explicit Euler Simulator
• At each step 𝑡𝑛 → 𝑡𝑛+1:
• For each particle:
• Compute 𝐟 𝑡𝑛 = Σ𝐟𝑖𝑗 using current particle positions: 𝐟𝑖𝑗 = 𝑘 𝐱𝑖𝑗 − 𝑙0
𝐱𝒊𝒋
𝐱𝒊𝒋
• Update its position 𝐱 𝑡𝑛+1 = 𝐱 𝑡𝑛 + 𝐯 𝑡𝑛 Δ𝑡
• Update its velocity 𝐯 𝑡𝑛+1 = 𝐯 𝑡𝑛 +
1
𝑚 𝐟 𝑡𝑛 Δt
5
```

### Slide 06

![Slide 06](assets/slides/10_MassSpring/10_MassSpring_p06.png)

```text
Here is the result
6
```

### Slide 07

![Slide 07](assets/slides/10_MassSpring/10_MassSpring_p07.png)

```text
To Improve Stability: Implicit Euler
• Explicit Euler:
• 𝐱 𝑡𝑛+1 = 𝐱 𝑡𝑛 + 𝐯 𝑡𝑛 Δ𝑡
• 𝐯 𝑡𝑛+1 = 𝐯 𝑡𝑛 +
1
𝑚 𝐟 𝑡𝑛 Δt
• Implicit Euler:
• 𝐱 𝑡𝑛+1 = 𝐱 𝑡𝑛 + 𝐯 𝑡𝑛+1 Δ𝑡
• 𝐯 𝑡𝑛+1 = 𝐯 𝑡𝑛 +
1
𝑚 𝐟 𝑡𝑛+1 Δt
• Later we’ll see why implicit Euler is stable
7
∆𝑡
𝑡[𝑛+1]𝑡[𝑛]
න
𝑡[𝑛]
𝑡[𝑛+1]
𝐯 𝑡 𝑑𝑡
𝐯 𝑡
𝑣 𝑡𝑛+1 Δ𝑡
```

### Slide 08

![Slide 08](assets/slides/10_MassSpring/10_MassSpring_p08.png)

```text
Implicit Euler
• Target: solve equations for 𝐱𝑛+1 and 𝐯𝑛+1
• For convenience, denote ℎ = 𝛥𝑡, 𝐱, 𝐯, 𝐟 ∈ 𝑅3𝑛 are collections of all particles
𝐱𝑛+1 = 𝐱𝑛 + ℎ 𝐯𝑛+1
𝐯𝑛+1 = 𝐯𝑛 + ℎ M−1 𝐟(𝐱𝑛+1)
𝐱𝑛+1 = 𝐱𝑛 + ℎ𝐯𝑛 + ℎ2 M−1 𝐟 𝐱𝑛+1
𝐱𝑛+1 = 𝐱𝑛 + ℎ𝐯𝑛 + ℎ2 M−1 𝐟𝑒𝑥𝑡 + ℎ2 M−1 𝐟𝑖𝑛𝑡 𝐱𝑛+1
independent of 𝐱𝑛+1 function of 𝐱𝑛+1
Implicit Euler
Substitute 𝐯𝑛+1 into 𝐱𝑛+1
Divide 𝑓 𝑥𝑛+1  into 2 parts
denote the first term as 𝐲
8
= 𝐱𝑛 + ℎ(𝐯𝑛+ℎ M−1 𝐟𝑒𝑥𝑡) + ℎ2 M−1 𝐟𝑖𝑛𝑡 𝐱𝑛+1
```

### Slide 09

![Slide 09](assets/slides/10_MassSpring/10_MassSpring_p09.png)

```text
Implicit Euler
• 𝐱𝑛+1 = 𝐲 + ℎ2M−1𝐟𝑖𝑛𝑡 𝐱𝑛+1
𝐱𝑛+1 = 𝑎𝑟𝑔𝑚𝑖𝑛𝐱 𝑔 𝐱 , for 𝑔 𝐱 =
1
2ℎ2 𝐱 − 𝐲 M
2 + 𝐸 𝐱
This is because:
9
𝐱 M
2 = 𝐱TM𝐱
𝐟𝑖𝑛𝑡 𝐱𝑛+1 = − 𝑑𝐸 𝐱
𝑑𝐱∇𝑔 𝐱 = 1
ℎ2M 𝐱 − 𝐲 − 𝐟𝑖𝑛𝑡 𝒙 = 0
𝐱 − 𝐲 − ℎ2M−1𝐟𝑖𝑛𝑡 𝑥 = 0
Applicable to every system
not just a mass-spring system
```

### Slide 10

![Slide 10](assets/slides/10_MassSpring/10_MassSpring_p10.png)

```text
Implicit Euler
• 𝐱𝑛+1 = 𝐲 + ℎ2M−1𝐟𝐢𝐧𝐭 𝐱𝐧+𝟏
𝐱𝑛+1 = 𝑎𝑟𝑔𝑚𝑖𝑛𝐱 𝑔 𝐱 , for 𝑔 𝐱 =
1
2ℎ2 𝐱 − 𝐲 M
2 + 𝐸 𝐱
• Implicit Euler = energy minimization:
𝑎𝑟𝑔𝑚𝑖𝑛𝐱
1
2ℎ2 𝐱 − 𝐲 M
2 + 𝐸 𝐱
• Stable under any timestep size
inertia elasticity
10
𝐸𝑖𝑗 = 1
2 𝑘( 𝐱i𝐣 − 𝑙0)2
𝐟𝑖𝑗 = 𝑘 𝐱i𝐣 − 𝑙0
𝐱i𝐣
𝐱i𝐣
𝐱 M
2 = 𝐱TM𝐱
𝐟𝑖𝑛𝑡 𝐱𝑛+1 = 𝑑𝐸 𝐱
𝑑𝐱
```

### Slide 11

![Slide 11](assets/slides/10_MassSpring/10_MassSpring_p11.png)

```text
Numerical Solver
𝐱𝑛+1 = 𝑎𝑟𝑔𝑚𝑖𝑛𝐱 𝑔
• Newton’s method:
Start from a guess 𝐱1,
update with
𝐱𝑘+1 = 𝐱𝑘 − H 𝑔 −1∇𝑔
11
𝑓 𝑥 = 𝑑 𝑔
𝑑 𝑥 = 𝛻𝑔
```

### Slide 12

![Slide 12](assets/slides/10_MassSpring/10_MassSpring_p12.png)

```text
Numerical Solver
𝐱𝑛+1 = 𝑎𝑟𝑔𝑚𝑖𝑛𝐱 𝑔
• Newton’s method:
Start from a guess 𝐱1,
update with
𝐱𝑘+1 = 𝐱𝑘 − H 𝑔 −1∇𝑔,
until converge.
12
𝑓 𝑥 = 𝑑 𝑔
𝑑 𝑥 = 𝛻𝑔
• Compute Hessian matrix H(𝑔) ∈ 𝑅3𝑛×3𝑛 at every step
• Solve Matrix equation at every step (main bottleneck)
• Line search: prevent overshoot
```

### Slide 13

![Slide 13](assets/slides/10_MassSpring/10_MassSpring_p13.png)

```text
Numerical Solver
To solve nonlinear equation systems:
• Newton’s methods
• Quasi-Newton Methods
• BFGS…
To solve matrix equations (linear or quadratic equation systems):
• Jacobi/Gauss-Seidel Iteration
• Conjugate Gradient
• Multigrid…
No one general optimal solver for all problems
13
```

### Slide 14

![Slide 14](assets/slides/10_MassSpring/10_MassSpring_p14.png)

```text
Implicit Euler Results
cloth rod
14
```

### Slide 15

![Slide 15](assets/slides/10_MassSpring/10_MassSpring_p15.png)

```text
This is Spring-Mass System
Huamin Wang SIGGRAPH2021
GPU-Based Simulation of Cloth Wrinkles at Submillimeter Levels
15
```

### Slide 16

![Slide 16](assets/slides/10_MassSpring/10_MassSpring_p16.png)

```text
This is Also Spring Mass
16
```

### Slide 17

![Slide 17](assets/slides/10_MassSpring/10_MassSpring_p17.png)

```text
17
Appendix : 1st&2nd -Order Derivatives
If 𝑓 𝐱 ∈ 𝐑, then:
𝐇 = 𝐉(∇𝑓 𝐱 ) =
𝜕2𝑓
𝜕𝑥2
𝜕2𝑓
𝜕𝑥𝜕𝑦
𝜕2𝑓
𝜕𝑥𝜕𝑧
𝜕2𝑓
𝜕𝑥𝜕𝑦
𝜕2𝑓
𝜕𝑦2
𝜕2𝑓
𝜕𝑦𝜕𝑧
𝜕2𝑓
𝜕𝑥𝜕𝑧
𝜕2𝑓
𝜕𝑦𝜕𝑧
𝜕2𝑓
𝜕𝑧2
Hessian
∇𝑓 𝐱 =
𝜕𝑓
𝜕𝑥
𝜕𝑓
𝜕𝑦
𝜕𝑓
𝜕𝑧
Gradient
𝐱1𝐱0
𝐱01 = 𝐱0 − 𝐱1
Energy: 𝐸 𝐱 = 𝑘
2 𝐱01 − 𝐿 2
𝐟 𝐱 = −∇𝐸 𝐱 = −∇0𝐸 𝐱
−∇1𝐸 𝐱 = 𝐟𝑒
−𝐟𝑒
Force:
Tangent
stiffness: 𝐇 𝐱 =
𝜕2𝐸
𝜕𝐱02
𝜕2𝐸
𝜕𝐱0𝜕𝐱1
𝜕2𝐸
𝜕𝐱0𝜕𝐱1
𝜕2𝐸
𝜕𝐱12
= 𝐇𝑒 −𝐇𝑒
−𝐇𝑒 𝐇𝑒
𝐇𝑒 = 𝑘𝐱01𝐱01T
𝐱01 2 + 𝑘 1 −
𝐿
𝐱01
𝐈 − 𝐱01𝐱01T
𝐱01 2
𝐟𝑒 = −𝑘 𝐱01 − 𝐿 𝐱01
𝐱01
𝜕 𝐱
𝜕𝐱 =
𝜕 𝐱T𝐱
1/2
𝜕𝐱 =
1
2 𝐱T𝐱
−1/2 𝜕 𝐱T𝐱
𝜕𝐱
= 1
2 𝐱 2𝐱T = 𝐱T
𝐱
Example: A Spring
Rest length 𝐿，
```

### Slide 18

![Slide 18](assets/slides/10_MassSpring/10_MassSpring_p18.png)

```text
Appendix: Spring-Mass System
18
Specifically to simulation, we have:
∇𝑔 𝐱(𝑘) = 1
ℎ2𝐌 − 𝐱(𝑘) − 𝒚 𝐟 𝐱(𝑘)
𝑔 𝐱 = 1
2ℎ2 𝐱 − 𝒚 𝐌
2 + 𝐸(𝐱)
𝜕2𝑔 𝐱(𝑘)
𝜕𝐱2 = 1
ℎ2𝐌 + 𝐇 𝐱 𝑘
𝐇 𝐱 = σ𝑒={𝑖,𝑗}
𝜕2𝐸
𝜕𝐱𝑖
2
𝜕2𝐸
𝜕𝐱𝑖𝜕𝐱𝑗
𝜕2𝐸
𝜕𝐱𝑖𝜕𝐱𝑗
𝜕2𝐸
𝜕𝐱𝑗
2
= σ𝑒={𝑖,𝑗}
𝐇𝑒 −𝐇𝑒
−𝐇𝑒 𝐇𝑒
𝐇𝑒 = 𝑘
𝐱𝑖𝑗𝐱𝑖𝑗
T
𝐱𝑖𝑗
2 + 𝑘 1 −
𝐿
𝐱𝑖𝑗
𝐈 −
𝐱𝑖𝑗𝐱𝑖𝑗
T
𝐱𝑖𝑗
2
```
