# 11 Cloth Simulation

Physics Simulation for Computer Graphics

- Source PDF: `/Users/charleschen/Library/Containers/com.apple.Safari/Data/tmp/WebKitPDFs-WGKAxE/11_Cloth.pdf`
- Slides: 38
- 说明：下面先整理与仿真实现直接相关的公式、推导和伪代码；后半部分按页保留课件转写文本，并嵌入每页原始截图，便于对照 PDF 中的图示、布局和符号。
## 仿真实现重点

### Cloth simulation 中的力

布料动力学：

$$
\frac{d\mathbf{v}}{dt}=\mathbf{M}^{-1}\mathbf{f}(\mathbf{x},\mathbf{v})
$$

力可分为：

- Internal forces：tension、shearing、bending
- External forces：contact、friction、collision、gravity、environmental force，例如 wind
- Tearable cloth 也可扩展

Tension 和 shearing 是 in-plane deformation；bending 是 out-of-plane deformation。

### Spring network 和 bending

结构化弹簧网络可加入：

- stretching/tension springs
- shearing springs
- bending springs

非结构化网格通常用 edge springs 表示面内拉伸/剪切，并对相邻三角形 pair 添加 bending 项。

Bending spring 的问题：当布料接近平面时，对角线长度变化很小，因此对弯曲的阻力很弱。

### Dihedral angle bending model

二面角模型把 bending force 写成：

$$
\mathbf{f}_i=f(\theta)\mathbf{u}_i
$$

其中 $\theta$ 是相邻两个三角形的二面角，$\mathbf{u}_i$ 是各顶点的方向向量。

对于共享边 $(\mathbf{x}_3,\mathbf{x}_4)$ 的两个三角形，课件记：

$$
\mathbf{E}=\mathbf{x}_4-\mathbf{x}_3
$$

$$
\mathbf{N}_1=(\mathbf{x}_1-\mathbf{x}_3)\times(\mathbf{x}_1-\mathbf{x}_4)
$$

$$
\mathbf{N}_2=(\mathbf{x}_2-\mathbf{x}_4)\times(\mathbf{x}_2-\mathbf{x}_3)
$$

方向向量：

$$
\mathbf{u}_1=\|\mathbf{E}\|\frac{\mathbf{N}_1}{\|\mathbf{N}_1\|^2}
$$

$$
\mathbf{u}_2=\|\mathbf{E}\|\frac{\mathbf{N}_2}{\|\mathbf{N}_2\|^2}
$$

$$
\mathbf{u}_3=\frac{(\mathbf{x}_1-\mathbf{x}_4)\cdot\mathbf{E}}{\|\mathbf{E}\|}\frac{\mathbf{N}_1}{\|\mathbf{N}_1\|^2}
+\frac{(\mathbf{x}_2-\mathbf{x}_4)\cdot\mathbf{E}}{\|\mathbf{E}\|}\frac{\mathbf{N}_2}{\|\mathbf{N}_2\|^2}
$$

$$
\mathbf{u}_4=-\frac{(\mathbf{x}_1-\mathbf{x}_3)\cdot\mathbf{E}}{\|\mathbf{E}\|}\frac{\mathbf{N}_1}{\|\mathbf{N}_1\|^2}
-\frac{(\mathbf{x}_2-\mathbf{x}_3)\cdot\mathbf{E}}{\|\mathbf{E}\|}\frac{\mathbf{N}_2}{\|\mathbf{N}_2\|^2}
$$

Planar material bending force：

$$
\mathbf{f}_i=k\frac{\|\mathbf{E}\|^2}{\|\mathbf{N}_1\|+\|\mathbf{N}_2\|}
\sin\left(\frac{\pi-\theta}{2}\right)\mathbf{u}_i
$$

Non-planar rest angle $\theta_0$：

$$
\mathbf{f}_i=k\frac{\|\mathbf{E}\|^2}{\|\mathbf{N}_1\|+\|\mathbf{N}_2\|}
\left(\sin\left(\frac{\pi-\theta}{2}\right)-
\sin\left(\frac{\pi-\theta_0}{2}\right)\right)\mathbf{u}_i
$$

实现提示：二面角模型 explicit integration 容易实现，但 derivative 难算，隐式实现会更复杂。

```cpp
for each adjacent triangle pair sharing edge (x3, x4):
    E  = x4 - x3
    N1 = cross(x1 - x3, x1 - x4)
    N2 = cross(x2 - x4, x2 - x3)
    theta = dihedral_angle(N1, N2, E)

    u1 = norm(E) * N1 / squaredNorm(N1)
    u2 = norm(E) * N2 / squaredNorm(N2)
    u3 = dot(x1 - x4, E) / norm(E) * N1 / squaredNorm(N1)
       + dot(x2 - x4, E) / norm(E) * N2 / squaredNorm(N2)
    u4 = -dot(x1 - x3, E) / norm(E) * N1 / squaredNorm(N1)
       - dot(x2 - x3, E) / norm(E) * N2 / squaredNorm(N2)

    scalar = k * squaredNorm(E) / (norm(N1) + norm(N2))
           * (sin((pi - theta) / 2) - sin((pi - theta0) / 2))

    f[x1] += scalar * u1
    f[x2] += scalar * u2
    f[x3] += scalar * u3
    f[x4] += scalar * u4
```

### Stretching 和 locking issue

课件假设 cloth planar deformation 和 cloth bending deformation 独立。问题是：若边都是约束，布料可能不能自由折叠。

约束自由度估算：

$$
DoFs=3N_{Vertices}-N_{Edges}
$$

对 manifold mesh，Euler formula：

$$
N_{Edges}=3N_{Vertices}-3N_{BoundaryEdges}
$$

因此系统自由度：

$$
DoFs=3N_{BoundaryEdges}
$$

现实布料很容易 bend，但不太 stretch。若 edge spring 太 stiff，会产生 locking issues、不稳定、极小时间步；若 edge spring 太 soft，会有橡胶状 stretching 和 coarse wrinkles。

### Position Based Dynamics (PBD)

PBD 基于 projection function。单根无限刚度弹簧可视为长度约束：

$$
\phi(\mathbf{x})=\|\mathbf{x}_i-\mathbf{x}_j\|-L=0
$$

投影目标：

$$
\{\mathbf{x}_i^{new},\mathbf{x}_j^{new}\}
=\arg\min\frac12\left(m_i\|\mathbf{x}_i^{new}-\mathbf{x}_i\|^2+
 m_j\|\mathbf{x}_j^{new}-\mathbf{x}_j\|^2\right)
$$

subject to

$$
\phi(\mathbf{x})=0
$$

投影公式：

$$
\mathbf{x}_i^{new}\leftarrow
\mathbf{x}_i-\frac{m_j}{m_i+m_j}(\|\mathbf{x}_i-\mathbf{x}_j\|-L)
\frac{\mathbf{x}_i-\mathbf{x}_j}{\|\mathbf{x}_i-\mathbf{x}_j\|}
$$

$$
\mathbf{x}_j^{new}\leftarrow
\mathbf{x}_j+\frac{m_i}{m_i+m_j}(\|\mathbf{x}_i-\mathbf{x}_j\|-L)
\frac{\mathbf{x}_i-\mathbf{x}_j}{\|\mathbf{x}_i-\mathbf{x}_j\|}
$$

默认 $m_i=m_j$；可设置 $m_i=\infty$ 表示 stationary node。

PBD simulator：

```cpp
// Do kinematics without constraints.
for each vertex i:
    v[i] += h * externalForce[i] / mass[i]
    x[i] += h * v[i]

// Constraint projection.
xNew = Projection(x)

// Velocity update following projection is important.
for each vertex i:
    v[i] += (xNew[i] - x[i]) / h
    x[i] = xNew[i]
```

Jacobi 多约束投影（课件结构）：

```cpp
for k = 0 ... K:
    for every vertex i:
        xNew[i] = Vec3(0)
        n[i] = 0

    for every edge e = {i, j}:
        dir = (x[i] - x[j]) / norm(x[i] - x[j])
        xNew[i] += x[i] - 0.5 * (norm(x[i] - x[j]) - L_e) * dir
        xNew[j] += x[j] + 0.5 * (norm(x[i] - x[j]) - L_e) * dir
        n[i] += 1
        n[j] += 1

    for every vertex i:
        xNew[i] = xNew[i] / n[i]
        x[i] = (xNew[i] + alpha * x[i]) / (1 + alpha)
```

PBD 特点：

- Pros：GPU parallelable、实现简单、低分辨率很快、通用，可处理 fluids 等约束。
- Cons：不物理正确；高分辨率低效；层次方法可能振荡；需要 Chebyshev 等加速方法。

### Strain limiting

Strain limiting 用 projection function 做 correction，而不是完全代替动力学。

模拟结构：

```cpp
// Do simulation with soft / relaxed constraints.
update x and v using dynamics

// Strain limiting starts.
xNew = Projection(x)
v += (xNew - x) / dt
x = xNew
```

#### Spring strain limit

限制拉伸比例：

$$
\sigma_{min}\le \frac{1}{L}\|\mathbf{x}_i-\mathbf{x}_j\|\le \sigma_{max}
$$

当前 strain：

$$
\sigma=\frac{1}{L}\|\mathbf{x}_i-\mathbf{x}_j\|
$$

clamp 后：

$$
\sigma_0\leftarrow \min(\max(\sigma,\sigma_{min}),\sigma_{max})
$$

投影公式：

$$
\mathbf{x}_i^{new}\leftarrow
\mathbf{x}_i-\frac{m_j}{m_i+m_j}(\|\mathbf{x}_i-\mathbf{x}_j\|-\sigma_0L)
\frac{\mathbf{x}_i-\mathbf{x}_j}{\|\mathbf{x}_i-\mathbf{x}_j\|}
$$

$$
\mathbf{x}_j^{new}\leftarrow
\mathbf{x}_j+\frac{m_i}{m_i+m_j}(\|\mathbf{x}_i-\mathbf{x}_j\|-\sigma_0L)
\frac{\mathbf{x}_i-\mathbf{x}_j}{\|\mathbf{x}_i-\mathbf{x}_j\|}
$$

PBD 是 $\sigma_0\equiv1$；no limit 相当于 $\sigma_{min},\sigma_{max}\leftarrow\infty$。

#### Triangle area limit

面积限制：

$$
A_{min}\le A\le A_{max}
$$

面积：

$$
A\leftarrow\frac12\|(\mathbf{x}_j-\mathbf{x}_i)\times(\mathbf{x}_k-\mathbf{x}_i)\|
$$

scale：

$$
s\leftarrow \sqrt{\frac{\min(\max(A,A_{min}),A_{max})}{A}}
$$

质量中心不动：

$$
\mathbf{c}\leftarrow\frac{m_i\mathbf{x}_i+m_j\mathbf{x}_j+m_k\mathbf{x}_k}{m_i+m_j+m_k}
$$

投影：

$$
\mathbf{x}_i^{new}\leftarrow\mathbf{c}+s(\mathbf{x}_i-\mathbf{c})
$$

$$
\mathbf{x}_j^{new}\leftarrow\mathbf{c}+s(\mathbf{x}_j-\mathbf{c})
$$

$$
\mathbf{x}_k^{new}\leftarrow\mathbf{c}+s(\mathbf{x}_k-\mathbf{c})
$$

Strain limiting 常用于：避免不稳定、避免大形变 artifacts、非线性效果、解决 locking issue。

### 相关方法

- Goldenthal et al. 2007, Efficient Simulation of Inextensible Cloth.
- Tournier et al. 2015, Stable Constrained Dynamics.
- Jin et al. 2017, Inequality Cloth.
- Macklin et al. 2013, Position Based Fluids.
- XPBD: Extended Position Based Dynamics.

## 逐页转写

### Slide 01

![Slide 01](assets/slides/11_Cloth/11_Cloth_p01.png)

```text
Physics Simulation for
Computer Graphics
11 Cloth Simulation
```

### Slide 02

![Slide 02](assets/slides/11_Cloth/11_Cloth_p02.png)

```text
Forces in Cloth Simulation
•
d𝐯
d𝑡 = 𝑀−1 𝑓(𝐱, 𝐯)
• 𝑓(𝐱, 𝐯):
• Internal Forces: Tension, Shearing, Bending
• External Forces: Contact, Friction, Collision, Gravity, Environmental force (wind)
• Tearable Cloth…
2
```

### Slide 03

![Slide 03](assets/slides/11_Cloth/11_Cloth_p03.png)

```text
Bending
• Tension : in-plane
• Shearing : in-plane
• Bending : out-of-plane
3
Internal Forces: Tension, Shearing, Bending
Tension Shearing
```

### Slide 04

![Slide 04](assets/slides/11_Cloth/11_Cloth_p04.png)

```text
Internal Forces: Tension, Shearing, Bending
• Structure Spring Networks
4
√ Tension
√ Shearing
× Bending
√ Tension
√ Shearing
√ Bending
√ Tension
× Shearing
× Bending
```

### Slide 05

![Slide 05](assets/slides/11_Cloth/11_Cloth_p05.png)

```text
Internal Forces: Tension, Shearing, Bending
• Unstructured Spring Networks
5
Edges
Bending (every
neighboring triangle pair)
√ Tension
√ Shearing
√ Bending
√ Tension
√ Shearing
× Bending
```

### Slide 06

![Slide 06](assets/slides/11_Cloth/11_Cloth_p06.png)

```text
Internal Forces: Tension, Shearing, Bending
• Spring Networks
6
√ Tension
√ Shearing
√ Bending
√ Tension
√ Shearing
√ Bending
Edges
Bending
Shearing
```

### Slide 07

![Slide 07](assets/slides/11_Cloth/11_Cloth_p07.png)

```text
The Bending Spring Issue
7
Bending spring: little resistance when cloth is nearly planar (length barely changes)
𝐱0𝐱1 𝐱2
```

### Slide 08

![Slide 08](assets/slides/11_Cloth/11_Cloth_p08.png)

```text
A Dihedral Angle Model
8
Bending forces as a function of 𝜃: 𝐟𝑖 = 𝑓 𝜃 𝐮𝑖.
𝐱1
𝐱2
𝐱3
𝐱4𝐮3
𝐮4
𝐮1 𝐮2
𝐧2
𝐧1
• First, 𝐮1 and 𝐮2 should be in the normal
directions 𝐧1 and 𝐧2.
• Second, bending doesn’t stretch the
edge, so 𝐮𝟒 − 𝐮𝟑 should be orthogonal
to the edge, i.e., in the span of 𝐧1 and
𝐧2.
• Finally, 𝐮1 + 𝐮2 + 𝐮3 + 𝐮4 = 𝟎
𝜃
𝐧2𝐧1
𝜋 − 𝜃
```

### Slide 09

![Slide 09](assets/slides/11_Cloth/11_Cloth_p09.png)

```text
9
A dihedral angle model defines bending forces as a function of 𝜃: 𝐟𝑖 = 𝑓 𝜃 𝐮𝑖.
𝐱1
𝐱2
𝐱3
𝐱4𝐮3
𝐮4
𝐮1
𝐧2
𝐧1
Conclusion:
𝐮1 = 𝐄 𝐍1
𝐍1 2 𝐮2 = 𝐄 𝐍2
𝐍2 2
𝐮3 = (𝐱1 − 𝐱4) ∙ 𝐄
𝐄
𝐍1
𝐍1 2 + (𝐱2 − 𝐱4) ∙ 𝐄
𝐄
𝐍2
𝐍2 2
𝐮4 = − (𝐱1 − 𝐱3) ∙ 𝐄
𝐄
𝐍1
𝐍1 2 − (𝐱2 − 𝐱3) ∙ 𝐄
𝐄
𝐍2
𝐍2 2
𝐍1 = (𝐱1 − 𝐱3) × (𝐱1 − 𝐱4)
𝐍2 = (𝐱2 − 𝐱4) × (𝐱2 − 𝐱3)
𝐸 = 𝐱4 − 𝐱3
𝜃
𝐧2𝐧1
𝜋 − 𝜃
A Dihedral Angle Model
𝐮2
```

### Slide 10

![Slide 10](assets/slides/11_Cloth/11_Cloth_p10.png)

```text
10
A dihedral angle model defines bending forces as a function of 𝜃: 𝐟𝑖 = 𝑓 𝜃 𝐮𝑖.
𝐱1
𝐱2
𝐱3
𝐱4𝐮3
𝐮4
𝐮1
𝐧2
𝐧1
𝐟𝑖 = 𝑘 𝐄 2
𝐍1 + 𝐍2
sin 𝜋 − 𝜃
2 𝐮𝑖
Planar Material:
𝐟𝑖 = 𝑘 𝐄 2
𝐍1 + 𝐍2
sin 𝜋 − 𝜃
2 − sin 𝜋 − 𝜃0
2 𝐮𝑖
Non-planar:
𝐧2𝐧1
𝜋 − 𝜃
𝜃
A Dihedral Angle Model
𝐮2
```

### Slide 11

![Slide 11](assets/slides/11_Cloth/11_Cloth_p11.png)

```text
• Distance constraint
• Simple, weak in flat state
11
𝐱1
𝐱2
𝐱3
𝐱4
𝐧2
𝐧1
𝜃
Bending
• Angle constraint
• Strong in flat state,
more expensive
```

### Slide 12

![Slide 12](assets/slides/11_Cloth/11_Cloth_p12.png)

```text
Bridson et al. 2003. Simulation of
Clothing with Folds and Wrinkles. SCA.
Reference
12
Explicit integration.
Derivative is difficult to compute.
```

### Slide 13

![Slide 13](assets/slides/11_Cloth/11_Cloth_p13.png)

```text
Stretching and Locking Issues
13
```

### Slide 14

![Slide 14](assets/slides/11_Cloth/11_Cloth_p14.png)

```text
14
Assumption: cloth planar deformation and cloth bending deformation are
independent.
Issue: Can a simulator fold cloth freely?
folding line folding line
The Locking Issue
```

### Slide 15

![Slide 15](assets/slides/11_Cloth/11_Cloth_p15.png)

```text
The Locking Issue
15
Edges are constraints: 𝐷𝑜𝐹𝑠 = 3 ∗ 𝑁Vertices – 𝑁Edges.
For a manifold mesh, Euler’s formula : 𝑁Edges = 3 ∗ 𝑁Vertices − 3 ∗ 𝑁BoundaryEdges
System DOFs: 3 ∗ 𝑁BoundaryEdges
(along edges) (against edges) (unstructured mesh)
```

### Slide 16

![Slide 16](assets/slides/11_Cloth/11_Cloth_p16.png)

```text
Stretching and Locking Issues
16
Real Cloth:
• Very likely to Bend,
• Not really stretch a lot…
Stiff Edge Springs
(locking issues, unstable, tiny time step)
Soft Edge Springs
(rubbery stretching with “coarse” wrinkles)
Elastic
Inequal Elastic, or Inextensible
```

### Slide 17

![Slide 17](assets/slides/11_Cloth/11_Cloth_p17.png)

```text
Stretching and Locking Issues
17
Real Cloth:
• Very likely to Bend,
• Not really stretch a lot…
Inequality Cloth (more “detailed” wrinkles)
[Jin et al. Inequality Cloth, SCA2017]
Stiff Edge Springs
(locking issues, unstable, tiny time step)
Soft Edge Springs
(rubbery stretching with “coarse” wrinkles)
Elastic
Inequal Elastic, or Inextensible
```

### Slide 18

![Slide 18](assets/slides/11_Cloth/11_Cloth_p18.png)

```text
Position Based Dynamics
18
```

### Slide 19

![Slide 19](assets/slides/11_Cloth/11_Cloth_p19.png)

```text
A Single Spring → One Position-Based Constraint
19
The spring is infinitely stiff. Its length is a constraint and defines a projection。
𝐱𝑖
rest length 𝐿
𝐱𝑗
Constraint 𝐱𝑖
𝐱𝑗
𝐱𝑖
new
𝐱𝑗
new
𝐱𝑖
𝐱𝑗
𝐱𝑖
new
𝐱𝑗
new
Stretched case Compressed Case
𝜙(𝐱) = 𝐱𝑖 − 𝐱𝑗 − 𝐿 = 0
```

### Slide 20

![Slide 20](assets/slides/11_Cloth/11_Cloth_p20.png)

```text
20
𝐱𝑖
new, 𝐱𝑗
new = argmin1
2 𝑚𝑖 𝐱𝑖
new − 𝐱𝑖 2 + 𝑚𝑗 𝐱𝑗
new − 𝐱𝑗
2
𝐱𝑖
rest length 𝐿
𝐱𝑗
Constraint
𝜙(𝐱) = 𝐱𝑖 − 𝐱𝑗 − 𝐿 = 0
Subject to 𝜙(𝐱) = 0
𝛀 = {𝐱 ∈ 𝐑6: 𝜙(𝐱) = 0}
Boundary 𝜕𝛀
𝐱 = {𝐱𝑖, 𝐱𝑗}
One Position-Based Constraint → Projection
The spring is infinitely stiff. Its length is a constraint and defines a projection。
```

### Slide 21

![Slide 21](assets/slides/11_Cloth/11_Cloth_p21.png)

```text
21
𝐱𝑖
rest length 𝐿
𝐱𝑗
Constraint
𝜙(𝐱) = 𝐱𝑖 − 𝐱𝑗 − 𝐿 = 0 𝐱𝑖
new ⟵ 𝐱𝑖 − 𝑚𝑗
𝑚𝑖+𝑚𝑗
𝐱𝑖 − 𝐱𝑗 − 𝐿 𝐱𝑖− 𝐱𝑗
𝐱𝑖− 𝐱𝑗
𝐱𝑗
new ⟵ 𝐱𝑗 + 𝑚𝑖
𝑚𝑖+𝑚𝑗
𝐱𝑖 − 𝐱𝑗 − 𝐿 𝐱𝑖− 𝐱𝑗
𝐱𝑖− 𝐱𝑗
𝜙 𝐱new = 𝐱𝑖
new − 𝐱𝑗
new − 𝐿 = 𝐱𝑖 − 𝐱𝑗 − 𝐱𝑖 + 𝐱𝑗 + 𝐿 − 𝐿 = 0
𝐱new ⟵ Projection(𝐱)
𝐱𝑖
new, 𝐱𝑗
new = argmin1
2 𝑚𝑖 𝐱𝑖
new − 𝐱𝑖 2 + 𝑚𝑗 𝐱𝑗
new − 𝐱𝑗
2
By default, 𝑚𝑖 = 𝑚𝑗, but we can also set 𝑚𝑖 = ∞ for stationary nodes.
One Position-Based Constraint → Projection
The spring is infinitely stiff. Its length is a constraint and defines a projection。
Subject to 𝜙(𝐱) = 0
```

### Slide 22

![Slide 22](assets/slides/11_Cloth/11_Cloth_p22.png)

```text
22
𝐱𝑖
new, 𝐱𝑗
new = argmin1
2 𝑚𝑖 𝐱𝑖
new − 𝐱𝑖 2 + 𝑚𝑗 𝐱𝑗
new − 𝐱𝑗
2
𝐱𝑖
rest length 𝐿
𝐱𝑗
Constraint
𝜙(𝐱) = 𝐱𝑖 − 𝐱𝑗 − 𝐿 = 0
Subject to 𝜙(𝐱) = 0
𝛀 = {𝐱 ∈ 𝐑6: 𝜙(𝐱) = 0}
Boundary 𝜕𝛀
One Position-Based Constraint → Projection
The spring is infinitely stiff. Its length is a constraint and defines a projection。
𝐱𝑡−1,
ሶ𝐱𝑡−1
{𝐱𝑡, 𝐱𝑡+1,…}
Spring-Mass System,
With Strong Stiffness,
Locking issues
```

### Slide 23

![Slide 23](assets/slides/11_Cloth/11_Cloth_p23.png)

```text
23
𝐱𝑖
new, 𝐱𝑗
new = argmin1
2 𝑚𝑖 𝐱𝑖
new − 𝐱𝑖 2 + 𝑚𝑗 𝐱𝑗
new − 𝐱𝑗
2
𝐱𝑖
rest length 𝐿
𝐱𝑗
Constraint
𝜙(𝐱) = 𝐱𝑖 − 𝐱𝑗 − 𝐿 = 0
Subject to 𝜙(𝐱) = 0
𝛀 = {𝐱 ∈ 𝐑6: 𝜙(𝐱) = 0}
Boundary 𝜕𝛀
𝐱 = {𝐱𝑖, 𝐱𝑗}
One Position-Based Constraint → Projection
The spring is infinitely stiff. Its length is a constraint and defines a projection。
𝐱𝑡−1,
ሶ𝐱𝑡−1
```

### Slide 24

![Slide 24](assets/slides/11_Cloth/11_Cloth_p24.png)

```text
Position Based Dynamics (PBD)
24
Position based dynamics (PBD) is based on the projection function.
A PBD Simulator
// Do Kinematics w.o. Constraints
// update 𝐱 and 𝐯
𝐱new ⟵ Projection(𝐱)
𝐯 ⟵ ⋯
𝐱 ⟵ ⋯
𝐯 ⟵ 𝐯 + (𝐱new − 𝐱)/∆𝑡
𝐱 ⟵ 𝐱new
𝐱𝑖
new += 𝐱𝑖 − 1
2 𝐱𝑖 − 𝐱𝑗 − 𝐿𝑒
𝐱𝑖− 𝐱𝑗
𝐱𝑖− 𝐱𝑗
𝐱j
new += 𝐱𝑗 + 1
2 𝐱𝑖 − 𝐱𝑗 − 𝐿𝑒
𝐱𝑖− 𝐱𝑗
𝐱𝑖− 𝐱𝑗
For every edge 𝑒 = {𝑖, 𝑗}
For 𝑘 = 0 … 𝐾
𝐱𝑖
new ⟵ 𝟎
𝑛𝑖 ⟵ 0
For every vertex 𝑖
𝑛𝑖 ⟵ 𝑛𝑖 + 1
𝑛𝑗 ⟵ 𝑛𝑗 + 1
𝐱𝑖 ⟵ (𝐱𝑖
new + 𝛼𝐱𝑖)/(1 + 𝛼)
For every vertex 𝑖
Projection of Multiple Constraints (by Jacobi)
𝐱𝑖
new ⟵ (𝐱𝑖
new)/(𝑛𝑖)
//Now Constraint Projection starts
```

### Slide 25

![Slide 25](assets/slides/11_Cloth/11_Cloth_p25.png)

```text
Position Based Dynamics (PBD)
25
Position based dynamics (PBD) is based on the projection function.
For every edge 𝑒 = {𝑖, 𝑗}
For 𝑘 = 0 … 𝐾
𝐱𝑖
new ⟵ 𝟎
𝑛𝑖 ⟵ 0
For every vertex 𝑖
𝑛𝑖 ⟵ 𝑛𝑖 + 1
𝑛𝑗 ⟵ 𝑛𝑗 + 1
𝐱𝑖 ⟵ (𝐱𝑖
new + 𝛼𝐱𝑖)/(1 + 𝛼)
For every vertex 𝑖
Projection of Multiple Constraints (by Jacobi)
𝐱𝑖
new ⟵ (𝐱𝑖
new)/(𝑛𝑖)
• Projects all constraints, then
linearly blend the results.
• More iterations it uses, the
better the constraints are
enforced.
𝐱𝑖
new += 𝐱𝑖 − 1
2 𝐱𝑖 − 𝐱𝑗 − 𝐿𝑒
𝐱𝑖− 𝐱𝑗
𝐱𝑖− 𝐱𝑗
𝐱j
new += 𝐱𝑗 + 1
2 𝐱𝑖 − 𝐱𝑗 − 𝐿𝑒
𝐱𝑖− 𝐱𝑗
𝐱𝑖− 𝐱𝑗
```

### Slide 26

![Slide 26](assets/slides/11_Cloth/11_Cloth_p26.png)

```text
Position Based Dynamics (PBD)
26
Position based dynamics (PBD) is based on the projection function.
A PBD Simulator
// Do Kinematics w.o. Constraints
// update 𝐱 and 𝐯
𝐱new ⟵ Projection(𝐱)
𝐯 ⟵ ⋯
𝐱 ⟵ ⋯
𝐯 ⟵ 𝐯 + (𝐱new − 𝐱)/∆𝑡
𝐱 ⟵ 𝐱new
For every edge 𝑒 = {𝑖, 𝑗}
For 𝑘 = 0 … 𝐾
𝐱𝑖
new ⟵ 𝟎
𝑛𝑖 ⟵ 0
For every vertex 𝑖
𝑛𝑖 ⟵ 𝑛𝑖 + 1
𝑛𝑗 ⟵ 𝑛𝑗 + 1
𝐱𝑖 ⟵ (𝐱𝑖
new + 𝛼𝐱𝑖)/(1 + 𝛼)
For every vertex 𝑖
Projection of Multiple Constraints (by Jacobi)
𝐱𝑖
new ⟵ (𝐱𝑖
new)/(𝑛𝑖)
//Now Constraint Projection starts𝐱𝑖
new += 𝐱𝑖 − 1
2 𝐱𝑖 − 𝐱𝑗 − 𝐿𝑒
𝐱𝑖− 𝐱𝑗
𝐱𝑖− 𝐱𝑗
𝐱j
new += 𝐱𝑗 + 1
2 𝐱𝑖 − 𝐱𝑗 − 𝐿𝑒
𝐱𝑖− 𝐱𝑗
𝐱𝑖− 𝐱𝑗
```

### Slide 27

![Slide 27](assets/slides/11_Cloth/11_Cloth_p27.png)

```text
Position Based Dynamics (PBD)
27
A PBD Simulator
// Do Kinematics w.o. Constraints
// update 𝐱 and 𝐯
𝐱new ⟵ Projection(𝐱)
𝐯 ⟵ ⋯
𝐱 ⟵ ⋯
//Now Constraint Projection starts
𝐯 ⟵ 𝐯 + (𝐱new − 𝐱)/∆𝑡
𝐱 ⟵ 𝐱new
•
• The stiffness (how tightly constraints are
enforced) is subject to non-physical factors:
• The number of iterations
• The mesh resolution
• The velocity update following projection is
important to dynamic effects.
• Applicable to other constraints:
triangle constraints, volume constraints, collision
constraints.
✓Implement these constraints ∶= simply define their
projection functions.
```

### Slide 28

![Slide 28](assets/slides/11_Cloth/11_Cloth_p28.png)

```text
28
• Pros
• Parallelable on GPUs (PhysX)
• Easy to implement
• Fast in low resolutions
• Generic, can handle other coupling and
constraints, including fluids
• Cons
• Not physically correct
• Low performance in high resolutions
• Hierarchical approaches (can cause
oscillation and other issues…)
• Acceleration approaches, like Chebyshev
Position Based Dynamics (PBD)
```

### Slide 29

![Slide 29](assets/slides/11_Cloth/11_Cloth_p29.png)

```text
29
Resources
• https://matthias-
research.github.io/pages/tenMinutePhysics/index.html
Macklin et al. SIGGRAPH 2013
Position based fluids
```

### Slide 30

![Slide 30](assets/slides/11_Cloth/11_Cloth_p30.png)

```text
XPBD：Extended Position Based Dynamics
30
diffxpbd
```

### Slide 31

![Slide 31](assets/slides/11_Cloth/11_Cloth_p31.png)

```text
Strain Limiting
32
```

### Slide 32

![Slide 32](assets/slides/11_Cloth/11_Cloth_p32.png)

```text
Strain Limiting
33
Strain limiting aims at using the projection function for correction only.
A Simulator with Strain Limiting
//Do Simulation, with soft/relaxed constraints
𝐱new ⟵ Projection(𝐱)
𝐯 ⟵ ⋯
𝐱 ⟵ ⋯
//Now strain limiting starts.
𝐯 ⟵ 𝐯 + (𝐱new − 𝐱)/∆𝑡
𝐱 ⟵ 𝐱new
Still do some
dynamics
//update 𝐱 and 𝐯
//Use projection to correct constraints
```

### Slide 33

![Slide 33](assets/slides/11_Cloth/11_Cloth_p33.png)

```text
Spring Strain Limit
34
𝐱𝑖
rest length 𝐿
𝐱𝑗
Constraint 𝐱𝑖
𝐱𝑗
𝐱𝑖
new
𝐱𝑗
new
𝐱𝑖
𝐱𝑗
𝐱𝑖
new
𝐱𝑗
new
Stretched case Compressed Case
𝐿
𝜎min ≤ 1
𝐿 𝐱𝑖 − 𝐱𝑗 ≤ 𝜎max
We can set the spring strain, i.e., the stretching ratio 𝜎, to be within a limit.
```

### Slide 34

![Slide 34](assets/slides/11_Cloth/11_Cloth_p34.png)

```text
Spring Strain Limit
35
We can set the spring strain, i.e., the stretching ratio 𝜎, to be within a limit.
𝐱𝑖
rest length 𝐿
𝐱𝑗
𝐱𝑖
new ⟵ 𝐱𝑖 − 𝑚𝑗
𝑚𝑖+𝑚𝑗
𝐱𝑖 − 𝐱𝑗 − 𝜎0𝐿 𝐱𝑖− 𝐱𝑗
𝐱𝑖− 𝐱𝑗
𝐱𝑗
new ⟵ 𝐱𝑗 + 𝑚𝑗
𝑚𝑖+𝑚𝑗
𝐱𝑖 − 𝐱𝑗 − 𝜎0𝐿 𝐱𝑖− 𝐱𝑗
𝐱𝑖− 𝐱𝑗
𝐱new ⟵ Projection(𝐱)
𝜎0 ⟵ min max 𝜎, 𝜎min , 𝜎max
𝜎 ⟵ 1
𝐿 𝐱𝑖 − 𝐱𝑗
PBD: 𝜎0 ≡ 1; No limit: 𝜎min, 𝜎max ⟵ ∞
Constraint
𝜎min ≤ 1
𝐿 𝐱𝑖 − 𝐱𝑗 ≤ 𝜎max
```

### Slide 35

![Slide 35](assets/slides/11_Cloth/11_Cloth_p35.png)

```text
Triangle Area Limit
36
We can limit the triangle area as well.  To do so, we define a scaling factor.
𝐱𝑖
Area 𝐴
𝐱𝑗
Constraint
𝐴min ≤ 𝐴 ≤ 𝐴max
𝐱𝑘
𝐱𝑖
new
Area 𝑠2𝐴
𝐱𝑗
new
𝐱𝑘
new
𝐱𝑖
new, 𝐱𝑖
new, 𝐱𝑘
new = argmin1
2 𝑚𝑖 𝐱𝑖
new − 𝐱𝑖 2 + 𝑚𝑗 𝐱𝑗
new − 𝐱𝑗
2
+ 𝑚𝑗 𝐱𝑘
new − 𝐱𝑘 2
s.t. constraint
Scaling by 𝑠
𝑠 = min max 𝐴, 𝐴min , 𝐴max /𝐴
```

### Slide 36

![Slide 36](assets/slides/11_Cloth/11_Cloth_p36.png)

```text
𝐱𝑖
new ⟵ 𝐜 + 𝑠 𝐱𝑖 − 𝐜
𝐱new ⟵ Projection(𝐱)
𝐴 ⟵ 1
2 𝐱𝑗 − 𝐱𝑖 × 𝐱𝑘 − 𝐱𝑖
Triangle Area Limit
37
To limit the area, we use the fact that the mass center doesn’t move.
𝐱𝑖
Area 𝐴
𝐱𝑗
Constraint
𝐴min ≤ 𝐴 ≤ 𝐴max
𝐱𝑘
𝐱𝑖
new, 𝐱𝑖
new, 𝐱𝑘
new = argmin1
2 𝑚𝑖 𝐱𝑖
new − 𝐱𝑖 2 + 𝑚𝑗 𝐱𝑗
new − 𝐱𝑖
2
+ 𝑚𝑗 𝐱𝑗
new − 𝐱𝑖
2
such that the constraint is satisfied.
𝑠 ⟵ min max 𝐴, 𝐴min , 𝐴max /𝐴
𝐜 ⟵ 1
𝑚𝑖+𝑚𝑗+𝑚𝑘
𝑚𝑖𝐱𝑖 + 𝑚𝑗𝐱𝑗 + 𝑚𝑘𝐱𝑘
𝐱𝑗
new ⟵ 𝐜 + 𝑠 𝐱𝑗 − 𝐜
𝐱𝑘
new ⟵ 𝐜 + 𝑠 𝐱𝑘 − 𝐜
```

### Slide 37

![Slide 37](assets/slides/11_Cloth/11_Cloth_p37.png)

```text
Strain Limiting in Simulation
38
• Widely used in physics-based simulation,
• to avoid instability
• to avoid artifacts (due to large deformation)
• for nonlinear effects
• to address the locking issue
Strain (deformation)
Force
Phase 1:
Elastic
Model
Phase 2:
Strain
Limiting
```

### Slide 38

![Slide 38](assets/slides/11_Cloth/11_Cloth_p38.png)

```text
Other methods
39
• Solving Strain Limiting Constraints with Lagrange Multiplier.
Goldenthal et al. Efficient Simulation of Inextensible Cloth. SIGGRAPH 2007
Tournier et al. Stable Constrained Dynamics. SIGGRAPH 2015
Jin et al. Inequality Cloth, SCA2017
```
