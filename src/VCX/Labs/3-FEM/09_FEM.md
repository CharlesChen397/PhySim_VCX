# 09 FEM

Physics Simulation for Computer Graphics

- Source PDF: `/Users/charleschen/Library/Containers/com.apple.Safari/Data/tmp/WebKitPDFs-WGKAxE/09_FEM.pdf`
- Slides: 58
- 说明：下面先整理与仿真实现直接相关的公式、推导和伪代码；后半部分按页保留课件转写文本，并嵌入每页原始截图，便于对照 PDF 中的图示、布局和符号。
## 仿真实现重点

### 连续介质守恒律

质量守恒：

$$
\frac{d}{dt}\int_{\Omega}\rho\,dV = -\int_{\partial\Omega}\rho \mathbf{v}\cdot \mathbf{n}\,dS
$$

由散度定理得到连续性方程：

$$
\int_\Omega \left(\frac{\partial \rho}{\partial t}+\nabla\cdot(\rho\mathbf{v})\right)dV = 0,
\qquad
\frac{\partial \rho}{\partial t}=-\nabla\cdot(\rho\mathbf{v})
$$

不可压缩情形：

$$
\nabla\cdot \mathbf{v}=0,\qquad \frac{D\rho}{Dt}=0
$$

线动量平衡：

$$
\int_{\Omega}\rho\frac{d\mathbf{v}}{dt}\,dV
=\int_{\partial\Omega}\mathbf{f}_{surface}\,dS+\int_{\Omega}\mathbf{f}_{body}\,dV
$$

牵引力和 Cauchy stress：

$$
\mathbf{t}=\boldsymbol{\sigma}\mathbf{n}
$$

由散度定理：

$$
\int_{\partial\Omega}\boldsymbol{\sigma}\mathbf{n}\,dS
=\int_\Omega \nabla\cdot\boldsymbol{\sigma}\,dV
$$

Cauchy 运动方程：

$$
\rho\frac{d\mathbf{v}}{dt}=\nabla\cdot\boldsymbol{\sigma}+\mathbf{f}_{body}
$$

角动量守恒要求 Cauchy stress 对称：

$$
\boldsymbol{\sigma}=\boldsymbol{\sigma}^{T}
$$

Newtonian fluid 的 constitutive law 示例：

$$
\boldsymbol{\sigma}=-p\mathbf{I}+\mu(\nabla\mathbf{v}+\nabla\mathbf{v}^{T})
$$

### 弹性动力学和时间积分

弹性模拟关注：shape change、deformation measure、strain、elastic energy、forces、dynamics。

显式或 symplectic Euler 的课件写法：

$$
\mathbf{x}(t_{n+1})=\mathbf{x}(t_n)+\left(\mathbf{v}(t_n)+\frac{1}{m}\mathbf{f}(t_n)\Delta t\right)\Delta t
$$

隐式积分可写成能量最小化：

$$
\mathbf{x}^{n+1}=\arg\min_{\mathbf{x}} g(\mathbf{x}),
\qquad
 g(\mathbf{x})=\frac{1}{2h^2}\|\mathbf{x}-\mathbf{y}\|_{M}^{2}+E(\mathbf{x})
$$

Newton 迭代：

$$
\mathbf{x}_{k+1}=\mathbf{x}_{k}-\left(\nabla^2g\right)^{-1}\nabla g
$$

### Spring-mass 到 FEM 的区别

Spring-mass 对一条边：

$$
F=\frac{l}{l_0},\qquad
G=\frac{l}{l_0}-1,
\qquad
E=\frac12 kG^2
$$

力为能量负梯度：

$$
\mathbf{f}_i=-\left(\frac{\partial E}{\partial\mathbf{x}_i}\right)^T
=-kG\frac{\mathbf{x}_{ij}}{\|\mathbf{x}_{ij}\|}
$$

Spring-mass 的问题：需要好的弹簧布局、弹簧参数，不同弹簧会耦合 stretch/bending，且很难直接映射到真实材料参数。

FEM 用 continuum mechanics：deformation gradient $\mathbf{F}$、strain $\mathbf{G}$、energy density $W$ 和 stress 表示材料，网格结构对材料行为影响更小。

### 形变场与 Deformation Gradient

形变函数：

$$
\boldsymbol{\varphi}: \mathbf{X}\mapsto \mathbf{x}
$$

局部线性近似：

$$
\mathbf{x}\approx\mathbf{F}\mathbf{X}+\mathbf{b}
$$

Taylor 展开：

$$
\mathbf{x}=\boldsymbol{\varphi}(\mathbf{X}_k)+\frac{\partial\mathbf{x}}{\partial\mathbf{X}}(\mathbf{X}-\mathbf{X}_k)+O(\|\mathbf{X}-\mathbf{X}_k\|^2)
$$

Deformation gradient：

$$
\mathbf{F}=\frac{\partial\mathbf{x}}{\partial\mathbf{X}}
$$

2D 三角形单元中，令

$$
\mathbf{E}=[\mathbf{X}_{10}\ \mathbf{X}_{20}],\qquad
\mathbf{e}=[\mathbf{x}_{10}\ \mathbf{x}_{20}]
$$

则

$$
\mathbf{F}=\mathbf{e}\mathbf{E}^{-1}
=[\mathbf{x}_{10}\ \mathbf{x}_{20}][\mathbf{X}_{10}\ \mathbf{X}_{20}]^{-1}
$$

3D 四面体单元中：

$$
\mathbf{E}=[\mathbf{X}_{10}\ \mathbf{X}_{20}\ \mathbf{X}_{30}],
\qquad
\mathbf{F}=[\mathbf{x}_{10}\ \mathbf{x}_{20}\ \mathbf{x}_{30}]\mathbf{E}^{-1}
$$

### Green Strain

Deformation gradient 包含旋转。用 polar decomposition：

$$
\mathbf{F}=\mathbf{R}\mathbf{S}
$$

Green strain：

$$
\mathbf{G}=\frac12(\mathbf{F}^T\mathbf{F}-\mathbf{I})
$$

若 $\mathbf{F}=\mathbf{U}\mathbf{D}\mathbf{V}^T$，则

$$
\mathbf{G}=\frac12(\mathbf{V}\mathbf{D}^2\mathbf{V}^T-\mathbf{I})
$$

Green strain 和长度变化的关系：

$$
\frac{l^2-l_0^2}{l_0^2}=2\mathbf{n}^T\mathbf{G}\mathbf{n}
$$

### StVK 能量、Second PK stress 和力

三角形上的总能量：

$$
E=\int W(\mathbf{G})\,dA=A^{ref}W(\epsilon_{uu},\epsilon_{vv},\epsilon_{uv})
$$

St. Venant-Kirchhoff model：

$$
W(\epsilon_{uu},\epsilon_{vv},\epsilon_{uv})
=\frac{\lambda}{2}(\epsilon_{uu}+\epsilon_{vv})^2
+\mu(\epsilon_{uu}^2+\epsilon_{vv}^2+2\epsilon_{uv}^2)
$$

Second Piola-Kirchhoff stress：

$$
\mathbf{S}=\frac{\partial W}{\partial\mathbf{G}}
=2\mu\mathbf{G}+\lambda\operatorname{trace}(\mathbf{G})\mathbf{I}
$$

First Piola-Kirchhoff stress：

$$
\mathbf{P}=\frac{\partial W}{\partial\mathbf{F}}=\mathbf{F}\mathbf{S}
$$

单元力是能量负梯度：

$$
\mathbf{f}_i=-\left(\frac{\partial E}{\partial\mathbf{x}_i}\right)^T
$$

2D 三角形：

$$
[\mathbf{f}_1\ \mathbf{f}_2]
=-A^{ref}\mathbf{F}\mathbf{S}[\mathbf{X}_{10}\ \mathbf{X}_{20}]^{-T},
\qquad
\mathbf{f}_0=-\mathbf{f}_1-\mathbf{f}_2
$$

3D 四面体：

$$
[\mathbf{f}_1\ \mathbf{f}_2\ \mathbf{f}_3]
=-V^{ref}\mathbf{F}\mathbf{S}[\mathbf{X}_{10}\ \mathbf{X}_{20}\ \mathbf{X}_{30}]^{-T},
\qquad
\mathbf{f}_0=-\mathbf{f}_1-\mathbf{f}_2-\mathbf{f}_3
$$

### Stress 的几种配置

Cauchy stress：current-state normal 到 current-state traction：

$$
\mathbf{t}=\boldsymbol{\sigma}\mathbf{n}
$$

Second Piola-Kirchhoff stress：reference-state normal 到 reference-state traction：

$$
\mathbf{T}=\mathbf{S}\mathbf{N}
$$

First Piola-Kirchhoff stress：reference-state normal 到 current-state traction，且

$$
\mathbf{P}=\mathbf{F}\mathbf{S}
$$

Nanson formula：

$$
A\mathbf{n}=\det(\mathbf{F})\mathbf{F}^{-T}(A_{ref}\mathbf{N})
$$

Cauchy stress 和 First PK stress 的关系：

$$
\boldsymbol{\sigma}=\det(\mathbf{F})^{-1}\mathbf{P}\mathbf{F}^{T}
$$

### Linear FEM Framework 伪代码

对每个四面体单元：

```cpp
// Rest positions X0, X1, X2, X3; current positions x0, x1, x2, x3.
Mat3 E = Mat3(X1 - X0, X2 - X0, X3 - X0);
Mat3 e = Mat3(x1 - x0, x2 - x0, x3 - x0);
Mat3 invE = inverse(E);
Real Vref = abs(determinant(E)) / 6.0;

Mat3 F = e * invE;
Mat3 G = 0.5 * (transpose(F) * F - Mat3::Identity());
Mat3 S = 2.0 * mu * G + lambda * trace(G) * Mat3::Identity();
Mat3 P = F * S;

Mat3 H = -Vref * P * transpose(invE);
Vec3 f1 = H.col(0);
Vec3 f2 = H.col(1);
Vec3 f3 = H.col(2);
Vec3 f0 = -f1 - f2 - f3;

force[i0] += f0;
force[i1] += f1;
force[i2] += f2;
force[i3] += f3;
```

课件 quick summary 写法：

$$
\mathbf{E}=[\mathbf{X}_{10}\ \mathbf{X}_{20}\ \mathbf{X}_{30}]
$$

$$
\mathbf{F}=[\mathbf{x}_{10}\ \mathbf{x}_{20}\ \mathbf{x}_{30}]\mathbf{E}^{-1}
$$

$$
\mathbf{G}=\frac12(\mathbf{F}^T\mathbf{F}-\mathbf{I})
$$

$$
\mathbf{P}=\mathbf{F}\mathbf{S},
\qquad
\mathbf{S}_{StVK}=2\mu\mathbf{G}+\lambda\operatorname{trace}(\mathbf{G})\mathbf{I}
$$

$$
[\mathbf{f}_1\ \mathbf{f}_2\ \mathbf{f}_3]
=-V^{ref}\mathbf{P}\mathbf{E}^{-T}
=-\frac{1}{6\det(\mathbf{E}^{-1})}\mathbf{P}\mathbf{E}^{-T}
$$

$$
\mathbf{f}_0=-\mathbf{f}_1-\mathbf{f}_2-\mathbf{f}_3
$$

### 材料参数和超弹性模型

Lame 参数与 Young's modulus $Y$、Poisson ratio $P$：

$$
\mu=\frac{Y}{2(1+P)},
\qquad
\lambda=\frac{YP}{(1+P)(1-2P)}
$$

反推：

$$
Y=\frac{\mu(3\lambda+2\mu)}{\lambda+\mu},
\qquad
P=\frac{\lambda}{2(\lambda+\mu)}
$$

StVK：

$$
\mathbf{G}=\frac12(\mathbf{F}^T\mathbf{F}-\mathbf{I})
$$

$$
W=\frac{\lambda}{2}\operatorname{tr}^{2}(\mathbf{G})+
\mu\|\mathbf{G}\|_{F}^{2}
$$

$$
\mathbf{P}=\mathbf{F}\left[2\mu\mathbf{G}+\lambda\operatorname{tr}(\mathbf{G})\mathbf{I}\right]
$$

Neo-Hookean：

$$
J=\det(\mathbf{F}),
\qquad
I_C=\|\mathbf{F}\|_{F}^{2}
$$

$$
W=\frac{\lambda}{2}\log^2(J)+\frac{\mu}{2}(I_C-3)-\mu\log(J)
$$

$$
\mathbf{P}=\mu(\mathbf{F}-\mathbf{F}^{-T})+
\lambda\log(J)\mathbf{F}^{-T}
$$

Cauchy-Green invariants：

$$
\mathbf{C}=\mathbf{F}^{T}\mathbf{F}
$$

$$
I_C=\operatorname{tr}(\mathbf{C})=\|\mathbf{F}\|_{F}^{2}
$$

$$
II_C=\operatorname{tr}(\mathbf{C}^{2}),
\qquad
III_C=\det(\mathbf{C})=\det(\mathbf{F})^2=J^2
$$

旋转不变性：

$$
W(\mathbf{R}\mathbf{F})=W(\mathbf{F})
$$

Isotropic material 还满足：

$$
W(\mathbf{F}\mathbf{Q})=W(\mathbf{F}),
\qquad
W(\mathbf{R}\mathbf{F}\mathbf{Q})=W(\mathbf{F})
$$

若 $\mathbf{F}=\mathbf{U}\boldsymbol{\Lambda}\mathbf{V}^{T}$，则各向同性模型可写为主伸长的函数：

$$
W(\mathbf{F})=W(\lambda_0,\lambda_1,\lambda_2)
$$

主伸长形式的 First PK stress：

$$
\mathbf{P}(\mathbf{F})=
\mathbf{U}\operatorname{diag}\left(
\frac{\partial W}{\partial \lambda_0},
\frac{\partial W}{\partial \lambda_1},
\frac{\partial W}{\partial \lambda_2}
\right)\mathbf{V}^{T}
$$

FEM/FVM for isotropic material 的 quick summary：

```cpp
Mat3 E = Mat3(X1 - X0, X2 - X0, X3 - X0);
Mat3 F = Mat3(x1 - x0, x2 - x0, x3 - x0) * inverse(E);
SVD(F, U, Lambda, V);
Mat3 P = U * diag(dW_dlambda0, dW_dlambda1, dW_dlambda2) * transpose(V);
Mat3 forces123 = -(1.0 / (6.0 * determinant(inverse(E)))) * P * transpose(inverse(E));
f0 = -f1 - f2 - f3;
```

### 课件提到的阅读和实现参考

- Sifakis and Barbic 2012, FEM simulation of 3D deformable solids: theory, discretization and model reduction.
- Teran et al. 2003, Finite Volume Methods for the Simulation of Skeleton Muscles.
- Xu et al. 2015, Nonlinear Material Design Using Principal Stretches.
- Huamin Wang and Yin Yang 2016, Descent Methods for Elastic Body Simulation on the GPU.
- SIGGRAPH 2022 Course, Dynamic Deformables: Implementation and Production Practicalities.

## 逐页转写

### Slide 01

![Slide 01](assets/slides/09_FEM/09_FEM_p01.png)

```text
Physics Simulation for
Computer Graphics
09 FEM
```

### Slide 02

![Slide 02](assets/slides/09_FEM/09_FEM_p02.png)

```text
Conservation Laws for
Continua
Mass Conservation
Linear momentum balance
Angular momentum balance
2
```

### Slide 03

![Slide 03](assets/slides/09_FEM/09_FEM_p03.png)

```text
Mass Conservation
• Mass conservation should be maintained：
3
න
Ω
𝜌d𝑉d
d𝑡 = −ׯ𝜕Ω 𝜌𝐯 · 𝐧d𝑆
= −׬Ω ∇ ⋅ (𝜌𝐯) dV
න
Ω
𝜕𝜌
𝜕𝑡 + ∇ ⋅ 𝜌𝐯 d𝑉 = 0
Continuity Equation
𝜕𝜌
𝜕𝑡 = −∇ ⋅ (𝜌𝐯)
∇ · 𝐟 𝐱 = 𝑑
𝑑𝐱 · 𝐟 = 𝜕𝑓
𝜕𝑥 + 𝜕𝑔
𝜕𝑦 + 𝜕ℎ
𝜕𝑧
𝐟 𝐱 = 𝑓, 𝑔, ℎ ∈ 𝐑3,
Divergence ∈ 𝐑
𝐷𝜌
𝐷𝑡 = 0
𝜕𝜌
𝜕𝑡 = −∇ ⋅ (𝜌𝐯) &                  ∇ ⋅ 𝐯 = 0
For
incompressible
Fluids:
```

### Slide 04

![Slide 04](assets/slides/09_FEM/09_FEM_p04.png)

```text
Linear momentum balance
4
(a volume element): ׬Ω ρ
𝑑𝐯
𝑑𝑡 d𝑉 =ׯ𝜕Ω 𝐟surface d𝑆 +׬Ω 𝐟bodyd𝑉 , For e.g., 𝐟body = 𝜌𝐠
𝐟surface
Traction 𝐭, (𝑁/𝑚2, internal
force per unit area/length )
𝐭 = 𝛔𝐧,
𝛔: Cauchy Stress Tensor,
𝐧y𝑧 = 𝒆𝑥 & 𝐟y𝑧 = 𝛔𝑥
𝐧𝑧𝑥 = 𝒆𝑦 & 𝐟𝑧𝑥 = 𝛔𝑦
𝐧𝑥y = 𝒆𝑧 & 𝐟xy = 𝛔𝑧
𝐧 = 𝑛𝑥𝒆𝑥 + 𝑛𝑦𝒆𝑦 + 𝑛𝑧𝒆𝑧;
= 𝑛𝑥𝛔𝑥 + 𝑛𝑥𝛔𝑦 + 𝑛𝑥𝛔𝑧
= 𝛔𝑥 𝛔𝑦 𝛔𝑧 𝐧 = 𝛔𝐧
𝐭
ර
𝜕Ω
𝛔𝐧 d𝑆 = න
Ω
∇ · 𝝈 d𝑉
Cauchy’s equation of motion ρ
𝑑𝐯
𝑑𝑡 = ∇ · 𝝈 + 𝐟𝑏𝑜𝑑𝑦
The equation of motion of an infinitesimal volume of any continuum.
```

### Slide 05

![Slide 05](assets/slides/09_FEM/09_FEM_p05.png)

```text
Angular momentum balance
• The Traction Force do not exert a torque on the volume element
6
−𝛔𝑥
𝛔𝑦
𝛔y
𝛔𝑥 𝛔01 𝑑ℎ = 𝛔10 𝑑ℎ
𝛔 = 𝛔T
𝛔00
𝛔10
𝛔11
𝛔01
−𝛔00
−𝛔10
−𝛔01
−𝛔11
```

### Slide 06

![Slide 06](assets/slides/09_FEM/09_FEM_p06.png)

```text
Conservation Laws for Continua
7
"𝛔 = f 𝐱, v, … " and "∇ · 𝛔" leads to different behaviors for different materials
```

### Slide 07

![Slide 07](assets/slides/09_FEM/09_FEM_p07.png)

```text
ρ
𝑑𝐮
𝑑𝑡 = ∇ · 𝝈 + 𝐟𝑏𝑜𝑑𝑦
• For a Newtonian Fluid, the stress tensor can be expressed as
𝝈 = −𝑝𝐈 + 𝜇(∇𝐯 + ∇𝐯 T)
8
```

### Slide 08

![Slide 08](assets/slides/09_FEM/09_FEM_p08.png)

```text
Learning Neural Constitutive Laws
9
https://sites.google.com/view/nclaw
ICML 2023
```

### Slide 09

![Slide 09](assets/slides/09_FEM/09_FEM_p09.png)

```text
Elasticity
Spring-Mass System
Finite Elements
10
```

### Slide 10

![Slide 10](assets/slides/09_FEM/09_FEM_p10.png)

```text
Elasticity
• Shape Changes:
• Deformation Measure
• Strain
• Elastic Energy
• Forces: ρ 𝑑𝐮
𝑑𝑡 = ∇ · 𝝈 + 𝐟𝑏𝑜𝑑𝑦
• Dynamics:
• Explicit Time Integration (Symplectic Euler):
𝐱 𝑡𝑛+1 = 𝐱 𝑡𝑛 + 𝐯 𝑡𝑛 +
1
𝑚 𝐟 𝑡𝑛 Δt Δ𝑡
• Implicit Time Integration:
𝐱𝑛+1 = 𝑎𝑟𝑔𝑚𝑖𝑛𝐱 𝑔 𝐱 , for 𝑔 𝐱 =
1
2ℎ2 𝐱 − 𝐲 M
2 + 𝐸 𝐱
Iterative solver: 𝐱𝑘+1 = 𝐱𝑘 − ∇2𝑔 −1∇𝑔 11
```

### Slide 11

![Slide 11](assets/slides/09_FEM/09_FEM_p11.png)

```text
Elasticity using Spring-Mass System
• Shape Changes:
• Deformation Measure
• Strain
• Elastic Energy
• Forces: ρ 𝑑𝐮
𝑑𝑡 = ∇ · 𝝈 + 𝐟𝑏𝑜𝑑𝑦
• Dynamics:
• Explicit Time Integration (Symplectic Euler):
𝐱 𝑡𝑛+1 = 𝐱 𝑡𝑛 + 𝐯 𝑡𝑛 +
1
𝑚 𝐟 𝑡𝑛 Δt Δ𝑡
• Implicit Time Integration:
𝐱𝑛+1 = 𝑎𝑟𝑔𝑚𝑖𝑛𝐱 𝑔 𝐱 , for 𝑔 𝐱 =
1
2ℎ2 𝐱 − 𝐲 M
2 + 𝐸 𝐱
Iterative solver: 𝐱𝑘+1 = 𝐱𝑘 − H 𝑔 −1∇𝑔 12
F = 𝑙
𝑙0
G = 𝑙
𝑙0
− 1
𝐸 = 1
2 𝑘G2
𝐟𝑖 = −( 𝜕𝐸
𝜕𝐱𝑖
)T= −𝑘G
𝐱𝑖𝒋
|𝐱𝑖𝒋|
Xj Xi
l0
xj xi
l
l = | xij |;  l0 = | Xij|
• Need good spring layout!
• Need good spring params!
• Spring can interfere (stretching & bending)!
• No direct map to material properties!
```

### Slide 12

![Slide 12](assets/slides/09_FEM/09_FEM_p12.png)

```text
Elasticity using FEM (Continuum Mechanics)
• Shape Changes:
• Deformation Measure
• Strain
• Elastic Energy
• Forces: ρ 𝑑𝐮
𝑑𝑡 = ∇ · 𝝈 + 𝐟𝑏𝑜𝑑𝑦
• Dynamics:
• Explicit Time Integration (Symplectic Euler):
𝐱 𝑡𝑛+1 = 𝐱 𝑡𝑛 + 𝐯 𝑡𝑛 +
1
𝑚 𝐟 𝑡𝑛 Δt Δ𝑡
• Implicit Time Integration:
𝐱𝑛+1 = 𝑎𝑟𝑔𝑚𝑖𝑛𝐱 𝑔 𝐱 , for 𝑔 𝐱 =
1
2ℎ2 𝐱 − 𝐲 M
2 + 𝐸 𝐱
Iterative solver: 𝐱𝑘+1 = 𝐱𝑘 − H 𝑔 −1∇𝑔 13
DeformationGradient, 𝐅
𝑎 𝑡𝑒𝑛𝑠𝑜𝑟
Strain, 𝐆
𝑎 𝑡𝑒𝑛𝑠𝑜𝑟
EnergyDensity, W& Stress, 𝛔
𝑎 𝑠𝑐𝑎𝑙𝑎𝑟& 𝑎 𝑡𝑒𝑛𝑠𝑜𝑟
𝐟𝑖 = −( 𝜕𝐸
𝜕𝐱𝑖
)T= ර
𝐿
𝛔𝐧𝑑𝑙
𝐱0
𝐱𝟏
midpoint
• In consistent with continuum mechanics
• Finite Element Discretization
• Advantages:
➢ Accurate and controllable material behavior
➢ Largely independent of mesh structure
```

### Slide 13

![Slide 13](assets/slides/09_FEM/09_FEM_p13.png)

```text
Finite Elements
14
```

### Slide 14

![Slide 14](assets/slides/09_FEM/09_FEM_p14.png)

```text
Deformation Measure
DeformationGradient, 𝐅
Strain, 𝐆
15
```

### Slide 15

![Slide 15](assets/slides/09_FEM/09_FEM_p15.png)

```text
Deformation Field
16
• Deformation function, 𝛗: 𝐗 → 𝐱
• E.g.:  𝐱 = 𝛗 𝐗 = 𝑋 + 𝑠𝑖𝑛 𝑌
𝑌 + 𝑠𝑖𝑛 𝑋
• Locally Linear
• 𝐱 ≈ F𝐗 + 𝐛
• Taylor expansion:
𝐱 = 𝛗(𝐗𝐤) +
𝜕𝐱
𝜕𝐗 𝐗 − 𝐗𝐤 + 𝑶(𝟐)
𝐱 ≈
𝜕𝑥
𝜕𝑋
𝜕𝑦
𝜕𝑋
X − Xk ,
𝜕𝑥
𝜕𝑌
𝜕𝑦
𝜕𝑌
Y − Yk + 𝛗 𝐗𝐤
• F =
𝜕𝐱
𝜕𝐗 =
𝜕𝑥
𝜕𝑋
𝜕𝑥
𝜕𝑌
𝜕𝑦
𝜕𝑋
𝜕𝑦
𝜕𝑌
，Deformation Gradient
```

### Slide 16

![Slide 16](assets/slides/09_FEM/09_FEM_p16.png)

```text
• Deformation function, 𝛗: 𝐗 → 𝐱
• E.g.:  𝐱 = 𝛗 𝐗 = 𝑋 + 𝑠𝑖𝑛 𝑌
𝑌 + 𝑠𝑖𝑛 𝑋
• Locally Linear
• 𝐱 ≈ F𝐗 + 𝐛
• Taylor expansion:
𝐱 = 𝛗(𝐗𝐤) +
𝜕𝐱
𝜕𝐗 𝐗 − 𝐗𝐤 + 𝑶(𝟐)
𝐱 ≈
𝜕𝑥
𝜕𝑋
𝜕𝑦
𝜕𝑋
X − Xk ,
𝜕𝑥
𝜕𝑌
𝜕𝑦
𝜕𝑌
Y − Yk + 𝛗 𝐗𝐤
• F =
𝜕𝐱
𝜕𝐗 =
𝜕𝑥
𝜕𝑋
𝜕𝑥
𝜕𝑌
𝜕𝑦
𝜕𝑋
𝜕𝑦
𝜕𝑌
，Deformation Gradient
Deformation Field
17
```

### Slide 17

![Slide 17](assets/slides/09_FEM/09_FEM_p17.png)

```text
• Deformation function, 𝛗: 𝐗 → 𝐱
• E.g.:  𝐱 = 𝛗 𝐗 = 𝑋 + 𝑠𝑖𝑛 𝑌
𝑌 + 𝑠𝑖𝑛 𝑋
• Locally Linear
• 𝐱 ≈ F𝐗 + 𝐛
• Taylor expansion:
𝐱 = 𝛗(𝐗𝐤) +
𝜕𝐱
𝜕𝐗 𝐗 − 𝐗𝐤 + 𝑶( 𝐗 − 𝐗𝐤 𝟐)
• F =
𝜕𝐱
𝜕𝐗 =
𝜕𝑥
𝜕𝑋
𝜕𝑥
𝜕𝑌
𝜕𝑦
𝜕𝑋
𝜕𝑦
𝜕𝑌
，Deformation Gradient
• 𝐱 ≈
𝜕𝑥
𝜕𝑋
𝜕𝑦
𝜕𝑋
X − Xk ,
𝜕𝑥
𝜕𝑌
𝜕𝑦
𝜕𝑌
Y − Yk + 𝛗 𝐗𝐤
Deformation Field and Deformation Gradient
18
𝐅 = 1 cos(Y)
cos(X) 1

1 cos(1)
cos(−2) 1 = 1 0.5403
−0.416 1
```

### Slide 18

![Slide 18](assets/slides/09_FEM/09_FEM_p18.png)

```text
Deformation Field and Deformation Gradient
19
```

### Slide 19

![Slide 19](assets/slides/09_FEM/09_FEM_p19.png)

```text
Deformation using Linear FEM
20
𝐱0
𝐱𝟏
𝐅1
𝐅2
𝐅3 𝐅4
𝐅5
𝐅6Approx. continuous deformation field 𝛗
with piece-wise linear deformation fields {F}
𝐗0
𝐗1
𝐱 = 𝛗(𝐗𝐤) +
𝜕𝐱
𝜕𝐗 𝐗 − 𝐗𝐤 + 𝑶( 𝐗 − 𝐗𝐤 𝟐) 𝐱 ≈ F𝑖𝐗 + 𝐛
𝐗0
𝐗1
```

### Slide 20

![Slide 20](assets/slides/09_FEM/09_FEM_p20.png)

```text
Deformation Gradient, F=
𝜕𝐱
𝜕𝐗
21
For any vector inside:
𝐅 converts it from reference
to deformed:
𝐱𝑏𝑎 = 𝐱𝑏 − 𝐱𝑎
= 𝐅𝐗𝑏 + 𝐜 − 𝐅𝐗𝑎 − 𝐜
𝐱𝑏𝑎 = 𝐅𝐗𝑏𝑎
𝐱 = 𝐅𝐗 + 𝐜
𝐅𝐗10 = 𝐱10
𝐅𝐗20 = 𝐱20
𝐅 𝐗10 𝐗20 = 𝐱10 𝐱20
𝐅 = 𝐱10 𝐱20 𝐗10 𝐗20 −1
𝐗0 𝐗1
𝐗2
Reference (undeformed)
𝐗𝑏𝑎
𝐗𝑎
𝐗𝑏
𝐱0
𝐱1
𝐱2
Current (deformed)
𝐱𝑏
𝐱𝑎
𝐱𝑏𝑎
𝐅 = 𝒆𝐄−1
Problem: 𝐅 contains
rotation
```

### Slide 21

![Slide 21](assets/slides/09_FEM/09_FEM_p21.png)

```text
Green Strain
22
𝐆 =
1
2 𝐅T𝐅 − 𝐈 = 𝜀𝑢𝑢 𝜀𝑢𝑣
𝜀𝑢𝑣 𝜀𝑣𝑣
Polar Decomposition : 𝐅 = 𝐑𝐒,
𝐅 = 𝐔𝐕T 𝐕𝐃𝐕T , 𝐆 = 1
2 𝐅T𝐅 − 𝐈 = 1
2 𝐕𝐃2𝐕T − 𝐈
Rotation 𝑹Scaling S
𝐅 = 𝐑𝐒
𝐅 = 𝐔𝐕T 𝐕𝐃𝐕T
𝐆 = 1
2 𝐅T𝐅 − 𝐈 = 1
2 𝐒T𝐒 − 𝐈
```

### Slide 22

![Slide 22](assets/slides/09_FEM/09_FEM_p22.png)

```text
𝑙2 − 𝑙0
2
𝑙0
2 = 𝟐𝒏𝐓𝐆 𝒏
Green Strain
23
𝐆 =
1
2 𝐅T𝐅 − 𝐈 = 𝜀𝑢𝑢 𝜀𝑢𝑣
𝜀𝑢𝑣 𝜀𝑣𝑣
𝐗𝑏𝑎
𝐗0 𝐗1
𝐗2
𝐱0
𝐱1
𝐱2
𝐗𝑎
𝐗𝑏 𝐱𝑏
𝐱𝑎
𝐱𝑏𝑎
Reference (undeformed) Current (deformed)
𝐱𝑏𝑎 = 𝐅𝐗𝑏𝑎
𝑙2−𝑙02
𝑙02 =
𝐱𝑏𝑎 𝟐− 𝐗𝑏𝑎 𝟐
𝐗𝑏𝑎 𝟐
=
𝟏
𝐗𝑏𝑎 𝟐 (𝐱𝒃𝒂
𝑇 𝐱𝑏𝑎 − 𝐗𝒃𝒂
𝑇 𝐗𝑏𝑎 )
=
𝟏
𝐗𝑏𝑎 𝟐 (𝐗𝒃𝒂
𝑇 𝐅T𝐅𝐗𝑏𝑎 − 𝐗𝒃𝒂
𝑇 𝐗𝑏𝑎 )
= 𝐗𝒃𝒂
𝑇 (𝐅T𝐅 − 𝐈)𝐗𝑏𝑎 / 𝐗𝑏𝑎 𝟐
𝒏 = 𝑿𝑏𝑎
𝐗𝑏𝑎
,
```

### Slide 23

![Slide 23](assets/slides/09_FEM/09_FEM_p23.png)

```text
Energy and Stress
24
```

### Slide 24

![Slide 24](assets/slides/09_FEM/09_FEM_p24.png)

```text
Strain Energy Density Function
• Energy Density Function: 𝑊 𝐆 = 𝑊 𝜀𝑢𝑢, 𝜀𝑣𝑣, 𝜀𝑢𝑣
• Total Energy: 𝐸 =׬𝑊 𝐆 𝑑𝐴 = 𝐴ref𝑊 𝜀𝑢𝑢, 𝜀𝑣𝑣, 𝜀𝑢𝑣
• E.g., The Saint Venant-Kirchhoff Model (StVK):
𝑊 𝜀𝑢𝑢, 𝜀𝑣𝑣, 𝜀𝑢𝑣 =
𝜆
2 𝜀𝑢𝑢 + 𝜀𝑣𝑣 2 + 𝜇(𝜀𝑢𝑢2 + 𝜀𝑣𝑣2 + 2𝜀𝑢𝑣2 ), 𝜆 and 𝜇 are Lamé(拉梅) parameters
• StVK’s Second Piola-Kirchhoff stress tensor:
25
Constant within the triangle
𝐒 = 𝜕𝑊
𝜕𝐆 =
𝜕𝑊
𝜕𝜀𝑢𝑢
1
2
𝜕𝑊
𝜕𝜀𝑢𝑣
1
2
𝜕𝑊
𝜕𝜀𝑢𝑣
𝜕𝑊
𝜕𝜀𝑣𝑣
= 2𝜇𝜀𝑢𝑢 + 𝜆𝜀𝑢𝑢 + 𝜆𝜀𝑣𝑣 2𝜇𝜀𝑢𝑣
2𝜇𝜀𝑢𝑣 2𝜇𝜀𝑣𝑣 + 𝜆𝜀𝑢𝑢 + 𝜆𝜀𝑣𝑣
= 2𝜇𝐆 + 𝜆 trace𝐆 𝐈
https://en.wikipedia.org/wiki/Lam%C3%A9_parameters Poisson's ratio 𝜈 =
𝜆
2 𝜆+𝜇
```

### Slide 25

![Slide 25](assets/slides/09_FEM/09_FEM_p25.png)

```text
Force, the negative Gradient of the Energy
• 𝐟𝑖 = −
𝜕𝐸
𝜕𝐱𝑖
T
= −𝐴ref 𝜕𝑊
𝜕𝐱𝑖
T
= −𝐴ref 𝜕𝑊
𝜕𝜀𝑢𝑢
𝜕𝜀𝑢𝑢
𝜕𝐱𝑖
+
𝜕𝑊
𝜕𝜀𝑣𝑣
𝜕𝜀𝑣𝑣
𝜕𝐱𝑖
+
𝜕𝑊
𝜕𝜀𝑢𝑣
𝜕𝜀𝑢𝑣
𝜕𝐱𝑖
T
• 𝐆 =
1
2 𝐅T𝐅 − 𝐈 = 𝜀𝑢𝑢 𝜀𝑢𝑣
𝜀𝑢𝑣 𝜀𝑣𝑣
, 𝐅 = 𝒆𝐄−1 = 𝐱10 𝐱20 𝐗10 𝐗20 −1 = 𝐱10 𝐱20 𝑎 𝑏
𝑐 𝑑 = 𝑎𝐱10 + 𝑐𝐱20 𝑏𝐱10 + 𝑑𝐱20
27
𝜕𝜀𝑢𝑢
𝜕𝐱1
= 𝑎 𝑎𝐱10 + 𝑐𝐱20 T 𝜕𝜀𝑣𝑣
𝜕𝐱1
= 𝑏 𝑏𝐱10 + 𝑑𝐱20 T 𝜕𝜀𝑢𝑣
𝜕𝐱1
=
1
2 𝑎 𝑏𝐱10 + 𝑑𝐱20 T +
1
2 𝑏 𝑎𝐱10 + 𝑐𝐱20 T
𝜕𝜀𝑢𝑢
𝜕𝐱2
= 𝑐 𝑎𝐱10 + 𝑐𝐱20 T 𝜕𝜀𝑣𝑣
𝜕𝐱2
= 𝑑 𝑏𝐱10 + 𝑑𝐱20 T 𝜕𝜀𝑢𝑣
𝜕𝐱2
=
1
2 𝑐 𝑏𝐱10 + 𝑑𝐱20 T +
1
2 𝑑 𝑎𝐱10 + 𝑐𝐱20 T
𝐗0 𝐗1
𝐗2
Reference
𝐟1 = −𝐴ref 𝑎𝐱10 + 𝑐𝐱20 𝑏𝐱10 + 𝑑𝐱20
𝜕𝑊
𝜕𝜀𝑢𝑢
1
2
𝜕𝑊
𝜕𝜀𝑢𝑣
1
2
𝜕𝑊
𝜕𝜀𝑢𝑣
𝜕𝑊
𝜕𝜀𝑣𝑣
𝑎
𝑏 = −𝐴ref𝐅𝐒 𝑎
𝑏
𝐟𝑖 = −𝐴ref 𝜕𝑊
𝜕𝐱𝑖
T
= −𝐴ref 𝜕𝑊
𝜕𝐅 : 𝜕𝐅
𝜕𝐱𝑖
T
, 𝜕𝑊
𝜕𝑭 = 𝜕𝑊
𝜕𝑮 : 𝜕𝑮
𝜕𝐱𝑖
= 𝐒: 𝜕𝑮
𝜕𝐱𝑖
= 𝐅𝐒
𝜕𝑊
𝜕𝐱𝑖
= theith row of FS E−𝟏 , for i = 1,2
𝜕𝑊
𝜕𝐱0
= − 𝜕𝑊
𝜕𝐱1
− 𝜕𝑊
𝜕𝐱2
https://www.bilibili.com/video/BV1eY411x7m
K?p=5 太极图形课 p5 线性有限元能量求导
𝐒StVK = 𝜕𝑊
𝜕𝐆 = 2𝜇𝐆 + 𝜆 trace𝐆 𝐈
```

### Slide 26

![Slide 26](assets/slides/09_FEM/09_FEM_p26.png)

```text
Force, the Gradient of the Energy
• 𝐟𝑖 = −
𝜕𝐸
𝜕𝐱𝑖
T
= −𝐴ref 𝜕𝑊
𝜕𝐱𝑖
T
= −𝐴ref 𝜕𝑊
𝜕𝜀𝑢𝑢
𝜕𝜀𝑢𝑢
𝜕𝐱𝑖
+
𝜕𝑊
𝜕𝜀𝑣𝑣
𝜕𝜀𝑣𝑣
𝜕𝐱𝑖
+
𝜕𝑊
𝜕𝜀𝑢𝑣
𝜕𝜀𝑢𝑣
𝜕𝐱𝑖
T
28
𝐗0 𝐗1
𝐗2
Reference
𝐟1 𝐟2 = −𝐴ref𝐅𝐒 𝐗10 𝐗20 −T
𝐟0 = −𝐟𝟏 − 𝐟2
𝐟1 𝐟2 𝐟3 = −𝑉ref𝐅𝐒 𝐗10 𝐗20 𝐗30 −T
𝐟0 = −𝐟𝟏 − 𝐟2 −𝐟3
𝐗0
𝐗1
𝐗2
𝐗3
```

### Slide 27

![Slide 27](assets/slides/09_FEM/09_FEM_p27.png)

```text
𝐭
−𝐭
Stress and Traction
29
• Traction 𝐭, (𝑁/𝑚2, internal force per unit area/length )
• Cauchy Stress tensor 𝛔 (interface normal → traction):
• Total interface force:
𝐟 = ර
𝜕𝐵𝑡
𝐭 𝑑𝑠
𝐭 = 𝛔𝐧
𝐟 = ර
𝜕𝐵𝑡
𝛔𝐧𝑑𝑠
𝛔: Cauchy Stress tensor (in deformed shape!)
𝐵𝑡
𝐭
𝐧
```

### Slide 28

![Slide 28](assets/slides/09_FEM/09_FEM_p28.png)

```text
Stress in rest and deformed shape
30
Stress tensor: mapping from the interface normal to the traction, it can be defined
by different configurations.
𝐗0
The energy density W is defined in the reference
state. An area element has E = ΔAref W.
Therefore,  𝐒 is a mapping from the normal 𝐍 to
the traction 𝐓, both in the reference state.
𝐱0
𝛔 converts the normal into 𝐭 for force
calculation. Therefore, this stress assumes
the normal 𝐧 and the traction 𝐭 are in the
deformed state.
𝐍
Reference (undeformed) Current (deformed)
𝐧
𝐗𝑎0
𝐗𝑏0 𝐓 = 𝐒 𝐍
𝐭 = 𝛔𝐧
𝐱𝑎0
𝐱𝑏0
S, Second Piola-Kirchhoff stress tensor 𝛔, Cauchy Stress
```

### Slide 29

![Slide 29](assets/slides/09_FEM/09_FEM_p29.png)

```text
Different Stresses (traction= ? normal)
31
Interface normal 𝐍 in the reference state
(undeformed N on undeformed unit area 𝐴𝑟𝑒𝑓)
Interface normal 𝐧 in the current
state (deformed 𝐧 on deformed
unit area 𝐴)
Traction 𝐓 in the
reference state
(undeformed)
Second Piola–Kirchhoff stress (𝐒)
𝐓 = 𝐒 𝐍
Traction 𝐭 in the
current state
(deformed)
First Piola–Kirchhoff stress (𝐏)
𝐅𝐓 = 𝐅𝐒 𝐍 = 𝐏𝐍
Cauchy Stress (𝝈)
𝐭 = 𝝈𝐧
Input
Output
𝐏 = 𝐅𝐒
Consider the force on a small deformed area  Δ𝐴 (undeformed Δ𝐴𝑟𝑒𝑓) : Δ𝐴𝐭 = Δ𝐴𝑟𝑒𝑓𝐅𝐓
𝝈 Δ𝐴𝐧 = 𝐏 Δ𝐴𝑟𝑒𝑓𝐍
According to Nanson‘s formula: Δ𝐴𝐧 = det 𝐅 𝐅−T(Δ𝐴ref𝐍), we have: 𝝈 = det−1(𝐅)𝐏𝐅T
𝝈 = det−1(𝐅)𝐏𝐅T
```

### Slide 30

![Slide 30](assets/slides/09_FEM/09_FEM_p30.png)

```text
Nanson‘s formula, 𝐴𝐧 = det 𝐅 𝐅−T(𝐴ref𝐍)
32
2𝐴ref𝐍 = 𝐗𝑎0 × 𝐗𝑏0
2𝐴𝐧 = 𝐱𝑎0 × 𝐱𝑏0 = 𝐅𝐗𝑎0 × 𝐅𝐗𝑏0 = 𝐔𝐃𝐕T𝐗𝑎0 × 𝐔𝐃𝐕T𝐗𝑏0
𝐗0
𝐱0
𝐍
𝐧
𝐗𝑎0
𝐗𝑏0
𝐱𝑎0
𝐱𝑏0
= 𝐔 𝐃𝐕T𝐗𝑎0 × 𝐃𝐕T𝐗𝑏0
= 𝐔
𝑑1𝑑2
𝑑0𝑑2
𝑑0𝑑1
𝐕T𝐗𝑎0 × 𝐕T𝐗𝑏0
= 𝐔
𝑑1𝑑2
𝑑0𝑑2
𝑑0𝑑1
𝐕T 𝐗𝑎0 × 𝐗𝑏0
= 𝑑0𝑑1𝑑2𝐔
1/𝑑0
1/𝑑1
1/𝑑2
𝐕T 𝐗𝑎0 × 𝐗𝑏0
= det 𝐅 𝐅−T 𝐗𝑎0 × 𝐗𝑏0 = det 𝐅 𝐅−T(2𝐴ref𝐍)
Reference
𝐃 =
𝑑0
𝑑1
𝑑2
```

### Slide 31

![Slide 31](assets/slides/09_FEM/09_FEM_p31.png)

```text
Force, an Integration of Traction
33
𝐱0
𝐱1
𝐱2
midpoint Interface L
𝐟0 = ර
𝐿
𝛔𝐧𝑑𝑙
Force contributed by an element:
Since 𝛔 is constant within the element,
ර
𝐿
𝛔𝐧𝑑𝑙 + ර
𝐿20
𝛔𝐧𝑑𝑙 + ර
𝐿10
𝛔𝐧𝑑𝑙 = 0
(Divergence Theorem)
The force is:
𝐟0 = − ර
𝐿20
𝛔𝐧20𝑑𝑙 − ර
𝐿10
𝛔𝐧10𝑑𝑙 = −𝛔 𝐱20
2 𝐧20 + 𝐱10
2 𝐧10
𝐧20
𝐧10
```

### Slide 32

![Slide 32](assets/slides/09_FEM/09_FEM_p32.png)

```text
Force, an Integration of Traction
34
𝐟0 = − ර
𝐿20
𝛔𝐧20𝑑𝑙 − ර
𝐿10
𝛔𝐧10𝑑𝑙 = −𝛔 𝐱20
2 𝐧20 + 𝐱10
2 𝐧10
𝐱0
𝐱1
𝐱2
𝐧20
𝐧10
𝐟0 = − ර
𝛀
𝛔𝐧𝑑𝐴 = −𝛔 𝐴012
3 𝐧012 + 𝐴023
3 𝐧023 + 𝐴031
3 𝐧031
= − 𝛔
6 𝐱10 × 𝐱20 + 𝐱20 × 𝐱30 + 𝐱30 × 𝐱10
𝐱0
𝐱1
𝐱2
𝐱3
𝐧012
𝐧023
𝛀
𝝈 Δ𝐴𝐧 = 𝐏 Δ𝐴𝑟𝑒𝑓𝐍
```

### Slide 33

![Slide 33](assets/slides/09_FEM/09_FEM_p33.png)

```text
Force, the Gradient of the Energy = an Integration of Traction
35
𝐟1 𝐟2 𝐟3 = −𝑉ref𝐅𝐒 𝐗10 𝐗20 𝐗30 −T
𝐟0 = −𝐟𝟏 − 𝐟2 −𝐟3
𝐱0
𝐱1
𝐱2
𝐱3
𝐱0
𝐱1
𝐱2
𝐱3
𝐧012
𝐧023
𝛀
𝐟1 = −𝛔
𝐴012
3 𝐧012 +
𝐴023
3 𝐧023 +
𝐴031
3 𝐧031
= −𝑷
𝐴012
𝑟𝑒𝑓
3 𝐍012 +
𝐴023
𝑟𝑒𝑓
3 𝐍023 +
𝐴031
𝑟𝑒𝑓
3 𝐍031
= − 𝐏
6 𝐗01 × 𝐗21 + 𝐗21 × 𝐗31 + 𝐗31 × 𝐗01 = − 𝐅𝐒
6 𝐛1
```

### Slide 34

![Slide 34](assets/slides/09_FEM/09_FEM_p34.png)

```text
Force, the Gradient of the Energy = an Integration of Traction
36
𝐟1 𝐟2 𝐟3 = −𝑉ref𝐅𝐒 𝐗10 𝐗20 𝐗30 −T
𝐟0 = −𝐟𝟏 − 𝐟2 −𝐟3
𝐱0
𝐱1
𝐱2
𝐱3
𝐱0
𝐱1
𝐱2
𝐱3
𝐧012
𝐧023
𝛀
𝐟1 𝐟2 𝐟3 = − 𝐅𝐒
6 𝐛1 𝐛2 𝐛3
𝐛1 𝐛2 𝐛3 = 6𝑉ref 𝐗10 𝐗20 𝐗30 −T𝐛1 𝐛2 𝐛3 = 6𝑉ref 𝐗10 𝐗20 𝐗30 −T
𝐛1 = 𝐗01 × 𝐗21 + 𝐗21 × 𝐗31 + 𝐗31 × 𝐗01
```

### Slide 35

![Slide 35](assets/slides/09_FEM/09_FEM_p35.png)

```text
37
𝐛1 𝐛2 𝐛3 = 6𝑉ref 𝐗10 𝐗20 𝐗30 −T
𝐗1
𝐗0
𝐗2
𝐍102
𝐍123
𝐗10
T 𝐛1 = 𝐗10
T 𝐗01 × 𝐗21 + 𝐗21 × 𝐗31 + 𝐗31 × 𝐗01 = 𝐗10
T 𝐗21 × 𝐗31
= 𝐗01
T 𝐗31 × 𝐗21 = 6𝑉ref
Meanwhile,
𝐗20
T 𝐛1 = 𝐗20
T 𝐗01 × 𝐗21 + 𝐗21 × 𝐗31 + 𝐗31 × 𝐗01
= 𝐗20
T 𝐗20 × 𝐗10 + 𝐗20 × 𝐗31 = 0
𝐗30
T 𝐛1 = 𝐗30
T 𝐗01 × 𝐗21 + 𝐗21 × 𝐗31 + 𝐗31 × 𝐗01
= 𝐗30
T 𝐗21 × 𝐗30 + 𝐗10 × 𝐗30 = 0
𝐍130
𝐗10 𝐗20 𝐗30 T𝐛1 = 𝐗10 𝐗20 𝐗30 T 𝐗01 × 𝐗21 + 𝐗21 × 𝐗31 + 𝐗31 × 𝐗01 =
6𝑉ref
0
0
=
6𝑉ref
0
0
```

### Slide 36

![Slide 36](assets/slides/09_FEM/09_FEM_p36.png)

```text
38
𝐛1 𝐛2 𝐛3 = 6𝑉ref 𝐗10 𝐗20 𝐗30 −T
𝐗10 𝐗20 𝐗30 T𝐛2 =
0
6𝑉ref
0
𝐗10 𝐗20 𝐗30 T𝐛3 =
0
0
6𝑉ref
𝐛1 𝐛2 𝐛3 = 6𝑉ref 𝐗10 𝐗20 𝐗30 −T
= 1
det( 𝐗10 𝐗20 𝐗30 −1) 𝐗10 𝐗20 𝐗30 −T
𝐗10 𝐗20 𝐗30 T𝐛1 = 𝐗10 𝐗20 𝐗30 T 𝐗01 × 𝐗21 + 𝐗21 × 𝐗31 + 𝐗31 × 𝐗01 =
6𝑉ref
0
0
=
6𝑉ref
0
0
= 1
det(𝐄−1)𝐄−T
```

### Slide 37

![Slide 37](assets/slides/09_FEM/09_FEM_p37.png)

```text
Linear FEM Framework
A Quick Summary
39
𝐄 = 𝐗10 𝐗20 𝐗30
𝐅 = 𝐱10 𝐱20 𝐱30 𝐄−1
𝐏 = 𝐅𝐒
𝐟1 𝐟2 𝐟3 = −𝑉ref𝐏𝐄−T = − 1
6det(𝐄−𝟏)𝐏𝐄−T
𝐟0 = −𝐟1 − 𝐟2 − 𝐟3
Deformation gradient
Green strain
First PK Stress
Forces
𝐆 = 1
2 𝐅T𝐅 − 𝐈
e.g., 𝐒 =
𝜕WStVK
𝜕𝐆 = 2𝜇𝐆 + 𝜆 trace𝐆 𝐈
Foreach Tetrahedra
```

### Slide 38

![Slide 38](assets/slides/09_FEM/09_FEM_p38.png)

```text
Explicit or Implicit Timestep
40
Sifakis, E. and J. Barbic (2012). FEM simulation of 3d
deformable solids: A practitioner’zs guide to theory,
discretization and model reduction. In SIGGRAPH
Courses, pp. 20:1–20:50
Implicit: Sec. 4.3
```

### Slide 39

![Slide 39](assets/slides/09_FEM/09_FEM_p39.png)

```text
Hyperelastic Models
42
```

### Slide 40

![Slide 40](assets/slides/09_FEM/09_FEM_p40.png)

```text
Hyperelasticity
Conservative Energy and Force: 𝐸 =׬Ω Ψ 𝐹 𝑑𝐱 ,   𝐟 = −(
𝜕𝐸
𝜕𝐱𝑖
)T
43
```

### Slide 41

![Slide 41](assets/slides/09_FEM/09_FEM_p41.png)

```text
Elasticity
• Shape Changes:
• Deformation Measure
44
DeformationGradient, 𝐅
𝑎 𝑡𝑒𝑛𝑠𝑜𝑟
Strain, 𝐆
𝑎 𝑡𝑒𝑛𝑠𝑜𝑟
EnergyDensity, W& Stress, 𝛔
𝑎 𝑠𝑐𝑎𝑙𝑎𝑟& 𝑎 𝑡𝑒𝑛𝑠𝑜𝑟
𝐟𝑖 = −( 𝜕𝐸
𝜕𝐱𝑖
)T= ර
𝐿
𝛔𝐧𝑑𝑙
• Strain
• Elastic Energy
• Forces
𝐟𝑖 = −𝑉ref𝐏 𝐗10 𝐗20 𝐗30 −T
```

### Slide 42

![Slide 42](assets/slides/09_FEM/09_FEM_p42.png)

```text
Elasticity
45
• Strain Energy Function
DeformationGradient, 𝐅
𝑎 𝑡𝑒𝑛𝑠𝑜𝑟
StrainEnergyDensity, W 𝐅
& First Piola–Kirchhoff stress P(𝐅)
𝑎 𝑠𝑐𝑎𝑙𝑎𝑟& 𝑎 𝑡𝑒𝑛𝑠𝑜𝑟
• Forces 𝐟𝑖 = −( 𝜕𝐸
𝜕𝐱𝑖
)T= ර
𝐿
𝛔𝐧𝑑𝑙
𝐟𝑖 = −𝑉ref𝐏 𝐗10 𝐗20 𝐗30 −T
• Shape Changes:
• Deformation Measure
```

### Slide 43

![Slide 43](assets/slides/09_FEM/09_FEM_p43.png)

```text
Material Models and Parameters
46
Lame parameters 𝝀 and 𝝁 (http://en.wikipedia.org/wiki/Lamé_parameters）
𝜆: Lamé’s first parameter, more related to volume
𝜇: Lamé’s second parameter, more related to length (Shearing)
𝐟𝑖 = −𝑉ref𝐏 𝐗10 𝐗20 𝐗30 −T
Related to the fundamental physical parameters: Poisson’s Ratio 𝑷 (a measure of incompressibility)
and Young’s modulus 𝒀(a measure of stretch resistance)
𝜇 = 𝑌
2(1 + 𝑃) ; 𝜆= 𝑌𝑃
(1 + 𝑃)(1 − 2𝑃)
𝑌 = 𝜇 3𝜆 + 2𝜇
𝜆 + 𝜇 ; 𝑃 = 𝜆
2 𝜆 + 𝜇
St. Venant-Kirchhoff material
𝐆 = 1
2 𝐅T𝐅 − 𝐈
𝑊 = 𝝀
2tr2(𝐆) + 𝝁 𝐆 𝑭
𝐏 = 𝜕𝑊
𝜕𝐹 = 𝜕𝐺
𝜕𝐹
𝜕𝑊
𝜕𝐺 = 𝐅𝐒 = 𝐅[2𝝁𝐆 + 𝝀tr 𝐆 𝐈]
Neohookean elasticity
J = det 𝐅 ; 𝑰𝑪 = 𝐅 𝑭
𝟐
𝑊 = 𝝀
2log2 𝐽 + 𝝁
2 𝑰𝑪 − 3 − 𝝁log(J)
𝐏 = 𝜕𝑊
𝜕𝐹 = 𝝁 𝐅 − 𝐅−T + 𝝀log(J)𝐅−T
```

### Slide 44

![Slide 44](assets/slides/09_FEM/09_FEM_p44.png)

```text
Material Models and Parameters
47
St. Venant-Kirchhoff material
𝐆 = 1
2 𝐅T𝐅 − 𝐈
𝑊 = 𝝀
2tr2(𝐆) + 𝝁 𝐆 𝑭
𝐏 = 𝜕𝑊
𝜕𝐹 = 𝜕𝐺
𝜕𝐹
𝜕𝑊
𝜕𝐺 = 𝐅𝐒 = 𝐅[2𝝁𝐆 + 𝝀tr 𝐆 𝐈]
Neohookean elasticity
J = det 𝐅 ; 𝑰𝑪 = 𝐅 𝑭
𝟐
𝑊 = 𝝀
2log2 𝐽 + 𝝁
2 𝑰𝑪 − 3 − 𝝁log(J)
𝐏 = 𝜕𝑊
𝜕𝐹 = 𝝁 𝐅 − 𝐅−T + 𝝀log(J)𝐅−T
𝐟𝑖 = −𝑉ref𝐏 𝐗10 𝐗20 𝐗30 −T
```

### Slide 45

![Slide 45](assets/slides/09_FEM/09_FEM_p45.png)

```text
Material Models and The Cauchy-Green Invariants
48
𝐂 = 𝐅T𝐅 is the right Cauchy-Green deformation tensor.
𝐼𝑪 = 𝐅 𝑭
𝟐 = ෍
𝑖,𝑗
𝐹𝑖𝑗
2 = tr(𝐅T𝐅) = tr(𝐂)
𝐼𝐼𝑪 = 𝐅T𝐅 𝑭
𝟐
= ෍
𝑖,𝑗
(Σ𝑘𝐹𝑖𝑘𝐹𝑘𝑗)2 = trace𝐂2
𝐼𝐼𝐼𝑪 = det 𝐅T𝐅 = det 𝐅 2 = 𝐽2 = det(𝐂)
Length
Area
Volume
𝐖 𝐑𝐅 = 𝐖(𝐅)
𝐖 𝐅 = 𝐖(𝑰𝑪, 𝑰𝑰𝑪, 𝑰𝑰𝑰𝑪 )
𝐖 𝐑𝐅 = 𝐖(𝑰𝑪, 𝑰𝑰𝑪, 𝑰𝑰𝑰𝑪 ) = 𝐖(𝐅)
𝑪 𝐑𝐅 = 𝐅T𝐑T𝐑𝐅 = 𝑪 𝐅
𝐼𝑪 𝐑𝐅 = 𝐼𝑪(𝐅)
𝐼𝐼𝑪 𝐑𝐅 = 𝐼𝐼𝑪(𝐅)
𝐼𝐼𝐼𝑪 𝐑𝐅 = 𝐼𝐼𝐼𝑪(𝐅)
𝐼𝐼𝑐∗ = 1
2 (𝐼𝑐2−𝐼𝐼𝐂)
```

### Slide 46

![Slide 46](assets/slides/09_FEM/09_FEM_p46.png)

```text
Material Models and The Cauchy-Green Invariants
49
𝐟𝑖 = −𝑉ref𝐏 𝐗10 𝐗20 𝐗30 −T; 𝜇: lengthpreservation; 𝜆: volumepreservation
𝑊STVK = 𝜆
2 𝐼𝐂 − 3 2 + 𝜇
4(𝐼𝐼𝐂 − 2𝐼𝐂 + 3) 𝑊BW08 = 𝜆
2 log2(𝐼𝐼𝐼𝐂
−1/2)+ 𝝁
2 𝐼𝐂 − 3 − 𝝁log(𝐼𝐼𝐼𝐂
−1/2)
StretchVolume
Bonet, J. and R. D. Wood (2008). Nonlinear continuum mechanics for finite element analysis. Cambridge university press.
Volume Stretch
= tr(𝐂)
= tr 𝐂2
= 𝐽2 = det(𝐂)
𝑰𝑪 = tr(𝐅T𝐅)
𝑰𝑰𝑪 = 𝐅T𝐅 𝑭
𝟐
𝑰𝑰𝑰𝑪 = det 𝐅 2
Length
Area
Volume
St. Venant-Kirchhoff material
𝐆 = 1
2 𝐅T𝐅 − 𝐈
𝑊 = 𝝀
2tr2(𝐆) + 𝝁 𝐆 𝑭
𝐏 = 𝜕𝑊
𝜕𝐹 = 𝜕𝐺
𝜕𝐹
𝜕𝑊
𝜕𝐺 = 𝐅𝐒 = 𝐅[2𝝁𝐆 + 𝝀tr 𝐆 𝐈]
Neohookean elasticity
J = det 𝐅 ; 𝑰𝑪 = 𝐅 𝑭
𝟐
𝑊 = 𝝀
2log2 𝐽 + 𝝁
2 𝑰𝑪 − 3 − 𝝁log(J)
𝐏 = 𝜕𝑊
𝜕𝐹 = 𝝁 𝐅 − 𝐅−T + 𝝀log(J)𝐅−T
oliver.rmee.upc.edu/xo/vpage/1/0/Teaching/Computational-Solid-Mechanics
```

### Slide 47

![Slide 47](assets/slides/09_FEM/09_FEM_p47.png)

```text
Material Models, Comparison
50
√Rotationally invariant
√ No SVD needed
× Weak resistance to compression
St. Venant-Kirchhoff material
× Inaccurate volume preservation
Neo-Hookean elasticity
√ Accurate volume preservation
√ Discourages compress
× Undefined when inverted
× Stiff compression
√Rotationally invariant
```

### Slide 48

![Slide 48](assets/slides/09_FEM/09_FEM_p48.png)

```text
Material Models and Strain Energy Function
51
Mooney-
Rivlin
Arruda-Boyce
Fung
Yeoh
Neo-Hookean 𝑊𝑁𝐻
𝑊𝑀𝑅
𝑊𝐴𝐵
𝑊𝐹𝑢𝑛𝑔
𝑊𝑌𝑒𝑜ℎ
stVK = 𝜆
2 𝐼C − 3 2 + 𝜇
4(𝐼𝐼C − 2𝐼𝐂 + 3)𝑊stVK
SIGGRAPH 2022 Course,
Dynamic Deformables:
Implementation and Production
Practicalities
Dynamic deformables | ACM
SIGGRAPH 2022 Courses
(The gradients and Hessians of
all three invariants)
If we can get the gradients and
Hessians of all three invariants,
then deriving the energy’s force
gradient becomes much easier
𝐼𝐼𝑐∗ = 1
2 (𝐼𝑐2−𝐼𝐼𝐂)
```

### Slide 49

![Slide 49](assets/slides/09_FEM/09_FEM_p49.png)

```text
Isotropic Material Models
53
𝐖 𝐑𝐅 = 𝐖(𝐅)
𝐖 𝐅𝐐 = 𝐖(𝐅)
𝐖 𝐑𝐅𝐐 = 𝐖(𝐅)
𝐖 𝐅 = 𝐖 𝐔𝚲𝐕T = 𝐖 𝚲 = 𝐖(𝜆0, 𝜆1 , 𝜆2)
```

### Slide 50

![Slide 50](assets/slides/09_FEM/09_FEM_p50.png)

```text
54
Strain of Isotropic Models
For Isotropic Model: 𝐏 𝐅 = 𝐏(𝐔𝚲𝐕T) = 𝐔𝐏(𝜆0, 𝜆1, 𝜆2)𝐕T
(𝜆0, 𝜆1, 𝜆2): Principal stretches: the singular values of 𝐅; 𝐂 = 𝚲T𝚲
𝐼𝐂 = trace𝐂 = 𝜆0
2 + 𝜆1
2 + 𝜆2
2
𝐼𝐼𝑐∗ = 1
2 (𝐼𝑐2−𝐼𝐼𝐂) = 1
2 trace2 𝐂 − trace𝐂2 = 𝜆0
2𝜆1
2 + 𝜆0
2𝜆2
2 + 𝜆1
2𝜆2
2
𝐼𝐼𝐼𝐂 = det 𝐂 = det 𝐅 2= (𝜆0𝜆1𝜆2)2
Squared
Length
Squared
Area
Squared
Volume
𝐼𝐼𝑪 = trace𝐂2 = 𝜆0
4 + 𝜆1
4 + 𝜆2
4
```

### Slide 51

![Slide 51](assets/slides/09_FEM/09_FEM_p51.png)

```text
55
Strain of Isotropic Models
For Isotropic Model: 𝐏 𝐅 = 𝐏(𝐔𝚲𝐕T) = 𝐔𝐏(𝜆0, 𝜆1, 𝜆2)𝐕T
(𝜆0, 𝜆1, 𝜆2): Principal stretches: the singular values of 𝐅; 𝐂 = 𝚲T𝚲
𝐼𝐂 = trace𝐂 = 𝜆0
2 + 𝜆1
2 + 𝜆2
2
𝐼𝐼𝑐∗ = 1
2 (𝐼𝑐2−𝐼𝐼𝐂) = 1
2 trace2 𝐂 − trace𝐂2 = 𝜆0
2𝜆1
2 + 𝜆0
2𝜆2
2 + 𝜆1
2𝜆2
2
𝐼𝐼𝐼𝐂 = det 𝐂 = det 𝐅 2= (𝜆0𝜆1𝜆2)2
Squared
Length
Squared
Area
Squared
Volume
𝐼𝐼𝑪 = trace𝐂2 = 𝜆0
4 + 𝜆1
4 + 𝜆2
4
SIGGRAPH 2022 Course, Dynamic Deformables: Implementation and Production Practicalities  Dynamic deformables | ACM SIGGRAPH 2022 Courses
```

### Slide 52

![Slide 52](assets/slides/09_FEM/09_FEM_p52.png)

```text
Material Models and Strain Energy Function
56
Mooney-
Rivlin
Arruda-Boyce
Fung
Yeoh
Neo-Hookean 𝑊𝑁𝐻
𝑊𝑀𝑅
𝑊𝐴𝐵
𝑊𝐹𝑢𝑛𝑔
𝑊𝑌𝑒𝑜ℎ
stVK = 𝜆
2 𝐼C − 3 2 + 𝜇
4(𝐼𝐼C − 2𝐼𝐂 + 3)𝑊stVK
SIGGRAPH 2022 Course,
Dynamic Deformables:
Implementation and Production
Practicalities
Dynamic deformables | ACM
SIGGRAPH 2022 Courses
(The gradients and Hessians of
all three invariants)
If we can get the gradients and
Hessians of all three invariants,
then deriving the energy’s force
gradient becomes much easier
𝐼𝐼𝑐∗ = 1
2 (𝐼𝑐2−𝐼𝐼𝐂)
```

### Slide 53

![Slide 53](assets/slides/09_FEM/09_FEM_p53.png)

```text
A FEM/FVM Framework
A Quick Summary (FEM for Isotropic Material )
57
𝐄 = 𝐗10 𝐗20 𝐗30
𝐅 = 𝐱10 𝐱20 𝐱30 𝐄−1
𝐔 𝚲 𝐕𝐓 = svd(𝐅)
𝐏 = 𝜕𝑊
𝜕𝑭 = 𝐔diag 𝜕𝑊
𝜕𝜆0
, 𝜕𝑊
𝜕𝜆1
, 𝜕𝑊
𝜕𝜆2
𝐕T
𝐟1 𝐟2 𝐟3 = − 1
6det(𝐄−1)𝐏𝐄−T
𝐟0 = −𝐟1 − 𝐟2 − 𝐟3
Deformation gradient
Principal stretches 𝚲
First PK Stress
Forces
```

### Slide 54

![Slide 54](assets/slides/09_FEM/09_FEM_p54.png)

```text
Descent Methods for Elastic Body Simulation on the GPU (SIGGRAPH Asia 2016)
58
```

### Slide 55

![Slide 55](assets/slides/09_FEM/09_FEM_p55.png)

```text
After-Class Reading
59
Teran et al. 2003. Finite Volume Methods for the
Simulation of Skeleton Muscles. SCA.
Sifakis, E. and J. Barbic (2012). FEM simulation of 3d
deformable solids: A practitioner’s guide to theory,
discretization and model reduction. In SIGGRAPH
Courses, pp. 20:1–20:50
```

### Slide 56

![Slide 56](assets/slides/09_FEM/09_FEM_p56.png)

```text
60
Xu et al. 2015. Nonlinear Material Design Using Principal
Stretches. TOG (SIGGRAPH).
Implicit Time Integration
After-Class Reading
弹性有限元方法 | Tom's develop Blog
(tomsworkspace.github.io)
```

### Slide 57

![Slide 57](assets/slides/09_FEM/09_FEM_p57.png)

```text
61
Optional Exercises
https://games103.games-cn.org/HW3/ https://github.com/dilevin/CSC417-a3-finite-elements-3d
CUDA code for GPU-based
hyperelastic simulation in C++:
https://wanghmin.github.io/Wang-
2016-DME/Wang-2016-DME.zip
Physics-based Animation by David Levin
Huamin Wang and Yin Yang.
2016. Descent Methods for
Elastic Body Simulation on the
GPU. (SIGGRAPH Asia)
StVK
```

### Slide 58

![Slide 58](assets/slides/09_FEM/09_FEM_p58.png)

```text
Conclusions
• FEM (Continuum Mechanics)
• Deformation Gradient and Strain
• Energy and Stress
• Linear FEM: Force, the Gradient of the Energy = an Integration of Traction
• Hyperelastic models
63
```
