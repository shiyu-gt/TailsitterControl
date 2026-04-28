# Twin-Rotor Tail-Sitter Aircraft 6DOF动力学与控制律详解

## 概述

本文档详细解释了双旋翼尾座式固定翼飞机的6自由度（6DOF）非线性动力学模型及其控制系统设计。该模型实现了：
- 基于表格数据的气动系数模型
- 考虑螺旋桨滑流效应的控制面效能增强（双动压模型）
- 四元数姿态表示，避免尾座式悬停时的万向锁（theta≈90°）
- 级联PID控制器用于悬停控制（位置→姿态→角速度）
- 线性化分析与模态分析

---

## 1. 坐标系与运动方程

### 1.1 坐标系约定
- **机体坐标系（Body Frame）**：$x_B$轴向前（机头方向），$y_B$轴向右，$z_B$轴向下（符合右手法则）
- **地面坐标系（Earth Frame）**：$x_E$轴指向北（或参考方向），$y_E$轴指向东，$z_E$轴垂直向下指向地心（NED）
- **运动方程**：使用机体坐标系描述速度和角速度，**四元数表示姿态**，地面坐标系描述位置

### 1.2 6DOF状态向量

当前实现采用**13状态四元数模型**，避免悬停时theta≈90°的万向锁奇异：

$$\mathbf{x} = [u, v, w, p, q, r, q_x, q_y, q_z, q_w, x_E, y_E, z_E]$$

- **速度分量**：$u, v, w$（机体坐标系下的速度分量，单位：m/s）
- **角速度**：$p, q, r$（机体坐标系下的角速度，单位：rad/s）
  - $p$：滚转角速度（绕$x_B$轴）
  - $q$：俯仰角速度（绕$y_B$轴）
  - $r$：偏航角速度（绕$z_B$轴）
- **姿态四元数**：$\mathbf{q} = [q_w, q_x, q_y, q_z]$（标量$q_w$在前，矢量$[q_x,q_y,q_z]$在后）
  - 存储顺序为状态向量中的$[q_x, q_y, q_z, q_w]$（标量在后），注意与接口函数的统一
- **位置**：$x_E, y_E, z_E$（地面坐标系下的位置，NED约定：$z_E$向下为正）

> **注**：早期版本使用欧拉角$[\phi, \theta, \psi]$表示姿态，但在尾座式悬停（$\theta \approx 90°$）时会出现万向锁。当前代码已全面迁移至四元数表示。

### 1.3 控制向量

真实飞机为**升降副翼（elevon）布局**，无独立副翼/方向舵：

$$\mathbf{u}_{ctrl} = [\delta T_{left}, \delta T_{right}, \delta e_{left}, \delta e_{right}]$$

- **$\delta T_{left}, \delta T_{right}$**：左右电机油门（0-1无量纲）
- **$\delta e_{left}, \delta e_{right}$**：左右升降副翼偏转角（弧度，向下偏转为正）
  - 对称偏转（$\delta e_{left} = \delta e_{right}$）产生俯仰力矩
  - 差动偏转（$\delta e_{left} \neq \delta e_{right}$）产生滚转力矩

---

## 2. 动力学方程

### 2.1 力与力矩计算

#### 总力方程
机体坐标系下的总力由气动力、推力和重力组成：

$$\begin{align*}
F_x &= F_{a_x} + G_x + T_{left} + T_{right} \\
F_y &= F_{a_y} + G_y \\
F_z &= F_{a_z} + G_z
\end{align*}$$

**计算逻辑**：
- $F_x, F_y, F_z$：机体坐标系下的总力分量（N）
- $F_{a_x}, F_{a_y}, F_{a_z}$：气动力分量
- $G_x, G_y, G_z$：重力分量（通过四元数旋转矩阵转换到机体坐标系）
- $T_{left}, T_{right}$：左右电机推力（沿机体$x_B$轴）

#### 重力分量（机体坐标系）

重力在地面坐标系中为$\mathbf{G}_E = [0, 0, mg]^T$（NED：$z$向下为正，重力指向$+z$）。
通过四元数旋转矩阵$\mathbf{R}_{E\to B}$转换到机体坐标系：

$$\mathbf{G}_B = \mathbf{R}_{E\to B} \mathbf{G}_E = \mathbf{R}^T \mathbf{G}_E$$

其中$\mathbf{R}$为机体到地面的旋转矩阵（由四元数构造）：

$$\mathbf{R} = \begin{bmatrix}
1-2(q_y^2+q_z^2) & 2(q_xq_y-q_wq_z) & 2(q_xq_z+q_wq_y) \\
2(q_xq_y+q_wq_z) & 1-2(q_x^2+q_z^2) & 2(q_yq_z-q_wq_x) \\
2(q_xq_z-q_wq_y) & 2(q_yq_z+q_wq_x) & 1-2(q_x^2+q_y^2)
\end{bmatrix}$$

#### 推力力矩（差动油门产生偏航力矩）

左右电机安装在机翼前缘，仅有$y$向错位：

$$N_{thrust} = T_{left} \cdot y_{motor\_left} + T_{right} \cdot y_{motor\_right}$$

**计算逻辑**：
- 电机位置：$y_{motor\_left} = -b/4$, $y_{motor\_right} = +b/4$（$b$为翼展）
- 推力沿机体$x_B$轴，叉乘$\mathbf{r} \times \mathbf{F}$严格产生绕$z_B$轴的偏航力矩
- 左油门增大 → $N < 0$（机头向左偏航）
- 右油门增大 → $N > 0$（机头向右偏航）

#### 总力矩

$$\begin{align*}
L &= L_a + L_{elevon\_diff} \\
M &= M_a + M_{elevon\_sym} \\
N &= N_a + N_{thrust}
\end{align*}$$

- $L_a, M_a, N_a$：基础气动力矩（机身，不含舵面）
- $L_{elevon\_diff}$：升降副翼差动产生的滚转力矩
- $M_{elevon\_sym}$：升降副翼对称偏转产生的俯仰力矩
- $N_{thrust}$：差动油门产生的偏航力矩

### 2.2 运动学方程

#### 平移运动（机体坐标系）

$$\begin{align*}
\dot{u} &= \frac{F_x}{m} + rv - qw \\
\dot{v} &= \frac{F_y}{m} - ru + pw \\
\dot{w} &= \frac{F_z}{m} + qu - pv
\end{align*}$$

**计算逻辑**：
- 牛顿第二定律在旋转坐标系中的形式
- $rv - qw$等：科里奥利加速度项，由于机体旋转产生的耦合加速度

#### 旋转运动（考虑$I_{xz}$耦合）

$$\Gamma = I_xI_z - I_{xz}^2$$

$$\begin{align*}
\dot{p} &= \frac{1}{\Gamma}[I_zL + I_{xz}N - (I_z(I_z-I_y) + I_{xz}^2)qr - I_{xz}(I_x-I_y+I_z)pq] \\
\dot{q} &= \frac{1}{I_y}[M - (I_x-I_z)pr - I_{xz}(p^2-r^2)] \\
\dot{r} &= \frac{1}{\Gamma}[I_xN + I_{xz}L - I_{xz}(I_x-I_y+I_z)qr - (I_x(I_x-I_y) + I_{xz}^2)pq]
\end{align*}$$

**计算逻辑**：
- 使用欧拉方程在机体坐标系中的形式
- 考虑了惯性积$I_{xz}$（陀螺耦合）
- $\Gamma$：特征行列式，用于消除耦合

#### 四元数运动学

$$\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \boldsymbol{\omega}$$

其中$\boldsymbol{\omega} = [0, p, q, r]$为纯虚四元数（标量部为0），$\otimes$为四元数乘法。

展开形式：

$$\begin{align*}
\dot{q}_w &= -\frac{1}{2}(q_x p + q_y q + q_z r) \\
\dot{q}_x &= \frac{1}{2}(q_w p + q_y r - q_z q) \\
\dot{q}_y &= \frac{1}{2}(q_w q + q_z p - q_x r) \\
\dot{q}_z &= \frac{1}{2}(q_w r + q_x q - q_y p)
\end{align*}$$

**计算逻辑**：
- 四元数导数的幅值上限由角速度自然约束（$|\dot{\mathbf{q}}| \leq 0.5|\boldsymbol{\omega}|$）
- 积分后需重新归一化，保持$|\mathbf{q}| = 1$
- 天然无万向锁，适用于全姿态范围（包括尾座式$\theta = 90°$）

#### 位置运动学（地面坐标系）

$$\begin{align*}
\dot{x}_E &= R_{11}u + R_{12}v + R_{13}w \\
\dot{y}_E &= R_{21}u + R_{22}v + R_{23}w \\
\dot{z}_E &= R_{31}u + R_{32}v + R_{33}w
\end{align*}$$

其中$\mathbf{R}$为机体到地面的旋转矩阵（由四元数构造）。

**计算逻辑**：
- 将机体速度$\mathbf{v}_B = [u, v, w]^T$通过旋转矩阵转换到地面坐标系
- NED约定：$z_E$向下为正，因此$\dot{z}_E > 0$表示飞机下降

---

## 3. 气动力与力矩模型

### 3.1 气动系数计算

#### 基本参数

$$V = \sqrt{u^2 + v^2 + w^2}$$
$$\alpha = \arctan2(w, u) \quad \text{(攻角)}$$
$$\beta = \arcsin\left(\frac{v}{V}\right) \quad \text{(侧滑角)}$$
$$q_{inf} = \frac{1}{2}\rho V^2 \quad \text{(自由流动压)}$$

**计算逻辑**：
- $V$：空速（相对空气的速度大小）
- $\alpha$：决定升力和阻力大小
- $\beta$：侧向流动引起的侧力
- $q_{inf}$：基础气动力的缩放因子

#### 无量纲角速率

$$\hat{p} = \frac{pb}{2V} \quad \hat{q} = \frac{q\bar{c}}{2V} \quad \hat{r} = \frac{rb}{2V}$$

当$V < 0.1$时，使用$V = 0.1$避免除零，低速时角速率项提供主要气动阻尼。

### 3.2 滑流效应计算（动量理论）

#### 滑流速度

基于真实自由流速度$V_{inf}$（而非硬编码常量）：

$$v_{slip} = V_{inf} + \sqrt{\frac{2T}{\rho A_{prop}}}$$

**计算逻辑**：
- $V_{inf} = \sqrt{u^2+v^2+w^2}$：真实空速
- $T$：单电机推力
- $A_{prop} = \pi (D_{prop}/2)^2$：螺旋桨盘面积
- 滑流速度增加量：$\Delta v = \sqrt{2T/(\rho A_{prop})}$

#### 滑流动压

$$q_{slip} = \frac{1}{2}\rho v_{slip}^2$$

左右电机各自计算滑流动压$q_{slip\_left}, q_{slip\_right}$，以处理不对称推力工况。

### 3.3 双动压气动力模型

当前代码采用**双动压模型**：机身基础气动使用自由流动压$q_{inf}$，舵面气动（位于滑流中）使用滑流动压$q_{slip}$。

#### 基础气动力（机身，使用$q_{inf}$）

$$\begin{align*}
F_{a_x}^{base} &= -q_{inf}S(C_D^{base}\cos\alpha - C_L^{base}\sin\alpha) \\
F_{a_y}^{base} &= q_{inf}S C_Y^{base} \\
F_{a_z}^{base} &= -q_{inf}S(C_D^{base}\sin\alpha + C_L^{base}\cos\alpha) \\
L_a^{base} &= q_{inf}S b C_l^{base} \\
M_a^{base} &= q_{inf}S \bar{c} C_m^{base} \\
N_a^{base} &= q_{inf}S b C_n^{base}
\end{align*}$$

其中基础系数$(C_L^{base}, C_D^{base}, C_Y^{base}, C_l^{base}, C_m^{base}, C_n^{base})$从表格插值获得，不含舵面增量。

#### 升降副翼气动力（使用$q_{slip}$）

**对称偏转（俯仰控制）**：
$$\delta e_{sym} = \frac{\delta e_{left} + \delta e_{right}}{2}$$

从数据表获取$\delta e_{sym}$对应的$(\Delta C_L, \Delta C_D, \Delta C_m)$，使用平均滑流动压$\bar{q}_{slip} = (q_{slip\_left} + q_{slip\_right})/2$计算：

$$M_{elevon\_sym} = \bar{q}_{slip} S \bar{c} \, \Delta C_m(\delta e_{sym})$$

**差动偏转（滚转控制）**：
$$\Delta C_L^{left} = \Delta C_L(\delta e_{left}), \quad \Delta C_L^{right} = \Delta C_L(\delta e_{right})$$

左右升力差产生滚转力矩（力臂为$y_{elevon}$）：

$$L_{elevon\_diff} = y_{elevon} \cdot \left(q_{slip\_right} S \frac{\Delta C_L^{right}}{2} - q_{slip\_left} S \frac{\Delta C_L^{left}}{2}\right)$$

> **物理意义**：悬停时$q_{inf} \approx 0$，机身不产生自由流动压升力；舵面完全处于螺旋桨滑流中，由$q_{slip}$驱动。这避免了旧模型中"增强因子"在$V \to 0$时的数值奇异性。

---

## 4. 悬停控制律设计

### 4.1 级联PID控制器结构（HoverPID）

```
位置误差(x,y) → [位置PID] → 期望倾斜角 → [四元数姿态PID] → 期望角速度 → [角速度PID] → 舵面/差动油门
高度误差(z)   → [高度PID]  → 总油门     → 左右电机
```

控制器采用**四级级联**：

| 控制环 | 输入 | 输出 | 执行器 |
|--------|------|------|--------|
| **外环：水平位置** (x, y) | 位置误差 | 期望倾斜角 | — |
| **高度控制** (z) | 高度误差 | 总油门 | 左右电机 |
| **中环：姿态控制** | 四元数误差 | 期望角速度 | — |
| **内环：角速度控制** | 角速度误差 | 舵面/差动油门指令 | 升降副翼 + 差动油门 |

### 4.2 水平位置控制（外环）

位置误差转换为期望加速度，再映射为期望倾斜角：

$$\mathbf{a}_{des,xy} = K_{p}^{pos} \mathbf{e}_{xy} + K_{i}^{pos} \int \mathbf{e}_{xy} dt + K_{d}^{pos} \dot{\mathbf{e}}_{xy}$$

$$\tilde{\theta}_{pitch} = -a_{des,x}/g, \quad \tilde{\phi}_{roll} = a_{des,y}/g$$

期望四元数由悬停基准$\mathbf{q}_{hover}$（$\theta = 90°$）叠加小倾斜角：

$$\mathbf{q}_{tilt} = \text{quaternion}(0, \tilde{\theta}_{pitch}, \tilde{\phi}_{roll})$$
$$\mathbf{q}_{desired} = \mathbf{q}_{hover} \otimes \mathbf{q}_{tilt}$$

> **尾座式映射修正**：$x$方向误差 → 俯仰倾斜（绕$y$轴），$y$方向误差 → 偏航/滚转映射（悬停时滚转不产生水平加速度，实际通过绕体轴的等效旋转实现）。

### 4.3 高度控制

$$e_z = z_E - z_{target} \quad \text{(NED：} z \text{向下为正)}$$
$$T = T_{hover} + K_p^{alt} e_z$$

**物理意义**：
- 飞机低于目标（$z_E > z_{target}$）→ $e_z > 0$ → **增大油门**
- 飞机高于目标（$z_E < z_{target}$）→ $e_z < 0$ → **减小油门**

### 4.4 姿态控制（中环）

四元数误差计算：

$$\mathbf{q}_{err} = \mathbf{q}_{current}^{-1} \otimes \mathbf{q}_{desired}$$

若$q_{err,w} < 0$，取$\mathbf{q}_{err} = -\mathbf{q}_{err}$（最短路径）。

旋转矢量近似（小角度）：

$$\mathbf{e}_{att} = 2 [q_{err,x}, q_{err,y}, q_{err,z}]$$

期望角速度：

$$\boldsymbol{\omega}_{des} = K_p^{att} \mathbf{e}_{att}$$

### 4.5 角速度控制（内环）

$$\mathbf{e}_{rate} = \boldsymbol{\omega}_{des} - [p, q, r]$$

$$\begin{align*}
\tau_{roll} &= K_p^{rate} e_{rate,p} + K_d^{rate} \dot{e}_{rate,p} \\
\tau_{pitch} &= K_p^{rate} e_{rate,q} + K_d^{rate} \dot{e}_{rate,q} \\
\tau_{yaw} &= K_p^{rate} e_{rate,r} + K_d^{rate} \dot{e}_{rate,r}
\end{align*}$$

### 4.6 控制分配

$$\begin{align*}
\Delta e_{diff} &= \tau_{roll} \cdot k_{roll}^{de} \\
\Delta e_{sym} &= \tau_{pitch} \cdot k_{pitch}^{de} \\
\Delta T_{diff} &= \tau_{yaw} \cdot k_{yaw}^{T}
\end{align*}$$

$$\begin{align*}
\delta e_{left} &= \Delta e_{sym} - \Delta e_{diff} \\
\delta e_{right} &= \Delta e_{sym} + \Delta e_{diff} \\
T_{left} &= T - \Delta T_{diff} \\
T_{right} &= T + \Delta T_{diff}
\end{align*}$$

**符号约定与物理映射**：

| 指令 | 执行器动作 | 产生的力矩 | 当前系数 |
|------|-----------|-----------|---------|
| $\tau_{roll}$ | 左右舵面差动偏转 | 绕 $x$ 轴滚转力矩 | $k_{roll}^{de} = +0.02$ |
| $\tau_{pitch}$ | 左右舵面同步偏转 | 绕 $y$ 轴俯仰力矩 | $k_{pitch}^{de} = +0.08$（飞翼Cm_de>0） |
| $\tau_{yaw}$ | 左右电机差动油门 | 绕 $z$ 轴偏航力矩 | $k_{yaw}^{T} = +0.02$ |

> **飞翼布局符号说明**：本机为飞翼布局，$\partial C_m / \partial \delta e > 0$（对称舵面下偏产生**抬头**力矩）。因此后仰（$\theta > 90°$）时需要**上偏**（$\delta e_{sym} < 0$）产生低头恢复力矩，故$k_{pitch}^{de}$取正值。
>
> **差动油门偏航**：左油门增大 → 左电机推力增大 → 由于$y_{motor\_left} < 0$，产生$N < 0$（机头向左偏航）。正偏航误差（机头偏右）需左油门增大，故$k_{yaw}^{T}$取正值时控制器内部为$\Delta T_{diff} = \tau_{yaw} \cdot k_{yaw}^{T}$，$T_{left} = T - \Delta T_{diff}$，$T_{right} = T + \Delta T_{diff}$。

### 4.7 控制器参数（保守初始值）

| 参数 | 值 | 说明 |
|------|-----|------|
| $K_p^{pos}$ | 0.80 | 位置比例增益 |
| $K_i^{pos}$ | 0.10 | 位置积分增益 |
| $K_d^{pos}$ | 0.60 | 位置微分增益（速度阻尼） |
| $K_p^{alt}$ | 0.065 | 高度比例增益 |
| $K_d^{alt}$ | 0.14 | 高度微分增益 |
| $K_p^{att}$ | 8.0 | 姿态比例增益 |
| $K_p^{rate}$ | 1.5 | 角速度比例增益 |
| $K_d^{rate}$ | 0.05 | 角速度微分增益 |
| 最大倾斜角 | 15° | 位置环输出限幅 |
| 最大舵面偏角 | 20° | 控制分配输出限幅 |
| 最大期望角速度 | 60°/s | 姿态环输出限幅 |

---

## 5. 线性化分析

### 5.1 数值雅可比计算

使用中心差分法计算状态矩阵$\mathbf{A}$和控制矩阵$\mathbf{B}$：

$$A_{i,j} = \frac{f(\mathbf{x}_0 + \epsilon\mathbf{e}_j, \mathbf{u}_0) - f(\mathbf{x}_0 - \epsilon\mathbf{e}_j, \mathbf{u}_0)}{2\epsilon}$$

**注意**：当前`linearize_analyze.py`假设12状态（欧拉角）系统，与13状态四元数模型不直接兼容。建议在四元数配平点附近进行小扰动线性化，或导出等效12状态模型后再分析经典模态。

### 5.2 模态分析

#### 纵向模态
- **短周期模态**：快速俯仰振荡，主导变量$q$和$\theta$
- **长周期模态**：速度和高度缓慢振荡，主导变量$u$和$h$

#### 横侧向模态
- **滚转衰减**：快速滚转收敛，主导变量$p$和$\phi$
- **荷兰滚**：耦合滚转-偏航振荡，主导变量$\beta, p, r, \phi$
- **螺旋模态**：缓慢滚转-偏航发散或收敛

> **尾座式悬停特殊性**：悬停时（$V \approx 0$）传统固定翼模态概念不再适用。飞机呈现类似倒立摆的不稳定特性，需依赖主动控制（升降副翼+差动油门）维持稳定。线性化应在悬停配平点（$\theta = 90°$）进行，分析姿态/高度/水平位置的耦合特性。

---

## 6. 仿真与控制性能

### 6.1 积分方法（RK4）

```
k1 = f(t_n, x_n, u_n)
k2 = f(t_n + h/2, x_n + h/2*k1, u_n)
k3 = f(t_n + h/2, x_n + h/2*k2, u_n)
k4 = f(t_n + h, x_n + h*k3, u_n)
x_{n+1} = x_n + h/6*(k1 + 2k2 + 2k3 + k4)
```

每步后重新归一化四元数：$\mathbf{q} = \mathbf{q} / |\mathbf{q}|$。

### 6.2 控制限制

$$|\tilde{\theta}_{pitch}|, |\tilde{\phi}_{roll}| \leq 15° \quad \text{（最大倾斜角）}$$
$$|\delta e_{left}|, |\delta e_{right}| \leq 20° \quad \text{（最大舵面偏角）}$$
$$|T_{left}|, |T_{right}| \leq 1.0 \quad \text{（最大油门）}$$

---

## 7. 实现说明

### 7.1 气动数据文件
- `aerodata_lon.xlsx`：纵向气动系数（$\alpha, C_L, C_D, C_m$）
- `aerodata_lat.xlsx`：横侧向气动系数（$\alpha, \beta, C_Y, C_l, C_n$）
- `aerodata_de.xlsx`：升降副翼增量系数（$\delta e, \Delta C_L, \Delta C_D, \Delta C_m$）
- `aerodata_throttle.xlsx`：推力-油门关系

### 7.2 大气模型（ISA）
标准大气模型，计算温度、压力、密度随高度变化。

### 7.3 关键代码文件
1. `aircraft_6dof.py`：核心动力学模型（13状态四元数、双动压气动、配平）
2. `hover_pid_controller.py`：悬停级联PID控制器
3. `hover_simulation.py`：悬停仿真主程序（RK4积分、绘图）
4. `hover_trim_manual.py`：配平验证
5. `linearize_analyze.py`：前飞模态分析（需适配四元数状态）

---

## 8. 验证清单

| 验证项 | 命令 | 期望结果 |
|--------|------|----------|
| 配平残差 | `python hover_trim_manual.py` | `max\|dx[0:9]\| < 1e-3` |
| 悬停稳定性 | `python hover_simulation.py` | 20s内位置误差收敛至 `<0.5m` |
| 极性验证 | `python test_compute_polarity.py` | 全部通道[OK] |
| 四元数归一化 | 检查仿真输出 `norm(q)` | 全程保持 `1.0 ± 1e-6` |

---

*本文档基于2026-04-26的代码快照编写。*
