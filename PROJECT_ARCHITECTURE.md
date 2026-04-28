# 尾座式无人机过渡飞行仿真 — 项目架构与工作状态

> 生成日期：2026-04-27
> 工作目录：`e:\workshop`
> 核心文件：`aircraft_6dof.py`, `hover_pid_controller.py`, `transition_simulation.py`

---

## 1. 项目目标

实现双旋翼尾座式无人机（tail-sitter VTOL）的完整三段式飞行仿真：

**悬停（竖直） → 前向过渡 → 巡航（水平） → 回收过渡 → 悬停回收**

关键指标：
- 巡航速度：20 m/s
- 巡航高度：50 m（NED 坐标系，z 向下为正）
- 悬停高度：20 m
- 终点位置误差 < 1 m，终点速度 < 1 m/s
- **全程避免在高速（V > 10 m/s）时进入高迎角区（alpha > 35°）**

---

## 2. 系统架构

### 2.1 控制律调度架构（三段式）

旧方案（增强型方案 B）已废弃——预先规划姿态角剖面并注入悬停 PID，导致飞机被强迫进入舵面无法驾驭的高迎角区。

新架构采用**控制律切换 + 空速混合**：

```
MissionPlanner
    │  输出：V_des(t), z_des(t)
    ▼
TransitionController
    ├─ HoverPID        (悬停律，复用已验证模块)
    ├─ ForwardController (前飞律，速度→俯仰，高度→油门)
    ├─ Blender         (四元数 SLERP + 油门线性混合)
    │
    ▼  q_blend, throttle_blend
HoverPID.compute(q_override, throttle_override)
    │  执行姿态内环 + 角速度内环 + 控制分配
    ▼
    [throttle_L, throttle_R, de_L, de_R]
```

**核心原则**：任务规划器只输出目标空速和高度，不预先规划姿态角和油门。各控制律根据实时误差自主解算期望姿态与油门。

### 2.2 混合窗口设计

| 模式 | 条件 | blend | 控制行为 |
|------|------|-------|---------|
| **悬停** | `V ≤ 5 m/s` | `0` | 完全 HoverPID（位置外环→倾斜→竖直基准姿态） |
| **过渡** | `5 < V < 15 m/s` | 速度插值 | SLERP 混合两个控制律的期望姿态四元数 + 油门线性混合 |
| **前飞** | `V ≥ 15 m/s` | `1` | 完全 ForwardController（速度→俯仰，高度→油门） |

**特殊处理——前向过渡启动**：悬停初速 `V=0`，若纯按速度混合则 `blend=0`，ForwardController 永远没有机会影响姿态。因此**前向过渡（t < t_transition）采用时间混合**：`blend = smooth_ramp(t / t_transition)`，打破死锁。

---

## 3. 核心模块

### 3.1 HoverPID (`hover_pid_controller.py`)

已验证的悬停 PID 控制器，四元数姿态控制，机头朝上基准（theta ≈ 90°）。

**控制结构**：
- 外环：水平位置（x, y）→ 期望倾斜角
- 高度环：独立油门 PID
- 中环：姿态控制（四元数误差 → 旋转矢量）→ 期望角速度
- 内环：角速度控制 → 舵面/差动油门

**本次修改**：
- `compute()` 增加 `throttle_override=None` 参数。当传入非 None 时，完全绕过内部高度 PID，直接使用外部油门。
- `max_desired_rate` 设为可配置（默认 60°/s，过渡阶段由 TransitionController 覆盖为 360°/s）。

**关键参数锚点**：
```python
Kp_pos = 0.8, Ki_pos = 0.1, Kd_pos = 0.6
Kp_alt = 0.065, Kd_alt = 0.14
Kp_att = 8.0
Kp_rate = 1.5, Kd_rate = 0.05
pitch_to_elevator = 0.08      # 俯仰舵面效率
roll_to_de_diff = -0.02       # 滚转差动舵面
yaw_to_throttle_diff = 0.02   # 偏航差动油门
```

### 3.2 ForwardController (`transition_simulation.py`)

新增前飞控制律。

**纵向控制**：
- **速度环**：`V_err = V - V_des` → `theta_des = PID(V_err)`。限幅：`[-30°, +45°]`。
- **高度环**：`h_err = pz - z_des` → `throttle_des = throttle_trim + PID(h_err)`。限幅：`[0, 1]`。
- **条件积分**：仅当 `theta_des` 未饱和在 `theta_min/theta_max` 时才累加 `int_V`。

**横侧向**：保守策略，`phi_des = 0`（保持零滚转）。

**初始化**：`int_V` 预置为巡航配平俯仰角（来自 `ac.trim(V=20)`），避免从 90° 悬停启动时姿态突变。

**关键参数锚点**：
```python
Kp_vel = 0.05, Ki_vel = 0.005, Kd_vel = 0.01
max_int_vel = 2.0
theta_min = -30°, theta_max = +45°
Kp_alt = 0.01, Ki_alt = 0.001, Kd_alt = 0.05
max_int_h = 5.0
throttle_trim = cruise_throttle  # 来自 trim
```

### 3.3 Blender (`transition_simulation.py`)

混合逻辑位于 `TransitionController.compute()` 内。

**姿态混合——SLERP（强制球面线性插值）**：
```python
def _slerp(q1, q2, t):
    dot = np.dot(q1, q2)
    if dot < 0: q2 = -q2; dot = -dot
    if dot > 0.9995:
        return normalize(q1 + t * (q2 - q1))
    theta_0 = arccos(dot)
    sin_theta_0 = sin(theta_0)
    theta = theta_0 * t
    s0 = cos(theta) - dot * sin(theta) / sin_theta_0
    s1 = sin(theta) / sin_theta_0
    return s0*q1 + s1*q2
```

**油门混合**：线性插值 `throttle_blend = (1-blend)*throttle_hover + blend*throttle_fwd`。

**姿态变化率限幅**：`max_q_blend_rate = 3°/s`，防止混合输出阶跃导致角速度环饱和。

**积分器同步**：当 `blend < 0.1` 持续 3 步以上，将 `ForwardController` 的高度积分器等价同步到 `HoverPID.int_alt`，确保悬停回收后 HoverPID 内部状态不突变。

### 3.4 MissionPlanner (`transition_simulation.py`)

`get_profile(t)` 仅输出 `V_des, z_des`，不输出 `theta_ff/throttle_ff`。

**剖面**：
- 前向过渡（0–10 s）：`V_des` 0→20 m/s，`z_des` 20→50 m
- 巡航（10–20 s）：`V_des=20`, `z_des=50`
- 回收过渡（20–35 s）：`V_des` 20→0 m/s，`z_des` 50→20 m（15 s 时长，给速度自然衰减留出余量）

---

### 3.5 TransitionController.compute() 详细控制逻辑

`compute(t, state, dt)` 是 `transition_simulation.py` 的核心调度方法，每一步仿真调用一次。其内部执行顺序如下：

#### Step 1: 任务目标提取
```python
V_des, z_des = self.get_profile(t)
```
从任务规划器获取当前时刻的期望空速和期望高度。

#### Step 2: 状态提取与混合系数计算
```python
V = sqrt(u^2 + v^2 + w^2)
px, py, pz = state[10:13]
```

根据当前飞行阶段计算 `blend`：
- **前向过渡** (`t < t_transition`)：`blend = _smooth_ramp(t / t_transition)`。这是**时间混合**，用于打破 `V=0` 时的死锁。
- **巡航** (`t_transition ≤ t < t_transition + t_cruise`)：`blend = 1.0`（完全前飞）。
- **回收过渡** (`t_cruise_end ≤ t < t_total`)：`blend = clip((V - 5) / (15 - 5), 0, 1)`。这是**速度混合**，速度越低，悬停律权重越大。
- **末端悬停** (`t ≥ t_total`)：`blend = 0.0`。

#### Step 3: 悬停控制律外环 (`_compute_hover_outer`)
手动复现 HoverPID 的位置外环和高度环，得到 `q_hover_des` 和 `throttle_hover`。

**位置环（比例-only）**：
```python
pos_error_xy = -[px, py]
desired_accel_xy = Kp_pos * pos_error_xy  # 限幅 ±max_accel
tilt_roll  = clip(desired_accel_xy[1] / g, ...)
tilt_pitch = clip(-desired_accel_xy[0] / g, ...)
q_tilt = euler_to_quaternion(0, tilt_pitch, tilt_roll)
q_hover_des = normalize(q_hover_base ⊗ q_tilt)
```
注意：`q_hover_base` 当前设为 **60°**（而非 HoverPID 内部的 90°），目的是减小前向过渡的总行程，避免从竖直急剧低头。

**高度环（比例-only，限幅 ±0.2）**：
```python
h_err = pz - z_des
throttle_hover = hover_throttle + clip(0.02 * h_err, -0.2, 0.2)
```
此处故意弱化高度控制，防止过渡段因高度误差大而导致油门饱和。

#### Step 4: 前飞控制律 (`ForwardController.compute`)
```python
theta_fwd, phi_fwd, throttle_fwd = self.forward.compute(state, V_des, z_des, dt)
q_fwd_des = euler_to_quaternion(phi_fwd, theta_fwd, 0.0)
```
前飞律输出期望俯仰角 `theta_fwd`、滚转角 `phi_fwd`（当前固定为 0）、油门 `throttle_fwd`。

#### Step 5: 混合 (Blender)
```python
if blend <= 0.0:
    q_blend_target = q_hover_des
    throttle_blend = throttle_hover
elif blend >= 1.0:
    q_blend_target = q_fwd_des
    throttle_blend = throttle_fwd
else:
    q_blend_target = _slerp(q_hover_des, q_fwd_des, blend)
    throttle_blend = (1-blend)*throttle_hover + blend*throttle_fwd
```

**姿态变化率限幅**（`max_q_blend_rate = 3°/s`）：
若当前步的 `q_blend_target` 与上一帧 `_q_blend_last` 的夹角超过 `max_q_blend_rate * dt`，则用 SLERP 将姿态指令限制在该角度内：
```python
angle = arccos(dot(q_blend_last, q_blend_target))
if angle > max_angle:
    q_blend = _slerp(q_blend_last, q_temp, max_angle / angle)
```
这是为了防止 SLERP 在 blend 突变时产生阶跃式的姿态指令，导致角速度环饱和。

#### Step 6: 积分器同步
当 `blend < 0.1` 持续 3 步以上，认为即将进入纯悬停模式，将 `ForwardController` 的高度积分器等价映射到 `HoverPID.int_alt`：
```python
int_alt_eq = (throttle_blend - hover_throttle - Kp_alt*h_err - Kd_alt*alt_deriv) / Ki_alt
self.pid.int_alt = int_alt_eq
```
确保 HoverPID 从 `throttle_override` 切换回自主高度控制时，积分器状态不发生突变。

#### Step 7: 末端悬停模式切换
```python
use_hover_mode = (t >= t_total) or (V < 3.0 and blend < 0.05)
if use_hover_mode:
    self.pid.target_pos = [px, py, h_hover]
    return self.pid.compute(t, state, dt)  # 不使用 override
```
末端彻底回到 HoverPID 全权控制（位置外环 + 高度环 + 姿态内环），不再使用混合输出。

#### Step 8: 统一调用 HoverPID 内环
```python
self.pid.target_pos[2] = z_des
return self.pid.compute(t, state, dt,
                        q_desired_override=q_blend,
                        throttle_override=throttle_blend)
```
HoverPID 内部绕过位置外环和高度环，仅执行姿态中环 + 角速度内环 + 控制分配。

---

## 4. 气动模型关键修改

### 4.1 Alpha 限幅

保持 `alpha` 硬限幅在 **90°**（用户明确禁止恢复为 45°）。这意味着过渡段可以使用完整的高迎角气动数据。

### 4.2 Cm 软化（高迎角区间）

在 `aircraft_6dof.py::get_base_coefficients` 中，对 `|alpha| > 30°` 区间的 `Cm` 进行幅度缩减：

```python
# 30°->1.0, 45°->0.4, 线性过渡
if abs_a >= 45.0:
    scale = 0.4
else:
    scale = 1.0 - (abs_a - 30.0) / 15.0 * 0.6
Cm_b = Cm_b * scale
```

**动机**：原始 `Cm(40°)≈-0.13`、`Cm(50°)≈-0.19` 在过渡段产生 `-1.7 ~ -2.5 N·m` 的气动低头力矩，远超舵面最大抬头力矩（20° 舵面约 `0.94 N·m`）。这是导致回收过渡失控的硬物理约束。通过软化降低该力矩约 50–60%，试图恢复舵面可控裕度。

---

## 5. 当前工作状态

### 5.1 已验证通过

| 阶段 | 状态 | 说明 |
|------|------|------|
| 悬停 PID（阶段三） | ✅ 通过 | 单独验证，位置/高度/姿态控制稳定 |
| 架构实现 | ✅ 完成 | 三段式控制律、SLERP、混合逻辑、积分器同步均已编码 |
| 巡航段 | ✅ 基本稳定 | V≈20 m/s, θ≈5–10°, h≈50 m |
| 回收过渡 | ⚠️ 部分通过 | 最终可到达悬停姿态（V≈0.06 m/s, θ≈90°），但过程伴随大幅漂移 |

### 5.2 当前阻塞问题

**前向过渡严重失败**——飞机在 `theta≈30°–60°`、空速 `V≈5–15 m/s` 区间反复发生**俯冲翻转**（tumble/flip），伴随以下现象：

- 俯仰角冲过 0° 进入负值（机头下俯至 -60° ~ -90°）
- 高度严重超调（目标 50 m，实际峰值达 113 m，后修正至约 37 m 超调）
- 终点水平位置误差巨大（> 500 m）
- 最大迎角可达 179°（完全倒飞）

**根因判定**：这不是 PID 参数 tuning 问题，而是**物理力矩不匹配**。

在 `theta≈30°–60°`、低动压（滑流减弱）的过渡区：
- 重力力矩分量为正（使机头继续向下转）
- 气动低头力矩（负 `Cm`）叠加
- 舵面可用抬头力矩因滑流动压下降而不足

即使已将 `Cm` 高迎角段软化 60%，力矩缺口仍未完全消除。

### 5.3 已尝试的修复（未解决）

| 措施 | 效果 |
|------|------|
| 前向过渡改用时间混合 | 打破了 V=0 死锁，但姿态进入过渡区后仍失控 |
| 降低 HoverPID 基准姿态（90°→60°） | 减小了过渡总行程，未改变力矩平衡 |
| 增加 `max_desired_rate` 至 360°/s | 增大了角速度环输出限幅，未解决物理力矩不足 |
| 增加 `q_blend` 变化率限幅（6°/s→3°/s） | 缓解了指令阶跃，未解决根本力矩缺口 |
| 软化 `Cm(alpha)` 高迎角段（scale 0.4） | 降低了气动低头力矩，但仍未阻止翻转 |

---

## 6. 工作锚点

### 6.1 文件定位

| 文件 | 职责 | 关键行号/区域 |
|------|------|--------------|
| `aircraft_6dof.py` | 6DOF 动力学 + 气动模型 | `get_base_coefficients`（Cm 软化逻辑）；`compute_forces_moments`；alpha 限幅（~512 行） |
| `hover_pid_controller.py` | 悬停 PID（含内环） | `compute()`（throttle_override 注入点，~180–210 行）；控制分配（~280 行） |
| `transition_simulation.py` | 任务管理 + 前飞律 + 混合器 | `ForwardController` 类；`TransitionController.compute()`（混合逻辑）；`get_profile()`；`_slerp()`；`_compute_hover_outer()` |

### 6.2 关键接口

```python
# HoverPID 对外接口
hover_pid.compute(t, state, dt,
                  q_desired_override=q_blend,   # 混合后的期望姿态
                  throttle_override=throttle_blend)  # 混合后的油门

# ForwardController 对外接口
forward_ctrl.compute(state, V_des, z_des, dt)
# 返回: theta_des, phi_des, throttle_fwd

# TransitionController 对外接口
trans_ctrl.compute(t, state, dt)
# 返回: [throttle_L, throttle_R, de_L, de_R]
```

### 6.3 关键物理常数与约束

| 参数 | 值 | 备注 |
|------|-----|------|
| 巡航配平俯仰角 | ~5° | 来自 `ac.trim(V=20)` |
| 悬停基准俯仰角 | 90°（HoverPID 内部）/ 60°（TransitionController 外环复现） | 后者为过渡友好修正 |
| 舵面最大偏角 | ±20° | HoverPID 内限幅 |
| 混合速度窗口 | [5, 15] m/s | 回收过渡用；前向过渡用时间混合 |
| 最大指令姿态变化率 | 3°/s | Blender 内部限幅 |
| 前过渡时长 | 10 s | |
| 巡航时长 | 10 s | |
| 回收过渡时长 | 15 s | |

---

## 7. 调试思路与排查指南

### 7.1 前向过渡失败诊断流程

当前阻塞问题是**前向过渡俯冲翻转**。排查时应按以下顺序验证：

**第一步：确认指令侧是否正常**
在 `TransitionController.compute()` 末尾、`return` 之前插入打印：
```python
if t < self.t_transition:
    print(f"t={t:.2f} blend={blend:.3f} "
          f"theta_fwd={np.degrees(theta_fwd):.1f}° "
          f"q_blend_ang={np.degrees(2*arccos(q_blend[0])):.1f}° "
          f"thr_blend={throttle_blend:.3f}")
```
- 若 `theta_fwd` 从正值（抬头）突变为很大的负值（低头），说明 ForwardController 速度环在加速初期输出异常。检查 `V_err` 符号和 `int_V` 初值。
- 若 `q_blend_ang`（四元数旋转角）始终接近 90° 甚至超过 90°，说明混合后的期望姿态仍太竖直，ForwardController 的低头指令未有效传入。检查 `_slerp` 和 `q_blend` 限幅逻辑。

**第二步：确认执行侧（舵面）是否饱和**
在 `hover_pid_controller.py` 的 `compute()` 返回前插入打印：
```python
print(f"t={t:.2f} att_err=[{att_error[0]:.3f}, {att_error[1]:.3f}, {att_error[2]:.3f}] "
      f"rate_des=[{np.degrees(desired_rate[0]):.1f}, {np.degrees(desired_rate[1]):.1f}] "
      f"de=[{np.degrees(de_left):.1f}, {np.degrees(de_right):.1f}]")
```
- 若 `de_left/right` 持续打满 ±30°，说明力矩缺口确实存在（执行器已尽全力仍无法跟踪）。此时调参无效，必须修改物理模型（见 7.4 力矩验算）。
- 若舵面未饱和但姿态仍失控，说明控制极性错误（如 `pitch_to_elevator` 符号反了）或角速度环增益不足。

**第三步：确认动力学侧（气动力矩）**
在 `aircraft_6dof.py` 的 `compute_forces_moments` 中，返回前打印俯仰力矩分量：
```python
print(f"t={t:.2f} V={V:.1f} alpha={np.degrees(alpha):.1f}° "
      f"M_aero={M[1]:.3f} M_prop={M_prop[1]:.3f} M_total={M_total[1]:.3f} "
      f"q={state[4]:.3f}")
```
- 重点关注 `M_total[1]`（绕 y 轴力矩，抬头为正）。若该值持续为负且绝对值远大于 `M_prop[1]`，说明气动低头力矩占主导，舵面无法抵消。

### 7.2 关键监控指标

| 指标 | 正常范围 | 异常阈值 | 诊断意义 |
|------|---------|---------|---------|
| `blend` | 前过渡 0→1 平滑 | 在某值阶跃/振荡 | 混合逻辑或速度测量异常 |
| `theta_fwd` | 前过渡期应为负值（低头） | > 0°（抬头）或 < -45° | 速度环极性或积分器发散 |
| `q_blend` 旋转角 | 90° → 5° 单调递减 | 反向增大或跳变 | SLERP 或限幅逻辑错误 |
| `de_sym` (对称舵面) | ±20° 内，不持续饱和 | 持续 ±30° | 力矩缺口或姿态误差过大 |
| `alpha` | 过渡段 < 35° | > 45° 或 > 90° | 进入高迎角危险区，气动低头力矩剧增 |
| `V_err` | 巡航段 < ±2 m/s | 前过渡期 > ±50 m/s | 速度环未建立有效低头指令 |

### 7.3 调参方向速查表

| 现象 | 调整对象 | 方向 | 备注 |
|------|---------|------|------|
| 前过渡加速过慢 | `ForwardController.Kp_vel` | 增大 | 每 0.01 约增加 0.6°/(m/s) |
| 前过渡 theta 突变（0→-30°） | `ForwardController.Kd_vel` | 增大 | 抑制空速变化率冲击 |
| 巡航速度稳态误差 | `ForwardController.Ki_vel` | 增大 / 放宽 `max_int_vel` | 积分器需积累巡航配平角 |
| 巡航高度波动 | `ForwardController.Kp_alt` | 降低 | 前飞时油门-高度耦合弱，增益宜小 |
| 巡航高度稳态误差 | `ForwardController.Ki_alt` | 增大 | 通常非主要问题 |
| 回收减速过慢 | `ForwardController.Kp_vel` | 增大 | 减速时 `V > V_des`，速度环输出正 theta_des（抬头） |
| 回收进入高速大迎角 | `V_cruise_blend` | 降低（如 12 m/s） | 提前让悬停律介入，避免前飞律在高速下命令抬头 |
| 末端悬停油门突变 | `blend_low_threshold` / 同步逻辑 | 提前触发（如 0.2） | 给 HoverPID 积分器更长的收敛时间 |

### 7.4 实时力矩验算代码

若怀疑特定状态点存在力矩缺口，可在仿真循环中插入以下代码进行实时验算：

```python
# 在 integrate_6dof_quaternion 的循环内，计算完 u[i-1] 后
ac_temp = Aircraft6DOF()
rho_temp, _ = isa_atmosphere(0.0)
# 复现当前状态的气动力
alpha_rad = np.arctan2(w, u)
q_inf = 0.5 * rho_temp * V**2
# 查表得 Cm（需根据实际 aero 对象调整）
Cm_base = np.interp(np.degrees(alpha_rad), ac.aero._lon_alpha, ac.aero._lon_Cm)
# 当前舵面偏角
de = (de_left + de_right) / 2.0
dCm_de = 0.0036  # /deg，需根据实际测量值调整
Cm_total = Cm_base + dCm_de * np.degrees(de)
# 力矩估算（使用参考值 S=0.5, c_bar=0.3）
S, c_bar = 0.5, 0.3
M_aero_est = Cm_base * q_inf * S * c_bar
M_ctrl_est = dCm_de * np.degrees(de) * q_inf * S * c_bar  # 注意：若使用滑流需换 q_inf
print(f"t={t:.2f} alpha={np.degrees(alpha_rad):.1f}° M_aero={M_aero_est:+.2f} "
      f"M_ctrl={M_ctrl_est:+.2f} 缺口={M_ctrl_est+M_aero_est:+.2f}")
```

**判定标准**：若 `缺口` 持续为负且绝对值 > 0.3 N·m，说明当前状态不可控。

### 7.5 常见失败模式速查

| 失败模式 | 典型症状 | 最可能原因 | 排查位置 |
|---------|---------|-----------|---------|
| **前过渡翻转** | theta 从 60°→-60°，alpha > 90°，高度冲顶 | 过渡区力矩缺口（重力+气动低头 > 舵面抬头） | `aircraft_6dof.py` Cm 曲线；`compute_forces_moments` 力矩输出 |
| **前过渡不加速** | V 始终 < 5 m/s，theta 在 80° 附近 | blend 死锁或 theta_fwd 未生效 | `TransitionController.compute` blend 计算；`_slerp` 输出 |
| **巡航振荡** | V 和 h 正弦波动，舵面周期性饱和 | 速度环/高度环增益过高或耦合 | `ForwardController` Kp_vel / Kp_alt |
| **回收俯冲** | theta 从 20°→-10°，不抬头 | 回收初期 blend=1，前飞律未命令抬头 | `ForwardController` V_err 符号；`int_V` 是否饱和 |
| **末端漂移** | 终点 V≈0 但 px/py > 10 m | 悬停切换前未充分减速，水平速度残留 | 末端切换条件 `V < 3.0`；HoverPID 位置外环收敛时间 |

---

## 8. 下一步可选路径

基于当前物理阻塞，若继续推进，有以下方向：

1. **进一步软化/重构高迎角 Cm**：当前仅对 `Cm` 做了标量缩放。可尝试在 `theta=45°–60°` 区间引入更强的补偿（例如将 `Cm` 符号反转或置零），但会显著改变飞翼的静稳定性。
2. **引入额外俯仰控制通道**：如矢量推力（螺旋桨倾转 ±10°）或鸭翼。需修改 `aircraft_6dof.py` 的控制输入维度和气动力矩计算。
3. **采用“减速滑行”前过渡策略**：不直接命令低头加速，而是在悬停状态下先水平侧倾（利用推力矢量产生水平分力），以低迎角、低 theta 变化率的方式建立前向速度。这要求重写前向过渡的混合逻辑。
4. **调整任务目标**：降低巡航速度（如 10–12 m/s）或提高巡航俯仰角（如 20°），减小过渡段的姿态变化量和力矩缺口。

---

*本文档记录了截至 2026-04-27 的代码状态、架构决策与物理阻塞分析。*
