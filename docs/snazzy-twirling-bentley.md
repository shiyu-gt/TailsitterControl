# 过渡飞行仿真 — 控制律切换与混合架构重构计划

## Context

旧方案（增强型方案B）在 `transition_simulation.py` 中全程复用 `HoverPID`，通过 `q_desired_override` 注入预先规划的姿态角剖面（`theta_ff` + 速度闭环修正）。该方案已被证实不可行，根因为：

- **前向过渡**：真实高迎角气动数据（alpha 限幅 90°）导致阻力与低头力矩剧增，高度/速度失控；而截断至 45° 是物理错误的妥协。
- **回收过渡**：高速大迎角状态下，气动低头力矩（`Cm≈-0.13~-0.19`，产生 `-1.7~-2.5 N·m`）远超舵面最大抬头力矩（`20°` 舵面仅 `0.94 N·m`），存在硬力矩缺口，与参数 tuning 无关。

**根本原因**：在低速/大姿态过渡区，用悬停姿态控制逻辑去追前飞速度目标，导致飞机被强迫进入舵面无法驾驭的高迎角区。必须根据飞行阶段使用物理适配的控制律。

---

## 新架构概述：三段式控制律调度

```
Mission Planner          只输出目标状态：V_des(t), z_des(t), y_des(t)
       │
       ▼
┌──────────────────────────────────────────────────────────────────────┐
│                      TransitionController                            │
│  ┌──────────────┐   ┌────────────────────┐   ┌──────────────────┐  │
│  │ HoverPID     │   │ ForwardController  │   │ Blender          │  │
│  │ (悬停律)     │   │ (前飞律)           │   │ (混合器)         │  │
│  │ q_hover_des  │   │ q_fwd_des          │   │ q_blend, thr_blend│ │
│  │ throttle_hov │   │ throttle_fwd       │   │                  │  │
│  └──────────────┘   └────────────────────┘   └──────────────────┘  │
│                              │                                       │
│                              ▼                                       │
│              HoverPID.compute(q_override, throttle_override)         │
│                              │                                       │
│                    [throttle_L, throttle_R, de_L, de_R]              │
└──────────────────────────────────────────────────────────────────────┘
```

**核心原则**：任务规划器只改变目标空速和高度，**不预先规划姿态角和油门**。各控制律根据实时误差自主解算期望姿态与油门，混合器按空速进行平滑过渡。

---

## 关键设计：空速混合窗口

| 模式 | 空速条件 | blend | 控制行为 |
|------|---------|-------|---------|
| **悬停** | `V ≤ V_hover = 5 m/s` | `0` | 完全使用 HoverPID（位置外环→倾斜→90°基准姿态） |
| **过渡** | `5 < V < V_cruise = 15 m/s` | `(V-5)/10` | 对两个控制律的**期望姿态四元数**和**油门**进行球面/线性混合 |
| **前飞** | `V ≥ 15 m/s` | `1` | 完全使用前飞控制律（速度→俯仰，高度→油门） |

> **设计意图**：前飞律在 `V≥15 m/s` 时让飞机保持低姿态（`theta≈0~10°`）前飞；随着速度自然降低（回收段 `V_des` 下降），blend 逐渐减小，HoverPID 的悬停姿态权重增大，飞机**随动压降低而自然抬头**，避免在高速下强行进入高迎角区。

---

## 模块一：前飞控制律（ForwardController）

### 1.1 纵向控制律 — 速度→俯仰，高度→油门

输入：`state`, `V_des`, `z_des`, `dt`
输出：`theta_des`, `phi_des`, `throttle_des`, `q_fwd_des`

```python
# ── 速度环：生成期望俯仰角 theta_des ──
V_err   = V - V_des                     # 后项减去：V慢(V<V_des)→V_err负→theta_des负(低头加速)
int_V  += V_err * dt
int_V   = clip(int_V, -max_int_V, max_int_V)
dV_dt   = (V_err - last_V_err) / dt    # 空速变化率

theta_des = Kp_vel * V_err + Ki_vel * int_V + Kd_vel * dV_dt
theta_des = clip(theta_des, theta_min, theta_max)

# ── 高度环：生成总油门 throttle_des ──
# NED: z向下为正。飞机在目标下方(pz>z_des)→h_err>0→增大油门爬升
h_err    = pz - z_des
int_h   += h_err * dt
int_h    = clip(int_h, -max_int_h, max_int_h)
h_rate   = -w_vel                        # 爬升率（NED下 w 向下为正）

throttle_des = throttle_trim + Kp_alt * h_err + Ki_alt * int_h + Kd_alt * h_rate
throttle_des = clip(throttle_des, 0.0, 1.0)

# ── 横侧向：保持直线（y_des = 0），可选协调转弯 ──
# y_err = py - y_des  （右偏为正）
# phi_des = -Kp_lat * y_err  （右偏需左滚，phi_des为负）
phi_des = 0.0  # 保守起见，第一阶段保持零滚转

# ── 构建期望四元数 ──
# 前飞时 psi 保持当前航向或 0
psi_des = 0.0
q_fwd_des = euler_to_quaternion(phi_des, theta_des, psi_des)
```

### 1.2 前飞 PID 保守初值建议

前飞气动特性与悬停完全不同：速度对俯仰角敏感，高度对油门敏感，耦合较弱。参数需独立整定。

| 参数 | 初值 | 单位 | 物理意义 |
|------|------|------|---------|
| `Kp_vel` | **0.02** | rad/(m/s) ≈ 1.15°/(m/s) | 速度误差→俯仰角比例 |
| `Ki_vel` | **0.005** | rad/(m/s·s) | 积分消除稳态误差，自动积累配平俯仰角 |
| `Kd_vel` | **0.01** | rad·s/(m/s) | 空速变化阻尼，抑制振荡 |
| `max_int_V` | **2.0** | rad·s | 抗积分饱和 |
| `theta_min` | **-30°** | rad | 最大低头角（俯冲加速） |
| `theta_max` | **+45°** | rad | 最大抬头角（减速爬升） |
| `Kp_alt` | **0.02** | 1/m | 高度误差→油门比例（前飞时油门效率低，增益宜小） |
| `Ki_alt` | **0.001** | 1/(m·s) | 高度稳态误差消除 |
| `Kd_alt` | **0.05** | s/m | 爬升率阻尼 |
| `max_int_h` | **5.0** | m·s | 抗积分饱和 |
| `throttle_trim` | `cruise_throttle` | — | 平飞配平油门（来自 `ac.trim(V=20)`） |
| `Kp_lat` | **0.05** | rad/m | 横向位置误差→滚转角（如启用协调转弯） |

> **为什么 Ki_vel 必须存在**：前飞控制律不预设 `theta_ff`，稳态配平角完全由积分器提供。若 `Ki_vel=0`，`V=V_des` 时 `theta_des=0`，但真实巡航需要 `theta≈5°`，系统将产生稳态速度误差。

### 1.3 前飞控制律与悬停控制律的本质区别

| 特性 | 悬停律（HoverPID） | 前飞律（ForwardController） |
|------|-------------------|---------------------------|
| 姿态基准 | 固定竖直（90°） | 水平（0°），由速度误差动态调节 |
| 位置→姿态映射 | `px,py`→倾斜角（推力矢量控制） | 无（前飞位置由航向/滚转管理） |
| 高度执行器 | 油门（直接产生升力） | 油门（固定翼中控制能量/高度） |
| 速度执行器 | 无（悬停速度≈0） | 俯仰角（固定翼中控制速度） |
| 迎角范围 | 80°~100°（推力为主） | 0°~15°（气动升力为主） |

---

## 模块二：混合器（Blender）

### 2.1 混合系数计算

```python
V_hover  = 5.0   # m/s，低于此速度完全悬停
V_cruise = 15.0  # m/s，高于此速度完全前飞

blend = (V - V_hover) / (V_cruise - V_hover)
blend = clip(blend, 0.0, 1.0)
```

### 2.2 姿态混合（四元数 SLERP）

悬停律输出 `q_hover_des`（由 HoverPID 位置环生成，以 `q_hover=90°` 为基准叠加小倾斜）。  
前飞律输出 `q_fwd_des`（由 `theta_des` 和 `phi_des` 构建）。

```python
# 球面线性插值（SLERP）确保平滑旋转，避免欧拉角万向锁
dot = np.dot(q_hover_des, q_fwd_des)
if dot < 0:
    q_fwd_des = -q_fwd_des
    dot = -dot

if dot > 0.9995:
    # 过于接近，直接用线性插值
    q_blend = q_hover_des + blend * (q_fwd_des - q_hover_des)
    q_blend = q_blend / norm(q_blend)
else:
    theta_0 = arccos(dot)
    sin_theta_0 = sin(theta_0)
    theta = theta_0 * blend
    sin_theta = sin(theta)
    s0 = cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    q_blend = s0 * q_hover_des + s1 * q_fwd_des
```

> 若 SLERP 过于复杂，初期可退化为 **简单线性混合** `q_blend = normalize((1-blend)*q_hover_des + blend*q_fwd_des)`，在小角度混合误差可接受。

### 2.3 油门混合

```python
throttle_blend = (1 - blend) * throttle_hover + blend * throttle_fwd
```

### 2.4 高度目标同步

HoverPID 内部有高度控制，其 `target_pos[2]` 始终设为当前任务高度 `z_des`：
```python
pid.target_pos[2] = z_des
```
这样 `throttle_hover` 始终对应 `z_des` 的高度误差响应。

---

## 模块三：HoverPID 最小扩展

当前 `HoverPID.compute()` 已支持 `q_desired_override`。需再增加 `throttle_override`，使前飞律可以完全接管油门。

**修改 `hover_pid_controller.py`**（高度控制段）：

```python
def compute(self, t, state, dt=0.01, q_desired_override=None, throttle_override=None):
    ...
    # ============================================================
    # 2. 高度控制 → 油门
    # ============================================================
    if throttle_override is not None:
        throttle = throttle_override
    else:
        alt_error = pz - self.target_pos[2]
        if self._first_call or self.last_alt_error is None:
            alt_deriv = 0.0
        else:
            alt_deriv = (alt_error - self.last_alt_error) / dt if dt > 0 else 0.0
        self.last_alt_error = alt_error
        if abs(alt_error) < 0.05:
            self.int_alt *= 0.95
        self.int_alt += alt_error * dt
        self.int_alt = np.clip(self.int_alt, -self.max_int_alt, self.max_int_alt)
        throttle = (self.hover_throttle +
                    self.Kp_alt * alt_error +
                    self.Ki_alt * self.int_alt +
                    self.Kd_alt * alt_deriv)
        throttle = np.clip(throttle, 0.0, self.max_throttle)
    ...
```

**影响**：完全向后兼容。当 `throttle_override=None` 时行为不变。

---

## 模块四：任务管理器重构

移除旧方案中 `get_profile()` 输出的 `theta_ff` 和 `throttle_ff`。任务规划器仅输出目标状态。

```python
def get_profile(self, t):
    """只输出目标空速和高度，不规划姿态/油门"""
    t1 = self.t_transition       # 前过渡结束
    t2 = t1 + self.t_cruise      # 巡航结束
    t3 = t2 + self.t_transition_rec  # 回收结束

    if t < t1:
        r = self._smooth_ramp(t / t1)
        V_des = r * self.V_cruise
        z_des = self.h_hover + r * (self.h_cruise - self.h_hover)
    elif t < t2:
        V_des = self.V_cruise
        z_des = self.h_cruise
    elif t < t3:
        r = self._smooth_ramp((t - t2) / self.t_transition_rec)
        V_des = self.V_cruise * (1.0 - r)
        z_des = self.h_cruise - r * (self.h_cruise - self.h_hover)
    else:
        V_des = 0.0
        z_des = self.h_hover

    return V_des, z_des
```

> 回收段时长仍建议 `15 s`（而非 `10 s`），给速度自然衰减和混合过渡留出足够时间。

---

## 主控制循环（compute 方法）伪代码

```python
def compute(self, t, state, dt):
    # ── 1. 任务目标 ──
    V_des, z_des = self.get_profile(t)

    # ── 2. 当前状态提取 ──
    u, v, w = state[0:3]
    V = sqrt(u^2 + v^2 + w^2)
    px, py, pz = state[10:13]
    qx, qy, qz, qw = state[6:10]
    q_current = [qw, qx, qy, qz]

    # ── 3. 计算混合系数 ──
    blend = (V - self.V_hover) / (self.V_cruise - self.V_hover)
    blend = clip(blend, 0.0, 1.0)

    # ── 4. 悬停控制律（始终运行，用于混合） ──
    self.pid.target_pos = np.array([0.0, 0.0, z_des])  # 或保持最终悬停点
    # 让 HoverPID 计算它自己的 q_hover_des 和 throttle_hover
    # 注意：为了提取 q_hover_des，需要临时让 HoverPID 只算到姿态输出
    # 实现方式：HoverPID 新增一个方法 compute_attitude_desired() 返回 q_des, throttle
    # 或：在 TransitionController 中手动复现 HoverPID 的外环逻辑

    # 推荐：在 TransitionController 内手动实现 HoverPID 外环（位置→倾斜），
    #       直接得到 q_hover_des，避免两次调用 compute()

    # ── 5. 前飞控制律（始终运行，用于混合） ──
    theta_des, phi_des, throttle_fwd = self.forward_ctrl.compute(
        state, V_des, z_des, dt
    )
    q_fwd_des = euler_to_quaternion(phi_des, theta_des, 0.0)

    # ── 6. 混合 ──
    if blend <= 0.0:
        q_blend = q_hover_des
        throttle_blend = throttle_hover
    elif blend >= 1.0:
        q_blend = q_fwd_des
        throttle_blend = throttle_fwd
    else:
        q_blend = slerp(q_hover_des, q_fwd_des, blend)
        throttle_blend = (1-blend)*throttle_hover + blend*throttle_fwd

    # ── 7. 统一调用 HoverPID 内环 ──
    return self.pid.compute(t, state, dt,
                            q_desired_override=q_blend,
                            throttle_override=throttle_blend)
```

### 关于 HoverPID 外环的提取问题

`HoverPID.compute()` 在 `q_desired_override=None` 时会执行位置环并返回最终控制量。为了得到 `q_hover_des` 而不执行完整内环，有两种实现方式：

**方式A（推荐，最小侵入）**：在 `TransitionController.compute()` 中**手动复现** HoverPID 的位置环逻辑（仅约 20 行代码），直接计算 `q_hover_des` 和 `throttle_hover`（高度 PID 部分也复现）。这样不需要修改 `HoverPID` 的接口结构。

**方式B**：在 `HoverPID` 中拆分 `compute()` 为 `compute_outer_loop()`（返回 `q_desired, throttle`）和 `compute_inner_loop(q_desired, throttle)`（返回最终控制量）。但这会重构 `HoverPID` 内部，风险较高。

**本计划采用方式A**：TransitionController 内部自己计算悬停模式的 `q_hover_des`（位置误差→倾斜角→`q_hover ⊗ q_tilt`）和 `throttle_hover`（高度 PID），然后只把 `q_desired_override` 和 `throttle_override` 传给 `HoverPID.compute()` 来执行姿态+角速度内环+控制分配。

---

## 关键文件与修改点

| 文件 | 操作 | 说明 |
|------|------|------|
| `hover_pid_controller.py` | **修改** | `compute()` 增加 `throttle_override=None` 参数，允许外部注入油门 |
| `transition_simulation.py` | **重构** | 重写 `TransitionController`：移除 `theta_ff/throttle_ff` 轨迹规划；新增 `ForwardController` 类；实现混合逻辑；简化任务管理器 |
| `aircraft_6dof.py` | **不修改 alpha 限幅** | 保持 90°，依靠控制律避免进入高危迎角区 |

---

## 验证策略

**运行命令**：`python transition_simulation.py`

### 预期行为（成功标志）

| 阶段 | 预期现象 |
|------|---------|
| **前过渡 (0~10s)** | `V` 从 0 平滑上升至 20 m/s；`theta` 从 90° 随速度建立而自然下降至 ~5°；高度从 20m 爬升至 50m；全程 `alpha` 不超过 25°，舵面不饱和 |
| **巡航 (10~20s)** | `V` 稳定 20±2 m/s；高度稳定 50±2 m；`theta` 稳定在 0~10°；滚转角≈0 |
| **回收 (20~35s)** | `V_des` 下降 → 前飞律输出正 `theta_des`（抬头减速）；随着 `V` 降至 15 以下，blend 减小，悬停姿态权重增加；`theta` 平滑从 5° 经过 ~45° 过渡到 90°；**全程避免在 V>10 时进入 alpha>35°**；终点 `V<1 m/s`，位置误差 `<1 m` |

### 调参方向（若初次失败）

| 现象 | 根因推断 | 调整方向 |
|------|---------|---------|
| 前过渡加速过慢 | 速度环增益不足或低头角限幅过严 | 增大 `Kp_vel` 或放宽 `theta_min` |
| 前过渡高度超调 | 爬升率过高/油门前馈不足 | 降低 `Kp_alt`，检查 `throttle_trim` |
| 回收减速过慢 | 抬头力矩不足，前飞律抬头不够 | 增大 `Kp_vel`（减速时 V>V_des，产生正 theta_des） |
| 回收进入高速大迎角 | 混合窗口过宽或速度跟踪滞后 | 降低 `V_cruise` 混合上限（如 12 m/s），或增大 `Ki_vel` |
| 巡航速度稳态误差 | 积分器未积累足够配平角 | 增大 `Ki_vel` 或降低 `max_int_V` 限制 |
| 巡航高度波动 | 高度-油门耦合振荡 | 降低 `Kp_alt` 或增大 `Kd_alt` |

---

## 风险与应对

1. **混合区姿态振荡**：两个控制律输出差异巨大的四元数（90° vs 5°），线性混合可能产生非物理的中间姿态。应对：使用 SLERP 或确保混合过程中飞机实际动压已足够低（混合区主要在 `V<15`）。
2. **前飞律积分器初始化**：若 `int_V` 初始为 0，飞机从 90° 悬停启动前飞时，`theta_des` 会从 0° 开始（即命令水平姿态），造成姿态突变。应对：前过渡初期 `blend=0`（完全悬停），待速度自然建立到 `>5 m/s` 后 blend 才开始增大，前飞律此时已积累一定积分值；或给 `int_V` 预置巡航配平角初值。
3. **HoverPID 外环复现与内部状态一致性**：`TransitionController` 自己算 `throttle_hover` 时，HoverPID 内部的 `int_alt` 和滤波器状态不会同步更新。应对：由于最终油门由 `throttle_override` 注入，HoverPID 的高度环积分器在前飞/过渡阶段不会被使用；回到纯悬停（`blend=0`）后，HoverPID 内部积分器可能需要短暂时间重新收敛。可在 `blend` 接近 0 时提前几秒停止 `throttle_override`，让 HoverPID 自己接管。

---

*本计划基于对旧方案失败根因的力学分析，采用控制律切换+混合的标准航空工程实践，不再依赖姿态角预设剖面。*
