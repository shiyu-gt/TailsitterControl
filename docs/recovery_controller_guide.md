# RecoveryController 控制逻辑详解

> 文件：`controllers/recovery_controller.py`
> 更新：2026-05-17

---

## 1. 概述

RecoveryController 是回收过渡段的核心控制律，负责将飞机从巡航状态（V≈20 m/s, theta≈5°, h=50m）引导至悬停状态（V≈0, theta≈90°, h=20m）。

**输入**：飞行状态向量（13 维）、期望速度 V_des、期望高度 z_des、时间步长 dt、当前阶段 stage

**输出**：期望俯仰角 theta_des、期望滚转角 phi_des（恒为 0）、期望油门 throttle_des；Stage B 时额外输出升降副翼直接偏角 `_de_override`

**接口**：
```python
rec_ctrl = RecoveryController(cruise_theta, cruise_throttle, hover_throttle,
                               V_cruise=20.0, h_cruise=50.0, h_hover=20.0, t_rec=15.0)
V_des, z_des, stage = rec_ctrl.get_profile(t_rec, V, pz)
theta_des, phi_des, throttle_des = rec_ctrl.compute(state, V_des, z_des, dt, stage)
de_override = rec_ctrl._de_override  # Stage B 时非 None
```

---

## 2. 构造函数参数

| 参数 | 含义 | 来源 |
|------|------|------|
| `cruise_theta` | 巡航配平俯仰角（~5°） | `ac.trim(V=20)` 返回 |
| `cruise_throttle` | 巡航配平油门（~0.057） | `ac.trim(V=20)` 返回 |
| `hover_throttle` | 悬停配平油门（~0.634） | `ac.trim(V=0)` 返回 |
| `V_cruise` | 巡航速度（20 m/s） | 任务定义 |
| `h_cruise` | 巡航高度（50 m，NED） | 任务定义 |
| `h_hover` | 悬停高度（20 m，NED） | 任务定义 |
| `t_rec` | 回收时间预算（120 s） | 任务定义 |

---

## 3. 内部状态与参数

### 3.1 速度环 PID（控制俯仰修正 gamma_des）

```
Kp_vel = 0.1        比例增益
Ki_vel = 0.005      积分增益
Kd_vel = 0.02       微分增益
int_vel = 0.0       积分器状态
max_int_vel = 1.0   积分限幅
gamma_max = 20°     Stage A 最大修正角
gamma_max_B = 8°    Stage B 最大修正角（当前未使用，Stage B 速度环禁用）
```

### 3.2 Alpha 保护

```
alpha_warn = 20°    保护触发阈值
alpha_max = 25°     保护极限（smoothstep 终点）
_int_vel_frozen     积分器冻结标志
```

### 3.3 高度环 PID（控制油门）

```
Kp_alt = 0.005      比例增益
Ki_alt = 0.001      积分增益
Kd_alt = 0.02       微分增益
int_alt = 0.0       积分器状态
max_int_alt = 0.15  积分限幅
```

### 3.4 俯仰角限制

```
theta_min = -5°     最小俯仰角
theta_max = 88°     最大俯仰角
max_theta_rate = 30°/s  俯仰角速率限幅
```

### 3.5 油门限制

```
throttle_floor = 0.0
throttle_ceil = 1.0
```

---

## 4. 阶段切换逻辑 — `get_profile()`

### 4.1 切换条件

```
输入: t_rec（已用回收时间）, V_current（当前空速）, pz（当前高度）

回退触发:
  time_fallback = (t_rec >= t_rec_param * 0.8)     # 80% 时间用完
  alt_fallback  = (pz - h_cruise) > 15.0           # 下降超过 15m

切换逻辑（带迟滞）:
  if V <= 3.0 → Stage C
  elif 当前 A 且 (V <= 12.0 或 time_fallback 或 alt_fallback) → Stage B
  elif 当前 B/C:
    V <= 3.0 → Stage C
    否则保持 B
  else → Stage A
```

**迟滞机制**：一旦进入 B 或 C，不会退回 A。防止在 V_break_A 附近反复切换。

### 4.2 期望速度 V_des

```
Stage A:  V_des = V_cruise + (12.5 - V_cruise) * min(t_rec / 50.0, 1.0)
          线性从 20 m/s 降至 12.5 m/s，50s 内完成

Stage B:  V_des = max(V_current - 0.5, V_break_B)
          跟踪当前速度，保留 0.5 m/s 误差驱动微调

Stage C:  V_des = max(0.0, 3.0 - t_rec * 0.5)
          线性降至 0

所有阶段: V_des = max(V_des, 0.0)
```

**设计意图**：Stage A 的 V_des 比较保守（12.5 m/s），匹配飞机实际减速能力（~0.1 m/s²）。Stage B 跟踪当前速度，避免产生大的速度误差导致积分器饱和。

### 4.3 期望高度 z_des

```
Stage A:  z_des = h_cruise（保持 50m）

Stage B:  t_stage_B = t_rec * 0.4
          progress = clip(t_rec / t_stage_B, 0, 1)
          z_des = h_cruise + progress * (h_hover - h_cruise)
          从 50m 线性降至 20m

Stage C:  z_des = h_hover（20m）
```

---

## 5. 核心控制律 — `compute()`

### 5.1 整体流程

```
输入: state, V_des, z_des, dt, stage

Step 1: 阶段切换检测 → 重置积分器
Step 2: 计算 theta_base（阶段基准俯仰角）
Step 3: 速度环 → gamma_des（俯仰修正）
Step 4: Alpha 保护（限制抬头）
Step 5: theta_des = theta_base + gamma_des（限幅 + 速率限制）
Step 6: 高度环 → throttle_des
Step 7: Stage B 直接升降副翼控制
Step 8: 暴露诊断量

输出: theta_des, phi_des, throttle_des
副作用: self._de_override（Stage B 时设置）
```

### 5.2 Step 1: 阶段切换检测

```python
if self._prev_stage != stage:
    self.int_vel = 0.0          # 重置速度积分器
    self.vel_first_call = True  # 重置微分项
self._prev_stage = stage
```

**目的**：防止 Stage A 的积分器累积值（可能很大）在切换到 Stage B 时产生不期望的 gamma 偏移。

### 5.3 Step 2: 阶段基准俯仰角 theta_base

**Stage A**：
```python
theta_base = cruise_theta  # ~5°
```
保持巡航俯仰角，速度环通过 gamma_des 微调。

**Stage B**：
```python
progress = (V_break_A - V) / (V_break_A - V_break_B)  # 12→3 m/s 映射到 1→0
progress = clip(progress, 0, 1)
alpha_target = 18° + progress * (8° - 18°)  # 高速18°，低速8°
theta_base = alpha_target
```
直接将迎角目标设为俯仰基准。假设航迹角 gamma≈0（level flight），则 alpha ≈ theta。

**物理含义**：
- V=12 m/s（Stage B 入口）：alpha_target=18°，大迎角产生大阻力
- V=3 m/s（Stage B 出口）：alpha_target=8°，小迎角准备过渡到 Stage C
- 中间线性插值

**Stage C**：
```python
progress = max(0, 1 - V / V_break_B)  # 3→0 m/s 映射到 0→1
theta_base = 45° + progress * (85° - 45°)
```
从 45° 渐变到 85°，建立悬停姿态。

### 5.4 Step 3: 速度环 → gamma_des

```python
V_err = V - V_des  # 正值 = 比期望快 = 需要抬头减速

# 微分项
dV_dt = (V_err - last_V_err) / dt  # 首次调用时为 0

# Stage B: 速度环完全禁用
if stage == 'B':
    gmax = 0
    Kp_v, Ki_v, Kd_v = 0, 0, 0
else:
    gmax = gamma_max  # 20°
    Kp_v, Ki_v, Kd_v = Kp_vel, Ki_vel, Kd_vel

# PID 计算
gamma_raw = Kp_v * V_err + Ki_v * int_vel + Kd_v * dV_dt

# 条件积分（gamma_raw 在限幅内且积分器未冻结时才累积）
int_cap = 0.1 if stage == 'B' else max_int_vel
if -gmax < gamma_raw < gmax and not int_vel_frozen:
    int_vel += V_err * dt
    int_vel = clip(int_vel, -int_cap, int_cap)

gamma_des = clip(gamma_raw, -gmax, gmax)
```

**Stage A 行为**：
- V > V_des（飞得太快）→ V_err > 0 → gamma_des > 0 → theta 增大 → 抬头减速
- V < V_des（飞得太慢）→ V_err < 0 → gamma_des < 0 → theta 减小 → 低头加速
- 限幅 ±20°，防止过度修正

**Stage B 行为**：速度环禁用（Kp=Ki=Kd=0），gamma_des 恒为 0。俯仰完全由 theta_base（alpha 目标）驱动。

### 5.5 Step 4: Alpha 保护

```python
alpha = arctan2(w, u)  # 机体轴迎角

if alpha > alpha_warn (20°):
    int_vel_frozen = True        # 冻结积分器
    x = (alpha - alpha_warn) / (alpha_max - alpha_warn)  # 0→1
    fade = 3x² - 2x³            # smoothstep
    gamma_floor = -gmax * fade   # 负值 = 低头方向
    gamma_des = max(gamma_des, gamma_floor)

elif alpha < alpha_warn - 2° (18°):
    int_vel_frozen = False       # 解冻积分器（2° 迟滞）
```

**smoothstep 曲线**：

```
alpha:    20°    21.25°   22.5°    23.75°   25°
fade:     0.0    0.156    0.500    0.844    1.0
gamma_floor: 0°   -3.1°    -10°     -16.9°   -20°
```

alpha=20° 时无影响，alpha=25° 时强制 gamma_des = -20°（最大低头修正）。

**积分器冻结**：保护触发时冻结积分器，防止保护解除后积分器累积值导致再次触发。

**迟滞恢复**：alpha 必须降到 18° 以下才解冻，避免在阈值附近反复冻结/解冻。

### 5.6 Step 5: theta_des 计算

```python
theta_des_raw = theta_base + gamma_des

# 绝对限幅
theta_des = clip(theta_des_raw, theta_min=-5°, theta_max=88°)

# 速率限制
if last_theta_des is not None:
    dtheta = theta_des - last_theta_des
    max_dtheta = max_theta_rate * dt  # 30°/s * 0.01s = 0.3°/步
    if |dtheta| > max_dtheta:
        theta_des = last_theta_des + sign(dtheta) * max_dtheta

last_theta_des = theta_des
```

**速率限制作用**：防止阶段切换时 theta_des 阶跃（如从 Stage A 的 5° 跳到 Stage B 的 18°），给姿态环平滑跟踪的时间。30°/s 对应从 5° 到 18° 需要 0.43s。

### 5.7 Step 6: 高度环 → throttle_des

```python
h_err = pz - z_des  # NED: pz 向下为正，h_err>0 表示低于目标

# 微分项
h_rate = (h_err - last_alt_error) / dt  # 首次调用时为 0

# 各阶段油门策略
Stage A:
    throttle_base = 0.03
    Kp_a, Ki_a, Kd_a = 0.001, 0.0002, 0.005  # 极小增益

Stage B:
    progress = (V_break_A - V) / (V_break_A - V_break_B)
    throttle_base = 0.03 + progress * (hover_throttle - 0.03)
    throttle_base = min(throttle_base, cruise_throttle)  # 关键：不超过巡航油门
    Kp_a, Ki_a, Kd_a = Kp_alt*0.3, Ki_alt*0.1, Kd_alt*0.3

Stage C:
    throttle_base = hover_throttle
    Kp_a, Ki_a, Kd_a = Kp_alt, Ki_alt, Kd_alt
```

**Stage A 油门修正**：
```python
throttle_corr = Kp_a*h_err + Ki_a*int_alt + Kd_a*h_rate
throttle_corr = min(throttle_corr, 0.0)  # 只允许减油门
throttle_corr = max(throttle_corr, -0.03)  # 最低到 0
throttle_unsat = 0.03 + throttle_corr      # 结果在 [0, 0.03] 范围
```
油门只减不增，防止爬升。高度高于目标（h_err<0）时减油门加速下降。

**Stage B 油门修正**：
```python
throttle_corr = Kp_a*h_err + Ki_a*int_alt + Kd_a*h_rate
max_corr = max(throttle_base * 0.3, 0.03)
throttle_corr = clip(throttle_corr, -max_corr, max_corr)
throttle_unsat = throttle_base + throttle_corr
```
修正量限制在 throttle_base 的 30% 以内，防止高度环大幅改变油门。

**积分器条件累积**：
```python
if throttle_floor < throttle_unsat < throttle_ceil:
    int_alt += h_err * dt
    int_alt = clip(int_alt, -max_int_alt, max_int_alt)
```
仅在油门未饱和时累积，防止积分器 windup。

**最终输出**：
```python
throttle_des = clip(throttle_unsat, 0.0, 1.0)
```

### 5.8 Step 7: Stage B 直接升降副翼控制

```python
if stage == 'B':
    # 从四元数提取当前俯仰角
    qx, qy, qz, qw = state[6:10]
    theta_current = arcsin(clip(2*(qw*qy - qz*qx), -1, 1))

    # P 控制器
    theta_err = theta_des - theta_current
    Kp_elevon = 10.0
    de_sym = Kp_elevon * theta_err
    de_sym = clip(de_sym, -30°, +30°)

    self._de_override = de_sym
else:
    self._de_override = None
```

**为什么绕过 HoverPID**：HoverPID 的姿态→角速度级联回路在 Stage B 的动态条件下响应太慢。直接 P 控制器（Kp=10）可以更快地命令升降副翼偏转。

**Kp_elevon=10.0 的含义**：theta_err=1° 时 de=10°，theta_err=3° 时 de=30°（饱和）。这是一个激进的增益，目的是尽快将 theta 跟踪到目标值。

**四元数到俯仰角**：`theta = arcsin(2*(qw*qy - qz*qx))`，这是从四元数提取俯仰角的标准公式（scalar-first 四元数，ZYX 欧拉角顺序）。

### 5.9 Step 8: 诊断量暴露

```python
self._last_gamma_des = gamma_des      # 速度环输出
self._last_theta_base = theta_base    # 阶段基准俯仰角
self._last_throttle_base = throttle_base  # 阶段基准油门
```

这些量在 `recovery_simulation.py` 中被记录为诊断数据，用于事后分析。

---

## 6. 三阶段控制策略总结

### Stage A（V > 12 m/s）— 低油门重力减速

```
                    ┌──────────────────────────────────┐
                    │         Stage A 控制流            │
                    │                                  │
  V_des (20→12.5) ─┤→ 速度环 PID → gamma_des (±20°)  │
                    │                                  │
  theta_base=5° ───┤→ theta_des = 5° + gamma_des      │
                    │                                  │
  z_des=50m ───────┤→ 高度环 PID → throttle (0~0.03)  │
                    │                                  │
  alpha > 20° ─────┤→ 保护: 冻结积分器 + 限制抬头      │
                    └──────────────────────────────────┘
```

**物理原理**：油门降至 0.03（远低于巡航 0.057），推力小于阻力。飞机在巡航俯仰角下自然减速。高度环增益极小，油门只减不增。

**预期行为**：V 从 20 降至 ~15 m/s，耗时 ~40s，高度基本保持（轻微爬升或下降）。

### Stage B（3 < V < 12 m/s）— Alpha 指令减速

```
                    ┌──────────────────────────────────┐
                    │         Stage B 控制流            │
                    │                                  │
  V_des=V-0.5 ─────┤→ 速度环禁用 (Kp=Ki=Kd=0)        │
                    │                                  │
  alpha_target ─────┤→ theta_base = alpha_target       │
  (18°→8°)          │   (不经过速度环)                  │
                    │                                  │
  z_des (50→20) ───┤→ 高度环 PID → throttle           │
                    │   throttle_base ≤ cruise_throttle │
                    │                                  │
  theta_des ────────┤→ 直接升降副翼 P 控制器            │
                    │   de = 10.0 * theta_err          │
                    │   绕过 HoverPID                   │
                    └──────────────────────────────────┘
```

**物理原理**：以中等迎角（10°-18°）飞行，阻力增大。油门低于巡航值，推力小于阻力，净减速。升降副翼直接控制俯仰，快速响应。

**关键约束**：`throttle_base ≤ cruise_throttle` 确保不产生净加速。

**预期行为**：V 从 ~15 降至 ~3 m/s，耗时 ~60-80s。高度从 50m 降至 20m。

### Stage C（V < 3 m/s）— 建立悬停

```
                    ┌──────────────────────────────────┐
                    │         Stage C 控制流            │
                    │                                  │
  V_des (3→0) ─────┤→ 速度环恢复 → gamma_des          │
                    │                                  │
  theta_base ──────┤→ 从 45° 渐变到 85°               │
  (45°→85°)        │                                  │
                    │                                  │
  z_des=20m ───────┤→ 高度环恢复全增益                 │
                    │   throttle = hover_throttle       │
                    │                                  │
  theta_des ────────┤→ HoverPID 全权控制                │
                    │   (无 de_override)                │
                    └──────────────────────────────────┘
```

**物理原理**：速度已低，逐步抬头至近悬停姿态。油门恢复到悬停值，HoverPID 接管姿态和位置控制。

**预期行为**：theta 从 ~10° 升至 ~85°，V 降至 < 1 m/s，pz 稳定在 20m。

---

## 7. 数据流图

```
get_profile(t_rec, V, pz)
    │
    ├──→ V_des, z_des, stage
    │
    ▼
compute(state, V_des, z_des, dt, stage)
    │
    ├── state[0:3] → V, alpha
    ├── state[6:10] → theta_current (四元数提取)
    ├── state[12] → pz
    │
    ├─→ theta_base (阶段基准)
    ├─→ gamma_des (速度环修正，Stage B 为 0)
    ├─→ theta_des = theta_base + gamma_des (限幅+速率限制)
    │
    ├─→ throttle_base (阶段基准油门)
    ├─→ throttle_corr (高度环修正)
    ├─→ throttle_des = throttle_base + throttle_corr (限幅)
    │
    ├─→ [Stage B] de_override = Kp_elevon * (theta_des - theta_current)
    │
    └─→ return (theta_des, 0.0, throttle_des)
```

---

## 8. 仿真调用方式

在 `recovery_simulation.py` 中的调用流程：

```python
def control(t, state, dt_step):
    V = sqrt(state[0]**2 + state[1]**2 + state[2]**2)

    # Step 1: 获取阶段和期望值
    V_des, z_des, stage = rec_ctrl.get_profile(t, V, pz=state[12])

    # Step 2: 计算控制输出
    theta_des, phi_des, throttle_des = rec_ctrl.compute(state, V_des, z_des, dt_step, stage)

    # Step 3: 检查是否有直接升降副翼覆盖
    de_override = rec_ctrl._de_override

    if de_override is not None:
        # Stage B: 直接使用升降副翼，绕过 HoverPID
        u_out = [throttle_des, throttle_des, de_override, de_override]
    else:
        # Stage A/C: 通过 HoverPID 内环
        q_des = euler_to_quaternion(phi_des, theta_des, 0.0)
        u_out = pid.compute(t, state, dt_step,
                            q_desired_override=q_des,
                            throttle_override=throttle_des)
    return u_out
```

---

## 9. 已知问题与调参方向

| 问题 | 现象 | 调参方向 |
|------|------|---------|
| Stage B 不减速 | V 卡在 15 m/s | 降低 throttle_base，或降低 V_break_A 使 Stage B 更早进入 |
| Stage B 振荡 | theta 来回摆动 | 降低 Kp_elevon，或降低 alpha_target |
| Alpha 保护频繁触发 | 大量 smoothstep 干预 | 降低 alpha_warn，或降低 gamma_max |
| 高度大幅下降 | pz 偏离目标 > 15m | 增大 Stage A 高度环增益，或提高 throttle_base |
| Stage C 过渡不平滑 | theta 突变 | 降低 max_theta_rate |
