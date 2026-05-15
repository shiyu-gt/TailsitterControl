# 尾座式无人机回收仿真 — 项目架构

> 日期：2026-05-15
> 工作目录：`e:\workshop`

---

## 1. 项目目标

实现双旋翼尾座式无人机从**巡航状态（V≈20 m/s, h=50m）到悬停状态（V≈0, h=20m）**的回收过渡仿真。

关键指标：
- 终点空速 < 1 m/s，终点高度 20±2 m，终点位置误差 < 1 m
- 回收时间 < 150 s，最大迎角 < 25°
- 全程避免在高速时进入高迎角失控区

---

## 2. 系统架构

### 2.1 控制架构

```
RecoveryController（回收律，3 阶段策略）
    │
    ├─ Stage A/C: theta_des + throttle_des ──→ HoverPID（姿态内环）
    │                                            │
    │                                            ▼
    │                                    [thr_L, thr_R, de_L, de_R]
    │
    └─ Stage B: de_override ──→ 直接升降副翼（绕过 HoverPID）
                                [thr_L, thr_R, de_L, de_R]
```

**Stage A/C 路径**：RecoveryController 输出期望俯仰角和油门，通过 `q_desired_override` 和 `throttle_override` 注入 HoverPID，由 HoverPID 的姿态环+角速度环+控制分配生成执行器指令。

**Stage B 路径**：RecoveryController 直接计算升降副翼偏角（P 控制器），绕过 HoverPID 的级联回路，减少 Stage B 的响应延迟。

### 2.2 阶段切换

```
V > 12 m/s ─────────→ Stage A（低油门重力减速）
V ≤ 12 或回退触发 ──→ Stage B（alpha 指令抬头减速）
V ≤ 3 m/s ─────────→ Stage C（建立悬停）
```

回退触发条件：时间用完 80% 或高度下降超过 15m。带迟滞：一旦进入 B/C 不退回 A。

---

## 3. 核心模块

### 3.1 Aircraft6DOF (`core/aircraft_6dof.py`)

6DOF 刚体动力学 + 查表气动模型 + 配平求解器。

- 状态向量（13维）：`[u, v, w, p, q, r, qx, qy, qz, qw, px, py, pz]`
- 控制向量（4维）：`[throttle_left, throttle_right, de_left, de_right]`
- 气动模型：双动压（自由流 + 滑流），四张 Excel 查表
- 配平：Newton-Raphson 求解 `du=dw=dq=0`

### 3.2 HoverPID (`controllers/hover_pid_controller.py`)

四环级联悬停 PID：
- 外环：水平位置 → 倾斜四元数
- 高度环：独立油门 PD
- 中环：四元数误差 → 期望角速率
- 内环：角速率 → 舵面/差动油门

支持 `q_desired_override` 和 `throttle_override` 外部注入。

### 3.3 RecoveryController (`controllers/recovery_controller.py`)

三阶段回收律：

**Stage A**：低油门（0.03）减速，速度环 gamma 修正，alpha 保护。

**Stage B**：alpha 指令 theta（18°→8°），速度环禁用，直接升降副翼 P 控制器（Kp=10.0），油门低于巡航。

**Stage C**：theta 从 45° 渐变到 85°，恢复 HoverPID 全权控制。

---

## 4. 关键物理约束

### 4.1 力矩平衡硬约束

高迎角（alpha > 35°）时，气动低头力矩超过升降副翼最大抬头力矩：

| 状态 | M_aero | M_elevon (30°) | 缺口 |
|------|--------|----------------|------|
| alpha=40°, V=12 | -1.72 N·m | +1.40 N·m | -0.32 N·m |
| alpha=50°, V=12 | -2.50 N·m | +1.40 N·m | -1.10 N·m |

**结论**：alpha 必须控制在 25° 以下，否则不可控。

### 4.2 减速能力限制

在中等迎角（alpha < 25°）下，最大减速约 0.1-0.15 m/s²。从 20 m/s 减速到 0 需要 100-150 秒。

### 4.3 滑流失效边界

当 V < 12 m/s 且 throttle < 0.05 时，升降副翼效能显著下降（总动压不足）。

---

## 5. 调试指南

### 5.1 诊断流程

**第一步：控制侧**
在 `recovery_simulation.py` 的 `control()` 中监控：
- `V_des`, `z_des`, `stage` — 阶段切换是否正确
- `theta_des`, `throttle_des` — 控制指令是否合理
- `int_vel` — 积分器是否饱和

**第二步：执行侧**
监控 `de_left`, `de_right`：
- 持续 ±30° → 舵面饱和，力矩缺口或增益过高
- 正常范围 ±15° → 控制有效

**第三步：动力学侧**
计算力矩平衡：`M_total = M_aero + M_elevon`
若 `M_total` 持续为负且 |M_total| > 0.3 N·m → 状态不可控

### 5.2 常见问题速查

| 现象 | 最可能原因 | 排查位置 |
|------|-----------|---------|
| Stage B 不减速 | 油门高于巡航，净加速 | throttle_base 计算 |
| theta 不上升 | 升降副翼增益不足或力矩缺口 | Kp_elevon, M_aero vs M_elevon |
| alpha 振荡 | 保护 ↔ 升降副翼正反馈 | alpha_warn, Kp_elevon |
| 高度大幅下降 | Stage A 油门过低 | throttle_base |
| 积分器饱和 | V_des 过于激进 | V_des 剖面设计 |

---

## 6. 关键参数锚点

| 参数 | 值 | 位置 | 说明 |
|------|-----|------|------|
| V_break_A | 12.0 m/s | recovery_controller.py | A→B 切换速度 |
| V_break_B | 3.0 m/s | recovery_controller.py | B→C 切换速度 |
| alpha_warn | 20° | recovery_controller.py | 迎角保护阈值 |
| alpha_max | 25° | recovery_controller.py | 迎角保护极限 |
| Kp_elevon | 10.0 | recovery_controller.py | Stage B 升降副翼增益 |
| throttle_base (A) | 0.03 | recovery_controller.py | Stage A 固定油门 |
| gamma_max | 20° | recovery_controller.py | Stage A 速度环修正限幅 |
| max_theta_rate | 30°/s | recovery_controller.py | 俯仰角速率限幅 |
| hover_throttle | 0.634 | hover_pid_controller.py | 悬停油门（配平值） |
| cruise_throttle | 0.057 | trim(V=20) | 巡航油门（配平值） |
