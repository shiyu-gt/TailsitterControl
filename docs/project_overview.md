# 尾座式无人机回收过程仿真 — 项目说明文档

> 版本：v2.0
> 日期：2026-05-15
> 工作目录：`e:\workshop`

---

## 1. 研究对象

### 1.1 飞行器概述

本项目研究对象为一台**双旋翼尾座式无人机（Twin-Rotor Tail-Sitter VTOL）**，采用飞翼布局，具备垂直起降与水平巡航能力。

**气动外形**：无尾飞翼，翼展 2.0 m，参考面积 0.62 m²，平均气动弦长 0.31 m，展弦比 6.45。

**推进系统**：双电动旋翼，分置于翼尖附近（左 -0.5 m，右 +0.5 m），螺旋桨直径 0.4064 m。推力沿机体 x 轴方向，通过油门（0-1）控制。

**操纵面**：左右升降副翼（elevon），位于螺旋桨滑流中。对称偏转产生俯仰力矩，差动偏转产生滚转力矩。偏转限幅 ±30°。

**质量与惯量**：质量 6.5 kg，转动惯量 Ix=0.1、Iy=0.183、Iz=2.8 kg·m²，Ixz=0。

**飞行模式**：
- **悬停模式**：机头朝上（theta ≈ 90°），双旋翼推力平衡重力
- **巡航模式**：机头前倾（theta ≈ 5°），水平前飞
- **回收过渡**：从巡航减速到悬停的非线性过渡过程（本项目焦点）

### 1.2 气动数据来源

气动系数通过四张 Excel 查表获取：

| 数据文件 | 内容 | 自变量 |
|---------|------|--------|
| `aerodata_lon.xlsx` | 纵向系数 CL, CD, Cm | alpha（迎角） |
| `aerodata_lat.xlsx` | 横侧系数 CY, Cl, Cn | alpha, beta（侧滑角） |
| `aerodata_de.xlsx` | 升降副翼增量 dCL, dCD, dCm | delta_e（舵面偏角） |
| `aerodata_throttle.xlsx` | 推力-油门映射 | throttle（0-1） |

### 1.3 双动压模型

气动计算采用双动压模型：
- **机体气动力**：使用自由来流动压 q_inf = 0.5 * rho * V²
- **升降副翼气动力**：使用滑流动压 q_slip（动量理论计算），通过混合因子 epsilon = 1/(1+(V/V_ref)²) 在 V_ref=15 m/s 处平滑过渡

滑流动压计算：
```
v_slip = V_inf + sqrt(2 * T / (rho * A_prop))
q_slip = 0.5 * rho * v_slip²
```

---

## 2. 实现目标

### 2.1 任务定义

从巡航配平状态（V≈20 m/s, theta≈5°, pz=50m）出发，仅仿真**回收过渡段**（平飞→悬停），目标到达悬停状态（V≈0, theta≈90°, pz=20m）。

```
巡航（V≈20 m/s, theta≈5°, h=50m）
    → 回收过渡（RecoveryController 三阶段）
    → 悬停（V≈0, theta≈90°, h=20m）
```

### 2.2 性能指标

| 指标 | 目标值 | 说明 |
|------|--------|------|
| 终点空速 | < 1.0 m/s | 接近悬停 |
| 终点高度 | 20 ± 2 m | 回到悬停高度 |
| 终点位置误差 | < 1 m | 水平+垂直综合 |
| 回收时间 | < 150 s | 从巡航到悬停 |
| 最大迎角 | < 25° | 避免失速/失控 |
| 最大高度偏移 | < 15 m | 回收段不应大幅掉高 |
| 水平航程 | < 200 m | 回收段不应飞太远 |

### 2.3 当前完成状态

| 模块 | 状态 | 说明 |
|------|------|------|
| 6DOF 动力学模型 | **已完成** | 含配平求解、气动查表、双动压模型 |
| HoverPID 悬停控制 | **已完成** | 四环级联，位置/高度/姿态稳定 |
| RecoveryController | **开发中** | 三阶段策略，Stage A 已验证，Stage B 减速能力待优化 |

---

## 3. 系统架构

### 3.1 回收仿真架构

```
┌─────────────────────────────────────────┐
│          RecoveryController              │
│   输入: V_des, z_des, stage             │
│   输出: theta_des, throttle_des         │
│         + Stage B: de_override          │
└────────────────┬────────────────────────┘
                 │
    ┌────────────┴────────────┐
    │                         │
    ▼ Stage A/C               ▼ Stage B
┌─────────────┐        ┌──────────────┐
│  HoverPID   │        │ 直接升降副翼  │
│ 姿态环+角速度│        │ P控制器      │
│ + 控制分配   │        │ 绕过HoverPID │
└──────┬──────┘        └──────┬───────┘
       │                      │
       └──────────┬───────────┘
                  ▼
   [throttle_L, throttle_R, de_L, de_R]
```

**Stage A/C**：RecoveryController 输出 theta_des 和 throttle_des，通过 q_desired_override 和 throttle_override 注入 HoverPID，由 HoverPID 的姿态环+角速度环+控制分配生成执行器指令。

**Stage B**：RecoveryController 直接输出升降副翼偏角（de_override），绕过 HoverPID 的姿态环和角速度环，减少响应延迟。

### 3.2 坐标系与状态向量

**坐标系**：NED（北-东-下），机体轴 x-前 y-右 z-下。

**13 维状态向量**：

| 索引 | 符号 | 含义 |
|------|------|------|
| 0 | u | 机体轴前向速度 [m/s] |
| 1 | v | 机体轴右侧速度 [m/s] |
| 2 | w | 机体轴下向速度 [m/s] |
| 3 | p | 滚转角速度 [rad/s] |
| 4 | q | 俯仰角速度 [rad/s] |
| 5 | r | 偏航角速度 [rad/s] |
| 6-9 | qx,qy,qz,qw | 姿态四元数（标量在后） |
| 10 | px | 北向位置 [m] |
| 11 | py | 东向位置 [m] |
| 12 | pz | 下向位置 [m]（高度 = -pz） |

**4 维控制向量**：

| 索引 | 符号 | 含义 |
|------|------|------|
| 0 | throttle_left | 左电机油门 [0, 1] |
| 1 | throttle_right | 右电机油门 [0, 1] |
| 2 | de_left | 左升降副翼偏角 [rad] |
| 3 | de_right | 右升降副翼偏角 [rad] |

### 3.3 核心模块文件

| 文件 | 职责 | 关键类/函数 |
|------|------|------------|
| `core/aircraft_6dof.py` | 6DOF 动力学 + 气动模型 + 配平求解 | `Aircraft6DOF`, `AeroModel`, `isa_atmosphere` |
| `core/integrator.py` | RK4 积分器（四元数归一化 + 溢出保护） | `integrate_6dof_quaternion` |
| `controllers/hover_pid_controller.py` | 四环级联悬停 PID | `HoverPID` |
| `controllers/recovery_controller.py` | 回收控制律（3 阶段策略） | `RecoveryController` |
| `simulations/recovery_simulation.py` | 回收段独立仿真 | `run_recovery_simulation` |
| `simulations/hover_simulation.py` | 悬停独立仿真 | `run_hover_simulation` |
| `analysis/recovery_envelope.py` | 回收减速能力包络线分析 | `run_envelope_analysis` |

---

## 4. 控制逻辑

### 4.1 HoverPID — 悬停控制器

四环级联 PID，适用于尾座式悬停（theta ≈ 90°）。在回收仿真中作为内环执行器使用。

```
位置外环 (x,y) ──→ 倾斜四元数 ──┐
                                 ├──→ q_hover_des
悬停基准姿态 (theta=90°) ────────┘
                                 │
高度环 (z) ──→ throttle_hover    │
                                 │
       q_hover_des ──→ 姿态环 ──→ 角速度环 ──→ 控制分配
                                                    │
                                    ┌───────────────┼───────────────┐
                                    ▼               ▼               ▼
                              对称升降副翼    差动升降副翼    差动油门
                              (俯仰)         (滚转)         (偏航)
```

**关键增益**：

| 环路 | Kp | Ki | Kd | 输出限幅 |
|------|----|----|-----|---------|
| 位置 (x,y) | 0.8 | 0.1 | 0.6 | 倾斜角 ±15° |
| 高度 (z) | 0.065 | 0 | 0.14 | 油门 [0, 1] |
| 姿态 | 8.0 | 0 | 0 | 角速率 ±60°/s |
| 角速度 | 1.5 | 0 | 0.05 | — |

**控制分配**：
- 俯仰 → 对称升降副翼：`de_sym = pitch_cmd * 0.08`
- 滚转 → 差动升降副翼：`de_diff = roll_cmd * (-0.02)`
- 偏航 → 差动油门：`thr_diff = yaw_cmd * 0.02`

**外部注入接口**（回收仿真中使用）：
- `q_desired_override`：绕过位置外环，直接注入期望姿态四元数
- `throttle_override`：绕过高度环，直接注入油门值

### 4.2 RecoveryController — 回收控制器

三阶段策略，从巡航（V≈20 m/s）过渡到悬停（V≈0）。

#### 阶段切换逻辑

```
V > 12 m/s ─────────→ Stage A（低油门减速）
V ≤ 12 或时间/高度回退 → Stage B（抬头减速）
V ≤ 3 m/s ─────────→ Stage C（建立悬停）
```

带迟滞：一旦进入 B/C 不退回 A。时间回退：80% 时间用完后强制进入 B。高度回退：下降超过 15m 后强制进入 B。

#### Stage A（V > 12 m/s）— 低油门减速

```
theta_base = cruise_theta（~5°）
V_des = 线性从 20 降至 12.5 m/s
z_des = h_cruise（保持 50m）
throttle_base = 0.03（远低于巡航油门 0.057）
速度环 PID → gamma_des（俯仰修正，限幅 ±20°）
theta_des = theta_base + gamma_des
```

油门修正只允许减小（不加推力），靠重力分量减速。alpha 保护在 alpha > 20° 时冻结积分器并限制抬头。

#### Stage B（3 < V < 12 m/s）— 抬头减速

```
alpha_target = 从 18° 渐变到 8°（速度高→低）
theta_base = alpha_target（gamma≈0 假设）
速度环禁用（Kp=Ki=Kd=0, gamma_max=0）
V_des = V_current - 0.5（跟踪当前速度）
z_des = 从 h_cruise 渐变到 h_hover
throttle_base = 从 0.03 渐变到 hover_throttle（上限 cruise_throttle）
```

**直接升降副翼控制**：绕过 HoverPID 姿态环，P 控制器直接驱动升降副翼：
```
theta_err = theta_des - theta_current
de_sym = Kp_elevon * theta_err（Kp=10.0，限幅 ±30°）
```

#### Stage C（V < 3 m/s）— 建立悬停

```
theta_base = 从 45° 渐变到 85°（速度高→低）
V_des = 从 3 降至 0
z_des = h_hover（20m）
throttle_base = hover_throttle
速度环恢复，高度环恢复全增益
```

#### Alpha 保护

```
alpha_warn = 20°, alpha_max = 25°
alpha > 20° → 冻结积分器 + smoothstep 限制抬头
alpha < 18° → 解冻积分器（2° 迟滞）
```

smoothstep 渐变：alpha=20° 时无强制，alpha=25° 时全力低头。

---

## 5. 调试思路

### 5.1 已完成的调试历程

| 阶段 | 内容 | 结果 |
|------|------|------|
| 悬停 PID 调试 | 四环级联极性/增益/稳定性验证 | 通过 |
| Stage A 调试 | 低油门减速策略、alpha 保护、积分器管理 | 通过（V 从 20 降到 15.3 m/s） |
| Stage B 调试 | alpha 目标、升降副翼直接控制、油门策略 | 进行中（减速能力不足） |

### 5.2 诊断方法

**控制侧诊断**：在 `recovery_simulation.py` 的 `control()` 函数中记录 V_des、z_des、stage、theta_base、gamma_des、throttle_base、int_vel 等中间量。

**执行侧诊断**：监控升降副翼偏角是否饱和（持续 ±30° 表示力矩缺口或增益过高）。

**动力学侧诊断**：计算当前状态下的气动俯仰力矩 M_aero 和舵面力矩 M_elevon，检查力矩平衡。

**力矩平衡验算公式**：
```
M_aero = Cm(alpha) * q_inf * S * c_bar
M_elevon = dCm * delta_e * q_eff * S * c_bar
M_total = M_aero + M_elevon
若 M_total 持续为负且 |M_total| > 0.3 N·m → 不可控
```

### 5.3 关键监控指标

| 指标 | 正常范围 | 异常阈值 | 诊断意义 |
|------|---------|---------|---------|
| alpha | < 20° | > 25° | 高迎角气动低头力矩剧增 |
| de_sym | ±20° 内 | 持续 ±30° | 舵面饱和，力矩缺口或增益过高 |
| V_err | ±2 m/s | > ±5 m/s | 速度环未有效跟踪 |
| theta | 平滑过渡 | 突变 > 10°/s | 控制指令阶跃或积分器发散 |
| throttle | 合理范围 | 阶跃 > 0.1 | 油门环失控 |

---

## 6. 当前问题

### 6.1 问题一：回收 Stage B 减速能力不足（核心阻塞）

**现象**：飞机进入 Stage B 后，速度稳定在 V≈15 m/s 附近，几乎不减速。Stage B 平均减速率仅 -0.006 m/s²。

**根因链**：

```
Stage B 入口条件: V=15.3 m/s（时间/高度回退触发，非速度阈值）
    │
    ├─ alpha_target 计算: progress=(12-15.3)/(12-3) = -0.37 → clip→ 0
    │   → alpha_target = 18°（始终固定，因 V > V_break_A）
    │
    ├─ 升降副翼 P 控制器: theta_err = 18° - 8.6° = 9.4°
    │   → de = 10.0 * 9.4° = 94° → 限幅 30°
    │   → 但实际 theta 仅上升到 ~10°（力矩平衡点）
    │
    └─ 净推力问题: throttle=0.065 > cruise_throttle=0.057
        → 推力 > 阻力 → 净加速 → 抵消了高 alpha 的减速效果
```

**关键矛盾**：
1. 油门 0.065 高于巡航油门 0.057，产生净加速
2. alpha=10° 的阻力不足以抵消额外推力
3. alpha 目标 18° 虽然在力矩可达范围内，但升降副翼无法快速将 theta 从 8.6° 推到 18°

### 6.2 问题二：物理减速能力的根本限制

包络线分析表明该飞机的减速能力存在硬约束：

| 速度 | 配平 alpha | 配平 throttle | 净水平加速度 |
|------|-----------|--------------|-------------|
| 20 m/s | 4.9° | 0.057 | ~0 m/s² |
| 15 m/s | 8.0° | 0.042 | ~-0.08 m/s² |
| 12 m/s | 10.8° | 0.027 | ~-0.12 m/s² |
| 8 m/s | 15.8° | 0.014 | ~-0.15 m/s² |

从 20 m/s 减速到 0 m/s，即使保持最优配平状态，也需要约 100-150 秒。任何额外推力（如维持高度或升降副翼效能）都会延长这一时间。

### 6.3 问题三：升降副翼效能与速度的耦合

升降副翼位于螺旋桨滑流中，其效能取决于滑流动压：

| 速度 | throttle | q_slip/q_inf | 有效动压 |
|------|----------|-------------|---------|
| 20 m/s | 0.057 | 1.15 | 高（自由流主导） |
| 15 m/s | 0.03 | 1.33 | 中（滑流贡献增大） |
| 10 m/s | 0.03 | 1.89 | 中低 |
| 5 m/s | 0.03 | 4.42 | 低（滑流主导，但 q_inf 小） |

低速时虽然滑流比增大，但总动压仍然很低，导致升降副翼力矩不足。这形成了一个两难：需要低油门减速，但低油门又降低了舵面效能。

---

## 7. 目录结构

```
e:\workshop\
├── core/                          # 核心仿真模块
│   ├── aircraft_6dof.py           #   6DOF 动力学 + 气动模型 + 配平求解
│   ├── integrator.py              #   RK4 积分器
│   └── __init__.py                #   导出接口
├── controllers/                   # 控制器
│   ├── hover_pid_controller.py    #   悬停 PID（四环级联，回收仿真内环）
│   ├── recovery_controller.py     #   回收控制律（3 阶段策略）
│   └── __init__.py                #   导出接口
├── simulations/                   # 仿真脚本
│   ├── recovery_simulation.py     #   回收段独立仿真
│   ├── hover_simulation.py        #   悬停仿真
│   └── __init__.py
├── analysis/                      # 分析脚本
│   ├── recovery_envelope.py       #   回收减速能力包络线
│   ├── linearize_analyze.py       #   线性化与模态分析
│   └── hover_trim_manual.py       #   悬停配平验算
├── tests/                         # 测试
│   └── test_polarity.py           #   控制极性验证
├── data/                          # 气动数据
│   ├── aerodata_lon.xlsx          #   纵向气动系数
│   ├── aerodata_lat.xlsx          #   横侧气动系数
│   ├── aerodata_de.xlsx           #   升降副翼增量
│   └── aerodata_throttle.xlsx     #   推力-油门映射
├── docs/                          # 文档
│   ├── project_overview.md        #   本文档
│   ├── PROJECT_ARCHITECTURE.md    #   架构与调试指南
│   └── recovery_analysis_report.md #  回收仿真分析
└── scripts/                       # 工具脚本
    ├── save.sh / save.bat         #   Git 自动保存
    └── restore.sh                 #   Git 恢复
```

---

## 8. 运行方式

```bash
# 回收过程仿真（主仿真）
python -m simulations.recovery_simulation

# 悬停仿真
python -m simulations.hover_simulation

# 减速能力包络线分析
python -m analysis.recovery_envelope

# 线性化与模态分析
python -m analysis.linearize_analyze

# 控制极性验证
python -m tests.test_polarity
```

所有脚本需从项目根目录 `e:\workshop` 运行。
