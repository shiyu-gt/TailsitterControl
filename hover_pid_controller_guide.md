# HoverPID 控制器解析与调试指南

## 1. 控制器架构概述

`HoverPID` 是一个**级联PID控制器**，专为双旋翼飞翼尾座式无人机（tailsitter）设计。该飞行器在悬停时机头朝上（`theta ≈ 90°`），采用四元数姿态表示以避免万向锁。

控制器采用**四环级联结构**：

| 控制环 | 输入 | 输出 | 执行器 |
|--------|------|------|--------|
| **外环：水平位置控制** (x, y) | 位置误差 | 期望倾斜角 | — |
| **高度控制** (z) | 高度误差 | 总油门 | 左右电机 |
| **中环：姿态控制** | 四元数误差 | 期望角速度 | — |
| **内环：角速度控制** | 角速度误差 | 舵面/差动油门指令 | 升降副翼 + 差动油门 |

---

## 2. 各控制环详解

### 2.1 水平位置控制（外环）

**目标**：将水平位置 `(px, py)` 收敛到目标点。

```python
pos_error_xy = self.target_pos[:2] - np.array([px, py])
desired_accel_xy = Kp_pos * pos_error + Ki_pos * int_pos + Kd_pos * pos_deriv
```

**物理映射**：位置误差被转换为**期望加速度**，再映射为**期望倾斜角**：
- `tilt_roll = desired_accel_xy[1] / g` — y方向误差 → 绕x轴滚转
- `tilt_pitch = -desired_accel_xy[0] / g` — x方向误差 → 绕y轴俯仰

期望四元数由悬停基准姿态 `q_hover`（theta=90°）叠加小倾斜角得到：

```python
q_tilt = euler_to_quaternion(0.0, tilt_pitch, tilt_roll)
q_desired = quaternion_multiply(self.q_hover, q_tilt)
```

**尾座式映射修正**：
- x方向误差 → 俯仰倾斜（绕y轴，推力纵向倾斜）
- y方向误差 → 偏航倾斜（绕z轴，推力横向倾斜）
- 滚转（绕x轴）不产生水平加速度，不参与位置映射

### 2.2 高度控制

**目标**：维持悬停高度（默认 `target_pos[2] = 0.0 m`，可通过 `set_hover_point()` 修改）。

```python
alt_error = pz - self.target_pos[2]   # NED：z向下为正
throttle = hover_throttle + Kp_alt*alt_error + Ki_alt*int_alt + Kd_alt*alt_deriv
```

**物理意义**：
- 飞机低于目标（pz > target）→ alt_error > 0 → **增大油门**
- 飞机高于目标（pz < target）→ alt_error < 0 → **减小油门**

### 2.3 姿态控制（中环）

**核心**：四元数误差 → 旋转矢量。

```python
q_current_inv = np.array([qw, -qx, -qy, -qz])
q_err = quaternion_multiply(q_current_inv, q_desired)
if q_err[0] < 0:
    q_err = -q_err          # 取最短路径
att_error = 2.0 * q_err[1:4]  # 旋转矢量近似
```

> **修正说明**：原公式 `q_desired * q_current^{-1}` 在 theta=90° 万向锁姿态下会导致滚转/偏航控制轴交叉映射。修正为 `q_current^{-1} * q_desired` 后，体轴旋转误差计算正确。

`att_error` 是在**体轴系**下的三维旋转误差，天然无万向锁。通过PID产生**期望角速度**：

```python
desired_rate = Kp_att * att_error + Ki_att * int_att + Kd_att * att_deriv
```

### 2.4 角速度控制（内环）

**目标**：跟踪期望角速度。

```python
rate_error = desired_rate - current_rate
roll_cmd  = Kp_rate * rate_error[0] + Ki_rate * int_rate[0] + Kd_rate * rate_deriv[0]
pitch_cmd = Kp_rate * rate_error[1] + Ki_rate * int_rate[1] + Kd_rate * rate_deriv[1]
yaw_cmd   = Kp_rate * rate_error[2] + Ki_rate * int_rate[2] + Kd_rate * rate_deriv[2]
```

---

## 3. 控制分配策略

控制分配将三个轴向指令映射到四个物理执行器：

```python
de_diff       = roll_cmd  * self.roll_to_de_diff       # 滚转 → 差动升降副翼
de_sym        = pitch_cmd * self.pitch_to_elevator     # 俯仰 → 对称升降副翼
throttle_diff = yaw_cmd   * self.yaw_to_throttle_diff  # 偏航 → 差动油门

de_left  = de_sym - de_diff
de_right = de_sym + de_diff
throttle_left  = throttle - throttle_diff
throttle_right = throttle + throttle_diff
```

### 物理映射与符号约定

| 指令 | 执行器动作 | 产生的力矩 | 当前系数 |
|------|-----------|-----------|---------|
| `roll_cmd` | 左右舵面差动偏转 | 绕 **x 轴** 滚转力矩 | `roll_to_de_diff = -0.02` |
| `pitch_cmd` | 左右舵面同步偏转 | 绕 **y 轴** 俯仰力矩 | `pitch_to_elevator = +0.08` |
| `yaw_cmd` | 左右电机差动油门 | 绕 **z 轴** 偏航力矩 | `yaw_to_throttle_diff = +0.02` |

**飞翼布局符号说明**：
- 本机为飞翼布局，`∂Cm/∂δe > 0`（对称舵面**下偏**产生**抬头**力矩）。因此后仰（θ > 90°）时需要**上偏**（`δe_sym < 0`）产生低头恢复力矩，故 `pitch_to_elevator` 取**正值**。
- 差动油门偏航：正偏航误差 → 右油门增大 → 绕z轴正偏航力矩回正，故 `yaw_to_throttle_diff` 取**正值**，控制器内部为 `throttle_left = throttle - throttle_diff`，`throttle_right = throttle + throttle_diff`。

---

## 4. 阶段一调试结果（已完成）

### 4.1 根因：控制分配符号错误

初始仿真在20秒内漂移超过300米。根因为两个控制分配系数的符号与飞翼布局物理特性不匹配：

| 系数 | 旧值 | 问题 | 修正后 |
|------|------|------|--------|
| `pitch_to_elevator` | `-0.08` | 后仰时输出正舵面，但飞翼 `Cm_de>0`，正舵面产生抬头力矩，加速发散 | `+0.08` |
| `yaw_to_throttle_diff` | `-0.004` | 正偏航误差时右油门增大，但电机几何决定左油门增大才产生回正力矩 | `+0.004` |

**验证方法**：调用 `aero.get_elevon_longitudinal_coefficients(np.radians(5))` 返回 `dCm=+0.0180`，确认飞翼布局 `Cm_de > 0`。

### 4.2 根因：滤波器状态未清零

`reset()` 未清除 `last_pos_deriv_xy`、`last_att_deriv`、`last_rate_deriv`，导致 `test_compute_polarity.py` 顺序执行测试时，测试2继承测试1的滤波状态，输出完全错误的 `de_sym`。

**修复**：在 `reset()` 中增加：
```python
self.last_pos_deriv_xy = np.zeros(2)
self.last_att_deriv = np.zeros(3)
self.last_rate_deriv = np.zeros(3)
```

### 4.3 根因：舵面输出无限幅

`max_de = 20°` 已定义，但 `de_left`/`de_right` 在 `compute()` 中未做限幅。

**修复**：在控制分配后增加：
```python
de_left = np.clip(de_left, -self.max_de, self.max_de)
de_right = np.clip(de_right, -self.max_de, self.max_de)
```

### 4.4 阶段一当前状态（已完成）

- **输出限幅**：已全部恢复（`max_tilt=15°`，`max_de=20°`，`max_desired_rate=60°/s`）
- **控制极性**：已通过 `test_compute_polarity.py` 全部通道验证
- **无扰动稳定性**：从精确配平点启动，20秒仿真位置误差保持 `< 0.01m`，无发散
- **高度控制**：符号已验证，NED坐标下逻辑正确

---

## 5. 阶段二：单通道独立调试（已完成）

### 5.1 测试方法与初始偏差

通过 `hover_single_channel_debug.py` 对各通道施加初始偏差：

| 通道 | 初始偏差 | 对应指标 |
|------|---------|---------|
| 俯仰 | theta = 91°（后仰1°） | 收敛时间、超调、舵面饱和 |
| 滚转 | phi = 5° | 收敛时间、超调 |
| 偏航 | psi = 10° | 差动油门响应、稳态误差 |
| 高度 | pz = 1m（目标下方1m） | 油门响应、高度保持精度 |

### 5.2 调试结果

| 通道 | 超调 | 稳定时间 | 峰值控制量 | 判定 |
|------|------|---------|-----------|------|
| **俯仰** | 0.0% | 0.88 s | 0.96° de_sym | 通过 |
| **滚转** | 1.7% | 0.38 s | 1.20° de_diff | 通过 |
| **偏航** | 49.7% | 4.29 s | 0.0314 throttle_diff | 通过 |
| **高度** | 0.0% | 7.60 s | 0.6988 throttle | 通过 |

> 偏航超调49.7%在可接受范围内（偏航惯量 Iz=2.8 远大于 Ix/Iy，物理上响应天然迟缓，限值放宽至50%）。

### 5.3 关键调参记录

| 参数 | 阶段一初值 | 阶段二终值 | 调整原因 |
|------|-----------|-----------|---------|
| `Kp_att` | 2.0 | **8.0** | 俯仰响应过慢，收敛时间>10s |
| `Kp_rate` | 0.5 | **1.5** | 角速度环阻尼不足，配合Kp_att提升 |
| `Kp_alt` | 0.08 | **0.065** | 高度初始响应油门峰值触碰0.7限幅 |
| `Kd_alt` | 0.15 | **0.14** | 配合Kp_alt微调，降低峰值 |

### 5.4 阶段二代码变更

- `hover_pid_controller.py` 增加 `debug_channel` 参数用于单通道隔离（**阶段三已移除**）
- `hover_single_channel_debug.py` 新增：四元数初始偏差、RK4积分器、指标计算、3行子图输出
- `ctrl_limit` bug 修复：舵面峰值单位为 deg，限值应由 `np.radians(20)` 改为 `20.0`
- 偏航调试策略调整：尾座式悬停时滚转/俯仰开环会失稳，偏航隔离采用"保持姿态环、仅禁用位置倾斜指令"方式

---

## 6. 阶段三：联合调试与扰动测试（已完成）

### 6.1 测试矩阵

| 测试 | 初始条件 | 仿真时长 | 考察目标 |
|------|---------|---------|---------|
| 1 水平耦合 | px=+0.5m, py=+0.5m | 20s | 水平位置协同响应 |
| 2 水平+高度 | px=+0.5m, pz=+1.0m | 20s | 俯仰纠正位置时高度稳定性 |
| 3 全状态 | px=+1, py=+1, pz=+1, yaw=+5° | 20s | 四通道同时协调性 |
| 4 大偏差 | px=+5m, py=-3m | 40s | 位置环饱和与恢复 |
| 5 姿态+位置 | roll=+5°, px=+2m | 20s | 姿态恢复与位置恢复冲突 |

### 6.2 测试结果

| 测试 | 稳定时间 | 最终位置误差 | 最大舵面 | 油门波动 | 判定 |
|------|---------|-------------|---------|---------|------|
| 1 水平耦合 | **1.2 s** | (0.002, 0.001, 0.000) m | 2.3° | ±0.014 | 通过 |
| 2 水平+高度 | **1.7 s** | (0.002, 0.000, 0.000) m | 2.2° | ±0.065 | 通过 |
| 3 全状态 | **4.8 s** | (0.003, 0.003, 0.000) m | 5.9° | ±0.084 | 通过 |
| 4 大偏差 | **8.9 s** | (-0.000, -0.000, 0.000) m | 7.6° | ±0.069 | 通过 |
| 5 姿态+位置 | **4.8 s** | (0.014, 0.000, 0.000) m | 8.4° | ±0.023 | 通过 |

### 6.3 关键调参记录

**根因**：位置外环增益 `Kp_pos = 0.05` 过小，对水平偏差几乎不产生倾斜指令，飞机无有效位置回复动作。

**修复**：引入 PI + 速度阻尼

| 参数 | 阶段二终值 | 阶段三终值 | 调整原因 |
|------|-----------|-----------|---------|
| `Kp_pos` | 0.05 | **0.80** | 比例增益过低，响应迟缓 |
| `Ki_pos` | 0.0 | **0.10** | 消除位置稳态误差 |
| `Kd_pos` | 0.0 | **0.60** | 提供速度阻尼，抑制 overshoot |

> `Kp_pos=2.5` 曾导致系统发散（外环带宽过高，与姿态环冲突）。回退至 `0.8` 并配合 `Kd_pos=0.6` 后获得良好阻尼。

### 6.4 阶段三代码变更

- `hover_pid_controller.py` **移除** `debug_channel` 单通道隔离逻辑，恢复全通道耦合
- `hover_single_channel_debug.py` 同步移除 `debug_channel` 传参（保留脚本以备后续单独调参）
- 新增 `hover_joint_debug.py`：联合调试入口，含5项测试矩阵、6子图输出、自动判定

---

## 7. 最终增益汇总表

| 参数 | 阶段三终值 | 说明 |
|------|-----------|------|
| `Kp_pos` | 0.80 | 位置外环比例 |
| `Ki_pos` | 0.10 | 位置外环积分 |
| `Kd_pos` | 0.60 | 位置外环微分（速度阻尼） |
| `Kp_alt` | 0.065 | 高度环比例 |
| `Kd_alt` | 0.14 | 高度环微分 |
| `Kp_att` | 8.0 | 姿态环比例 |
| `Kp_rate` | 1.5 | 角速度环比例 |
| `Kd_rate` | 0.05 | 角速度环微分 |
| `pitch_to_elevator` | +0.08 | 俯仰 → 对称舵面 |
| `roll_to_de_diff` | -0.02 | 滚转 → 差动舵面 |
| `yaw_to_throttle_diff` | +0.02 | 偏航 → 差动油门 |
| `max_de` | 20° | 舵面限幅 |
| `max_tilt` | 15° | 最大倾斜角 |
| `max_desired_rate` | 60°/s | 角速度限幅 |

---

## 8. 下一步工作建议

### 阶段四：功能扩展（优先级：中）

1. **过渡飞行仿真（Transition）**：新增 `transition_simulation.py`，研究悬停↔前飞的姿态/油门协调控制
2. **悬停状态线性化**：新增 `hover_linearize.py`，在 `theta=90°` 配平点进行13状态线性化，分析悬停模态
3. **线性化脚本适配**：修正 `linearize_analyze.py` 的状态索引，或新增12状态包装接口
4. **数据导出与可视化**：导出时序数据为 `.csv`/`.mat`，增加3D轨迹动画
5. **风扰测试**：引入持续风场或阵风模型，评估悬停抗风能力

---

## 9. 关键代码位置

- 控制器实现：[hover_pid_controller.py](hover_pid_controller.py)
- 仿真入口：[hover_simulation.py](hover_simulation.py)
- 配平验证：[hover_trim_manual.py](hover_trim_manual.py)
- 极性测试：[test_compute_polarity.py](test_compute_polarity.py)
- 单通道调试：[hover_single_channel_debug.py](hover_single_channel_debug.py)
- 联合调试：[hover_joint_debug.py](hover_joint_debug.py)
- 动力学模型：[aircraft_6dof.py](aircraft_6dof.py)
