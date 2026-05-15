# 尾座式无人机回收仿真 — 代码操作指南

> 更新日期：2026-05-15

---

## 1. 文件关系总览

```
┌─────────────────────────────────────────────────────────────────┐
│                     aircraft_6dof.py                            │
│              （核心：6DOF动力学 + 四元数 + 双动压气动模型）          │
└──────────────────┬──────────────────────────────────────────────┘
                   │
    ┌──────────────┼──────────────┬──────────────────┐
    │              │              │                  │
    ▼              ▼              ▼                  ▼
┌─────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐
│hover_   │ │recovery_     │ │hover_        │ │linearize_    │
│pid_     │ │simulation.py │ │simulation.py │ │analyze.py    │
│controller│ │（回收仿真    │ │（悬停仿真    │ │（模态分析）  │
│.py      │ │ 主程序）     │ │）            │ │              │
│（悬停PID）│ │              │ │              │ │              │
└────┬────┘ └──────┬───────┘ └──────────────┘ └──────────────┘
     │             │
     │    ┌────────┴────────┐
     │    │                 │
     │    ▼                 ▼
     │ ┌──────────────┐ ┌──────────────┐
     │ │recovery_     │ │recovery_     │
     │ │controller.py │ │envelope.py   │
     │ │（回收控制律） │ │（包络线分析）│
     │ └──────────────┘ └──────────────┘
     │         │
     └─────────┘
     （HoverPID 作为内环被 RecoveryController 调用）
```

### 各文件职责

| 文件 | 职责 | 依赖关系 |
|------|------|---------|
| `core/aircraft_6dof.py` | 6DOF 动力学、气动模型、配平求解、ISA 大气 | 无 |
| `core/integrator.py` | RK4 积分器（四元数归一化） | aircraft_6dof |
| `controllers/hover_pid_controller.py` | 四环级联悬停 PID | aircraft_6dof |
| `controllers/recovery_controller.py` | 三阶段回收控制律 | 无（纯控制逻辑） |
| `simulations/recovery_simulation.py` | 回收段独立仿真 | 全部核心+控制器 |
| `simulations/hover_simulation.py` | 悬停仿真 | aircraft_6dof, hover_pid |
| `analysis/recovery_envelope.py` | 减速能力包络线分析 | aircraft_6dof |
| `analysis/linearize_analyze.py` | 线性化与模态分析 | aircraft_6dof |
| `tests/test_polarity.py` | 控制极性验证 | aircraft_6dof, hover_pid |

---

## 2. 当前代码状态

| 模块 | 状态 | 说明 |
|------|------|------|
| `core/aircraft_6dof.py` | 稳定 | 6DOF + 配平 + 气动查表，已验证 |
| `core/integrator.py` | 稳定 | RK4 + 四元数归一化，已验证 |
| `controllers/hover_pid_controller.py` | 稳定 | 四环级联 PID，悬停验证通过 |
| `controllers/recovery_controller.py` | **开发中** | Stage A 验证通过，Stage B 待优化 |
| `simulations/recovery_simulation.py` | 稳定 | 含诊断记录和图表生成 |
| `analysis/recovery_envelope.py` | 稳定 | 配平扫描和减速包络线 |

---

## 3. 验证检查清单

### 3.1 基础验证

- [ ] `python -m tests.test_polarity` — 极性验证通过
- [ ] `python -m simulations.hover_simulation` — 悬停稳定
- [ ] `python -m analysis.linearize_analyze` — 模态分析正常

### 3.2 回收仿真验证

- [ ] Stage A：V 从 20 降至 ~15 m/s，alpha < 20°，无振荡
- [ ] Stage B：进入后 V 持续下降（非停滞）
- [ ] Stage C：theta 平滑过渡到 ~85°，V 降至 < 1 m/s
- [ ] 终点：V < 1 m/s，pz 在 20±2 m
- [ ] 全程：alpha < 25°，无舵面持续饱和

---

## 4. 调试操作

### 4.1 添加诊断打印

在 `recovery_simulation.py` 的 `control()` 函数中：

```python
# 每 1 秒打印一次
if abs(t - round(t)) < dt/2:
    print(f"t={t:.1f} stage={stage} V={V:.1f} V_des={V_des:.1f} "
          f"theta={np.degrees(theta_des):.1f}° thr={throttle_des:.3f}")
```

### 4.2 检查力矩平衡

在仿真循环中插入：

```python
alpha = np.arctan2(state[2], state[0])
q_inf = 0.5 * rho * V**2
Cm = np.interp(np.degrees(alpha), ac.aero._lon_alpha, ac.aero._lon_Cm)
M_aero = Cm * q_inf * ac.aero.S * ac.aero.c_bar
print(f"alpha={np.degrees(alpha):.1f}° M_aero={M_aero:+.3f} N·m")
```

### 4.3 修改参数后清理缓存

```bash
find . -name "*.pyc" -delete
find . -name "__pycache__" -type d -exec rm -rf {} +
```

---

## 5. 关键接口

```python
# RecoveryController 接口
rec_ctrl = RecoveryController(cruise_theta, cruise_throttle, hover_throttle,
                               V_cruise, h_cruise, h_hover, t_rec)
V_des, z_des, stage = rec_ctrl.get_profile(t_rec, V_current, pz=pz)
theta_des, phi_des, throttle_des = rec_ctrl.compute(state, V_des, z_des, dt, stage)
de_override = rec_ctrl._de_override  # Stage B 时非 None

# HoverPID 接口
u_out = pid.compute(t, state, dt,
                    q_desired_override=q_des,
                    throttle_override=throttle_des)
```
