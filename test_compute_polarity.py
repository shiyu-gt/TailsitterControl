"""
直接测试 HoverPID.compute() 的控制输出符号，隔离动力学耦合
"""

import numpy as np
from aircraft_6dof import euler_to_quaternion
from hover_pid_controller import HoverPID

pid = HoverPID()
pid.set_hover_point(pos=[0.0, 0.0, 0.0], att=None, throttle=0.634)

def test_state(name, state, dt=0.01):
    """给定状态，打印控制输出"""
    print(f"\n{'='*60}")
    print(f"  测试: {name}")
    print(f"{'='*60}")
    pid.reset()
    u = pid.compute(0.0, state, dt)
    throttle_left, throttle_right, de_left, de_right = u
    de_sym = (de_left + de_right) / 2.0
    de_diff = de_right - de_left
    throttle_diff = throttle_right - throttle_left
    print(f"  油门: 左={throttle_left:.6f}, 右={throttle_right:.6f}, 差={throttle_diff:.6f}")
    print(f"  舵面: 左={np.degrees(de_left):.4f}°, 右={np.degrees(de_right):.4f}°")
    print(f"  对称分量 de_sym={np.degrees(de_sym):.4f}°, 差动分量 de_diff={np.degrees(de_diff):.4f}°")
    return de_sym, de_diff, throttle_diff, throttle_left, throttle_right


# 基准悬停状态
q_hover = euler_to_quaternion(0.0, np.radians(90.0), 0.0)
state_hover = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        q_hover[1], q_hover[2], q_hover[3], q_hover[0],
                        0.0, 0.0, 0.0])

# ============ 测试1: 俯仰偏差（theta=91°，后仰1°） ============
q_pitch = euler_to_quaternion(0.0, np.radians(91.0), 0.0)
state_pitch = state_hover.copy()
state_pitch[6:10] = [q_pitch[1], q_pitch[2], q_pitch[3], q_pitch[0]]
de_sym, de_diff, dthrottle, thl, thr = test_state("俯仰偏差: theta=91° (后仰1°)", state_pitch)
print("  [判定] 后仰应产生低头力矩。对于飞翼布局(Cm_de>0)，上偏(de_sym<0)产生低头力矩...")
if np.degrees(de_sym) < 0:
    print("  [OK] de_sym<0, 俯仰响应方向正确")
else:
    print("  [FAIL] de_sym>0, 俯仰响应方向相反！需修正 pitch_to_elevator 符号")

# ============ 测试2: x位置偏差（px=+0.5m，飞机在前方） ============
state_px = state_hover.copy()
state_px[10] = 0.5
de_sym, de_diff, dthrottle, thl, thr = test_state("x位置偏差: px=+0.5m", state_px)
print("  [判定] 飞机在前方，应向后倾斜产生-x加速度。de_sym>0 -> 后仰 -> 推力向后 -> -x加速度...")
if np.degrees(de_sym) > 0:
    print("  [OK] de_sym>0, x位置→俯仰映射方向正确")
else:
    print("  [FAIL] de_sym<0, x位置→俯仰映射方向相反")

# ============ 测试3: y位置偏差（py=+0.5m，飞机在右方） ============
state_py = state_hover.copy()
state_py[11] = 0.5
de_sym, de_diff, dthrottle, thl, thr = test_state("y位置偏差: py=+0.5m", state_py)
print("  [判定] 飞机在右方，应向左倾斜产生-y加速度。de_diff>0 -> 右舵面下偏 -> 左滚转...")
print("  [注意] 尾座式悬停时，y位置控制通过偏航实现，非滚转")

# ============ 测试4: 偏航偏差（psi=5°） ============
q_yaw = euler_to_quaternion(0.0, np.radians(90.0), np.radians(5.0))
state_yaw = state_hover.copy()
state_yaw[6:10] = [q_yaw[1], q_yaw[2], q_yaw[3], q_yaw[0]]
de_sym, de_diff, dthrottle, thl, thr = test_state("偏航偏差: psi=5°", state_yaw)
print("  [判定] 正偏航误差(机头偏右)应通过差动油门修正。需左油门增大 -> 负偏航力矩(机头向左回正)...")
if dthrottle < 0:
    print("  [OK] dthrottle<0 (左油门增大), 偏航响应方向正确")
else:
    print("  [FAIL] dthrottle>0, 偏航响应方向相反！需修正 yaw_to_throttle_diff 符号")

# ============ 测试5: 高度偏差（pz=+0.5m，飞机在下方） ============
state_pz = state_hover.copy()
state_pz[12] = 0.5
de_sym, de_diff, dthrottle, thl, thr = test_state("高度偏差: pz=+0.5m (低于目标)", state_pz)
throttle_avg = (thl + thr) / 2
print("  [判定] 飞机低于目标，应增大油门...")
if throttle_avg > 0.634:
    print("  [OK] 平均油门>配平值, 高度响应方向正确")
else:
    print("  [FAIL] 平均油门<配平值, 高度响应方向相反")

# ============ 测试6: 滚转偏差（phi=5°） ============
q_roll = euler_to_quaternion(np.radians(5.0), np.radians(90.0), 0.0)
state_roll = state_hover.copy()
state_roll[6:10] = [q_roll[1], q_roll[2], q_roll[3], q_roll[0]]
de_sym, de_diff, dthrottle, thl, thr = test_state("滚转偏差: phi=5°", state_roll)
print("  [判定] 在尾座式悬停时，滚转偏差被四元数误差映射为偏航误差")
print(f"  差动舵面 de_diff={np.degrees(de_diff):.4f}°, 差动油门 dthrottle={dthrottle:.6f}")

print("\n" + "="*60)
print("  compute() 极性验证完成")
print("="*60)
