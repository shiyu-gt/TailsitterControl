"""
阶段一：控制极性验证（修正版）
使用位置偏差和俯仰偏差来测试各通道响应极性
"""

import numpy as np
from aircraft_6dof import Aircraft6DOF, quaternion_to_euler, euler_to_quaternion
from hover_pid_controller import HoverPID
from hover_simulation import integrate_6dof_quaternion

ac = Aircraft6DOF()
rho = 1.225
pid = HoverPID()

# 获取配平状态
x0_trim, u0_trim, alpha_t, theta_t, rho = ac.trim(V_trim=0.0, h_trim=0.0)
hover_throttle = u0_trim[0]
pid.set_hover_point(pos=[0.0, 0.0, 0.0], att=None, throttle=hover_throttle)

def run_test(name, x0_init, t_end=5.0):
    """运行单次测试并打印结果"""
    print(f"\n{'='*60}")
    print(f"  测试: {name}")
    print(f"{'='*60}")

    pid.reset()

    def control_func(t, x, dt):
        return pid.compute(t, x, dt)

    t, x_hist, u_hist = integrate_6dof_quaternion(ac, x0_init, u0_trim, rho, control_func, (0, t_end), dt=0.01)

    # 提取关键指标
    px, py, pz = x_hist[:, 10:13].T
    u_body, v_body, w_body = x_hist[:, 0:3].T

    # 初始和最终状态
    px0, py0, pz0 = px[0], py[0], pz[0]
    pxf, pyf, pzf = px[-1], py[-1], pz[-1]

    # 前1秒的平均加速度（看初始响应方向）
    n = min(100, len(t))
    ax = np.mean(np.diff(u_body[:n+1]) / 0.01)
    ay = np.mean(np.diff(v_body[:n+1]) / 0.01)

    # 控制量均值（前1秒）
    thl_avg = np.mean(u_hist[:n, 0])
    thr_avg = np.mean(u_hist[:n, 1])
    evl_avg = np.mean(np.degrees(u_hist[:n, 2]))
    evr_avg = np.mean(np.degrees(u_hist[:n, 3]))

    print(f"  初始位置: px={px0:.3f}, py={py0:.3f}, pz={pz0:.3f}")
    print(f"  最终位置: px={pxf:.3f}, py={pyf:.3f}, pz={pzf:.3f}")
    print(f"  平均加速度(0-1s): ax={ax:.4f}, ay={ay:.4f}")
    print(f"  平均油门: 左={thl_avg:.4f}, 右={thr_avg:.4f}, 差={thr_avg-thl_avg:.6f}")
    print(f"  平均舵面: 左={evl_avg:.2f}°, 右={evr_avg:.2f}°, 差={evr_avg-evl_avg:.2f}°")

    return pxf-px0, pyf-py0, pzf-pz0, ax, ay, thr_avg-thl_avg, evr_avg-evl_avg


# ============ 测试1: x方向位置偏差（测试俯仰通道极性） ============
# 初始 px=+0.5m（飞机在目标前方，应向-x回正）
x0_px = x0_trim.copy()
x0_px[10] = 0.5
dx, dy, dz, ax, ay, dthrottle, dev_diff = run_test("x位置偏差: px=+0.5m", x0_px, t_end=5.0)
print("  [判定] 飞机在目标前方(px>0)，应产生负x加速度回正...")
if ax < 0:
    print("  [OK] ax<0, x位置→俯仰响应方向正确")
else:
    print("  [FAIL] ax>0, x位置→俯仰响应方向相反！需修正 pitch_to_elevator 符号或位置映射")

# ============ 测试2: y方向位置偏差（测试偏航通道极性） ============
# 初始 py=+0.5m（飞机在目标右方，应向-y回正）
x0_py = x0_trim.copy()
x0_py[11] = 0.5
dx, dy, dz, ax, ay, dthrottle, dev_diff = run_test("y位置偏差: py=+0.5m", x0_py, t_end=5.0)
print("  [判定] 飞机在目标右方(py>0)，应产生负y加速度回正...")
if ay < 0:
    print("  [OK] ay<0, y位置→偏航响应方向正确")
else:
    print("  [FAIL] ay>0, y位置→偏航响应方向相反！需修正 yaw_to_throttle_diff 符号或位置映射")

# ============ 测试3: 俯仰姿态偏差（测试对称舵面极性） ============
# 初始 theta=91°（机头后仰1°，应产生低头力矩回正）
x0_pitch = x0_trim.copy()
q_tilt = euler_to_quaternion(0.0, np.radians(91.0), 0.0)
x0_pitch[6:10] = [q_tilt[1], q_tilt[2], q_tilt[3], q_tilt[0]]
dx, dy, dz, ax, ay, dthrottle, dev_diff = run_test("俯仰偏差: theta=91° (后仰1°)", x0_pitch, t_end=2.0)
print("  [判定] 机头后仰应产生低头力矩回正（对称舵面下偏，de_sym>0）...")
# 对于尾座式，后仰时机头在上，要回正需产生低头力矩
# 对称舵面下偏(de>0) -> 尾部升力增大 -> 低头力矩（绕y轴负方向）
# 位置变化上，后仰会导致推力有向前的分量，产生负x加速度
if ax < 0:
    print("  [OK] ax<0, 俯仰响应方向正确（对称舵面下偏产生低头力矩）")
else:
    print("  [FAIL] ax>0, 俯仰响应方向相反！需修正 pitch_to_elevator 符号")

# ============ 测试4: 高度偏差（测试高度通道极性） ============
# 初始 pz=+0.5m（飞机在目标下方，应增大油门上升）
x0_pz = x0_trim.copy()
x0_pz[12] = 0.5
dx, dy, dz, ax, ay, dthrottle, dev_diff = run_test("高度偏差: pz=+0.5m (低于目标)", x0_pz, t_end=2.0)
print("  [判定] 飞机低于目标(pz>0)，应增大油门上升（产生负z加速度）...")
# NED坐标：z向下为正。上升 -> z减小 -> dz/dt < 0
if dz < 0:
    print("  [OK] dz<0 (pz减小), 高度响应方向正确")
else:
    print("  [FAIL] dz>0, 高度响应方向相反！")

print("\n" + "="*60)
print("  阶段一极性验证完成")
print("="*60)
