"""
阶段一：控制极性验证（修正版）
使用位置偏差和俯仰偏差来测试各通道响应极性
"""

import numpy as np
from core import Aircraft6DOF, quaternion_to_euler, euler_to_quaternion, integrate_6dof_quaternion
from controllers import HoverPID

ac = Aircraft6DOF()
rho = 1.225
pid = HoverPID()

# 获取配平状态
x0_trim, u0_trim, alpha_t, theta_t, rho = ac.trim(V_trim=0.0, h_trim=0.0)
hover_throttle = u0_trim[0]
pid.set_hover_point(pos=[0.0, 0.0, 0.0], att=None, throttle=hover_throttle)

def run_test(name, x0_init, t_end=5.0):
    """运行单个极性测试"""
    print(f"\n{'='*50}")
    print(f"  测试: {name}")
    print(f"{'='*50}")

    pid.reset()
    t, x, u = integrate_6dof_quaternion(ac, x0_init, rho,
                                         lambda t, x, dt: pid.compute(t, x, dt),
                                         (0, t_end), dt=0.01)

    # 提取最终状态
    px, py, pz = x[-1, 10:13]
    qx, qy, qz, qw = x[-1, 6:10]
    phi, theta, psi = quaternion_to_euler(np.array([qw, qx, qy, qz]))

    print(f"  终点位置: ({px:.3f}, {py:.3f}, {pz:.3f}) m")
    print(f"  终点姿态: phi={np.degrees(phi):.2f}° theta={np.degrees(theta):.2f}° psi={np.degrees(psi):.2f}°")
    print(f"  终点油门: L={u[-1,0]:.3f} R={u[-1,1]:.3f}")
    print(f"  终点舵面: L={np.degrees(u[-1,2]):.2f}° R={np.degrees(u[-1,3]):.2f}°")

    return t, x, u


if __name__ == "__main__":
    # 测试1: 俯仰偏差
    x0 = x0_trim.copy()
    x0[6:10] = euler_to_quaternion(0.0, np.radians(95.0), 0.0)
    run_test("俯仰+5°偏差", x0)

    # 测试2: 滚转偏差
    x0 = x0_trim.copy()
    x0[6:10] = euler_to_quaternion(np.radians(5.0), np.radians(90.0), 0.0)
    run_test("滚转+5°偏差", x0)

    # 测试3: 偏航偏差
    x0 = x0_trim.copy()
    x0[6:10] = euler_to_quaternion(0.0, np.radians(90.0), np.radians(10.0))
    run_test("偏航+10°偏差", x0)

    # 测试4: 位置偏差
    x0 = x0_trim.copy()
    x0[10] = 1.0  # x偏移1m
    run_test("x位置+1m偏差", x0)

    print("\n极性验证完成。")
