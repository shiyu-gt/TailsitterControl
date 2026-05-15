"""
test_forward_controller.py
Standalone test for ForwardController I/O logic without aircraft model.
"""

import numpy as np
import matplotlib
matplotlib.rcParams['font.family'] = 'DejaVu Sans'
import matplotlib.pyplot as plt
from controllers import ForwardController


def euler_to_quaternion(phi, theta, psi):
    """roll, pitch, yaw -> quaternion [w, x, y, z]"""
    cr, sr = np.cos(phi * 0.5), np.sin(phi * 0.5)
    cp, sp = np.cos(theta * 0.5), np.sin(theta * 0.5)
    cy, sy = np.cos(psi * 0.5), np.sin(psi * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y, z])


def make_state(V, pz, theta_deg=0.0):
    """构造简化的状态向量"""
    theta = np.radians(theta_deg)
    u = V * np.cos(theta)
    w = V * np.sin(theta)
    q = euler_to_quaternion(0.0, theta, 0.0)
    return np.array([u, 0, w, 0, 0, 0, q[1], q[2], q[3], q[0], 0, 0, pz])


def test_case(name, V, pz, V_des, z_des, theta_trim=0.1, throttle_trim=0.6):
    """运行单个测试用例"""
    print(f"\n{'='*50}")
    print(f"  {name}")
    print(f"{'='*50}")

    ctrl = ForwardController(theta_trim, throttle_trim)
    ctrl.reset(theta_trim=theta_trim)

    state = make_state(V, pz)
    theta_des, phi_des, throttle_des = ctrl.compute(state, V_des, z_des, dt=0.01)

    print(f"  输入: V={V:.1f} m/s, pz={pz:.1f} m, V_des={V_des:.1f}, z_des={z_des:.1f}")
    print(f"  输出: theta_des={np.degrees(theta_des):.2f}°, phi_des={np.degrees(phi_des):.2f}°, throttle={throttle_des:.3f}")

    return theta_des, phi_des, throttle_des


if __name__ == "__main__":
    # 测试1: 速度误差 → 俯仰角
    test_case("速度偏高 (+5 m/s)", V=25, pz=50, V_des=20, z_des=50)
    test_case("速度偏低 (-5 m/s)", V=15, pz=50, V_des=20, z_des=50)
    test_case("速度匹配", V=20, pz=50, V_des=20, z_des=50)

    # 测试2: 高度误差 → 油门
    test_case("高度偏高 (+10 m)", V=20, pz=60, V_des=20, z_des=50)
    test_case("高度偏低 (-10 m)", V=20, pz=40, V_des=20, z_des=50)
    test_case("高度匹配", V=20, pz=50, V_des=20, z_des=50)

    # 测试3: 综合
    test_case("速度高+高度低", V=25, pz=40, V_des=20, z_des=50)
    test_case("速度低+高度高", V=15, pz=60, V_des=20, z_des=50)

    print("\nForwardController 测试完成。")
