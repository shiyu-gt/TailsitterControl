"""
手动悬停配平
===========
手动设置一个正确的悬停状态
"""

import numpy as np
from aircraft_6dof import Aircraft6DOF, euler_to_quaternion

def manual_hover_trim():
    """手动设置悬停状态"""
    print("=" * 50)
    print("  手动悬停配平")
    print("=" * 50)

    ac = Aircraft6DOF()
    rho = 1.225  # 海平面密度

    # 手动设置悬停状态
    # 1. 零速度
    u = 0.0
    v = 0.0
    w = 0.0

    # 2. 零角速度
    p = 0.0
    q = 0.0
    r = 0.0

    # 3. 尾座式悬停姿态（机头朝上，theta=90°）
    phi = 0.0
    theta = np.radians(90.0)  # 机头朝上，尾座式悬停
    psi = 0.0

    # 4. 零位置
    px = 0.0
    py = 0.0
    pz = 0.0

    # 5. 四元数（无旋转）
    quat = euler_to_quaternion(phi, theta, psi)

    # 6. 计算所需推力（平衡重力）
    m = ac.mass
    g = ac.g
    total_thrust = m * g  # 总推力
    hover_throttle = np.interp(total_thrust / 2, ac.aero._thrust, ac.aero._throttle)

    # 左右对称油门
    throttle_left = hover_throttle
    throttle_right = hover_throttle

    # 零升降副翼偏角
    ev_left = 0.0
    ev_right = 0.0

    # 构建状态向量
    x0 = np.array([u, v, w, p, q, r, quat[1], quat[2], quat[3], quat[0], px, py, pz])
    u0 = np.array([throttle_left, throttle_right, ev_left, ev_right])

    print(f"  质量: {m} kg")
    print(f"  重量: {m * g:.1f} N")
    print(f"  总推力: {total_thrust:.1f} N")
    print(f"  单侧油门: {hover_throttle:.3f}")

    # 验证配平
    print("\n验证配平状态...")
    dx = ac.derivatives(x0, u0, rho)

    print("状态导数:")
    labels = ['du/dt', 'dv/dt', 'dw/dt', 'dp/dt', 'dq/dt', 'dr/dt',
              'dqx/dt', 'dqy/dt', 'dqz/dt', 'dqw/dt',
              'dpx/dt', 'dpy/dt', 'dpz/dt']
    for i, label in enumerate(labels):
        print(f"  {label}: {dx[i]:.6f}")

    # 检查是否平衡（仅检查前9个机械状态导数）
    mech_res = np.max(np.abs(dx[:9]))
    print(f"\n机械状态导数残差 max|dx[0:9]|: {mech_res:.2e}")
    tol = 1e-3
    if mech_res < tol:
        print("[OK] 状态平衡！")
    else:
        print("[FAIL] 状态不平衡！")
        for i in range(9):
            if abs(dx[i]) > tol:
                print(f"  警告: {labels[i]} = {dx[i]:.6f}")

    # 仿真测试
    print("\n仿真测试...")
    dt = 0.01
    steps = 1000
    x = np.zeros((steps, 13))
    x[0] = x0
    u = np.tile(u0, (steps, 1))

    for i in range(1, steps):
        dx = ac.derivatives(x[i-1], u[i-1], rho)
        x[i] = x[i-1] + dx * dt

        # 确保四元数归一化
        q = x[i, 6:10]
        q = q / np.linalg.norm(q)
        x[i, 6:10] = q

        # 限制位置范围
        x[i, 10:13] = np.clip(x[i, 10:13], -10, 10)

    # 分析结果
    final_pos = x[-1, 10:13]
    max_pos = np.max(np.sqrt(np.sum(x[:, 10:13]**2, axis=1)))

    print(f"\n仿真结果:")
    print(f"  最终位置: [{final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f}] m")
    print(f"  最大偏移: {max_pos:.3f} m")

    # 保存状态
    print(f"\n保存配平状态...")
    np.save('hover_trim_state.npy', x0)
    np.save('hover_trim_control.npy', u0)
    print(f"  hover_trim_state.npy")
    print(f"  hover_trim_control.npy")

    return x0, u0

if __name__ == "__main__":
    x0, u0 = manual_hover_trim()