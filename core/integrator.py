"""
6DOF 四元数积分器
================
统一的 RK4 积分器，支持四元数归一化和数值溢出保护。
从 hover_simulation.py 和 transition_simulation.py 中提取。
"""

import numpy as np


def integrate_6dof_quaternion(ac, x0, rho, control_func, t_span, dt=0.01, u0=None):
    """
    6DOF 四元数 RK4 积分器，带数值稳定性保护。

    参数:
        ac: Aircraft6DOF 实例
        x0: 初始状态向量 (13,) — [u,v,w, p,q,r, qx,qy,qz,qw, px,py,pz]
        rho: 空气密度 [kg/m^3]
        control_func: callable(t, x, dt) -> u (4,)
        t_span: (t_start, t_end)
        dt: 时间步长 [s]
        u0: 初始控制向量 (4,)，默认为零向量

    返回:
        t: 时间数组 (N,)
        x: 状态历史 (N, 13)
        u: 控制历史 (N, 4)
    """
    steps = int((t_span[1] - t_span[0]) / dt) + 1
    t = np.linspace(t_span[0], t_span[1], steps)
    x = np.zeros((steps, 13))
    u = np.zeros((steps, 4))

    x[0] = x0
    stop_idx = steps - 1

    for i in range(1, steps):
        if np.any(np.isnan(x[i - 1])) or np.any(np.abs(x[i - 1]) > 1e6):
            print(f"警告: 在t={t[i-1]:.2f}s时状态溢出，停止仿真")
            stop_idx = i - 1
            break

        u[i - 1] = control_func(t[i - 1], x[i - 1], dt)

        try:
            k1 = ac.derivatives(x[i - 1], u[i - 1], rho)
            k2 = ac.derivatives(x[i - 1] + 0.5 * dt * k1, u[i - 1], rho)
            k3 = ac.derivatives(x[i - 1] + 0.5 * dt * k2, u[i - 1], rho)
            k4 = ac.derivatives(x[i - 1] + dt * k3, u[i - 1], rho)
        except RuntimeError as e:
            print(f"警告: 在t={t[i-1]:.2f}s时导数计算失败: {e}")
            stop_idx = i - 1
            break

        if np.any(np.isnan(k1)) or np.any(np.abs(k1) > 1e6):
            print(f"警告: 在t={t[i-1]:.2f}s时导数溢出，停止仿真")
            stop_idx = i - 1
            break

        x[i] = x[i - 1] + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

        # 四元数归一化
        q = x[i, 6:10]
        q_norm = np.linalg.norm(q)
        if q_norm > 1e-10:
            x[i, 6:10] = q / q_norm

    if stop_idx < steps - 1:
        x[stop_idx + 1:, :] = x[stop_idx, :]
        u[stop_idx:, :] = u[stop_idx - 1, :]

    u[-1] = control_func(t[-1], x[-1], dt)
    return t, x, u
