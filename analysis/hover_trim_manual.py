"""
手动悬停配平
===========
手动设置一个正确的悬停状态
"""

import os
import numpy as np
from core import Aircraft6DOF, euler_to_quaternion

_DATA_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'data')

def manual_hover_trim():
    """手动设置悬停状态"""
    print("=" * 50)
    print("  手动悬停配平")
    print("=" * 50)

    ac = Aircraft6DOF()

    # 获取悬停配平
    x0, u0, alpha_t, theta_t, rho = ac.trim(V_trim=0.0, h_trim=0.0)

    print(f"\n配平结果:")
    print(f"  alpha: {np.degrees(alpha_t):.2f}°")
    print(f"  theta: {np.degrees(theta_t):.2f}°")
    print(f"  油门: L={u0[0]:.3f} R={u0[1]:.3f}")
    print(f"  舵面: L={np.degrees(u0[2]):.2f}° R={np.degrees(u0[3]):.2f}°")

    # 验证配平
    dx = ac.derivatives(x0, u0, rho)
    residual = np.max(np.abs(dx[:9]))
    print(f"\n配平残差: {residual:.2e}")

    # 简短仿真验证
    from core import integrate_6dof_quaternion
    from controllers import HoverPID

    pid = HoverPID()
    pid.set_hover_point(pos=[0.0, 0.0, 0.0], att=None, throttle=u0[0])
    pid.reset()

    t, x, u = integrate_6dof_quaternion(ac, x0, rho,
                                         lambda t, x, dt: pid.compute(t, x, dt),
                                         (0, 5.0), dt=0.01)

    final_pos = x[-1, 10:13]
    max_pos = np.max(np.sqrt(np.sum(x[:, 10:13]**2, axis=1)))

    print(f"\n仿真结果:")
    print(f"  最终位置: [{final_pos[0]:.3f}, {final_pos[1]:.3f}, {final_pos[2]:.3f}] m")
    print(f"  最大偏移: {max_pos:.3f} m")

    # 保存状态
    print(f"\n保存配平状态...")
    np.save(os.path.join(_DATA_DIR, 'hover_trim_state.npy'), x0)
    np.save(os.path.join(_DATA_DIR, 'hover_trim_control.npy'), u0)
    print(f"  hover_trim_state.npy")
    print(f"  hover_trim_control.npy")

    return x0, u0

if __name__ == "__main__":
    x0, u0 = manual_hover_trim()
