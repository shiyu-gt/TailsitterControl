"""
ForwardController 平飞仿真测试
===============================
使用 ForwardController 单独控制飞机完成平飞仿真，复用 HoverPID 的姿态中环 +
角速度内环 + 控制分配来执行期望姿态。

坐标系说明：
- aircraft_6dof 使用标准 NED（z 向下为正）
- ForwardController 内部 h_err = pz - z_des，其中 pz 和 z_des 均为 NED z 坐标
- 因此调用 ForwardController 时，需将高度目标转为 NED z 坐标：z_des_ned = -z_des_height
- 记录高度时：h = -pz（标准 NED）
"""

import numpy as np
import matplotlib.pyplot as plt

from core import Aircraft6DOF, euler_to_quaternion, isa_atmosphere, integrate_6dof_quaternion
from controllers import HoverPID, ForwardController


def main():
    # ---- 初始化 ----
    ac = Aircraft6DOF()
    rho, _ = isa_atmosphere(0.0)

    # 配平
    x_trim, u_trim, alpha_trim, theta_trim, _ = ac.trim(V_trim=20.0, h_trim=0.0)
    cruise_throttle = u_trim[0]

    # 控制器
    forward = ForwardController(theta_trim, cruise_throttle)
    forward.reset(theta_trim=theta_trim)

    pid = HoverPID()
    pid.hover_throttle = cruise_throttle
    pid.max_de = np.radians(30)
    pid.reset()

    # 仿真参数
    t_end = 30.0
    dt = 0.01
    V_des = 20.0
    z_des_ned = -50.0  # NED: 50m高度

    x0 = x_trim.copy()
    x0[12] = -50.0  # NED: 初始高度50m

    def control(t, state, dt_step):
        theta_des, phi_des, throttle_des = forward.compute(state, V_des, z_des_ned, dt_step)
        q_des = euler_to_quaternion(phi_des, theta_des, 0.0)
        pid.target_pos[2] = z_des_ned
        return pid.compute(t, state, dt_step,
                           q_desired_override=q_des,
                           throttle_override=throttle_des)

    print("开始平飞仿真...")
    t, x_hist, u_hist = integrate_6dof_quaternion(ac, x0, rho, control, (0, t_end), dt)
    print("仿真完成。")

    # 提取数据
    px = x_hist[:, 10]
    py = x_hist[:, 11]
    pz = x_hist[:, 12]
    h = -pz  # NED → 高度

    u_vel = x_hist[:, 0]
    v_vel = x_hist[:, 1]
    w_vel = x_hist[:, 2]
    V = np.sqrt(u_vel**2 + v_vel**2 + w_vel**2)

    theta_arr = []
    for i in range(len(x_hist)):
        qx, qy, qz, qw = x_hist[i, 6:10]
        from core import quaternion_to_euler
        _, theta, _ = quaternion_to_euler(np.array([qw, qx, qy, qz]))
        theta_arr.append(np.degrees(theta))
    theta_arr = np.array(theta_arr)

    # 绘图
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    ax = axes[0, 0]
    ax.plot(t, V, 'b-', lw=1.5, label='实际')
    ax.axhline(V_des, color='r', ls='--', label='期望')
    ax.set_ylabel('空速 (m/s)'); ax.set_title('空速'); ax.legend(); ax.grid(True)

    ax = axes[0, 1]
    ax.plot(t, h, 'b-', lw=1.5, label='实际')
    ax.axhline(50.0, color='r', ls='--', label='期望')
    ax.set_ylabel('高度 (m)'); ax.set_title('高度'); ax.legend(); ax.grid(True)

    ax = axes[1, 0]
    ax.plot(t, theta_arr, 'b-', lw=1.5)
    ax.axhline(np.degrees(theta_trim), color='r', ls='--', alpha=0.5, label='配平')
    ax.set_ylabel('俯仰角 (deg)'); ax.set_xlabel('时间 (s)'); ax.set_title('俯仰角'); ax.legend(); ax.grid(True)

    ax = axes[1, 1]
    ax.plot(t, u_hist[:, 0], 'b-', lw=1.5, label='左')
    ax.plot(t, u_hist[:, 1], 'r-', lw=1.5, label='右')
    ax.set_ylabel('油门'); ax.set_xlabel('时间 (s)'); ax.set_title('油门'); ax.legend(); ax.grid(True)

    fig.suptitle('ForwardController 平飞仿真', fontsize=14, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig('test_forward_flight.png', dpi=150, bbox_inches='tight')
    plt.close(fig)

    print(f"终点: V={V[-1]:.2f} m/s, h={h[-1]:.2f} m, theta={theta_arr[-1]:.2f}°")
    print("图表已保存: test_forward_flight.png")


if __name__ == "__main__":
    main()
