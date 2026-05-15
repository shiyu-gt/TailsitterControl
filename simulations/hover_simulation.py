"""
双旋翼尾座式悬停仿真（中文版）
=============================
使用四元数和PID控制器的悬停仿真，支持中文显示
"""

import numpy as np
import matplotlib.pyplot as plt
from core import Aircraft6DOF, quaternion_to_euler, quaternion_multiply, euler_to_quaternion, integrate_6dof_quaternion
from controllers import HoverPID


def setup_chinese_font():
    """设置matplotlib支持中文显示"""
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False


def get_hover_responses(t, x_hist, u_hist):
    """提取悬停仿真变量"""
    # 位置
    px, py, pz = x_hist[:, 10:13].T
    # 速度
    u, v, w = x_hist[:, 0:3].T
    V = np.sqrt(u**2 + v**2 + w**2)
    # 姿态（从四元数转换）
    phi_deg = []
    theta_deg = []
    psi_deg = []
    for i in range(len(x_hist)):
        qx, qy, qz, qw = x_hist[i, 6:10]
        phi, theta, psi = quaternion_to_euler(np.array([qw, qx, qy, qz]))
        phi_deg.append(np.degrees(phi))
        theta_deg.append(np.degrees(theta))
        psi_deg.append(np.degrees(psi))

    # 控制输入
    throttle_left = u_hist[:, 0]
    throttle_right = u_hist[:, 1]
    ev_left = np.degrees(u_hist[:, 2])
    ev_right = np.degrees(u_hist[:, 3])

    return t, px, py, pz, V, phi_deg, theta_deg, psi_deg, throttle_left, throttle_right, ev_left, ev_right


def run_hover_simulation(ac, t_end=20.0):
    """运行悬停仿真"""
    print("=" * 60)
    print("  双旋翼尾座式悬停仿真")
    print("=" * 60)

    # 初始化悬停点
    rho = 1.225  # 海平面密度

    # 创建悬停控制器（使用保守参数）
    pid = HoverPID()

    # 获取初始配平状态
    print("\n1. 计算悬停配平...")
    x0, u0, alpha_t, theta_t, rho = ac.trim(V_trim=0.0, h_trim=0.0)

    # 设置悬停点
    hover_throttle = u0[0]
    pid.set_hover_point(
        pos=[0.0, 0.0, 0.0],
        att=None,
        throttle=hover_throttle
    )
    pid.reset()

    print(f"   悬停高度: 0.0 m")
    print(f"   配平油门: {hover_throttle:.3f}")

    # 仿真控制函数
    def hover_control(t, x, dt):
        return pid.compute(t, x, dt)

    # 运行仿真
    print(f"\n2. 开始悬停仿真...")
    print(f"   仿真时长: {t_end} 秒")

    t, x, u = integrate_6dof_quaternion(ac, x0, rho, hover_control, (0, t_end), dt=0.01)

    # 提取结果
    t_arr, px, py, pz, V, _, _, _, thl, thr, _, _ = get_hover_responses(t, x, u)

    # 计算姿态误差（相对配平四元数，避免90°奇点导致的跳变）
    qx_trim, qy_trim, qz_trim, qw_trim = x0[6:10]
    q_trim_inv = np.array([qw_trim, -qx_trim, -qy_trim, -qz_trim])
    phi_err = []
    theta_err = []
    psi_err = []
    for i in range(len(x)):
        qx, qy, qz, qw = x[i, 6:10]
        q_current = np.array([qw, qx, qy, qz])
        q_err = quaternion_multiply(q_trim_inv, q_current)
        phi_e, theta_e, psi_e = quaternion_to_euler(q_err)
        phi_err.append(np.degrees(phi_e))
        theta_err.append(np.degrees(theta_e))
        psi_err.append(np.degrees(psi_e))
    phi_err = np.degrees(np.unwrap(np.radians(phi_err)))
    theta_err = np.degrees(np.unwrap(np.radians(theta_err)))
    psi_err = np.degrees(np.unwrap(np.radians(psi_err)))

    # 性能评估
    print(f"\n3. 仿真结果分析:")
    final_pos_error = np.sqrt((px[-1])**2 + (py[-1])**2 + (pz[-1])**2)
    print(f"   最终位置误差: {final_pos_error:.3f} m")

    settling_time = None
    for i in range(len(px)):
        error = np.sqrt(px[i]**2 + py[i]**2 + pz[i]**2)
        if error < 0.5:
            settling_time = t_arr[i]
            break

    if settling_time:
        print(f"   稳定时间 (±0.5m): {settling_time:.1f} s")
    else:
        print(f"   {t_end}秒内未达到稳定")

    max_pos_dev = np.max(np.sqrt(px**2 + py**2 + pz**2))
    print(f"   最大位置偏差: {max_pos_dev:.3f} m")

    # 设置中文字体
    setup_chinese_font()

    # 绘制结果
    plt.figure(figsize=(15, 10))

    ax1 = plt.subplot(2, 3, 1)
    ax1.plot(t_arr, px, 'b-', label='x')
    ax1.plot(t_arr, py, 'r-', label='y')
    ax1.plot(t_arr, pz, 'g-', label='z')
    ax1.axhline(0, color='k', linestyle='--', alpha=0.3)
    ax1.set_ylabel('位置 (m)')
    ax1.set_title('位置响应')
    ax1.legend()
    ax1.grid(True)
    ax1.ticklabel_format(axis='y', style='plain', useOffset=False)
    ax1.set_ylim([-2, 2])

    ax2 = plt.subplot(2, 3, 2)
    ax2.plot(t_arr, V, 'b-', linewidth=1.5)
    ax2.axhline(0, color='k', linestyle='--', alpha=0.3)
    ax2.set_ylabel('速度 (m/s)')
    ax2.set_title('空速响应')
    ax2.grid(True)
    ax2.ticklabel_format(axis='y', style='plain', useOffset=False)
    ax2.set_ylim([-2, 2])

    ax3 = plt.subplot(2, 3, 3)
    ax3.plot(t_arr, phi_err, 'b-', label='φ (滚转)')
    ax3.plot(t_arr, theta_err, 'r-', label='θ (俯仰)')
    ax3.plot(t_arr, psi_err, 'g-', label='ψ (偏航)')
    ax3.axhline(0, color='k', linestyle='--', alpha=0.3)
    ax3.set_ylabel('姿态误差 (deg)')
    ax3.set_title('姿态响应')
    ax3.legend()
    ax3.grid(True)
    ax3.ticklabel_format(axis='y', style='plain', useOffset=False)
    ax3.set_ylim([-20, 20])

    ax = plt.subplot(2, 3, 4, projection='3d')
    ax.plot(px, py, pz, 'b-', linewidth=2)
    ax.scatter([0], [0], [0], c='r', s=100, marker='o', label='目标点')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D轨迹')
    ax.legend()
    max_range = 2.0
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([-max_range, max_range])

    ax5 = plt.subplot(2, 3, 5)
    ax5.plot(t_arr, thl, 'b-', label='左油门')
    ax5.plot(t_arr, thr, 'r-', label='右油门')
    ax5.set_ylabel('油门')
    ax5.set_title('油门控制')
    ax5.legend()
    ax5.grid(True)
    ax5.ticklabel_format(axis='y', style='plain', useOffset=False)
    ax5.set_ylim([0, 1])

    ax6 = plt.subplot(2, 3, 6)
    pos_error = np.sqrt(px**2 + py**2 + pz**2)
    ax6.plot(t_arr, pos_error, 'r-', linewidth=2)
    ax6.axhline(0.5, color='k', linestyle='--', alpha=0.3, label='±0.5m')
    ax6.axvline(5, color='r', linestyle='--', alpha=0.3, label='扰动')
    ax6.set_xlabel('时间 (s)')
    ax6.set_ylabel('位置误差 (m)')
    ax6.set_title('位置误差')
    ax6.legend()
    ax6.grid(True)
    ax6.ticklabel_format(axis='y', style='plain', useOffset=False)
    ax6.set_ylim([0, 2])

    plt.tight_layout()
    plt.savefig('hover_simulation.png', dpi=150, bbox_inches='tight')
    print(f"\n4. 结果已保存: hover_simulation.png")

    return t_arr, x, u


if __name__ == "__main__":
    ac = Aircraft6DOF()
    t, x, u = run_hover_simulation(ac, t_end=20.0)
    print("\n悬停仿真完成！")
