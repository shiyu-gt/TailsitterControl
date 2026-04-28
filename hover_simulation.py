"""
双旋翼尾座式悬停仿真（中文版）
=============================
使用四元数和PID控制器的悬停仿真，支持中文显示
"""

import numpy as np
import matplotlib.pyplot as plt
from aircraft_6dof import Aircraft6DOF, quaternion_to_euler, quaternion_multiply, euler_to_quaternion
from hover_pid_controller import HoverPID
import matplotlib.font_manager as fm

# 设置中文字体
def setup_chinese_font():
    """设置matplotlib支持中文显示"""
    # 尝试使用SimHei字体
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

def integrate_6dof_quaternion(ac, x0, u0, rho, control_func, t_span, dt=0.01):
    """
    改进的6DOF积分器，支持四元数，带数值稳定性保护
    """
    steps = int((t_span[1] - t_span[0]) / dt) + 1
    t = np.linspace(t_span[0], t_span[1], steps)
    x = np.zeros((steps, 13))
    u = np.zeros((steps, 4))

    x[0] = x0
    for i in range(1, steps):
        # 检查状态是否有效
        if np.any(np.isnan(x[i-1])) or np.any(np.abs(x[i-1]) > 1e6):
            print(f"警告: 在t={t[i-1]:.2f}s时状态溢出，停止仿真")
            break

        u[i-1] = control_func(t[i-1], x[i-1], dt)

        # RK4 积分
        try:
            k1 = ac.derivatives(x[i-1], u[i-1], rho)
            k2 = ac.derivatives(x[i-1] + 0.5 * dt * k1, u[i-1], rho)
            k3 = ac.derivatives(x[i-1] + 0.5 * dt * k2, u[i-1], rho)
            k4 = ac.derivatives(x[i-1] + dt * k3, u[i-1], rho)
        except RuntimeError as e:
            print(f"警告: 在t={t[i-1]:.2f}s时导数计算失败: {e}")
            break

        # 检查中间导数是否有效
        if np.any(np.isnan(k1)) or np.any(np.abs(k1) > 1e6):
            print(f"警告: 在t={t[i-1]:.2f}s时导数溢出，停止仿真")
            break

        x[i] = x[i-1] + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

        # 确保四元数保持单位长度
        q = x[i, 6:10]
        q_norm = np.linalg.norm(q)
        if q_norm > 1e-10:
            q = q / q_norm
            x[i, 6:10] = q

    # 如果提前停止，填充剩余时间
    if i < steps - 1:
        x[i+1:, :] = x[i, :]
        u[i:, :] = u[i-1, :]

    u[-1] = control_func(t[-1], x[-1], dt)
    return t, x, u

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

    # 使用PID控制器默认参数（已针对四元数姿态控制调优）

    # 获取初始配平状态
    print("\n1. 计算悬停配平...")
    x0, u0, alpha_t, theta_t, rho = ac.trim(V_trim=0.0, h_trim=0.0)

    # 设置悬停点
    hover_throttle = u0[0]  # 假设对称油门
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
        # 阶段一调试：暂不施加外部扰动，仅通过初始姿态偏差测试控制极性
        return pid.compute(t, x, dt)

    # 阶段一调试：设置初始小姿态偏差以验证控制极性
    # 测试1: 俯仰偏差（正theta相对90°基准增加 -> 机头后仰 -> 应向-x回正）
    # x0[6:10] = ac.euler_to_quaternion(0.0, np.radians(95.0), 0.0)  # 俯仰+5°
    # 测试2: 滚转偏差（正phi -> 应向-y回正）
    # x0[6:10] = ac.euler_to_quaternion(np.radians(5.0), np.radians(90.0), 0.0)  # 滚转+5°
    # 测试3: 偏航偏差（正psi -> 差动油门应产生修正力矩）
    # x0[6:10] = ac.euler_to_quaternion(0.0, np.radians(90.0), np.radians(10.0))  # 偏航+10°

    # 运行仿真
    print(f"\n2. 开始悬停仿真...")
    print(f"   仿真时长: {t_end} 秒")

    t, x, u = integrate_6dof_quaternion(ac, x0, u0, rho, hover_control, (0, t_end), dt=0.01)

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
    # 最终位置误差
    final_pos_error = np.sqrt((px[-1])**2 + (py[-1])**2 + (pz[-1])**2)
    print(f"   最终位置误差: {final_pos_error:.3f} m")

    # 稳定时间（误差<0.5m）
    settling_idx = None
    settling_time = None
    for i in range(len(px)):
        error = np.sqrt(px[i]**2 + py[i]**2 + pz[i]**2)
        if error < 0.5:
            settling_idx = i
            settling_time = t_arr[i]
            break

    if settling_time:
        print(f"   稳定时间 (±0.5m): {settling_time:.1f} s")
    else:
        print(f"   {t_end}秒内未达到稳定")

    # 最大偏差
    max_pos_dev = np.max(np.sqrt(px**2 + py**2 + pz**2))
    print(f"   最大位置偏差: {max_pos_dev:.3f} m")

    # 设置中文字体
    setup_chinese_font()

    # 绘制结果
    plt.figure(figsize=(15, 10))

    # 位置
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

    # 速度
    ax2 = plt.subplot(2, 3, 2)
    ax2.plot(t_arr, V, 'b-', linewidth=1.5)
    ax2.axhline(0, color='k', linestyle='--', alpha=0.3)
    ax2.set_ylabel('速度 (m/s)')
    ax2.set_title('空速响应')
    ax2.grid(True)
    ax2.ticklabel_format(axis='y', style='plain', useOffset=False)
    ax2.set_ylim([-2, 2])

    # 姿态
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

    # 3D轨迹
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

    # 控制输入
    ax5 = plt.subplot(2, 3, 5)
    ax5.plot(t_arr, thl, 'b-', label='左油门')
    ax5.plot(t_arr, thr, 'r-', label='右油门')
    ax5.set_ylabel('油门')
    ax5.set_title('油门控制')
    ax5.legend()
    ax5.grid(True)
    ax5.ticklabel_format(axis='y', style='plain', useOffset=False)
    ax5.set_ylim([0, 1])

    # 位置误差
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
    # 创建飞机模型
    ac = Aircraft6DOF()

    # 运行悬停仿真
    t, x, u = run_hover_simulation(ac, t_end=20.0)

    print("\n悬停仿真完成！")