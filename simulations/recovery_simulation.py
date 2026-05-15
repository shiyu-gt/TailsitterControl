"""
回收过程单独仿真（Recovery-Only Simulation）
============================================
直接从巡航配平状态出发，仅仿真回收过渡段（平飞→悬停），
使用 RecoveryController + HoverPID 内环，详细记录控制中间量。
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from core import (
    Aircraft6DOF, quaternion_to_euler, euler_to_quaternion,
    isa_atmosphere, integrate_6dof_quaternion
)
from controllers import HoverPID, RecoveryController


def run_recovery_simulation(dt=0.01, t_end=60.0):
    """直接从巡航配平状态开始的回收仿真"""
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    ac = Aircraft6DOF()
    rho, _ = isa_atmosphere(0.0)

    # ── 直接获取巡航配平状态 ──
    x_cruise, u_cruise, _, theta_c, _ = ac.trim(V_trim=20.0, h_trim=0.0)
    x_cruise[12] = 50.0  # 设置巡航高度
    V_cruise = 20.0
    h_cruise = 50.0
    h_hover = 20.0
    cruise_throttle = u_cruise[0]

    # ── 悬停配平 ──
    _, u_hover, _, theta_hover, _ = ac.trim(V_trim=0.0, h_trim=0.0)
    hover_throttle = u_hover[0]

    print("=" * 60)
    print("  回收过程仿真 — 从巡航配平状态直接开始")
    print("=" * 60)
    print(f"巡航配平: theta={np.degrees(theta_c):.2f}°, throttle={cruise_throttle:.3f}")
    print(f"悬停配平: theta={np.degrees(theta_hover):.2f}°, throttle={hover_throttle:.3f}")
    print(f"初始高度: {h_cruise}m → 目标: {h_hover}m")
    print(f"回收时长: {t_end}s")

    # ── 创建控制器 ──
    rec_ctrl = RecoveryController(theta_c, cruise_throttle, hover_throttle,
                                   V_cruise, h_cruise, h_hover, t_rec=t_end)
    pid = HoverPID()
    pid.hover_throttle = hover_throttle
    pid.max_de = np.radians(30)
    pid.max_desired_rate = np.radians(360)
    pid.q_hover = euler_to_quaternion(0.0, theta_hover, 0.0)
    pid.reset()

    # ── 诊断记录 ──
    diag = {
        'V_des': [], 'z_des': [], 'stage': [],
        'theta_base': [], 'gamma_des': [],
        'throttle_base': [], 'throttle_rec': [],
        'h_err': [], 'vel_err': [],
    }

    def control(t, state, dt_step):
        V = np.sqrt(state[0]**2 + state[1]**2 + state[2]**2)
        V_des, z_des, stage = rec_ctrl.get_profile(t, V)

        theta_des, phi_des, throttle_des = rec_ctrl.compute(state, V_des, z_des, dt_step, stage)

        q_des = euler_to_quaternion(phi_des, theta_des, 0.0)

        pid.target_pos[2] = z_des
        u_out = pid.compute(t, state, dt_step,
                            q_desired_override=q_des,
                            throttle_override=throttle_des)

        # 记录诊断量
        pz = state[12]
        diag['V_des'].append(V_des)
        diag['z_des'].append(z_des)
        diag['stage'].append(stage)
        diag['throttle_rec'].append(throttle_des)
        diag['h_err'].append(pz - z_des)
        diag['vel_err'].append(V - V_des)

        return u_out

    # ── 运行仿真 ──
    print(f"\n开始回收仿真: 0 -> {t_end}s")
    t, x_hist, u_hist = integrate_6dof_quaternion(ac, x_cruise, rho, control, (0, t_end), dt)
    print("仿真完成。")

    # ── 提取数据 ──
    V = np.sqrt(x_hist[:, 0]**2 + x_hist[:, 1]**2 + x_hist[:, 2]**2)
    pz = x_hist[:, 12]
    px = x_hist[:, 10]
    py = x_hist[:, 11]

    theta_arr = []
    alpha_arr = []
    phi_arr = []
    psi_arr = []
    for i in range(len(x_hist)):
        qx, qy, qz, qw = x_hist[i, 6:10]
        phi, theta, psi = quaternion_to_euler(np.array([qw, qx, qy, qz]))
        theta_arr.append(np.degrees(theta))
        alpha_arr.append(np.degrees(np.arctan2(x_hist[i, 2], x_hist[i, 0])))
        phi_arr.append(np.degrees(phi))
        psi_arr.append(np.degrees(psi))
    theta_arr = np.array(theta_arr)
    alpha_arr = np.array(alpha_arr)
    phi_arr = np.array(phi_arr)
    psi_arr = np.array(psi_arr)

    de_left = np.degrees(u_hist[:, 2])
    de_right = np.degrees(u_hist[:, 3])
    thr_left = u_hist[:, 0]
    thr_right = u_hist[:, 1]

    V_des_arr = np.array(diag['V_des'])
    z_des_arr = np.array(diag['z_des'])
    stage_arr = diag['stage']
    throttle_rec_arr = np.array(diag['throttle_rec'])

    # ── 关键指标 ──
    final_V = V[-1]
    final_pz = pz[-1]
    final_px = px[-1]
    final_py = py[-1]
    final_pos_err = np.sqrt(final_px**2 + final_py**2 + (final_pz - h_hover)**2)
    max_alpha = np.max(np.abs(alpha_arr))
    min_theta = np.min(theta_arr)
    max_theta = np.max(theta_arr)
    max_de = max(np.max(np.abs(de_left)), np.max(np.abs(de_right)))
    de_sat = np.sum((np.abs(de_left) >= 29.9) | (np.abs(de_right) >= 29.9))
    V_err_max = np.max(np.abs(V - V_des_arr))

    print("\n" + "=" * 50)
    print("  回收仿真结果")
    print("=" * 50)
    print(f"终点空速:     {final_V:.2f} m/s  (目标: 0)")
    print(f"终点高度:     {final_pz:.2f} m   (目标: {h_hover})")
    print(f"终点水平位移: ({final_px:.2f}, {final_py:.2f}) m")
    print(f"终点位置误差: {final_pos_err:.2f} m  (目标: <1)")
    print(f"最大迎角:     {max_alpha:.2f}°")
    print(f"俯仰角范围:   [{min_theta:.2f}°, {max_theta:.2f}°]")
    print(f"最大舵面:     {max_de:.2f}°")
    print(f"舵面饱和次数: {de_sat}")
    print(f"最大速度误差: {V_err_max:.2f} m/s")

    # 分段统计
    print("\n" + "-" * 50)
    print("  分段统计")
    print("-" * 50)
    for stg in ['A', 'B', 'C']:
        mask = np.array(stage_arr) == stg
        if np.any(mask):
            idxs = np.where(mask)[0]
            print(f"Stage {stg}: t={t[idxs[0]]:.1f}~{t[idxs[-1]]:.1f}s  "
                  f"V={V[idxs].min():.1f}~{V[idxs].max():.1f} m/s  "
                  f"theta={theta_arr[idxs].min():.1f}~{theta_arr[idxs].max():.1f}°  "
                  f"alpha={alpha_arr[idxs].min():.1f}~{alpha_arr[idxs].max():.1f}°")

    # 详细时间序列
    print("\n" + "-" * 50)
    print("  详细时间序列 (每1s)")
    print("-" * 50)
    print(f"{'t':>5s} {'stage':>5s} {'V':>6s} {'V_des':>6s} {'pz':>6s} {'z_des':>6s} "
          f"{'theta':>7s} {'alpha':>7s} {'thr_rec':>7s} {'thr_L':>6s} {'thr_R':>6s} "
          f"{'de_L':>6s} {'de_R':>6s} {'h_err':>6s}")
    step = int(1.0 / dt)
    for i in range(0, len(t), step):
        if i >= len(t):
            break
        h_err = pz[i] - z_des_arr[min(i, len(z_des_arr)-1)]
        thr_r = throttle_rec_arr[min(i, len(throttle_rec_arr)-1)]
        stg = stage_arr[min(i, len(stage_arr)-1)]
        print(f"{t[i]:5.1f} {stg:>5s} {V[i]:6.1f} {V_des_arr[min(i, len(V_des_arr)-1)]:6.1f} "
              f"{pz[i]:6.1f} {z_des_arr[min(i, len(z_des_arr)-1)]:6.1f} "
              f"{theta_arr[i]:7.1f} {alpha_arr[i]:7.1f} {thr_r:7.3f} "
              f"{thr_left[i]:6.3f} {thr_right[i]:6.3f} "
              f"{de_left[i]:6.1f} {de_right[i]:6.1f} {h_err:6.1f}")

    # ── 作图 ──
    fig, axes = plt.subplots(3, 3, figsize=(18, 16))

    ax = axes[0, 0]
    ax.plot(t, V, 'b-', lw=1.5, label='实际')
    ax.plot(t, V_des_arr, 'r--', lw=1.5, label='期望')
    ax.set_ylabel('空速 (m/s)'); ax.set_title('空速'); ax.legend(); ax.grid(True)

    ax = axes[0, 1]
    ax.plot(t, pz, 'b-', lw=1.5, label='实际')
    ax.plot(t, z_des_arr, 'r--', lw=1.5, label='期望')
    ax.axhline(h_hover, color='k', ls=':', alpha=0.3)
    ax.set_ylabel('高度 (m)'); ax.set_title('高度'); ax.legend(); ax.grid(True)

    ax = axes[0, 2]
    ax.plot(t, theta_arr, 'b-', lw=1.5)
    ax.axhline(np.degrees(theta_c), color='g', ls='--', alpha=0.5, label='巡航配平')
    ax.axhline(np.degrees(theta_hover), color='r', ls=':', alpha=0.5, label='悬停配平')
    ax.set_ylabel('俯仰角 (deg)'); ax.set_title('俯仰角'); ax.legend(); ax.grid(True)

    ax = axes[1, 0]
    ax.plot(t, alpha_arr, 'b-', lw=1.5)
    ax.axhline(25, color='orange', ls='--', alpha=0.7, label='保护限25°')
    ax.axhline(-25, color='orange', ls='--', alpha=0.7)
    ax.set_ylabel('迎角 (deg)'); ax.set_title('迎角'); ax.legend(); ax.grid(True)

    ax = axes[1, 1]
    ax.plot(t, phi_arr, 'b-', lw=1.5)
    ax.set_ylabel('滚转角 (deg)'); ax.set_title('滚转'); ax.grid(True)

    ax = axes[1, 2]
    ax.plot(t, psi_arr, 'b-', lw=1.5)
    ax.set_ylabel('偏航角 (deg)'); ax.set_title('偏航'); ax.grid(True)

    ax = axes[2, 0]
    ax.plot(t, thr_left, 'b-', lw=1.5, label='左')
    ax.plot(t, thr_right, 'r-', lw=1.5, label='右')
    ax.plot(t[:len(throttle_rec_arr)], throttle_rec_arr, 'g--', lw=1.0, alpha=0.7, label='回收律')
    ax.axhline(rec_ctrl.throttle_floor, color='k', ls=':', alpha=0.3, label='下限')
    ax.set_ylabel('油门'); ax.set_xlabel('时间 (s)'); ax.set_title('油门'); ax.legend(); ax.grid(True)

    ax = axes[2, 1]
    ax.plot(t, de_left, 'b-', lw=1.5, label='左')
    ax.plot(t, de_right, 'r-', lw=1.5, label='右')
    ax.axhline(30, color='k', ls='--', alpha=0.3)
    ax.axhline(-30, color='k', ls='--', alpha=0.3)
    ax.set_ylabel('舵面 (deg)'); ax.set_xlabel('时间 (s)'); ax.set_title('舵面'); ax.legend(); ax.grid(True)

    ax = axes[2, 2]
    ax.plot(t[:len(V_des_arr)], V - V_des_arr, 'b-', lw=1.5)
    ax.axhline(0, color='k', ls=':', alpha=0.3)
    ax.set_ylabel('速度误差 (m/s)'); ax.set_xlabel('时间 (s)'); ax.set_title('速度跟踪误差'); ax.grid(True)

    fig.suptitle('回收过程仿真 — RecoveryController', fontsize=14, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig('recovery_result.png', dpi=150, bbox_inches='tight')
    plt.close(fig)

    # 3D轨迹
    fig2 = plt.figure(figsize=(10, 8))
    ax3d = fig2.add_subplot(111, projection='3d')
    ax3d.plot(px, py, pz, 'r-', lw=2)
    ax3d.scatter([px[0]], [py[0]], [pz[0]], c='green', s=100, marker='^', label='起点')
    ax3d.scatter([px[-1]], [py[-1]], [pz[-1]], c='blue', s=100, marker='D', label='终点')
    ax3d.set_xlabel('X (m)'); ax3d.set_ylabel('Y (m)'); ax3d.set_zlabel('Z (m)')
    ax3d.set_title('回收轨迹'); ax3d.legend()
    fig2.savefig('recovery_trajectory.png', dpi=150, bbox_inches='tight')
    plt.close(fig2)

    print("\n图表已保存: recovery_result.png, recovery_trajectory.png")
    return t, x_hist, u_hist


if __name__ == "__main__":
    run_recovery_simulation(dt=0.01, t_end=60.0)
