"""
阶段三：联合调试与扰动测试
==========================
全通道耦合，验证多轴协同响应与扰动抑制能力。
"""

import numpy as np
import matplotlib.pyplot as plt
from aircraft_6dof import Aircraft6DOF, quaternion_to_euler, euler_to_quaternion, quaternion_multiply
from hover_pid_controller import HoverPID


def setup_chinese_font():
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False


def integrate_6dof_quaternion(ac, x0, u0, rho, control_func, t_span, dt=0.01):
    """改进的6DOF积分器，支持四元数，带数值稳定性保护"""
    steps = int((t_span[1] - t_span[0]) / dt) + 1
    t = np.linspace(t_span[0], t_span[1], steps)
    x = np.zeros((steps, 13))
    u = np.zeros((steps, 4))

    x[0] = x0
    stop_idx = steps - 1
    for i in range(1, steps):
        if np.any(np.isnan(x[i-1])) or np.any(np.abs(x[i-1]) > 1e6):
            print(f"  警告: 在t={t[i-1]:.2f}s时状态溢出，停止仿真")
            stop_idx = i - 1
            break

        u[i-1] = control_func(t[i-1], x[i-1], dt)

        try:
            k1 = ac.derivatives(x[i-1], u[i-1], rho)
            k2 = ac.derivatives(x[i-1] + 0.5 * dt * k1, u[i-1], rho)
            k3 = ac.derivatives(x[i-1] + 0.5 * dt * k2, u[i-1], rho)
            k4 = ac.derivatives(x[i-1] + dt * k3, u[i-1], rho)
        except RuntimeError as e:
            print(f"  警告: 在t={t[i-1]:.2f}s时导数计算失败: {e}")
            stop_idx = i - 1
            break

        if np.any(np.isnan(k1)) or np.any(np.abs(k1) > 1e6):
            print(f"  警告: 在t={t[i-1]:.2f}s时导数溢出，停止仿真")
            stop_idx = i - 1
            break

        x[i] = x[i-1] + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

        q = x[i, 6:10]
        q_norm = np.linalg.norm(q)
        if q_norm > 1e-10:
            q = q / q_norm
            x[i, 6:10] = q

    if stop_idx < steps - 1:
        x[stop_idx+1:, :] = x[stop_idx, :]
        u[stop_idx:, :] = u[stop_idx-1, :]

    u[-1] = control_func(t[-1], x[-1], dt)
    return t, x, u


def run_test(name, x0_perturb, t_end, dt=0.01):
    """
    运行单个联合调试测试

    name: 测试名称
    x0_perturb: 对配平状态的扰动字典
        支持: 'px', 'py', 'pz', 'phi_deg', 'theta_deg', 'psi_deg'
    t_end: 仿真时长
    """
    setup_chinese_font()
    ac = Aircraft6DOF()
    rho = 1.225

    pid = HoverPID()

    # 获取悬停配平状态
    x0, u0, alpha_t, theta_t, rho = ac.trim(V_trim=0.0, h_trim=0.0)
    hover_throttle = u0[0]

    pid.set_hover_point(pos=[0.0, 0.0, 0.0], att=None, throttle=hover_throttle)
    pid.reset()

    # 应用初始扰动
    q_hover = euler_to_quaternion(0.0, np.radians(90.0), 0.0)

    if 'phi_deg' in x0_perturb or 'theta_deg' in x0_perturb or 'psi_deg' in x0_perturb:
        phi = np.radians(x0_perturb.get('phi_deg', 0.0))
        theta = np.radians(90.0 + x0_perturb.get('theta_deg', 0.0))
        psi = np.radians(x0_perturb.get('psi_deg', 0.0))
        q_perturb = euler_to_quaternion(phi, theta, psi)
        x0[6:10] = [q_perturb[1], q_perturb[2], q_perturb[3], q_perturb[0]]

    if 'px' in x0_perturb:
        x0[10] = x0_perturb['px']
    if 'py' in x0_perturb:
        x0[11] = x0_perturb['py']
    if 'pz' in x0_perturb:
        x0[12] = x0_perturb['pz']

    # 仿真
    def control_func(t, x, dt):
        return pid.compute(t, x, dt)

    t, x_hist, u_hist = integrate_6dof_quaternion(ac, x0, u0, rho, control_func, (0, t_end), dt=dt)

    # 提取响应
    px = x_hist[:, 10]
    py = x_hist[:, 11]
    pz = x_hist[:, 12]

    phi_deg = []
    theta_deg = []
    psi_deg = []
    for i in range(len(x_hist)):
        qx, qy, qz, qw = x_hist[i, 6:10]
        phi, theta, psi = quaternion_to_euler(np.array([qw, qx, qy, qz]))
        phi_deg.append(np.degrees(phi))
        theta_deg.append(np.degrees(theta))
        psi_deg.append(np.degrees(psi))
    phi_deg = np.array(phi_deg)
    theta_deg = np.array(theta_deg)
    psi_deg = np.array(psi_deg)

    throttle_left = u_hist[:, 0]
    throttle_right = u_hist[:, 1]
    de_left = np.degrees(u_hist[:, 2])
    de_right = np.degrees(u_hist[:, 3])

    # 计算指标
    pos_error = np.sqrt(px**2 + py**2 + pz**2)
    final_px_err = px[-1]
    final_py_err = py[-1]
    final_pz_err = pz[-1]
    final_pos_err = pos_error[-1]

    # 稳定时间（进入±0.5m并保持）
    settling_time = None
    for i in range(len(pos_error)):
        if pos_error[i] < 0.5:
            if np.all(pos_error[i:] < 0.5):
                settling_time = t[i]
                break

    # 最大舵面偏角
    max_de = max(np.max(np.abs(de_left)), np.max(np.abs(de_right)))

    # 最大倾斜角（偏离90°）
    max_tilt = max(np.max(np.abs(phi_deg)), np.max(np.abs(theta_deg - 90.0)))

    # 油门波动（相对悬停油门）
    throttle_dev = np.max(np.abs(throttle_left - hover_throttle))
    throttle_dev = max(throttle_dev, np.max(np.abs(throttle_right - hover_throttle)))

    # 绘图：6子图面板
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))

    # 左上：滚转
    ax = axes[0, 0]
    ax.plot(t, phi_deg, 'b-', linewidth=1.5)
    ax.axhline(0, color='r', linestyle='--', alpha=0.5)
    ax.set_ylabel('滚转角 φ (deg)')
    ax.set_title('滚转响应')
    ax.grid(True)

    # 中上：俯仰
    ax = axes[0, 1]
    ax.plot(t, theta_deg, 'r-', linewidth=1.5)
    ax.axhline(90, color='r', linestyle='--', alpha=0.5)
    ax.set_ylabel('俯仰角 θ (deg)')
    ax.set_title('俯仰响应')
    ax.grid(True)

    # 右上单独放偏航
    # 重新布局：3行2列，姿态占3个位置
    # 左中：偏航
    ax = axes[1, 0]
    ax.plot(t, psi_deg, 'g-', linewidth=1.5)
    ax.axhline(0, color='r', linestyle='--', alpha=0.5)
    ax.set_ylabel('偏航角 ψ (deg)')
    ax.set_title('偏航响应')
    ax.grid(True)

    # 右中：水平位置
    ax = axes[1, 1]
    ax.plot(t, px, 'b-', linewidth=1.5, label='px')
    ax.plot(t, py, 'r-', linewidth=1.5, label='py')
    ax.axhline(0, color='k', linestyle='--', alpha=0.3)
    ax.set_ylabel('水平位置 (m)')
    ax.set_title('水平位置响应')
    ax.legend(loc='upper right')
    ax.grid(True)

    # 左下：高度
    ax = axes[2, 0]
    ax.plot(t, pz, 'g-', linewidth=1.5)
    ax.axhline(0, color='r', linestyle='--', alpha=0.5)
    ax.set_ylabel('高度 pz (m)')
    ax.set_title('高度响应')
    ax.set_xlabel('时间 (s)')
    ax.grid(True)

    # 右下：控制量
    ax = axes[2, 1]
    ax.plot(t, throttle_left, 'b-', linewidth=1.5, label='左油门')
    ax.plot(t, throttle_right, 'r-', linewidth=1.5, label='右油门')
    ax.plot(t, de_left, 'c--', linewidth=1.0, label='左舵面')
    ax.plot(t, de_right, 'm--', linewidth=1.0, label='右舵面')
    ax.axhline(hover_throttle, color='k', linestyle=':', alpha=0.3)
    ax.axhline(0, color='k', linestyle=':', alpha=0.3)
    ax.set_ylabel('控制量')
    ax.set_title('控制输入')
    ax.set_xlabel('时间 (s)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True)

    fig.suptitle(f'测试: {name}', fontsize=14, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    fname = f'hover_joint_{name.replace(" ", "_").replace("+", "plus")}.png'
    plt.savefig(fname, dpi=150, bbox_inches='tight')
    plt.close()

    # 判定
    # 根据初始偏差确定标准
    init_px = x0_perturb.get('px', 0.0)
    init_py = x0_perturb.get('py', 0.0)
    init_pz = x0_perturb.get('pz', 0.0)
    init_pos = np.sqrt(init_px**2 + init_py**2 + init_pz**2)

    if init_pos <= 0.5:
        ts_limit = 10.0
        err_limit = 0.1
    elif init_pos <= 2.0:
        ts_limit = 15.0
        err_limit = 0.15
    else:
        ts_limit = 20.0
        err_limit = 0.2

    verdict = "通过"
    issues = []
    if settling_time is None:
        verdict = "未通过"
        issues.append(f"未在{t_end}s内稳定到±0.5m")
    elif settling_time > ts_limit:
        verdict = "未通过"
        issues.append(f"稳定时间{settling_time:.1f}s > 限值{ts_limit:.1f}s")
    if final_pos_err > err_limit:
        verdict = "未通过"
        issues.append(f"最终位置误差{final_pos_err:.3f}m > 限值{err_limit:.1f}m")
    if max_de > 15.0:
        verdict = "未通过"
        issues.append(f"最大舵面{max_de:.1f}° > 15°")
    if throttle_dev > 0.15:
        verdict = "未通过"
        issues.append(f"油门波动±{throttle_dev:.3f} > ±0.15")

    result = {
        'name': name,
        'final_px_err': final_px_err,
        'final_py_err': final_py_err,
        'final_pz_err': final_pz_err,
        'final_pos_err': final_pos_err,
        'settling_time': settling_time,
        'max_de': max_de,
        'max_tilt': max_tilt,
        'throttle_dev': throttle_dev,
        'verdict': verdict,
        'issues': issues,
        'fname': fname,
    }
    return result


def print_test_result(r):
    print(f"> 【{r['name']}】结果")
    print(f"> - 最终位置误差：({r['final_px_err']:.3f}, {r['final_py_err']:.3f}, {r['final_pz_err']:.3f}) m")
    if r['settling_time'] is not None:
        print(f"> - 稳定时间（进入 ±0.5m）：{r['settling_time']:.1f} s")
    else:
        print(f"> - 稳定时间：未达到")
    print(f"> - 最大舵面偏角：{r['max_de']:.1f}°")
    print(f"> - 最大倾斜角：{r['max_tilt']:.1f}°")
    print(f"> - 油门波动：±{r['throttle_dev']:.3f}")
    print(f"> - 判定：{r['verdict']}")
    if r['issues']:
        for issue in r['issues']:
            print(f">   ! {issue}")
    print()


if __name__ == "__main__":
    setup_chinese_font()
    print("=" * 60)
    print("  阶段三：联合调试与扰动测试")
    print("=" * 60)

    results = []

    # -------------------------------------------------------------
    # 测试1：水平耦合
    # -------------------------------------------------------------
    print("\n[1/5] 测试1：水平耦合（px=+0.5m, py=+0.5m）")
    r = run_test("水平耦合", {'px': 0.5, 'py': 0.5}, t_end=20.0)
    results.append(r)
    print_test_result(r)

    # -------------------------------------------------------------
    # 测试2：水平+高度耦合
    # -------------------------------------------------------------
    print("[2/5] 测试2：水平+高度耦合（px=+0.5m, pz=+1m）")
    r = run_test("水平高度耦合", {'px': 0.5, 'pz': 1.0}, t_end=20.0)
    results.append(r)
    print_test_result(r)

    # -------------------------------------------------------------
    # 测试3：全状态偏差
    # -------------------------------------------------------------
    print("[3/5] 测试3：全状态偏差（px=+1m, py=+1m, pz=+1m, yaw=+5°）")
    r = run_test("全状态偏差", {'px': 1.0, 'py': 1.0, 'pz': 1.0, 'psi_deg': 5.0}, t_end=20.0)
    results.append(r)
    print_test_result(r)

    # -------------------------------------------------------------
    # 测试4：大偏差
    # -------------------------------------------------------------
    print("[4/5] 测试4：大偏差（px=+5m, py=-3m）")
    r = run_test("大偏差", {'px': 5.0, 'py': -3.0}, t_end=40.0)
    results.append(r)
    print_test_result(r)

    # -------------------------------------------------------------
    # 测试5：姿态+位置混合扰动
    # -------------------------------------------------------------
    print("[5/5] 测试5：姿态+位置混合（roll=+5°, px=+2m）")
    r = run_test("姿态位置混合", {'phi_deg': 5.0, 'px': 2.0}, t_end=20.0)
    results.append(r)
    print_test_result(r)

    # -------------------------------------------------------------
    # 汇总
    # -------------------------------------------------------------
    print("=" * 60)
    print("  阶段三汇总")
    print("=" * 60)
    passed = sum(1 for r in results if r['verdict'] == '通过')
    print(f"通过 {passed}/{len(results)} 项测试")
    print()

    if passed == len(results):
        print("阶段三全部通过")
    else:
        print("阶段三未完成，部分测试需继续调参")
