"""
单通道独立调试脚本
==================
隔离各通道，逐一调参，观察阻尼与收敛特性。
"""

import numpy as np
import matplotlib.pyplot as plt
from aircraft_6dof import Aircraft6DOF, quaternion_to_euler, euler_to_quaternion, quaternion_multiply
from hover_pid_controller import HoverPID


def axis_angle_quat(axis, angle):
    """绕指定轴旋转angle弧度的四元数 [w, x, y, z]"""
    axis = np.array(axis) / np.linalg.norm(axis)
    return np.array([np.cos(angle/2),
                     axis[0]*np.sin(angle/2),
                     axis[1]*np.sin(angle/2),
                     axis[2]*np.sin(angle/2)])


def setup_chinese_font():
    """设置matplotlib支持中文显示"""
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
            print(f"警告: 在t={t[i-1]:.2f}s时状态溢出，停止仿真")
            stop_idx = i - 1
            break

        u[i-1] = control_func(t[i-1], x[i-1], dt)

        try:
            k1 = ac.derivatives(x[i-1], u[i-1], rho)
            k2 = ac.derivatives(x[i-1] + 0.5 * dt * k1, u[i-1], rho)
            k3 = ac.derivatives(x[i-1] + 0.5 * dt * k2, u[i-1], rho)
            k4 = ac.derivatives(x[i-1] + dt * k3, u[i-1], rho)
        except RuntimeError as e:
            print(f"警告: 在t={t[i-1]:.2f}s时导数计算失败: {e}")
            stop_idx = i - 1
            break

        if np.any(np.isnan(k1)) or np.any(np.abs(k1) > 1e6):
            print(f"警告: 在t={t[i-1]:.2f}s时导数溢出，停止仿真")
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


def compute_settling_time(t_arr, signal, target, deviation, threshold_ratio=0.02):
    """计算2%稳定时间（进入目标±threshold_ratio*deviation范围内且不再越出）"""
    threshold = threshold_ratio * abs(deviation)
    settled = False
    settling_time = None
    for i in range(len(signal)):
        if abs(signal[i] - target) <= threshold:
            # 检查之后是否一直保持在范围内
            if np.all(np.abs(signal[i:] - target) <= threshold):
                settling_time = t_arr[i]
                settled = True
                break
    return settled, settling_time, threshold


def compute_overshoot(signal, target, initial):
    """计算超调量百分比（相对于初始偏差）"""
    error_initial = abs(initial - target)
    if error_initial < 1e-9:
        return 0.0
    # 对于收敛到目标的信号，找越过目标的最大偏差
    if initial > target:
        # 初始大于目标，找最小值（过调）
        min_val = np.min(signal)
        if min_val < target:
            overshoot_abs = abs(min_val - target)
        else:
            overshoot_abs = 0.0
    else:
        # 初始小于目标，找最大值（过调）
        max_val = np.max(signal)
        if max_val > target:
            overshoot_abs = abs(max_val - target)
        else:
            overshoot_abs = 0.0

    overshoot_pct = (overshoot_abs / error_initial) * 100.0
    return overshoot_pct


def run_single_channel_debug(channel, Kp_att=None, Kp_rate=None, Ki_rate=None, Kd_rate=None,
                             Kp_alt=None, Kd_alt=None, t_end=5.0, dt=0.01, plot=True):
    """
    运行单通道独立调试仿真

    channel: 'roll', 'pitch', 'yaw', 'alt'
    返回: metrics字典
    """
    setup_chinese_font()
    ac = Aircraft6DOF()
    rho = 1.225

    # 创建控制器（阶段三已移除单通道隔离逻辑，全通道耦合）
    pid = HoverPID()

    # 应用外部传入的增益
    if Kp_att is not None:
        pid.Kp_att = Kp_att
    if Kp_rate is not None:
        pid.Kp_rate = Kp_rate
    if Ki_rate is not None:
        pid.Ki_rate = Ki_rate
    if Kd_rate is not None:
        pid.Kd_rate = Kd_rate
    if Kp_alt is not None:
        pid.Kp_alt = Kp_alt
    if Kd_alt is not None:
        pid.Kd_alt = Kd_alt

    # 获取悬停配平状态
    x0, u0, alpha_t, theta_t, rho = ac.trim(V_trim=0.0, h_trim=0.0)
    hover_throttle = u0[0]

    # 设置悬停点
    pid.set_hover_point(pos=[0.0, 0.0, 0.0], att=None, throttle=hover_throttle)
    pid.reset()

    q_hover = euler_to_quaternion(0.0, np.radians(90.0), 0.0)

    # 根据通道设置初始偏差（使用轴角四元数避免万向锁问题）
    if channel == 'roll':
        # 滚转偏差 5°（绕体轴x）
        init_deviation_deg = 5.0
        q_tilt = axis_angle_quat([1, 0, 0], np.radians(init_deviation_deg))
        q_init = quaternion_multiply(q_hover, q_tilt)
        x0[6:10] = [q_init[1], q_init[2], q_init[3], q_init[0]]
        target_val = 0.0
        deviation = init_deviation_deg
        rate_idx = 3  # p
        ctrl_label = 'de_diff'
        unit = 'deg'
        ylabel1 = '滚转角误差 (deg)'
        ylabel2 = '滚转角速度 p (deg/s)'
        ylabel3 = '差动舵面 de_diff (deg)'
        title = '滚转通道调试'
    elif channel == 'pitch':
        # 俯仰偏差 1°（后仰，绕体轴y正转）
        init_deviation_deg = 1.0
        q_tilt = axis_angle_quat([0, 1, 0], np.radians(init_deviation_deg))
        q_init = quaternion_multiply(q_hover, q_tilt)
        x0[6:10] = [q_init[1], q_init[2], q_init[3], q_init[0]]
        target_val = 90.0  # theta目标90°
        deviation = init_deviation_deg
        rate_idx = 4  # q
        ctrl_label = 'de_sym'
        unit = 'deg'
        ylabel1 = '俯仰角 θ (deg)'
        ylabel2 = '俯仰角速度 q (deg/s)'
        ylabel3 = '对称舵面 de_sym (deg)'
        title = '俯仰通道调试'
    elif channel == 'yaw':
        # 偏航偏差 10°（绕体轴z）
        init_deviation_deg = 10.0
        q_tilt = axis_angle_quat([0, 0, 1], np.radians(init_deviation_deg))
        q_init = quaternion_multiply(q_hover, q_tilt)
        x0[6:10] = [q_init[1], q_init[2], q_init[3], q_init[0]]
        target_val = 0.0
        deviation = init_deviation_deg
        rate_idx = 5  # r
        ctrl_label = 'throttle_diff'
        unit = ''
        ylabel1 = '偏航角误差 (deg)'
        ylabel2 = '偏航角速度 r (deg/s)'
        ylabel3 = '差动油门 throttle_diff'
        title = '偏航通道调试'
    elif channel == 'alt':
        # 高度偏差 1m（在目标下方1m）
        init_pz = 1.0
        x0[12] = init_pz
        pid.target_pos = np.array([0.0, 0.0, 0.0])
        target_val = 0.0  # pz目标0m
        deviation = init_pz
        rate_idx = 2  # w (爬升率)
        ctrl_label = 'throttle'
        unit = ''
        ylabel1 = '高度 pz (m)'
        ylabel2 = '爬升率 -w (m/s)'
        ylabel3 = '油门 throttle'
        title = '高度通道调试'
    else:
        raise ValueError(f"未知通道: {channel}")

    # 仿真
    def control_func(t, x, dt):
        return pid.compute(t, x, dt)

    t, x_hist, u_hist = integrate_6dof_quaternion(ac, x0, u0, rho, control_func, (0, t_end), dt=dt)

    # 提取响应
    if channel in ('roll', 'pitch', 'yaw'):
        phi_deg = []
        theta_deg = []
        psi_deg = []
        roll_err_deg = []
        pitch_err_deg = []
        yaw_err_deg = []
        for i in range(len(x_hist)):
            qx, qy, qz, qw = x_hist[i, 6:10]
            q_current = np.array([qw, qx, qy, qz])
            phi, theta, psi = quaternion_to_euler(q_current)
            phi_deg.append(np.degrees(phi))
            theta_deg.append(np.degrees(theta))
            psi_deg.append(np.degrees(psi))

            # 计算相对悬停姿态的旋转矢量（避免万向锁）
            q_current_inv = np.array([qw, -qx, -qy, -qz])
            q_rel = quaternion_multiply(q_current_inv, q_hover)
            if q_rel[0] < 0:
                q_rel = -q_rel
            att_vec = 2.0 * q_rel[1:4]  # [roll_err, pitch_err, yaw_err] in rad
            roll_err_deg.append(np.degrees(att_vec[0]))
            pitch_err_deg.append(np.degrees(att_vec[1]))
            yaw_err_deg.append(np.degrees(att_vec[2]))

        phi_deg = np.array(phi_deg)
        theta_deg = np.array(theta_deg)
        psi_deg = np.array(psi_deg)
        roll_err_deg = np.array(roll_err_deg)
        pitch_err_deg = np.array(pitch_err_deg)
        yaw_err_deg = np.array(yaw_err_deg)

        if channel == 'roll':
            signal = roll_err_deg
            rate_signal = np.degrees(x_hist[:, rate_idx])
        elif channel == 'pitch':
            signal = theta_deg
            rate_signal = np.degrees(x_hist[:, rate_idx])
        elif channel == 'yaw':
            signal = yaw_err_deg
            rate_signal = np.degrees(x_hist[:, rate_idx])

        throttle_left = u_hist[:, 0]
        throttle_right = u_hist[:, 1]
        de_left = np.degrees(u_hist[:, 2])
        de_right = np.degrees(u_hist[:, 3])
        de_sym = 0.5 * (de_left + de_right)
        de_diff = 0.5 * (de_right - de_left)
        throttle_diff = 0.5 * (throttle_right - throttle_left)

        if ctrl_label == 'de_diff':
            ctrl_signal = de_diff
        elif ctrl_label == 'de_sym':
            ctrl_signal = de_sym
        else:
            ctrl_signal = throttle_diff

    else:  # alt
        signal = x_hist[:, 12]
        rate_signal = -x_hist[:, 2]  # NED坐标，-w为爬升率
        ctrl_signal = u_hist[:, 0]  # 左油门（左右对称）

    # 计算指标
    overshoot = compute_overshoot(signal, target_val, signal[0])
    settled, settling_time, threshold = compute_settling_time(t, signal, target_val, deviation)
    ctrl_peak = np.max(np.abs(ctrl_signal))
    ctrl_unit = unit if channel != 'alt' else ''

    # 最终误差
    final_error = abs(signal[-1] - target_val)

    # 绘图
    if plot:
        fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

        # 上图：被控量
        ax = axes[0]
        ax.plot(t, signal, 'b-', linewidth=1.5, label='响应')
        ax.axhline(target_val, color='r', linestyle='--', alpha=0.7, label=f'目标值 {target_val}')
        if settled and settling_time is not None:
            ax.axvline(settling_time, color='g', linestyle='--', alpha=0.7, label=f'稳定时间 {settling_time:.2f}s')
            ax.axhline(target_val + threshold, color='g', linestyle=':', alpha=0.5)
            ax.axhline(target_val - threshold, color='g', linestyle=':', alpha=0.5)
        ax.set_ylabel(ylabel1)
        ax.set_title(title)
        ax.legend(loc='upper right')
        ax.grid(True)

        # 标注超调量
        if overshoot > 0.1:
            if signal[0] > target_val:
                overshoot_idx = np.argmin(signal)
            else:
                overshoot_idx = np.argmax(signal)
            ax.annotate(f'超调 {overshoot:.1f}%',
                        xy=(t[overshoot_idx], signal[overshoot_idx]),
                        xytext=(t[overshoot_idx] + 0.2, signal[overshoot_idx]),
                        arrowprops=dict(arrowstyle='->', color='purple'),
                        color='purple')

        # 中图：速率
        ax = axes[1]
        ax.plot(t, rate_signal, 'b-', linewidth=1.5)
        ax.axhline(0, color='k', linestyle='--', alpha=0.3)
        ax.set_ylabel(ylabel2)
        ax.grid(True)

        # 下图：控制量
        ax = axes[2]
        ax.plot(t, ctrl_signal, 'b-', linewidth=1.5)
        if channel == 'pitch':
            limit = np.degrees(pid.max_de)
        elif channel == 'roll':
            limit = np.degrees(pid.max_de)
        elif channel == 'yaw':
            limit = pid.max_throttle
        else:
            limit = pid.max_throttle
        ax.axhline(limit * 0.7, color='orange', linestyle='--', alpha=0.5, label='70%限幅')
        ax.axhline(-limit * 0.7, color='orange', linestyle='--', alpha=0.5)
        ax.axhline(limit, color='r', linestyle='--', alpha=0.5, label='限幅')
        ax.axhline(-limit, color='r', linestyle='--', alpha=0.5)
        ax.set_ylabel(ylabel3)
        ax.set_xlabel('时间 (s)')
        ax.legend(loc='upper right')
        ax.grid(True)

        plt.tight_layout()
        fname = f'hover_debug_{channel}.png'
        plt.savefig(fname, dpi=150, bbox_inches='tight')
        plt.close()
        print(f"  图形已保存: {fname}")

    metrics = {
        'channel': channel,
        'Kp_att': pid.Kp_att,
        'Kp_rate': pid.Kp_rate,
        'Ki_rate': pid.Ki_rate,
        'Kd_rate': pid.Kd_rate,
        'Kp_alt': pid.Kp_alt,
        'overshoot': overshoot,
        'settled': settled,
        'settling_time': settling_time,
        'ctrl_peak': ctrl_peak,
        'ctrl_unit': ctrl_unit,
        'final_error': final_error,
        't_end': t_end,
    }
    return metrics


def print_channel_result(metrics):
    """按指定格式输出调试结论"""
    channel = metrics['channel']
    if channel == 'roll':
        name = '【滚转通道】'
    elif channel == 'pitch':
        name = '【俯仰通道】'
    elif channel == 'yaw':
        name = '【偏航通道】'
    elif channel == 'alt':
        name = '【高度通道】'
    else:
        name = f'【{channel}】'

    print(f"> {name}调试结果")
    if channel == 'alt':
        print(f"> - 最终增益：Kp_alt={metrics['Kp_alt']:.2f}")
    else:
        print(f"> - 最终增益：Kp_att={metrics['Kp_att']:.2f}, Kp_rate={metrics['Kp_rate']:.2f}, Ki_rate={metrics['Ki_rate']:.2f}")
    print(f"> - 超调量：{metrics['overshoot']:.1f}%")
    if metrics['settled'] and metrics['settling_time'] is not None:
        print(f"> - 2%稳定时间：{metrics['settling_time']:.2f}s")
    else:
        print(f"> - 2%稳定时间：未在{metrics['t_end']:.1f}s内达到")
    print(f"> - 控制量峰值：{metrics['ctrl_peak']:.4f} ({metrics['ctrl_unit']})")

    # 判定
    ch = metrics['channel']
    # 稳定时间阈值：滚转/俯仰<3s，偏航<5s（惯性大），高度<8s
    if ch == 'alt':
        ts_limit = 8.0
    elif ch == 'yaw':
        ts_limit = 5.0
    else:
        ts_limit = 3.0
    # 控制量限幅阈值（舵面已转为deg，油门为无量纲）
    ctrl_limit = 20.0 if ch in ('roll', 'pitch') else 0.7

    if not metrics['settled']:
        verdict = "需继续调"
        next_step = "增益不足或发散，检查极性并适当增大Kp_rate/Kp_att"
    elif metrics['overshoot'] > 50:
        verdict = "需继续调"
        next_step = "超调过大，增大Kd_rate或减小Kp_att"
    elif metrics['ctrl_peak'] > ctrl_limit:
        verdict = "需继续调"
        next_step = "控制量触碰限幅，降低对应增益"
    elif metrics['settling_time'] and metrics['settling_time'] > ts_limit:
        verdict = "需继续调"
        next_step = "响应过慢，逐步增大Kp_rate/Kp_att"
    else:
        verdict = "通过"
        next_step = ""

    print(f"> - 判定：{verdict}")
    if next_step:
        print(f"> - 若未通过，下一步调整方向：{next_step}")
    print()
    return verdict == "通过"


if __name__ == "__main__":
    setup_chinese_font()
    print("=" * 60)
    print("  单通道独立调试")
    print("=" * 60)

    results = {}
    passed = {}

    # --------------------------------------------------------------
    # 1. 俯仰通道调试
    # --------------------------------------------------------------
    print("\n[1/4] 俯仰通道调试：初始偏差 theta=91°")
    # 先尝试当前默认参数
    m = run_single_channel_debug('pitch', t_end=5.0)
    results['pitch'] = m
    passed['pitch'] = print_channel_result(m)

    # --------------------------------------------------------------
    # 2. 滚转通道调试
    # --------------------------------------------------------------
    print("\n[2/4] 滚转通道调试：初始偏差 phi=5°")
    m = run_single_channel_debug('roll', t_end=5.0)
    results['roll'] = m
    passed['roll'] = print_channel_result(m)

    # --------------------------------------------------------------
    # 3. 偏航通道调试
    # --------------------------------------------------------------
    print("\n[3/4] 偏航通道调试：初始偏差 psi=10°")
    m = run_single_channel_debug('yaw', t_end=5.0)
    results['yaw'] = m
    passed['yaw'] = print_channel_result(m)

    # --------------------------------------------------------------
    # 4. 高度通道调试
    # --------------------------------------------------------------
    print("\n[4/4] 高度通道调试：初始偏差 pz=1m")
    m = run_single_channel_debug('alt', t_end=10.0)
    results['alt'] = m
    passed['alt'] = print_channel_result(m)

    # --------------------------------------------------------------
    # 汇总
    # --------------------------------------------------------------
    print("=" * 60)
    print("  增益汇总表")
    print("=" * 60)
    print(f"{'通道':<10} {'Kp_att':<10} {'Kp_rate':<10} {'Ki_rate':<10} {'Kp_alt':<10} {'状态':<6}")
    print("-" * 60)
    for ch in ['pitch', 'roll', 'yaw', 'alt']:
        m = results[ch]
        status = "通过" if passed[ch] else "待调"
        if ch == 'alt':
            print(f"{ch:<10} {'-':<10} {'-':<10} {'-':<10} {m['Kp_alt']:<10.2f} {status:<6}")
        else:
            print(f"{ch:<10} {m['Kp_att']:<10.2f} {m['Kp_rate']:<10.2f} {m['Ki_rate']:<10.2f} {'-':<10} {status:<6}")

    if all(passed.values()):
        print("\n阶段二全部通过")
    else:
        print("\n阶段二未完成，部分通道需继续调参")
