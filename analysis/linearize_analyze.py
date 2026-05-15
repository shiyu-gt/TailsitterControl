"""
固定翼飞机线性化与模态分析
===============================================
从 aircraft_6dof.py 中获取 6DOF 模型，在配平条件下进行线性化，
并将结果系统分离为：

  - 纵向状态空间：  A_lon, B_lon  (状态: Δu, Δw, Δq, Δθ)
  - 横航向状态空间： A_lat, B_lat  (状态: Δv, Δp, Δr, Δφ)

然后对每个子系统进行特征值/模态分析。

使用方法：
    python -m analysis.linearize_analyze
"""

import numpy as np
from core import Aircraft6DOF, isa_atmosphere


# ═══════════════════════════════════════════════════════════════════════════════
#  本脚本通过12状态包装函数调用13状态四元数模型，欧拉角仅用于线性化和
#  模态分析输出，实际动力学仍由四元数模型计算。
# ═══════════════════════════════════════════════════════════════════════════════

def euler_to_quat_yaw_first(yaw, pitch, roll):
    """
    将欧拉角 (yaw, pitch, roll) 转换为四元数 [w, x, y, z]。
    顺序与 aircraft_6dof.py 的 euler_to_quaternion(phi, theta, psi) 一致（ZYX）。
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([w, x, y, z])


def quat_to_euler(q0, q1, q2, q3):
    """
    从四元数（标量 q0 在前）提取欧拉角 [phi, theta, psi]。
    与 aircraft_6dof.py 的 quaternion_to_euler(q) 一致。
    """
    w, x, y, z = q0, q1, q2, q3

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    phi = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        theta = np.copysign(np.pi / 2, sinp)
    else:
        theta = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    psi = np.arctan2(siny_cosp, cosy_cosp)

    return phi, theta, psi


def derivatives_12state(x_12, u, rho, ac):
    """
    12状态包装函数：将12状态(欧拉角)转换为13状态(四元数)，
    调用 ac.derivatives()，再将结果转换回12状态导数。

    参数：
        x_12: [u, v, w, p, q, r, phi, theta, psi, px, py, pz]
        u:    [throttle_left, throttle_right, de_left, de_right] (4维真实控制)
        rho:  空气密度
        ac:   Aircraft6DOF 实例
    """
    phi, theta, psi = x_12[6:9]
    q = euler_to_quat_yaw_first(psi, theta, phi)  # [qw, qx, qy, qz]

    x_13 = np.array([
        x_12[0], x_12[1], x_12[2],   # u, v, w
        x_12[3], x_12[4], x_12[5],   # p, q, r
        q[1], q[2], q[3], q[0],      # qx, qy, qz, qw
        x_12[9], x_12[10], x_12[11]  # px, py, pz
    ])

    dx_13 = ac.derivatives(x_13, u, rho)

    du, dv, dw = dx_13[0:3]
    dp, dq, dr = dx_13[3:6]
    dpx, dpy, dpz = dx_13[10:13]

    p, q_rate, r = x_12[3:6]

    # 欧拉角速率转换矩阵（仅用于前飞，theta 远离 90°）
    dphi = p + q_rate * np.sin(phi) * np.tan(theta) + r * np.cos(phi) * np.tan(theta)
    dtheta = q_rate * np.cos(phi) - r * np.sin(phi)
    dpsi = q_rate * np.sin(phi) / np.cos(theta) + r * np.cos(phi) / np.cos(theta)

    dx_12 = np.array([du, dv, dw, dp, dq, dr, dphi, dtheta, dpsi, dpx, dpy, dpz])
    return dx_12


# ═══════════════════════════════════════════════════════════════════════════════
#  使用中心差分法进行数值雅可比计算
# ═══════════════════════════════════════════════════════════════════════════════

def jacobian(f, x0, u0, args=(), eps=1e-6):
    """
    通过中心差分法计算 A = ∂f/∂x 和 B = ∂f/∂u。
    f : 可调用函数(x, u, *args) → dx (与 x 形状相同)
    """
    nx = len(x0)
    nu = len(u0)
    f0 = f(x0, u0, *args)
    A = np.zeros((nx, nx))
    B = np.zeros((nx, nu))
    for j in range(nx):
        xp = x0.copy(); xp[j] += eps
        xm = x0.copy(); xm[j] -= eps
        A[:, j] = (f(xp, u0, *args) - f(xm, u0, *args)) / (2 * eps)
    for j in range(nu):
        up = u0.copy(); up[j] += eps
        um = u0.copy(); um[j] -= eps
        B[:, j] = (f(x0, up, *args) - f(x0, um, *args)) / (2 * eps)
    return A, B


# ═══════════════════════════════════════════════════════════════════════════════
#  配平与完整线性化
# ═══════════════════════════════════════════════════════════════════════════════

def full_linearisation(V_trim, gamma_trim=0.0, h_trim=0.0):
    """
    在机翼水平配平条件下对 6DOF 模型进行线性化（通过12状态包装函数）。
    返回：
        A_full, B_full, x_trim_12, u_equiv, alpha_trim, theta_trim, rho, ac, x_trim_13, u_trim_4d
    """
    ac = Aircraft6DOF()
    x_trim_13, u_trim_4d, alpha_trim, theta_trim, rho = ac.trim(V_trim, gamma_trim, h_trim)

    # 将13状态配平向量转换为12状态
    phi, theta, psi = quat_to_euler(x_trim_13[9], x_trim_13[6], x_trim_13[7], x_trim_13[8])
    x_trim_12 = np.array([
        *x_trim_13[:6],
        phi, theta, psi,
        *x_trim_13[10:]
    ])

    # 将4维真实控制映射为等效控制量
    throttle_left, throttle_right, de_left, de_right = u_trim_4d
    de_sym = (de_left + de_right) / 2.0
    de_diff = (de_right - de_left) / 2.0
    throttle_diff = (throttle_right - throttle_left) / 2.0
    total_throttle = (throttle_left + throttle_right) / 2.0
    u_equiv = np.array([de_sym, de_diff, throttle_diff, total_throttle])

    def f_12state_equiv(x_12, u_equiv, rho, ac):
        """等效控制 → 真实4维控制 → 12状态导数"""
        de_sym, de_diff, throttle_diff, total_throttle = u_equiv
        de_left = de_sym - de_diff
        de_right = de_sym + de_diff
        throttle_left = total_throttle - throttle_diff
        throttle_right = total_throttle + throttle_diff
        u_real = np.array([throttle_left, throttle_right, de_left, de_right])
        return derivatives_12state(x_12, u_real, rho, ac)

    A, B = jacobian(f_12state_equiv, x_trim_12, u_equiv, args=(rho, ac))
    return A, B, x_trim_12, u_equiv, alpha_trim, theta_trim, rho, ac, x_trim_13, u_trim_4d


# ═══════════════════════════════════════════════════════════════════════════════
#  分离为纵向/横航向子系统
# ═══════════════════════════════════════════════════════════════════════════════

def extract_subsystems(A, B, x_trim, u_trim, alpha_trim, theta_trim, rho):
    """
    从 12 状态线性化系统中提取 4 状态纵向
    和 4 状态横航向子系统。

    纵向状态（索引）: 0=u, 2=w, 4=q, 7=theta
    横航向状态（索引）: 1=v, 3=p, 5=r, 6=phi

    纵向输入：0=de_sym, 3=T_total
    横航向输入：1=de_diff, 2=throttle_diff
    """
    # ── Longitudinal ──
    lon_states = [0, 2, 4, 7]   # u, w, q, theta
    lon_inputs = [0, 3]         # de, T
    A_lon = A[np.ix_(lon_states, lon_states)]
    B_lon = B[np.ix_(lon_states, lon_inputs)]

    # ── 横航向 ──
    lat_states = [1, 3, 5, 6]   # v, p, r, phi
    lat_inputs = [1, 2]         # da, dr
    A_lat = A[np.ix_(lat_states, lat_states)]
    B_lat = B[np.ix_(lat_states, lat_inputs)]

    return A_lon, B_lon, A_lat, B_lat


# ═══════════════════════════════════════════════════════════════════════════════
#  模态分析
# ═══════════════════════════════════════════════════════════════════════════════

def modal_analysis(A, state_names, subsystem_name):
    """
    计算特征值和模态特性。
    返回模式字典列表，每个模式一个字典。
    """
    eigvals, eigvecs = np.linalg.eig(A)

    modes = []
    used = set()
    for i, ev in enumerate(eigvals):
        if i in used:
            continue
        mode = {"eigenvalue": ev}
        real, imag = ev.real, ev.imag
        mode["real"] = real
        mode["imag"] = imag

        if abs(imag) > 1e-8:
            # Oscillatory mode — pair with conjugate
            j = None
            for k in range(i + 1, len(eigvals)):
                if k in used:
                    continue
                if abs(eigvals[k] - np.conj(ev)) < 1e-6:
                    j = k
                    break
            if j is not None:
                used.add(j)
            wn = np.sqrt(real**2 + imag**2)
            zeta = -real / wn if wn > 1e-10 else 0.0
            T = 2 * np.pi / wn if wn > 1e-10 else np.inf
            mode["type"] = "oscillatory"
            mode["wn_rad_s"] = wn
            mode["wn_hz"] = wn / (2 * np.pi)
            mode["zeta"] = zeta
            mode["period_s"] = T
            mode["time_to_half_s"] = (np.log(2) / abs(real)) if abs(real) > 1e-10 else np.inf
            mode["time_to_double_s"] = (-np.log(2) / real) if real < -1e-10 else np.inf
        else:
            # Real mode
            mode["type"] = "real"
            if abs(real) > 1e-10:
                tau = 1.0 / abs(real)
                mode["tau_s"] = tau
                if real < 0:
                    mode["time_const_s"] = tau
                else:
                    mode["time_to_double_s"] = np.log(2) / real
        # 特征向量主导状态（取模最大分量）
        vec = eigvecs[:, i]
        dom_idx = int(np.argmax(np.abs(vec)))
        mode["dominant_state_idx"] = dom_idx
        mode["dominant_state_name"] = state_names[dom_idx]

        modes.append(mode)
        used.add(i)

    # Sort by natural frequency descending
    modes.sort(key=lambda m: abs(m["eigenvalue"]), reverse=True)
    return modes


def print_modes(modes, subsystem_name, state_names):
    """美观地打印模态分析结果。"""
    hline = "=" * 72
    print(f"\n{hline}")
    print(f"  {subsystem_name} — Modal Analysis")
    print(f"  States: {state_names}")
    print(hline)

    for i, m in enumerate(modes):
        print(f"\n  Mode {i+1}:")
        ev = m["eigenvalue"]
        if m["type"] == "oscillatory":
            print(f"    Eigenvalue   : {ev: .5f}")
            print(f"    ω_n          : {m['wn_rad_s']:.4f} rad/s  ({m['wn_hz']:.4f} Hz)")
            print(f"    ζ            : {m['zeta']:.5f}")
            print(f"    Period       : {m['period_s']:.3f} s")
            if m["zeta"] > 0:
                print(f"    t_{1/2}       : {m['time_to_half_s']:.3f} s")
            else:
                print(f"    t_2          : {m['time_to_double_s']:.3f} s")
        else:
            print(f"    Eigenvalue   : {ev: .5f}")
            if abs(ev) < 1e-10:
                print(f"    Neutral mode : marginally stable (zero root)")
            elif ev < 0:
                print(f"    Time const   : {m['tau_s']:.3f} s  (stable convergence)")
            else:
                print(f"    t_2          : {m['time_to_double_s']:.3f} s  (unstable divergence)")

    print(f"\n{hline}")


# ═══════════════════════════════════════════════════════════════════════════════
#  识别和标记经典模式
# ═══════════════════════════════════════════════════════════════════════════════

def identify_longitudinal_modes(modes):
    """标记短周期和长周期模式。"""
    labels = []
    osc = [m for m in modes if m["type"] == "oscillatory"]
    real = [m for m in modes if m["type"] == "real"]

    if len(osc) >= 2:
        # Higher frequency → short period
        sp = max(osc, key=lambda m: m["wn_rad_s"])
        ph = min(osc, key=lambda m: m["wn_rad_s"])
        labels.append(("Short Period", sp))
        labels.append(("Phugoid", ph))
    elif len(osc) == 1:
        # 综合频率与主导状态判断
        o = osc[0]
        is_low_freq = o["wn_rad_s"] < 2.0
        is_u_dominant = o["dominant_state_idx"] == 0   # Δu 主导
        if is_low_freq and is_u_dominant:
            labels.append(("Phugoid (probable)", o))
        else:
            labels.append(("Short Period (probable)", o))

    for m in real:
        if abs(m.get("tau_s", np.inf)) < 5:
            labels.append(("Fast real mode", m))
        else:
            labels.append(("Slow real mode", m))

    return labels


def identify_lateral_modes(modes):
    """标记荷兰滚、滚转和螺旋模式。"""
    labels = []
    osc = [m for m in modes if m["type"] == "oscillatory"]
    real = sorted([m for m in modes if m["type"] == "real"],
                  key=lambda m: m.get("tau_s", 1e9))

    if osc:
        o = osc[0]
        # 荷兰滚典型频率 0.1~2.0 rad/s；若频率异常高/低则加 probable 标记
        if 0.1 < o["wn_rad_s"] < 3.0:
            labels.append(("Dutch Roll", o))
        else:
            labels.append(("Dutch Roll (probable)", o))
    if len(real) >= 2:
        labels.append(("Roll subsidence", real[0]))
        labels.append(("Spiral mode", real[1]))
    elif len(real) == 1:
        labels.append(("Roll/Spiral", real[0]))

    return labels


# ═══════════════════════════════════════════════════════════════════════════════
#  打印状态空间矩阵
# ═══════════════════════════════════════════════════════════════════════════════

def print_ss(name, A, B, state_names, input_names):
    print(f"\n{'─' * 72}")
    print(f"  {name}")
    print(f"  States : {state_names}")
    print(f"  Inputs : {input_names}")
    print(f"{'─' * 72}")
    print("\n  A matrix:")
    row_fmt = "    {:>10s}" + "  {:>12.5f}" * A.shape[1]
    for i, row in enumerate(A):
        print(row_fmt.format(state_names[i], *row))
    print("\n  B matrix:")
    b_fmt = "    {:>10s}" + "  {:>12.5f}" * B.shape[1]
    for i, row in enumerate(B):
        print(b_fmt.format(state_names[i], *row))


# ═══════════════════════════════════════════════════════════════════════════════
#  主函数
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    # ── 配置 ──
    V_trim = 20.0       # trim airspeed [m/s]
    gamma_trim = 0.0    # flight-path angle [rad] (0 = level)
    h_trim = 20.0        # altitude [m]

    print("=" * 72)
    print("  固定翼飞机 6DOF 线性化与模态分析")
    print("=" * 72)
    print(f"\n  配平速度      : {V_trim:.1f} m/s ({V_trim * 3.6:.1f} km/h)")
    print(f"  航迹角      : {np.degrees(gamma_trim):.1f}°")
    print(f"  高度          : {h_trim:.0f} m")

    # ── 线性化 ──
    A, B, x_trim_12, u_equiv, alpha_t, theta_t, rho, ac, x_trim_13, u_trim_4d = full_linearisation(
        V_trim, gamma_trim, h_trim)

    print(f"\n  配平攻角 : {np.degrees(alpha_t):.3f}°")
    print(f"  配平俯仰角 : {np.degrees(theta_t):.3f}°")
    print(f"  配平对称舵面 : {np.degrees(u_equiv[0]):.3f}°")
    print(f"  配平油门 : {u_equiv[3]:.3f}")
    print(f"  空气密度: {rho:.4f} kg/m^3")

    # ── 验证配平残差（13状态原始模型）──
    dx_13 = ac.derivatives(x_trim_13, u_trim_4d, rho)
    mech_residual = np.max(np.abs(dx_13[:9]))
    print(f"  配平残差（13状态） max|dx[0:9]|: {mech_residual:.2e}")

    # ── 验证12状态包装函数一致性 ──
    dx_12 = derivatives_12state(x_trim_12, u_trim_4d, rho, ac)
    mech_residual_12 = np.max(np.abs(dx_12[:9]))
    print(f"  配平残差（12状态） max|dx[0:9]|: {mech_residual_12:.2e}")

    # ── 提取子系统 ──
    A_lon, B_lon, A_lat, B_lat = extract_subsystems(
        A, B, x_trim_12, u_equiv, alpha_t, theta_t, rho)

    # ── 纵向 ──
    lon_states = ["Δu", "Δw", "Δq", "Δθ"]
    lon_inputs = ["δe_sym", "T_total"]
    print_ss("纵向子系统", A_lon, B_lon, lon_states, lon_inputs)
    modes_lon = modal_analysis(A_lon, lon_states, "纵向")
    print_modes(modes_lon, "纵向", lon_states)

    lon_labels = identify_longitudinal_modes(modes_lon)
    print("\n  识别的经典纵向模式：")
    for label, m in lon_labels:
        ev_str = f"{m['eigenvalue']:.5f}"
        if m["type"] == "oscillatory":
            extra = f"  ωn={m['wn_rad_s']:.3f} rad/s, ζ={m['zeta']:.4f}"
        else:
            extra = f"  τ={m.get('tau_s', np.inf):.3f} s"
        print(f"    {label:30s}  λ={ev_str}{extra}")

    # ── 横航向 ──
    lat_states = ["Δv", "Δp", "Δr", "Δφ"]
    lat_inputs = ["δe_diff", "δT_diff"]
    print_ss("横航向子系统", A_lat, B_lat, lat_states, lat_inputs)
    modes_lat = modal_analysis(A_lat, lat_states, "横航向")
    print_modes(modes_lat, "横航向", lat_states)

    lat_labels = identify_lateral_modes(modes_lat)
    print("\n  识别的经典横航向模式：")
    for label, m in lat_labels:
        ev_str = f"{m['eigenvalue']:.5f}"
        if m["type"] == "oscillatory":
            extra = f"  ωn={m['wn_rad_s']:.3f} rad/s, ζ={m['zeta']:.4f}"
        else:
            extra = f"  τ={m.get('tau_s', np.inf):.3f} s"
        print(f"    {label:30s}  λ={ev_str}{extra}")

    # ── 稳定性总结 ──
    print(f"\n{'=' * 72}")
    print("  稳定性总结")
    print(f"{'=' * 72}")
    all_eigs = np.linalg.eigvals(np.vstack([
        np.hstack([A_lon, np.zeros((4, 4))]),
        np.hstack([np.zeros((4, 4)), A_lat])]))
    stable = all(np.real(e) < 0 for e in all_eigs)
    print(f"  总体线性稳定性 : {'稳定 [OK]' if stable else '不稳定 [FAIL]'}")
    print(f"  纵向特征值 : {np.linalg.eigvals(A_lon)}")
    print(f"  横航向特征值 : {np.linalg.eigvals(A_lat)}")

    return A_lon, B_lon, A_lat, B_lat


if __name__ == "__main__":
    main()
