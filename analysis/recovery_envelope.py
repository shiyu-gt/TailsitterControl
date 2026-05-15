"""
回收过程减速能力包络线分析
============================
扫描不同速度/迎角/油门组合下的减速能力、升降副翼效能和滑流效率，
为 RecoveryController 调参提供定量依据。
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from core import Aircraft6DOF, euler_to_quaternion, isa_atmosphere


def build_state(V, alpha, theta, throttle, de_sym, rho, h=50.0):
    """构造给定飞行条件的状态向量和控制向量。"""
    u = V * np.cos(alpha)
    w = V * np.sin(alpha)
    q = euler_to_quaternion(0.0, theta, 0.0)
    x = np.array([u, 0.0, w, 0.0, 0.0, 0.0,
                   q[1], q[2], q[3], q[0],
                   0.0, 0.0, h])
    u_ctrl = np.array([throttle, throttle, de_sym, de_sym])
    return x, u_ctrl


def run_envelope_analysis():
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    ac = Aircraft6DOF()
    rho, _ = isa_atmosphere(50.0)

    speeds = np.arange(20, 2, -1)  # 20 down to 3 m/s
    alphas_deg = [5, 10, 15, 18, 20, 22, 25]
    alphas_rad = [np.radians(a) for a in alphas_deg]

    # ── 1. 配平扫描 ──
    print("=" * 60)
    print("  配平扫描 (V=20..3 m/s)")
    print("=" * 60)
    trim_results = []
    for V in speeds:
        try:
            x_trim, u_trim, alpha_t, theta_t, _ = ac.trim(V_trim=V, h_trim=50.0)
            # 有效性检查
            if u_trim[0] < 0 or u_trim[0] > 1 or abs(alpha_t) > np.radians(90):
                print(f"  V={V:5.1f}  配平无效 (throttle={u_trim[0]:.3f}, alpha={np.degrees(alpha_t):.1f}°)")
                continue
            # 计算配平状态下的 du（零附加俯仰）
            dx = ac.derivatives(x_trim, u_trim, rho)
            du = dx[0]
            throttle_t = u_trim[0]
            de_t = u_trim[2]
            trim_results.append({
                'V': V, 'alpha_deg': np.degrees(alpha_t), 'theta_deg': np.degrees(theta_t),
                'throttle': throttle_t, 'de_deg': np.degrees(de_t), 'du': du
            })
            print(f"  V={V:5.1f}  alpha={np.degrees(alpha_t):6.2f}°  "
                  f"theta={np.degrees(theta_t):6.2f}°  throttle={throttle_t:.4f}  "
                  f"de={np.degrees(de_t):6.2f}°  du={du:+.4f} m/s²")
        except Exception as e:
            print(f"  V={V:5.1f}  配平失败: {e}")

    # ── 2. Alpha-限制减速扫描 (throttle=0.05, Stage A) ──
    print("\n" + "=" * 60)
    print("  Alpha-限制减速扫描 (throttle=0.05)")
    print("=" * 60)
    throttle_fixed = 0.05
    decel_table = {}  # {V: {alpha_deg: decel}}
    for V in speeds:
        decel_table[V] = {}
        for alpha_d, alpha in zip(alphas_deg, alphas_rad):
            theta = alpha  # 简化：theta ≈ alpha (gamma=0)
            x, u_ctrl = build_state(V, alpha, theta, throttle_fixed, 0.0, rho)
            dx = ac.derivatives(x, u_ctrl, rho)
            du = dx[0]
            decel_table[V][alpha_d] = du
        row = [f"{decel_table[V][a]:+.3f}" for a in alphas_deg]
        print(f"  V={V:5.1f}  decel by alpha: {'  '.join(row)}")

    # ── 3. 不同油门下的减速能力 ──
    print("\n" + "=" * 60)
    print("  不同油门下 alpha=15° 的减速能力")
    print("=" * 60)
    throttles = [0.02, 0.03, 0.05, 0.08, 0.10]
    alpha_fixed = np.radians(15)
    throttle_decel = {}
    for thr in throttles:
        throttle_decel[thr] = []
        for V in speeds:
            x, u_ctrl = build_state(V, alpha_fixed, alpha_fixed, thr, 0.0, rho)
            dx = ac.derivatives(x, u_ctrl, rho)
            throttle_decel[thr].append(dx[0])
        avg = np.mean(throttle_decel[thr])
        print(f"  throttle={thr:.3f}  avg_decel={avg:+.4f} m/s²  "
              f"range=[{min(throttle_decel[thr]):+.3f}, {max(throttle_decel[thr]):+.3f}]")

    # ── 4. 升降副翼效能 vs 速度 ──
    print("\n" + "=" * 60)
    print("  升降副翼效能 (de=30° 俯仰力矩)")
    print("=" * 60)
    de_max = np.radians(30)
    elevon_moments = []
    for V in speeds:
        # 计算 q_inf
        q_inf = 0.5 * rho * V**2
        # 舵面增量
        dCL_e, dCD_e, dCm_e = ac.aero.get_elevon_longitudinal_coefficients(de_max)
        M_elevon = q_inf * ac.aero.S * ac.aero.c_bar * dCm_e
        elevon_moments.append(M_elevon)
        print(f"  V={V:5.1f}  q_inf={q_inf:7.2f} Pa  dCm={dCm_e:+.4f}  M_elevon={M_elevon:+.4f} Nm")

    # ── 5. 滑流效率分析 ──
    print("\n" + "=" * 60)
    print("  滑流效率 (throttle=0.05)")
    print("=" * 60)
    slipstream_ratios = []
    for V in speeds:
        T = ac.aero.thrust_from_throttle(throttle_fixed)
        v_slip = V + np.sqrt(2 * T / (rho * ac.aero.A_prop))
        q_inf = 0.5 * rho * V**2
        q_slip = 0.5 * rho * v_slip**2
        ratio = q_slip / q_inf if q_inf > 0 else 1.0
        slipstream_ratios.append(ratio)
        print(f"  V={V:5.1f}  T={T:.2f}N  v_slip={v_slip:.2f}m/s  "
              f"q_slip/q_inf={ratio:.2f}")

    # ── 6. 高迎角低头力矩 vs 升降副翼抬头力矩 ──
    print("\n" + "=" * 60)
    print("  力矩平衡检查 (高迎角低头 vs 升降副翼抬头)")
    print("=" * 60)
    for V in [20, 15, 12, 10, 8, 5]:
        q_inf = 0.5 * rho * V**2
        for alpha_deg in [15, 20, 25]:
            alpha = np.radians(alpha_deg)
            CL_b, CD_b, CY_b, Cl_b, Cm_b, Cn_b = ac.aero.get_base_coefficients(
                alpha, 0.0, 0.0, 0.0, 0.0)
            M_aero = q_inf * ac.aero.S * ac.aero.c_bar * Cm_b
            # 升降副翼最大抬头力矩
            _, _, dCm_e = ac.aero.get_elevon_longitudinal_coefficients(de_max)
            M_elevon = q_inf * ac.aero.S * ac.aero.c_bar * dCm_e
            balance = M_aero + M_elevon
            print(f"  V={V:5.1f} alpha={alpha_deg:2d}°  "
                  f"M_aero={M_aero:+.4f}  M_elevon={M_elevon:+.4f}  "
                  f"M_total={balance:+.4f} Nm  {'OK' if balance > 0 else 'INSUFFICIENT'}")

    # ── 作图 ──
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # (a) 配平减速度 vs 速度
    ax = axes[0, 0]
    V_trim = [r['V'] for r in trim_results]
    du_trim = [r['du'] for r in trim_results]
    ax.plot(V_trim, du_trim, 'bo-', lw=1.5, markersize=4)
    ax.axhline(0, color='k', ls=':', alpha=0.3)
    ax.set_xlabel('空速 (m/s)'); ax.set_ylabel('du (m/s²)')
    ax.set_title('配平状态净水平加速度'); ax.grid(True)

    # (b) Alpha-限制减速包络线
    ax = axes[0, 1]
    for alpha_d in [10, 15, 18, 20, 22]:
        decels = [decel_table[V][alpha_d] for V in speeds]
        ax.plot(speeds, decels, '-o', ms=3, lw=1.2, label=f'α={alpha_d}°')
    ax.axhline(0, color='k', ls=':', alpha=0.3)
    ax.set_xlabel('空速 (m/s)'); ax.set_ylabel('du (m/s²)')
    ax.set_title('Alpha-限制减速能力 (thr=0.05)'); ax.legend(fontsize=8); ax.grid(True)

    # (c) 滑流效率
    ax = axes[1, 0]
    ax.plot(speeds, slipstream_ratios, 'ro-', lw=1.5, markersize=4)
    ax.axhline(1.0, color='k', ls=':', alpha=0.3, label='q_slip=q_inf')
    ax.axhline(1.5, color='orange', ls='--', alpha=0.5, label='50%增强')
    ax.set_xlabel('空速 (m/s)'); ax.set_ylabel('q_slip / q_inf')
    ax.set_title('滑流效率 (thr=0.05)'); ax.legend(fontsize=8); ax.grid(True)

    # (d) 升降副翼力矩
    ax = axes[1, 1]
    ax.plot(speeds, elevon_moments, 'go-', lw=1.5, markersize=4)
    ax.axhline(0, color='k', ls=':', alpha=0.3)
    ax.set_xlabel('空速 (m/s)'); ax.set_ylabel('M_elevon (Nm)')
    ax.set_title('升降副翼最大抬头力矩 (de=30°)'); ax.grid(True)

    fig.suptitle('回收过程减速能力包络线分析', fontsize=14, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig('recovery_envelope.png', dpi=150, bbox_inches='tight')
    plt.close(fig)

    # ── 汇总表 ──
    print("\n" + "=" * 60)
    print("  汇总表")
    print("=" * 60)
    print(f"{'V':>5s} {'alpha_t':>8s} {'theta_t':>8s} {'thr_t':>7s} {'du_t':>8s} "
          f"{'du_a15':>8s} {'du_a20':>8s} {'q_slip':>7s} {'M_elev':>8s}")
    for i, V in enumerate(speeds):
        r = trim_results[i] if i < len(trim_results) else None
        du_a15 = decel_table.get(V, {}).get(15, float('nan'))
        du_a20 = decel_table.get(V, {}).get(20, float('nan'))
        ss = slipstream_ratios[i] if i < len(slipstream_ratios) else float('nan')
        em = elevon_moments[i] if i < len(elevon_moments) else float('nan')
        if r:
            print(f"{V:5.1f} {r['alpha_deg']:8.2f} {r['theta_deg']:8.2f} {r['throttle']:7.4f} "
                  f"{r['du']:+8.4f} {du_a15:+8.4f} {du_a20:+8.4f} {ss:7.2f} {em:+8.4f}")

    print("\n图表已保存: recovery_envelope.png")


if __name__ == "__main__":
    run_envelope_analysis()
