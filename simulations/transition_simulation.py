"""
过渡飞行仿真（Transition Simulation）
=====================================
尾座式无人机：悬停(竖直) -> 平飞(水平巡航) -> 悬停回收
新架构：控制律切换 + 空速混合（HoverPID / ForwardController）
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from core import (
    Aircraft6DOF, quaternion_to_euler, euler_to_quaternion,
    quaternion_multiply, isa_atmosphere, integrate_6dof_quaternion
)
from controllers import HoverPID, ForwardController, RecoveryController


def setup_chinese_font():
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False


class TransitionController:
    def __init__(self, ac):
        self.ac = ac

        # 任务参数（必须在 RecoveryController 之前定义）
        self.t_transition = 20.0
        self.t_cruise = 10.0
        self.t_transition_rec = 15.0
        self.V_cruise = 20.0
        self.h_hover = 20.0
        self.h_cruise = 50.0

        # 悬停配平
        self.x_hover, self.u_hover, _, self.hover_theta, _ = ac.trim(V_trim=0.0, h_trim=0.0)
        self.hover_throttle = self.u_hover[0]

        # 平飞配平
        self.x_cruise, self.u_cruise, _, self.cruise_theta, _ = ac.trim(V_trim=20.0, h_trim=0.0)
        self.cruise_throttle = self.u_cruise[0]

        # 悬停PID（统一内环 + 姿态/角速度环）
        self.pid = HoverPID()
        self.pid.hover_throttle = self.hover_throttle
        self.pid.max_de = np.radians(30)
        self.pid.max_desired_rate = np.radians(360)
        self.pid.reset()

        # 前飞控制律
        self.forward = ForwardController(self.cruise_theta, self.cruise_throttle)
        self.forward.reset(theta_trim=self.cruise_theta)

        # 回收控制律
        self.recovery = RecoveryController(
            self.cruise_theta, self.cruise_throttle, self.hover_throttle,
            V_cruise=self.V_cruise, h_cruise=self.h_cruise,
            h_hover=self.h_hover, t_rec=self.t_transition_rec
        )

        # 混合窗口
        self.V_hover_blend = 5.0
        self.V_cruise_blend = 15.0

        # 积分器同步
        self.blend_low_threshold = 0.1
        self.blend_low_count = 0

        # 期望姿态变化率限制
        self._q_blend_last = None
        self.max_q_blend_rate = np.radians(3.0)

    @staticmethod
    def _smooth_ramp(r):
        if r <= 0.0:
            return 0.0
        if r >= 1.0:
            return 1.0
        return np.sin(np.pi / 2.0 * r) ** 2

    def get_profile(self, t):
        t1 = self.t_transition
        t2 = t1 + self.t_cruise
        t3 = t2 + self.t_transition_rec

        if t < t1:
            r = self._smooth_ramp(t / t1)
            V_des = r * self.V_cruise
            z_des = self.h_hover + r * (self.h_cruise - self.h_hover)
        elif t < t2:
            V_des = self.V_cruise
            z_des = self.h_cruise
        elif t < t3:
            r = self._smooth_ramp((t - t2) / self.t_transition_rec)
            V_des = self.V_cruise * (1.0 - r)
            z_des = self.h_cruise - r * (self.h_cruise - self.h_hover)
        else:
            V_des = 0.0
            z_des = self.h_hover

        return V_des, z_des

    def _compute_hover_outer(self, state, z_des, base_att_deg=60.0):
        """手动复现 HoverPID 外环，得到 q_hover_des 和 throttle_hover"""
        px, py, pz = state[10:13]

        pos_error_xy = -np.array([px, py])
        Kp_pos = self.pid.Kp_pos
        max_tilt = self.pid.max_tilt

        desired_accel_xy = Kp_pos * pos_error_xy
        max_accel = 9.81 * np.tan(max_tilt)
        desired_accel_xy = np.clip(desired_accel_xy, -max_accel, max_accel)

        tilt_roll = np.clip(desired_accel_xy[1] / 9.81, -np.sin(max_tilt), np.sin(max_tilt))
        tilt_pitch = np.clip(-desired_accel_xy[0] / 9.81, -np.sin(max_tilt), np.sin(max_tilt))

        q_tilt = euler_to_quaternion(0.0, tilt_pitch, tilt_roll)
        q_hover_base = euler_to_quaternion(0.0, np.radians(base_att_deg), 0.0)
        q_hover_des = quaternion_multiply(q_hover_base, q_tilt)
        q_hover_des = q_hover_des / np.linalg.norm(q_hover_des)

        h_err = pz - z_des
        throttle_hover = self.pid.hover_throttle + np.clip(0.02 * h_err, -0.2, 0.2)
        throttle_hover = np.clip(throttle_hover, 0.0, 1.0)

        return q_hover_des, throttle_hover

    @staticmethod
    def _slerp(q1, q2, t):
        """四元数球面线性插值（SLERP）"""
        dot = np.dot(q1, q2)
        if dot < 0:
            q2 = -q2
            dot = -dot
        if dot > 0.9995:
            q_blend = q1 + t * (q2 - q1)
            q_blend = q_blend / np.linalg.norm(q_blend)
            return q_blend
        theta_0 = np.arccos(dot)
        sin_theta_0 = np.sin(theta_0)
        theta = theta_0 * t
        sin_theta = np.sin(theta)
        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        return s0 * q1 + s1 * q2

    def compute(self, t, state, dt):
        u, v, w = state[0:3]
        V = np.sqrt(u**2 + v**2 + w**2)
        px, py, pz = state[10:13]

        # ---- 阶段判断 ----
        t1 = self.t_transition
        t2 = t1 + self.t_cruise
        t3 = t2 + self.t_transition_rec
        in_recovery = (t >= t2) and (t < t3)
        post_recovery = (t >= t3)

        if in_recovery:
            t_rec = t - t2
            V_des_rec, z_des_rec, stage = self.recovery.get_profile(t_rec, V)

            theta_rec, phi_rec, throttle_rec = self.recovery.compute(
                state, V_des_rec, z_des_rec, dt, stage)

            q_hover_des, throttle_hover = self._compute_hover_outer(state, z_des_rec, base_att_deg=90.0)

            if stage == 'A':
                blend_rec = 0.8
            elif stage == 'B':
                blend_rec = 0.5
            else:
                blend_rec = 0.2

            q_rec_des = euler_to_quaternion(phi_rec, theta_rec, 0.0)

            q_blend_target = self._slerp(q_hover_des, q_rec_des, blend_rec)
            throttle_blend = (1.0 - blend_rec) * throttle_hover + blend_rec * throttle_rec
            throttle_blend = np.clip(throttle_blend, self.recovery.throttle_floor, 1.0)

            last_print = getattr(self, '_last_diag_t', -1.0)
            if t - last_print >= 0.099:
                self._last_diag_t = t
                print(f"[REC] t={t:.1f} t_rec={t_rec:.1f} V={V:.1f} stage={stage} "
                      f"theta_rec={np.degrees(theta_rec):.1f}° "
                      f"thr_rec={throttle_rec:.3f} thr_hov={throttle_hover:.3f} "
                      f"thr_out={throttle_blend:.3f} blend={blend_rec:.1f}")

        elif post_recovery:
            V_des, z_des = 0.0, self.h_hover
            q_hover_des, throttle_hover = self._compute_hover_outer(state, z_des, base_att_deg=90.0)
            q_blend_target = q_hover_des
            throttle_blend = throttle_hover
            blend_rec = 0.0
        else:
            V_des, z_des = self.get_profile(t)

            if t < t1:
                blend = self._smooth_ramp(t / t1)
            else:
                blend = 1.0

            q_hover_des, throttle_hover = self._compute_hover_outer(state, z_des, base_att_deg=60.0)

            theta_fwd, phi_fwd, throttle_fwd = self.forward.compute(state, V_des, z_des, dt)
            q_fwd_des = euler_to_quaternion(phi_fwd, theta_fwd, 0.0)

            if blend <= 0.0:
                q_blend_target = q_hover_des
                throttle_blend = throttle_hover
            elif blend >= 1.0:
                q_blend_target = q_fwd_des
                throttle_blend = throttle_fwd
            else:
                q_blend_target = self._slerp(q_hover_des, q_fwd_des, blend)
                throttle_blend = (1.0 - blend) * throttle_hover + blend * throttle_fwd

            blend_rec = blend

        # 限制期望姿态变化率
        if self._q_blend_last is not None:
            dot = np.dot(self._q_blend_last, q_blend_target)
            if dot < 0:
                q_temp = -q_blend_target
            else:
                q_temp = q_blend_target
            dot = np.clip(np.abs(dot), -1.0, 1.0)
            angle = np.arccos(dot)
            max_angle = self.max_q_blend_rate * dt
            if angle > max_angle:
                q_blend = self._slerp(self._q_blend_last, q_temp, max_angle / angle)
            else:
                q_blend = q_temp
        else:
            q_blend = q_blend_target
        self._q_blend_last = q_blend.copy()

        # ---- 诊断打印（每 0.1 s） ----
        last_print = getattr(self, '_last_diag_t', -1.0)
        if t - last_print >= 0.099:
            self._last_diag_t = t
            theta_fwd_deg = np.degrees(theta_fwd)
            q_blend_ang_deg = 2.0 * np.degrees(np.arccos(np.clip(q_blend[0], -1.0, 1.0)))
            diag = (f"[DIAG] t={t:.2f}  V={V:.1f}  blend={blend:.3f}  "
                    f"theta_fwd={theta_fwd_deg:.1f}°  q_blend_ang={q_blend_ang_deg:.1f}°")
            if blend < 1.0:
                theta_hover_deg = np.degrees(quaternion_to_euler(q_hover_des)[1])
                diag += f"  theta_hover={theta_hover_deg:.1f}°"
            diag += (f"  thr_hov={throttle_hover:.3f}  thr_fwd={throttle_fwd:.3f}  "
                     f"thr_blend={throttle_blend:.3f}")
            print(diag)

        # ---- HoverPID 高度积分器同步 ----
        if blend < self.blend_low_threshold:
            self.blend_low_count += 1
        else:
            self.blend_low_count = 0

        if self.blend_low_count >= 3:
            h_err = pz - z_des
            if self.pid.last_alt_error is not None and dt > 0:
                alt_deriv = (h_err - self.pid.last_alt_error) / dt
            else:
                alt_deriv = 0.0

            if abs(self.pid.Ki_alt) > 1e-9:
                int_alt_eq = ((throttle_blend - self.pid.hover_throttle
                               - self.pid.Kp_alt * h_err
                               - self.pid.Kd_alt * alt_deriv) / self.pid.Ki_alt)
            else:
                int_alt_eq = self.forward.int_alt
            self.pid.int_alt = int_alt_eq

        # ---- 末端悬停模式切换 ----
        t_total = self.t_transition + self.t_cruise + self.t_transition_rec
        use_hover_mode = (t >= t_total) or (V < 3.0 and blend < 0.05)

        if use_hover_mode:
            self.pid.target_pos = np.array([px, py, self.h_hover])
            self.pid.q_hover = euler_to_quaternion(0.0, self.hover_theta, 0.0)
            self.pid.hover_throttle = self.hover_throttle
            return self.pid.compute(t, state, dt)

        self.pid.target_pos[2] = z_des
        return self.pid.compute(t, state, dt,
                                q_desired_override=q_blend,
                                throttle_override=throttle_blend)


def run_transition_simulation(t_end=45.0, dt=0.01):
    setup_chinese_font()
    ac = Aircraft6DOF()
    rho, _ = isa_atmosphere(0.0)

    ctrl = TransitionController(ac)

    print("=" * 60)
    print("  过渡飞行仿真 — 控制律切换与混合架构")
    print("=" * 60)
    print(f"悬停配平: theta={np.degrees(ctrl.hover_theta):.2f}°, throttle={ctrl.hover_throttle:.3f}")
    print(f"平飞配平: theta={np.degrees(ctrl.cruise_theta):.2f}°, throttle={ctrl.cruise_throttle:.3f}")

    x0 = ctrl.x_hover.copy()
    x0[12] = ctrl.h_hover

    print(f"\n开始仿真: 0 -> {t_end}s, dt={dt}s")
    t, x_hist, u_hist = integrate_6dof_quaternion(
        ac, x0, rho,
        lambda t, x, dt: ctrl.compute(t, x, dt),
        (0, t_end), dt=dt
    )
    print("仿真完成。")

    # 提取响应
    px = x_hist[:, 10]
    py = x_hist[:, 11]
    pz = x_hist[:, 12]

    u_vel = x_hist[:, 0]
    v_vel = x_hist[:, 1]
    w_vel = x_hist[:, 2]
    V = np.sqrt(u_vel**2 + v_vel**2 + w_vel**2)

    phi_deg = []
    theta_deg = []
    psi_deg = []
    alpha_deg = []
    for i in range(len(x_hist)):
        qx, qy, qz, qw = x_hist[i, 6:10]
        phi, theta, psi = quaternion_to_euler(np.array([qw, qx, qy, qz]))
        phi_deg.append(np.degrees(phi))
        theta_deg.append(np.degrees(theta))
        psi_deg.append(np.degrees(psi))
        alpha_deg.append(np.degrees(np.arctan2(w_vel[i], u_vel[i])))
    phi_deg = np.array(phi_deg)
    theta_deg = np.array(theta_deg)
    psi_deg = np.array(psi_deg)
    alpha_deg = np.array(alpha_deg)

    throttle_left = u_hist[:, 0]
    throttle_right = u_hist[:, 1]
    de_left = np.degrees(u_hist[:, 2])
    de_right = np.degrees(u_hist[:, 3])

    # 期望剖面
    V_des_arr = np.zeros_like(t)
    h_des_arr = np.zeros_like(t)
    for i, ti in enumerate(t):
        Vd, hd = ctrl.get_profile(ti)
        V_des_arr[i] = Vd
        h_des_arr[i] = hd

    # 混合系数
    blend_arr = np.clip((V - ctrl.V_hover_blend) / (ctrl.V_cruise_blend - ctrl.V_hover_blend), 0.0, 1.0)

    # 关键指标
    t1 = ctrl.t_transition
    t2 = t1 + ctrl.t_cruise
    t3 = t2 + ctrl.t_transition_rec

    mask_fwd = (t >= 0) & (t <= t1)
    mask_rec = (t >= t2) & (t <= t3)
    overshoot_fwd = np.max(pz[mask_fwd] - h_des_arr[mask_fwd]) if np.any(mask_fwd) else 0.0
    overshoot_rec = np.max(h_des_arr[mask_rec] - pz[mask_rec]) if np.any(mask_rec) else 0.0

    V_err_arr = V - V_des_arr
    max_V_err = np.max(np.abs(V_err_arr))
    rms_V_err_cruise = np.sqrt(np.mean(V_err_arr[(t >= t1) & (t <= t2)]**2))

    h_err_arr = pz - h_des_arr
    rms_h_err_cruise = np.sqrt(np.mean(h_err_arr[(t >= t1) & (t <= t2)]**2))

    end_idx = -1
    final_pos_err = np.sqrt(px[end_idx]**2 + py[end_idx]**2 + (pz[end_idx] - ctrl.h_hover)**2)
    final_V = V[end_idx]

    max_de = max(np.max(np.abs(de_left)), np.max(np.abs(de_right)))
    de_saturated = np.sum((np.abs(de_left) >= 29.9) | (np.abs(de_right) >= 29.9))

    max_alpha = np.max(np.abs(alpha_deg))

    print("\n" + "=" * 60)
    print("  关键指标")
    print("=" * 60)
    print(f"前过渡最大高度超调: {overshoot_fwd:.2f} m")
    print(f"回收过渡最大高度欠调: {overshoot_rec:.2f} m")
    print(f"全程最大空速误差: {max_V_err:.2f} m/s")
    print(f"巡航段空速误差 RMS: {rms_V_err_cruise:.2f} m/s")
    print(f"巡航段高度误差 RMS: {rms_h_err_cruise:.2f} m")
    print(f"终点位置误差: {final_pos_err:.3f} m")
    print(f"终点空速: {final_V:.2f} m/s")
    print(f"全程最大舵面: {max_de:.2f}°")
    print(f"舵面限幅触发次数: {de_saturated}")
    print(f"全程最大迎角绝对值: {max_alpha:.2f}°")

    # 可视化
    idx1 = np.searchsorted(t, t1)
    idx2 = np.searchsorted(t, t2)

    # ---------- 图1: 3D 轨迹 ----------
    fig1 = plt.figure(figsize=(10, 8))
    ax3d = fig1.add_subplot(111, projection='3d')
    ax3d.plot(px[:idx1], py[:idx1], pz[:idx1], 'b-', linewidth=2, label='前过渡')
    ax3d.plot(px[idx1:idx2], py[idx1:idx2], pz[idx1:idx2], 'g-', linewidth=2, label='巡航')
    ax3d.plot(px[idx2:], py[idx2:], pz[idx2:], 'r-', linewidth=2, label='回收')
    ax3d.scatter([px[0]], [py[0]], [pz[0]], c='red', s=100, marker='o', label='起点(悬停)')
    ax3d.scatter([px[idx1]], [py[idx1]], [pz[idx1]], c='lime', s=100, marker='^', label='过渡完成')
    ax3d.scatter([px[idx2]], [py[idx2]], [pz[idx2]], c='orange', s=100, marker='s', label='巡航结束')
    ax3d.scatter([px[-1]], [py[-1]], [pz[-1]], c='blue', s=100, marker='D', label='终点(回收)')
    xx, yy = np.meshgrid(np.linspace(px.min(), px.max(), 2), np.linspace(py.min(), py.max(), 2))
    ax3d.plot_surface(xx, yy, np.full_like(xx, ctrl.h_hover), alpha=0.1, color='cyan')
    ax3d.plot_surface(xx, yy, np.full_like(xx, ctrl.h_cruise), alpha=0.1, color='cyan')
    ax3d.set_xlabel('X (m)')
    ax3d.set_ylabel('Y (m)')
    ax3d.set_zlabel('Z (m)')
    ax3d.set_title('过渡飞行 3D 轨迹')
    ax3d.legend(loc='upper left', fontsize=8)
    fig1.savefig('transition_3d_trajectory.png', dpi=150, bbox_inches='tight')
    plt.close(fig1)

    # ---------- 图2: 时间历程（6子图） ----------
    fig2, axes = plt.subplots(2, 3, figsize=(16, 10))

    ax = axes[0, 0]
    ax.plot(t, V, 'b-', linewidth=1.5, label='实际')
    ax.plot(t, V_des_arr, 'r--', linewidth=1.5, label='期望')
    ax.set_ylabel('空速 (m/s)')
    ax.set_title('空速响应')
    ax.legend(loc='lower right')
    ax.grid(True)

    ax = axes[0, 1]
    ax.plot(t, pz, 'b-', linewidth=1.5, label='实际')
    ax.plot(t, h_des_arr, 'r--', linewidth=1.5, label='期望')
    ax.axhline(ctrl.h_hover, color='k', linestyle=':', alpha=0.3)
    ax.axhline(ctrl.h_cruise, color='k', linestyle=':', alpha=0.3)
    ax.set_ylabel('高度 (m)')
    ax.set_title('高度响应')
    ax.legend(loc='lower right')
    ax.grid(True)

    ax = axes[0, 2]
    ax.plot(t, theta_deg, 'b-', linewidth=1.5, label='实际')
    ax.plot(t, blend_arr * 15.0 + (1 - blend_arr) * 90.0, 'g--', linewidth=1.0, alpha=0.5, label='混合示意')
    ax.axhline(90, color='k', linestyle=':', alpha=0.3)
    ax.axhline(np.degrees(ctrl.cruise_theta), color='k', linestyle='--', alpha=0.3)
    ax.set_ylabel('俯仰角 (deg)')
    ax.set_title('俯仰角响应')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True)

    ax = axes[1, 0]
    ax.plot(t, alpha_deg, 'b-', linewidth=1.5)
    ax.axhline(0, color='r', linestyle='--', alpha=0.5)
    ax.axhline(35, color='orange', linestyle='--', alpha=0.7, label='高危迎角 35°')
    ax.axhline(-35, color='orange', linestyle='--', alpha=0.7)
    ax.set_ylabel('迎角 (deg)')
    ax.set_title('迎角（关键监控指标）')
    ax.legend(loc='upper right')
    ax.grid(True)

    ax = axes[1, 1]
    ax.plot(t, phi_deg, 'b-', linewidth=1.5)
    ax.axhline(0, color='r', linestyle='--', alpha=0.5)
    ax.set_ylabel('滚转角 (deg)')
    ax.set_title('滚转响应')
    ax.grid(True)

    ax = axes[1, 2]
    ax.plot(t, psi_deg, 'b-', linewidth=1.5)
    ax.axhline(0, color='r', linestyle='--', alpha=0.5)
    ax.set_ylabel('偏航角 (deg)')
    ax.set_title('偏航响应')
    ax.grid(True)

    fig2.suptitle('过渡飞行状态时间历程', fontsize=14, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    fig2.savefig('transition_state_history.png', dpi=150, bbox_inches='tight')
    plt.close(fig2)

    # ---------- 图3: 控制量 ----------
    fig3, axes = plt.subplots(1, 2, figsize=(14, 5))

    ax = axes[0]
    ax.plot(t, throttle_left, 'b-', linewidth=1.5, label='左油门')
    ax.plot(t, throttle_right, 'r-', linewidth=1.5, label='右油门')
    ax.axhline(ctrl.hover_throttle, color='k', linestyle=':', alpha=0.3)
    ax.axhline(ctrl.cruise_throttle, color='k', linestyle='--', alpha=0.3)
    ax.set_ylabel('油门')
    ax.set_xlabel('时间 (s)')
    ax.set_title('油门控制')
    ax.legend(loc='upper right')
    ax.grid(True)

    ax = axes[1]
    ax.plot(t, de_left, 'b-', linewidth=1.5, label='左舵面')
    ax.plot(t, de_right, 'r-', linewidth=1.5, label='右舵面')
    ax.axhline(0, color='k', linestyle=':', alpha=0.3)
    ax.axhline(30, color='k', linestyle='--', alpha=0.3)
    ax.axhline(-30, color='k', linestyle='--', alpha=0.3)
    ax.set_ylabel('舵面偏角 (deg)')
    ax.set_xlabel('时间 (s)')
    ax.set_title('舵面控制')
    ax.legend(loc='upper right')
    ax.grid(True)

    fig3.suptitle('过渡飞行控制输入', fontsize=14, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    fig3.savefig('transition_controls.png', dpi=150, bbox_inches='tight')
    plt.close(fig3)

    print("\n图表已保存:")
    print("  - transition_3d_trajectory.png")
    print("  - transition_state_history.png")
    print("  - transition_controls.png")

    return {
        't': t,
        'x': x_hist,
        'u': u_hist,
        'overshoot_fwd': overshoot_fwd,
        'overshoot_rec': overshoot_rec,
        'max_V_err': max_V_err,
        'rms_V_err_cruise': rms_V_err_cruise,
        'rms_h_err_cruise': rms_h_err_cruise,
        'final_pos_err': final_pos_err,
        'final_V': final_V,
        'max_de': max_de,
        'de_saturated': de_saturated,
        'max_alpha': max_alpha,
    }


if __name__ == "__main__":
    run_transition_simulation(t_end=60.0, dt=0.01)
