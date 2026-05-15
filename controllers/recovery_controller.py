"""
回收控制律（Recovery Controller）
=================================
专用的平飞→悬停回收控制器。

控制架构：
  速度环 → gamma_des（俯仰修正）
  高度环 → throttle_des（油门维持高度）
  theta_des = theta_base + gamma_des

三阶段策略：
  Stage A（V > 8 m/s）: 低油门下降减速，用高度换速度
  Stage B（3 < V < 8） : 逐步抬头，高度环控制油门
  Stage C（V < 3 m/s）: 大幅抬头建立悬停

物理约束：
  - 中等迎角(<25°)下，升力前向分量抵消阻力，无法有效减速
  - 大迎角(>30°)时升降副翼效率急剧下降
  - 唯一有效减速方式：减小油门，靠重力分量减速
"""

import numpy as np


class RecoveryController:
    """回收控制律"""

    def __init__(self, cruise_theta, cruise_throttle, hover_throttle,
                 V_cruise=20.0, h_cruise=50.0, h_hover=20.0, t_rec=15.0):
        self.cruise_theta = cruise_theta
        self.cruise_throttle = cruise_throttle
        self.hover_throttle = hover_throttle
        self.V_cruise = V_cruise
        self.h_cruise = h_cruise
        self.h_hover = h_hover
        self.t_rec = t_rec

        # 阶段切换速度阈值
        self.V_break_A = 12.0  # A→B（基于包络线分析：V<12 时配平发散）
        self.V_break_B = 3.0   # B→C

        # ---- 速度环 PID（控制俯仰）----
        self.Kp_vel = 0.1
        self.Ki_vel = 0.005
        self.Kd_vel = 0.02
        self.int_vel = 0.0
        self.max_int_vel = 1.0
        self.last_V_err = 0.0
        self.vel_first_call = True
        self.gamma_max = np.radians(20.0)       # Stage A
        self.gamma_max_B = np.radians(8.0)      # Stage B: tighter to avoid alpha overshoot

        # ---- alpha 保护 ----
        self.alpha_warn = np.radians(20.0)
        self.alpha_max = np.radians(25.0)
        self._int_vel_frozen = False

        # ---- 俯仰角限制 ----
        self.theta_min = np.radians(-5.0)
        self.theta_max = np.radians(88.0)
        self.max_theta_rate = np.radians(30.0)  # alpha protection is the real limiter

        # ---- 高度环 PID（控制油门）----
        self.Kp_alt = 0.005
        self.Ki_alt = 0.001
        self.Kd_alt = 0.02
        self.int_alt = 0.0
        self.max_int_alt = 0.15
        self.last_alt_error = None
        self.alt_first_call = True

        # ---- 油门 ----
        self.throttle_floor = 0.0
        self.throttle_ceil = 1.0

        # ---- 状态 ----
        self._last_theta_des = None
        self._stage = 'A'

        # ---- 诊断量 ----
        self._last_gamma_des = 0.0
        self._last_theta_base = 0.0
        self._last_throttle_base = 0.0
        self._last_alpha_prot_active = False

    def reset(self):
        self.int_vel = 0.0
        self.int_alt = 0.0
        self.last_V_err = 0.0
        self.vel_first_call = True
        self.last_alt_error = None
        self.alt_first_call = True
        self._last_theta_des = None
        self._stage = 'A'
        self._last_gamma_des = 0.0
        self._last_theta_base = 0.0
        self._last_throttle_base = 0.0
        self._last_alpha_prot_active = False
        self._int_vel_frozen = False

    def get_profile(self, t_rec, V_current, pz=None):
        """基于当前速度动态决定阶段，返回 (V_des, z_des, stage)"""
        # 时间回退：80% 时间用完后强制进入 Stage B
        time_fallback = (t_rec >= self.t_rec * 0.8)
        # 高度回退：下降超过 15m（已交换高度换速度）
        alt_fallback = (pz is not None and (pz - self.h_cruise) > 15.0)

        # 带迟滞的阶段切换：一旦进入 B/C 不退回 A
        prev = self._stage
        if V_current <= self.V_break_B:
            stage = 'C'
        elif prev == 'A' and (V_current <= self.V_break_A or time_fallback or alt_fallback):
            stage = 'B'
        elif prev in ('B', 'C'):
            stage = 'C' if V_current <= self.V_break_B else 'B'
        else:
            stage = 'A'
        self._stage = stage

        # 期望速度：分阶段剖面
        # Stage A: 缓慢下降到 12.5 m/s（物理减速极限）
        # Stage B: 跟踪当前速度（V_des = V - 0.5），消除持续速度误差
        # Stage C: 目标 0
        if stage == 'A':
            V_des = self.V_cruise + (12.5 - self.V_cruise) * min(t_rec / 50.0, 1.0)
        elif stage == 'B':
            # 跟踪当前速度，保留 0.5 m/s 的微小误差驱动 gamma 微调
            V_des = max(V_current - 0.5, self.V_break_B)
        else:
            V_des = max(0.0, 3.0 - t_rec * 0.5)
        V_des = max(V_des, 0.0)

        # 期望高度
        if stage == 'A':
            z_des = self.h_cruise
        elif stage == 'B':
            # 时间进度：Stage B 预计耗时 t_rec 的 40%
            t_stage_B = self.t_rec * 0.4
            progress = np.clip(t_rec / max(t_stage_B, 1.0), 0.0, 1.0)
            z_des = self.h_cruise + progress * (self.h_hover - self.h_cruise)
        else:
            z_des = self.h_hover

        return V_des, z_des, stage

    def compute(self, state, V_des, z_des, dt, stage='A'):
        """返回 (theta_des, phi_des, throttle_des)"""
        u, v, w = state[0:3]
        pz = state[12]
        V = np.sqrt(u**2 + v**2 + w**2)

        # ---- 阶段切换检测：重置积分器 ----
        if hasattr(self, '_prev_stage') and self._prev_stage != stage:
            self.int_vel = 0.0
            self.vel_first_call = True
        self._prev_stage = stage

        # ---- 阶段基准俯仰角 ----
        if stage == 'A':
            theta_base = self.cruise_theta
        elif stage == 'B':
            # Stage B: alpha-commanded theta for drag-based deceleration
            # At high V: high alpha (18°) for maximum drag
            # At low V: low alpha (8°) to prepare for Stage C transition
            progress = (self.V_break_A - V) / (self.V_break_A - self.V_break_B)
            progress = np.clip(progress, 0.0, 1.0)
            alpha_target = np.radians(18.0) + progress * (np.radians(8.0) - np.radians(18.0))
            theta_base = alpha_target
        else:
            progress = max(0.0, 1.0 - V / self.V_break_B)
            theta_base = np.radians(45.0) + progress * (np.radians(85.0) - np.radians(45.0))

        # ---- 速度环 → gamma_des（俯仰修正）----
        V_err = V - V_des
        if self.vel_first_call:
            dV_dt = 0.0
        else:
            dV_dt = (V_err - self.last_V_err) / dt if dt > 0 else 0.0
        self.last_V_err = V_err
        self.vel_first_call = False

        # Stage-dependent gains: Stage B uses alpha-commanded theta (no gamma correction)
        if stage == 'B':
            gmax = 0.0  # no velocity loop correction in Stage B
            Kp_v, Ki_v, Kd_v = 0.0, 0.0, 0.0
        else:
            gmax = self.gamma_max
            Kp_v, Ki_v, Kd_v = self.Kp_vel, self.Ki_vel, self.Kd_vel
        gamma_raw = Kp_v * V_err + Ki_v * self.int_vel + Kd_v * dV_dt
        int_cap = 0.1 if stage == 'B' else self.max_int_vel
        if -gmax < gamma_raw < gmax and not self._int_vel_frozen:
            self.int_vel += V_err * dt
            self.int_vel = np.clip(self.int_vel, -int_cap, int_cap)
        gamma_des = np.clip(gamma_raw, -gmax, gmax)

        # ---- alpha 保护：高迎角时限制抬头 ----
        alpha = np.arctan2(w, u)
        self._last_alpha_prot_active = False
        if alpha > self.alpha_warn:
            self._last_alpha_prot_active = True
            self._int_vel_frozen = True
            # smoothstep 渐变：alpha_warn 时无强制，alpha_max 时全力
            x = (alpha - self.alpha_warn) / max(self.alpha_max - self.alpha_warn, 1e-6)
            x = np.clip(x, 0.0, 1.0)
            fade = 3.0 * x**2 - 2.0 * x**3  # smoothstep
            gamma_floor = -gmax * fade
            gamma_des = max(gamma_des, gamma_floor)
        elif alpha < self.alpha_warn - np.radians(2.0):
            # 迟滞恢复：alpha 低于 warn-2° 时解冻积分器
            self._int_vel_frozen = False

        # ---- theta_des ----
        theta_des_raw = theta_base + gamma_des
        theta_des = np.clip(theta_des_raw, self.theta_min, self.theta_max)

        if self._last_theta_des is not None:
            dtheta = theta_des - self._last_theta_des
            max_dtheta = self.max_theta_rate * dt
            if abs(dtheta) > max_dtheta:
                theta_des = self._last_theta_des + np.sign(dtheta) * max_dtheta
        self._last_theta_des = theta_des

        phi_des = 0.0

        # ---- 高度环 → throttle_des ----
        h_err = pz - z_des  # NED：pz 向下为正
        if self.alt_first_call or self.last_alt_error is None:
            h_rate = 0.0
        else:
            h_rate = (h_err - self.last_alt_error) / dt if dt > 0 else 0.0
        self.last_alt_error = h_err
        self.alt_first_call = False

        # 各阶段油门策略
        if stage == 'A':
            # Stage A: 低油门，靠重力减速
            # 油门只减不增：高度高于目标时减油门加速下降，低于目标时不加
            throttle_base = 0.03
            Kp_a, Ki_a, Kd_a = 0.001, 0.0002, 0.005
        elif stage == 'B':
            # Stage B: throttle below cruise to decelerate, rising toward hover_throttle at low V
            progress = (self.V_break_A - V) / (self.V_break_A - self.V_break_B)
            progress = np.clip(progress, 0.0, 1.0)
            throttle_base = 0.03 + progress * (self.hover_throttle - 0.03)
            throttle_base = min(throttle_base, self.cruise_throttle)
            Kp_a, Ki_a, Kd_a = self.Kp_alt * 0.3, self.Ki_alt * 0.1, self.Kd_alt * 0.3
        else:
            throttle_base = self.hover_throttle
            Kp_a, Ki_a, Kd_a = self.Kp_alt, self.Ki_alt, self.Kd_alt

        throttle_unsat = (throttle_base
                          + Kp_a * h_err
                          + Ki_a * self.int_alt
                          + Kd_a * h_rate)

        # 各阶段油门修正限制
        if stage == 'A':
            # Stage A: 油门只减不增，防止爬升失控
            throttle_corr = Kp_a * h_err + Ki_a * self.int_alt + Kd_a * h_rate
            throttle_corr = min(throttle_corr, 0.0)  # 只允许减油门
            throttle_corr = max(throttle_corr, -0.03)  # 最低到 0
            throttle_unsat = throttle_base + throttle_corr
        elif stage == 'B':
            # Stage B: 油门修正量限制在 throttle_base 的 30% 以内
            throttle_corr = Kp_a * h_err + Ki_a * self.int_alt + Kd_a * h_rate
            max_corr = max(throttle_base * 0.3, 0.03)
            throttle_corr = np.clip(throttle_corr, -max_corr, max_corr)
            throttle_unsat = throttle_base + throttle_corr

        if self.throttle_floor < throttle_unsat < self.throttle_ceil:
            self.int_alt += h_err * dt
            self.int_alt = np.clip(self.int_alt, -self.max_int_alt, self.max_int_alt)

        throttle_des = np.clip(throttle_unsat, self.throttle_floor, self.throttle_ceil)

        # ---- Stage B: 直接升降副翼控制（绕过 HoverPID）----
        # P-controller on theta error with anti-windup
        if stage == 'B':
            theta_err = theta_des - np.arctan2(w, u)  # 简化：theta_err ≈ theta_des - alpha
            # 更精确的 theta 计算：从四元数提取
            qx, qy, qz, qw = state[6:10]
            theta_current = np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
            theta_err = theta_des - theta_current
            Kp_elevon = 10.0  # aggressive gain: command large elevon for theta tracking
            de_sym = Kp_elevon * theta_err
            de_sym = np.clip(de_sym, np.radians(-30), np.radians(30))
            self._de_override = de_sym
        else:
            self._de_override = None

        # ---- 诊断量暴露 ----
        self._last_gamma_des = gamma_des
        self._last_theta_base = theta_base
        self._last_throttle_base = throttle_base

        return theta_des, phi_des, throttle_des
