"""
回收控制律（Recovery Controller）
=================================
专用的平飞→悬停回收控制器。

控制架构：
  速度环 → gamma_des（俯仰修正）
  高度环 → throttle_des（油门维持高度）
  theta_des = theta_base + gamma_des

三阶段策略：
  Stage A（V > 12 m/s）: 低油门减速，巡航俯仰角，速度环微调
  Stage B（3 < V < 12） : 渐进抬头，速度环修正，低油门下降
  Stage C（V < 3 m/s）  : 大幅抬头建立悬停

物理约束：
  - V > 12 m/s 时任何正 alpha 都产生超过重力的升力，飞机无法下降
  - Stage A 必须靠低油门缓慢减速到 V_break_A
  - Stage B 在 V < 12 m/s 后可以适度抬头减速+下降
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
        self.V_break_A = 12.0  # A→B（V<12 时飞机可以正 alpha 下降）
        self.V_break_B = 3.0   # B→C

        # ---- 速度环 PID（控制俯仰修正 gamma_des）----
        self.Kp_vel = 0.1
        self.Ki_vel = 0.005
        self.Kd_vel = 0.02
        self.int_vel = 0.0
        self.max_int_vel = 1.0
        self.last_V_err = 0.0
        self.vel_first_call = True
        self.gamma_max = np.radians(20.0)       # Stage A
        self.gamma_max_B = np.radians(10.0)     # Stage B: moderate

        # ---- alpha 保护 ----
        self.alpha_warn = np.radians(20.0)
        self.alpha_max = np.radians(25.0)
        self._int_vel_frozen = False

        # ---- 俯仰角限制 ----
        self.theta_min = np.radians(-5.0)
        self.theta_max = np.radians(88.0)
        self.max_theta_rate = np.radians(30.0)

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

        # ---- Stage B 升降副翼 P 控制器 ----
        self.Kp_elevon = 3.0

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
        # 带迟滞的阶段切换：一旦进入 B/C 不退回 A
        prev = self._stage
        if V_current <= self.V_break_B:
            stage = 'C'
        elif prev == 'A' and V_current <= self.V_break_A:
            stage = 'B'
        elif prev in ('B', 'C'):
            stage = 'C' if V_current <= self.V_break_B else 'B'
        else:
            stage = 'A'
        self._stage = stage

        # 期望速度
        if stage == 'A':
            # 线性从 V_cruise 降到 12.5 m/s，50s 内完成
            V_des = self.V_cruise + (12.5 - self.V_cruise) * min(t_rec / 50.0, 1.0)
        elif stage == 'B':
            # 从当前速度缓慢降到 V_break_B
            V_des = max(V_current - 0.5, self.V_break_B)
        else:
            V_des = max(0.0, 3.0 - t_rec * 0.5)
        V_des = max(V_des, 0.0)

        # 期望高度
        if stage == 'A':
            z_des = self.h_cruise
        elif stage == 'B':
            # 从 h_cruise 线性降到 h_hover
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
            # 渐进抬头：从巡航角到 45°
            progress = (self.V_break_A - V) / (self.V_break_A - self.V_break_B)
            progress = np.clip(progress, 0.0, 1.0)
            theta_base = self.cruise_theta + progress * (np.radians(45.0) - self.cruise_theta)
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

        # Stage B 使用降低的增益
        if stage == 'B':
            gmax = self.gamma_max_B
            Kp_v = self.Kp_vel * 0.5
            Ki_v = self.Ki_vel * 0.5
            Kd_v = self.Kd_vel * 0.5
        else:
            gmax = self.gamma_max
            Kp_v, Ki_v, Kd_v = self.Kp_vel, self.Ki_vel, self.Kd_vel

        gamma_raw = Kp_v * V_err + Ki_v * self.int_vel + Kd_v * dV_dt
        int_cap = self.max_int_vel
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
            x = (alpha - self.alpha_warn) / max(self.alpha_max - self.alpha_warn, 1e-6)
            x = np.clip(x, 0.0, 1.0)
            fade = 3.0 * x**2 - 2.0 * x**3  # smoothstep
            gamma_floor = -gmax * fade
            gamma_des = max(gamma_des, gamma_floor)
        elif alpha < self.alpha_warn - np.radians(2.0):
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
            throttle_base = 0.03
            Kp_a, Ki_a, Kd_a = 0.001, 0.0002, 0.005
        elif stage == 'B':
            # 低于巡航油门，确保推力不足→减速
            throttle_base = 0.04
            Kp_a, Ki_a, Kd_a = self.Kp_alt * 0.3, self.Ki_alt * 0.1, self.Kd_alt * 0.3
        else:
            throttle_base = self.hover_throttle
            Kp_a, Ki_a, Kd_a = self.Kp_alt, self.Ki_alt, self.Kd_alt

        # 各阶段油门修正限制
        if stage == 'A':
            throttle_corr = Kp_a * h_err + Ki_a * self.int_alt + Kd_a * h_rate
            throttle_corr = min(throttle_corr, 0.0)
            throttle_corr = max(throttle_corr, -0.03)
            throttle_unsat = throttle_base + throttle_corr
        elif stage == 'B':
            throttle_corr = Kp_a * h_err + Ki_a * self.int_alt + Kd_a * h_rate
            max_corr = max(throttle_base * 0.3, 0.03)
            throttle_corr = np.clip(throttle_corr, -max_corr, max_corr)
            throttle_unsat = throttle_base + throttle_corr
        else:
            throttle_unsat = (throttle_base
                              + Kp_a * h_err
                              + Ki_a * self.int_alt
                              + Kd_a * h_rate)

        if self.throttle_floor < throttle_unsat < self.throttle_ceil:
            self.int_alt += h_err * dt
            self.int_alt = np.clip(self.int_alt, -self.max_int_alt, self.max_int_alt)

        throttle_des = np.clip(throttle_unsat, self.throttle_floor, self.throttle_ceil)

        # ---- Stage B: 直接升降副翼控制（绕过 HoverPID）----
        if stage == 'B':
            qx, qy, qz, qw = state[6:10]
            theta_current = np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
            theta_err = theta_des - theta_current
            de_sym = self.Kp_elevon * theta_err
            de_sym = np.clip(de_sym, np.radians(-30), np.radians(30))
            self._de_override = de_sym
        else:
            self._de_override = None

        # ---- 诊断量暴露 ----
        self._last_gamma_des = gamma_des
        self._last_theta_base = theta_base
        self._last_throttle_base = throttle_base

        return theta_des, phi_des, throttle_des
