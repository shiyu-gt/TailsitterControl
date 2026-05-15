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
        self.V_break_A = 8.0   # A→B
        self.V_break_B = 3.0   # B→C

        # ---- 速度环 PID（控制俯仰）----
        self.Kp_vel = 0.1
        self.Ki_vel = 0.005
        self.Kd_vel = 0.02
        self.int_vel = 0.0
        self.max_int_vel = 1.0
        self.last_V_err = 0.0
        self.vel_first_call = True
        self.gamma_max = np.radians(10.0)

        # ---- alpha 保护 ----
        self.alpha_warn = np.radians(18.0)
        self.alpha_max = np.radians(22.0)

        # ---- 俯仰角限制 ----
        self.theta_min = np.radians(-5.0)
        self.theta_max = np.radians(88.0)
        self.max_theta_rate = np.radians(10.0)

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

    def reset(self):
        self.int_vel = 0.0
        self.int_alt = 0.0
        self.last_V_err = 0.0
        self.vel_first_call = True
        self.last_alt_error = None
        self.alt_first_call = True
        self._last_theta_des = None
        self._stage = 'A'

    def get_profile(self, t_rec, V_current):
        """基于当前速度动态决定阶段，返回 (V_des, z_des, stage)"""
        if V_current > self.V_break_A:
            stage = 'A'
        elif V_current > self.V_break_B:
            stage = 'B'
        else:
            stage = 'C'
        self._stage = stage

        # 期望速度：指数衰减
        k_decel = 0.04
        V_des = self.V_cruise * np.exp(-k_decel * t_rec)
        V_des = max(V_des, 0.0)

        # 期望高度
        if stage == 'A':
            z_des = self.h_cruise
        elif stage == 'B':
            progress = (self.V_break_A - V_current) / (self.V_break_A - self.V_break_B)
            progress = np.clip(progress, 0.0, 1.0)
            z_des = self.h_cruise + progress * (self.h_hover - self.h_cruise)
        else:
            z_des = self.h_hover

        return V_des, z_des, stage

    def compute(self, state, V_des, z_des, dt, stage='A'):
        """返回 (theta_des, phi_des, throttle_des)"""
        u, v, w = state[0:3]
        pz = state[12]
        V = np.sqrt(u**2 + v**2 + w**2)

        # ---- 阶段基准俯仰角 ----
        if stage == 'A':
            theta_base = self.cruise_theta
        elif stage == 'B':
            progress = (self.V_break_A - V) / (self.V_break_A - self.V_break_B)
            progress = np.clip(progress, 0.0, 1.0)
            theta_base = np.radians(15.0) + progress * (np.radians(45.0) - np.radians(15.0))
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

        gamma_raw = self.Kp_vel * V_err + self.Ki_vel * self.int_vel + self.Kd_vel * dV_dt
        if -self.gamma_max < gamma_raw < self.gamma_max:
            self.int_vel += V_err * dt
            self.int_vel = np.clip(self.int_vel, -self.max_int_vel, self.max_int_vel)
        gamma_des = np.clip(gamma_raw, -self.gamma_max, self.gamma_max)

        # ---- alpha 保护：高迎角时限制抬头 ----
        alpha = np.arctan2(w, u)
        if alpha > self.alpha_warn:
            fade = max(0.0, (self.alpha_max - alpha) / (self.alpha_max - self.alpha_warn))
            gamma_floor = -self.gamma_max * fade
            gamma_des = max(gamma_des, gamma_floor)
            self.int_vel = min(self.int_vel, 0.0)

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
            # 高度高于目标时进一步减油门加速下降
            throttle_base = 0.05
            if h_err < -10.0:  # 飞机高于目标 10m 以上
                throttle_base = 0.02
            Kp_a, Ki_a, Kd_a = 0.0, 0.0, 0.0
        elif stage == 'B':
            progress = (self.V_break_A - V) / (self.V_break_A - self.V_break_B)
            progress = np.clip(progress, 0.0, 1.0)
            throttle_base = self.cruise_throttle + progress * (self.hover_throttle - self.cruise_throttle)
            Kp_a, Ki_a, Kd_a = self.Kp_alt, self.Ki_alt, self.Kd_alt
        else:
            throttle_base = self.hover_throttle
            Kp_a, Ki_a, Kd_a = self.Kp_alt, self.Ki_alt, self.Kd_alt

        throttle_unsat = (throttle_base
                          + Kp_a * h_err
                          + Ki_a * self.int_alt
                          + Kd_a * h_rate)

        if self.throttle_floor < throttle_unsat < self.throttle_ceil:
            self.int_alt += h_err * dt
            self.int_alt = np.clip(self.int_alt, -self.max_int_alt, self.max_int_alt)

        throttle_des = np.clip(throttle_unsat, self.throttle_floor, self.throttle_ceil)

        return theta_des, phi_des, throttle_des
