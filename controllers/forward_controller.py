import numpy as np


class ForwardController:
    """前飞控制律：速度→俯仰，高度→油门（固定翼模式）"""

    def __init__(self, trim_pitch, cruise_throttle):
        # 配平点
        self.cruise_theta = trim_pitch
        self.cruise_throttle = cruise_throttle

        # 速度环 PID
        self.Kp_vel = 0.05
        self.Ki_vel = 0.005
        self.Kd_vel = 0.01
        self.int_vel = 0.0
        self.max_int_vel = 2.0
        self.last_V_err = 0.0
        self.vel_first_call = True

        # 高度环 PID（降低 Kp_alt 使前飞律在过渡段不剧烈响应高度误差）
        self.Kp_alt = 0.01
        self.Ki_alt = 0.001
        self.Kd_alt = 0.05
        self.int_alt = 0.0
        self.max_int_alt = 5.0
        self.last_alt_error = None
        self.alt_first_call = True

        # 输出限制
        self.theta_min = np.radians(-30)
        self.theta_max = np.radians(45)
        self.throttle_trim = self.cruise_throttle

    def reset(self, theta_trim=0.0):
        self.int_vel = theta_trim
        self.last_V_err = 0.0
        self.vel_first_call = True
        self.int_alt = 0.0
        self.last_alt_error = None
        self.alt_first_call = True

    def compute(self, state, V_des, z_des, dt):
        u, v, w = state[0:3]
        V = np.sqrt(u**2 + v**2 + w**2)
        pz = state[12]

        # ---- 速度环 → 期望俯仰角 ----
        V_err = V - V_des
        if self.vel_first_call:
            dV_dt = 0.0
        else:
            dV_dt = (V_err - self.last_V_err) / dt if dt > 0 else 0.0
        self.last_V_err = V_err
        self.vel_first_call = False

        theta_des_unsat = (self.Kp_vel * V_err +
                           self.Ki_vel * self.int_vel +
                           self.Kd_vel * dV_dt)

        # 条件积分：仅当未触及限幅时累加
        if self.theta_min < theta_des_unsat < self.theta_max:
            self.int_vel += V_err * dt
            self.int_vel = np.clip(self.int_vel, -self.max_int_vel, self.max_int_vel)

        theta_des = np.clip(theta_des_unsat, self.theta_min, self.theta_max)

        # ---- 高度环 → 油门 ----
        h_err = pz - z_des
        if self.alt_first_call or self.last_alt_error is None:
            h_rate = 0.0
        else:
            h_rate = (h_err - self.last_alt_error) / dt if dt > 0 else 0.0
        self.last_alt_error = h_err
        self.alt_first_call = False

        throttle_unsat = (self.throttle_trim +
                          self.Kp_alt * h_err +
                          self.Ki_alt * self.int_alt +
                          self.Kd_alt * h_rate)

        # 条件积分：仅当油门未饱和时累加
        if 0.0 < throttle_unsat < 1.0:
            self.int_alt += h_err * dt
            self.int_alt = np.clip(self.int_alt, -self.max_int_alt, self.max_int_alt)

        throttle_des = np.clip(throttle_unsat, 0.0, 1.0)

        phi_des = 0.0
        return theta_des, phi_des, throttle_des
