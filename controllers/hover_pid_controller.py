"""
悬停PID控制器
==============
双旋翼尾座式悬停控制（机头朝上，theta≈90°）
- 使用四元数姿态控制，避免万向锁
- 外环：水平位置控制（x,y）→ 期望倾斜
- 高度控制：独立油门PID
- 中环：姿态控制（四元数误差→旋转矢量）→ 期望角速度
- 内环：角速度控制 → 舵面/差动油门
"""

import numpy as np
from core import euler_to_quaternion, quaternion_multiply, rotation_matrix_from_quaternion


class HoverPID:
    """悬停PID控制器，四元数姿态控制，优化收敛性"""

    def __init__(self):
        # 控制限制
        self.max_throttle = 1.0
        # 恢复输出限幅（阶段一调试）
        self.max_tilt = np.radians(15)   # 最大倾斜角
        self.max_de = np.radians(20)     # 最大舵面偏角

        # 位置控制器 (外环, x,y) - PID (微分由姿态环提供)
        self.Kp_pos = 0.8
        self.Ki_pos = 0.1
        self.Kd_pos = 0.6

        # 高度控制器 - PD
        self.Kp_alt = 0.065
        self.Ki_alt = 0.0
        self.Kd_alt = 0.14

        # 姿态控制器 (中环) - P only (微分由角速度环提供)
        self.Kp_att = 8.0
        self.Ki_att = 0.0
        self.Kd_att = 0.0

        # 角速度控制器 (内环) - PD (仅此处保留微分阻尼)
        self.Kp_rate = 1.5
        self.Ki_rate = 0.0
        self.Kd_rate = 0.05
        self.max_desired_rate = np.radians(60)  # 可配置，过渡阶段可能需要更大值

        # 积分器
        self.int_pos = np.zeros(2)
        self.int_att = np.zeros(3)
        self.int_rate = np.zeros(3)
        self.int_alt = 0.0

        # 上一次误差（用于微分）
        self.last_pos_error = np.zeros(2)
        self.last_att_error = np.zeros(3)
        self.last_rate_error = np.zeros(3)
        self.last_alt_error = 0.0

        # 抗积分饱和 - 去除积分限制
        self.max_int_pos = 2.0      # 降低位置积分限制
        self.max_int_att = np.radians(10)  # 降低姿态积分限制
        self.max_int_rate = np.radians(5)  # 降低速率积分限制
        self.max_int_alt = 0.2     # 降低高度积分限制

        # 悬停参考 - 设置20m高度悬停
        self.target_pos = np.array([0.0, 0.0, 20.0])
        self.hover_throttle = 0.65

        # 悬停基准四元数（theta=90°，机头朝上）
        self.q_hover = euler_to_quaternion(0.0, np.radians(90.0), 0.0)

        # 控制分配系数（匹配重构后模型物理）
        # 阶段一极性修正：
        #   pitch_to_elevator 取反（后仰时应下偏产生低头力矩）
        #   yaw_to_throttle_diff 取反（正偏航误差应右油门增大）
        self.roll_to_de_diff = -0.02       # 滚转 → 差动升降副翼（修正：符号取反以匹配实际滚转力矩方向）
        self.pitch_to_elevator = 0.08      # 俯仰 → 对称升降副翼（修正：飞翼布局Cm_de>0，后仰需上偏产生低头力矩）
        self.yaw_to_throttle_diff = 0.02   # 偏航 → 差动油门（增大至0.02以提供足够偏航控制力矩）

        # 低通滤波器参数
        self.filter_freq = 10.0  # Hz
        self.alpha_filter = 0.0  # 将在compute中计算
        self.last_pos_deriv_xy = np.zeros(2)
        self.last_att_deriv = np.zeros(3)
        self.last_rate_deriv = np.zeros(3)

    def set_hover_point(self, pos, att, throttle):
        """设置悬停参考点"""
        self.target_pos = np.array(pos)
        self.hover_throttle = throttle

    def reset(self):
        """重置控制器状态"""
        self.int_pos = np.zeros(2)
        self.int_att = np.zeros(3)
        self.int_rate = np.zeros(3)
        self.int_alt = 0.0
        self.last_pos_error = np.zeros(2)
        self.last_att_error = np.zeros(3)
        self.last_rate_error = np.zeros(3)
        self.last_alt_error = None
        self.last_pos_deriv_xy = np.zeros(2)
        self.last_att_deriv = np.zeros(3)
        self.last_rate_deriv = np.zeros(3)
        self._first_call = True

    def compute(self, t, state, dt=0.01, q_desired_override=None, throttle_override=None):
        """
        计算控制输出

        参数：
            t: 时间 [s]
            state: [u, v, w, p, q, r, qx, qy, qz, qw, px, py, pz]
            dt: 时间步长 [s]
            q_desired_override: 若提供，直接作为期望姿态四元数 [qw, qx, qy, qz]，跳过位置环

        返回：
            [throttle_left, throttle_right, de_left, de_right]
        """
        # 计算低通滤波器系数
        if dt > 0:
            self.alpha_filter = dt / (dt + 1.0/(2*np.pi*self.filter_freq))

        # 提取状态
        px, py, pz = state[10:13]
        p_rate, q_rate, r_rate = state[3:6]
        qx, qy, qz, qw = state[6:10]
        q_current = np.array([qw, qx, qy, qz])

        # ============================================================
        # 1. 水平位置控制 (x, y) → 期望倾斜角
        # ============================================================
        if q_desired_override is not None:
            q_desired = q_desired_override / np.linalg.norm(q_desired_override)
        else:
            pos_error_xy = self.target_pos[:2] - np.array([px, py])

            # 添加低通滤波的位置微分
            if self._first_call:
                pos_deriv_xy = np.zeros(2)
            else:
                pos_deriv_xy = (pos_error_xy - self.last_pos_error) / dt if dt > 0 else np.zeros(2)
                pos_deriv_xy = self.alpha_filter * pos_deriv_xy + (1-self.alpha_filter) * self.last_pos_deriv_xy
            self.last_pos_deriv_xy = pos_deriv_xy
            self.last_pos_error = pos_error_xy.copy()

            # 抗积分饱和：当接近目标时衰减积分
            if abs(pos_error_xy[0]) < 0.1 and abs(pos_error_xy[1]) < 0.1:
                self.int_pos *= 0.95

            self.int_pos += pos_error_xy * dt
            self.int_pos = np.clip(self.int_pos, -self.max_int_pos, self.max_int_pos)

            desired_accel_xy = (self.Kp_pos * pos_error_xy +
                                self.Ki_pos * self.int_pos +
                                self.Kd_pos * pos_deriv_xy)

            # 恢复加速度和倾斜角限幅
            max_accel = 9.81 * np.tan(self.max_tilt)
            desired_accel_xy = np.clip(desired_accel_xy, -max_accel, max_accel)

            # 加速度→倾斜角（绕体轴的小角度旋转）
            # y方向加速 → 绕体轴x旋转（滚转）
            # x方向加速 → 绕体轴y旋转（俯仰）
            tilt_roll = np.clip(desired_accel_xy[1] / 9.81, -np.sin(self.max_tilt), np.sin(self.max_tilt))
            tilt_pitch = np.clip(-desired_accel_xy[0] / 9.81, -np.sin(self.max_tilt), np.sin(self.max_tilt))

            # 尾座式悬停映射修正：
            #   x轴朝天，推力沿x轴。
            #   x方向误差 -> 绕y轴俯仰（推力纵向倾斜）
            #   y方向误差 -> 绕z轴偏航（推力横向倾斜）
            #   滚转（绕x轴）不产生水平加速度，不参与位置映射
            q_tilt = euler_to_quaternion(0.0, tilt_pitch, tilt_roll)
            q_desired = quaternion_multiply(self.q_hover, q_tilt)
            q_desired = q_desired / np.linalg.norm(q_desired)

        # ============================================================
        # 2. 高度控制 → 油门
        # ============================================================
        if throttle_override is not None:
            throttle = throttle_override
        else:
            # NED：z向下为正
            # 飞机在目标下方 → pz > target → alt_error > 0 → 需增大油门
            # 飞机在目标上方 → pz < target → alt_error < 0 → 需减小油门
            alt_error = pz - self.target_pos[2]

            if self._first_call or self.last_alt_error is None:
                alt_deriv = 0.0
            else:
                alt_deriv = (alt_error - self.last_alt_error) / dt if dt > 0 else 0.0
            self.last_alt_error = alt_error

            # 抗积分饱和：当接近目标时衰减积分
            if abs(alt_error) < 0.05:
                self.int_alt *= 0.95

            self.int_alt += alt_error * dt
            self.int_alt = np.clip(self.int_alt, -self.max_int_alt, self.max_int_alt)

            throttle = (self.hover_throttle +
                        self.Kp_alt * alt_error +
                        self.Ki_alt * self.int_alt +
                        self.Kd_alt * alt_deriv)
            throttle = np.clip(throttle, 0.0, self.max_throttle)

        # ============================================================
        # 3. 姿态控制（四元数误差→旋转矢量）
        # ============================================================
        # 误差四元数：q_err = q_current^{-1} * q_desired
        # 修正：原公式 q_desired * q_current^{-1} 在 theta=90° 万向锁姿态下
        # 会导致滚转/偏航控制轴交叉映射。正确顺序应为 q_current^{-1} * q_desired。
        q_current_inv = np.array([qw, -qx, -qy, -qz])
        q_err = quaternion_multiply(q_current_inv, q_desired)

        # 确保最短路径（旋转角≤π）
        # 如果q_err[0] < 0，取反方向
        if q_err[0] < 0:
            q_err = -q_err

        # 旋转矢量：小角度下 att_error ≈ 2 * [qx_err, qy_err, qz_err]
        # 这是在体轴下的旋转误差，无万向锁
        att_error = 2.0 * q_err[1:4]

        # 添加低通滤波的姿态微分
        if self._first_call:
            att_deriv = np.zeros(3)
        else:
            att_deriv = (att_error - self.last_att_error) / dt if dt > 0 else np.zeros(3)
            att_deriv = self.alpha_filter * att_deriv + (1-self.alpha_filter) * self.last_att_deriv
        self.last_att_deriv = att_deriv
        self.last_att_error = att_error.copy()

        # 抗积分饱和：当接近目标时衰减积分
        if np.linalg.norm(att_error) < np.radians(2):
            self.int_att *= 0.95

        self.int_att += att_error * dt
        self.int_att = np.clip(self.int_att, -self.max_int_att, self.max_int_att)

        desired_rate = (self.Kp_att * att_error +
                        self.Ki_att * self.int_att +
                        self.Kd_att * att_deriv)

        # 角速度限幅
        desired_rate = np.clip(desired_rate, -self.max_desired_rate, self.max_desired_rate)

        # ============================================================
        # 4. 角速度控制 → 控制指令
        # ============================================================
        current_rate = np.array([p_rate, q_rate, r_rate])
        rate_error = desired_rate - current_rate

        # 添加低通滤波的角速度微分
        if self._first_call:
            rate_deriv = np.zeros(3)
        else:
            rate_deriv = (rate_error - self.last_rate_error) / dt if dt > 0 else np.zeros(3)
            rate_deriv = self.alpha_filter * rate_deriv + (1-self.alpha_filter) * self.last_rate_deriv
        self.last_rate_deriv = rate_deriv
        self.last_rate_error = rate_error.copy()

        # 抗积分饱和：当接近目标时衰减积分
        if np.linalg.norm(rate_error) < np.radians(2):
            self.int_rate *= 0.95

        self.int_rate += rate_error * dt
        self.int_rate = np.clip(self.int_rate, -self.max_int_rate, self.max_int_rate)

        roll_cmd  = self.Kp_rate * rate_error[0] + self.Ki_rate * self.int_rate[0] + self.Kd_rate * rate_deriv[0]
        pitch_cmd = self.Kp_rate * rate_error[1] + self.Ki_rate * self.int_rate[1] + self.Kd_rate * rate_deriv[1]
        yaw_cmd   = self.Kp_rate * rate_error[2] + self.Ki_rate * self.int_rate[2] + self.Kd_rate * rate_deriv[2]

        # ============================================================
        # 5. 控制分配（匹配重构后模型物理）
        # ============================================================
        # 滚转 → 差动升降副翼（左右升力差产生绕x轴力矩）
        # 俯仰 → 对称升降副翼（同步偏转产生绕y轴力矩）
        # 偏航 → 差动油门（左右推力差产生绕z轴力矩）
        de_diff = roll_cmd * self.roll_to_de_diff
        de_sym = pitch_cmd * self.pitch_to_elevator
        throttle_diff = yaw_cmd * self.yaw_to_throttle_diff

        # 符号约定：正roll_cmd → 右舵面升力增大 → de_right 增大
        de_left  = de_sym - de_diff
        de_right = de_sym + de_diff

        # 舵面限幅
        de_left = np.clip(de_left, -self.max_de, self.max_de)
        de_right = np.clip(de_right, -self.max_de, self.max_de)

        # 符号约定：正yaw_cmd → 右油门增大 → 绕z轴正偏航力矩
        throttle_left  = throttle - throttle_diff
        throttle_right = throttle + throttle_diff

        # 油门限幅（已在高度控制中限幅，此处再限幅差动后的值）
        throttle_left = np.clip(throttle_left, 0.0, self.max_throttle)
        throttle_right = np.clip(throttle_right, 0.0, self.max_throttle)

        self._first_call = False
        return np.array([throttle_left, throttle_right, de_left, de_right])
