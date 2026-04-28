"""
ForwardController 平飞仿真测试
===============================
使用 ForwardController 单独控制飞机完成平飞仿真，复用 HoverPID 的姿态中环 +
角速度内环 + 控制分配来执行期望姿态。

坐标系说明：
- aircraft_6dof 使用标准 NED（z 向下为正）
- ForwardController 内部 h_err = pz - z_des，其中 pz 和 z_des 均为 NED z 坐标
- 因此调用 ForwardController 时，需将高度目标转为 NED z 坐标：z_des_ned = -z_des_height
- 记录高度时：h = -pz（标准 NED）
"""

import numpy as np
import matplotlib.pyplot as plt

from aircraft_6dof import Aircraft6DOF, euler_to_quaternion, isa_atmosphere
from forward_controller import ForwardController
from hover_pid_controller import HoverPID


def main():
    # ---- 初始化 ----
    ac = Aircraft6DOF()
    hover_pid = HoverPID()
    hover_pid.reset()

    dt = 0.01
    T_total = 30.0
    n_steps = int(T_total / dt)
    record_interval = int(0.1 / dt)  # 每 0.1s 记录一次

    # ---- 配平：巡航状态 V=20 m/s（海平面） ----
    V_trim = 20.0
    x_trim, u_trim, alpha_trim, theta_trim, rho_trim = ac.trim(V_trim=V_trim)
    trim_pitch_deg = np.degrees(theta_trim)
    cruise_throttle = u_trim[0]

    print("=" * 60)
    print("ForwardController 平飞仿真开始")
    print("=" * 60)
    print(f"配平结果: 俯仰角 = {trim_pitch_deg:.2f} deg, 巡航油门 = {cruise_throttle:.3f}")
    print(f"初始状态: V={V_trim:.1f} m/s, 高度=50.0 m (NED pz=-50.0)")
    print("=" * 60)

    # ---- 初始化控制器 ----
    fwd_ctrl = ForwardController(trim_pitch=theta_trim, cruise_throttle=cruise_throttle)
    fwd_ctrl.reset(theta_trim=theta_trim)

    # ---- 初始状态（NED: 高度 50m -> pz = -50.0） ----
    state = x_trim.copy()
    state[12] = -50.0

    # ---- 数据记录 ----
    records = {
        't': [],
        'V': [],
        'h': [],
        'theta': [],
        'throttle_fwd': [],
        'theta_fwd': [],
        'de_sym': [],
    }

    def get_V(state):
        u, v, w = state[0:3]
        return np.sqrt(u**2 + v**2 + w**2)

    def record_data(t, state, throttle_fwd, theta_fwd_deg, de_sym_deg):
        records['t'].append(t)
        records['V'].append(get_V(state))
        records['h'].append(-state[12])  # h = -pz
        _, theta, _, _ = ac.get_attitude(state)
        records['theta'].append(np.degrees(theta))
        records['throttle_fwd'].append(throttle_fwd)
        records['theta_fwd'].append(theta_fwd_deg)
        records['de_sym'].append(de_sym_deg)

    # 记录初始状态
    record_data(0.0, state, cruise_throttle, trim_pitch_deg, np.degrees(u_trim[2]))

    # ---- 主仿真循环 ----
    for i in range(1, n_steps + 1):
        t = i * dt

        # 阶段切换（高度目标为正值，表示地面以上）
        if t < 10.0:
            V_des = 20.0
            z_des_height = 50.0
        elif t < 20.0:
            V_des = 25.0
            z_des_height = 55.0
        else:
            V_des = 15.0
            z_des_height = 45.0

        # ForwardController 期望 z_des 为 NED z 坐标
        z_des_ned = -z_des_height

        # 1. ForwardController 计算期望姿态和油门
        #    传入的 state 保持 NED 坐标系
        theta_fwd, phi_fwd, throttle_fwd = fwd_ctrl.compute(
            state, V_des, z_des_ned, dt
        )

        # 2. 转为期望四元数 (yaw=0)
        q_des = euler_to_quaternion(phi_fwd, theta_fwd, 0.0)

        # 3. HoverPID 计算执行器输出（绕过位置环和高度环）
        ctrl = hover_pid.compute(
            t, state, dt,
            q_desired_override=q_des,
            throttle_override=throttle_fwd
        )
        throttle_L, throttle_R, de_L, de_R = ctrl
        de_sym = 0.5 * (de_L + de_R)

        # 4. 推进飞机模型一步（RK4）
        h = -state[12]
        rho, _ = isa_atmosphere(h)

        def f(s):
            return ac.derivatives(s, np.array([throttle_L, throttle_R, de_L, de_R]), rho)

        k1 = f(state)
        k2 = f(state + 0.5 * dt * k1)
        k3 = f(state + 0.5 * dt * k2)
        k4 = f(state + dt * k3)
        state = state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

        # 归一化四元数
        qx, qy, qz, qw = state[6:10]
        q_norm = np.linalg.norm([qx, qy, qz, qw])
        state[6:10] = [qx / q_norm, qy / q_norm, qz / q_norm, qw / q_norm]

        # 记录数据
        if i % record_interval == 0:
            record_data(t, state, throttle_fwd, np.degrees(theta_fwd), np.degrees(de_sym))

        # 每 2 秒打印
        if i % int(2.0 / dt) == 0:
            V = get_V(state)
            h = -state[12]
            _, theta, _, _ = ac.get_attitude(state)
            print(f"t={t:5.1f}s  V={V:6.2f} m/s  h={h:7.2f} m  "
                  f"theta={np.degrees(theta):7.2f} deg  throttle={throttle_fwd:.3f}")

    # ---- 仿真结束打印 ----
    V_end = get_V(state)
    h_end = -state[12]
    _, theta_end, _, _ = ac.get_attitude(state)
    print("=" * 60)
    print("仿真结束")
    print(f"终点状态: V={V_end:.2f} m/s, h={h_end:.2f} m, theta={np.degrees(theta_end):.2f} deg")
    print("=" * 60)

    # ---- 可视化 ----
    fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    t_arr = np.array(records['t'])

    # 阶段划分辅助线颜色
    phase_color = '#cccccc'

    # 子图 1: 空速
    axs[0].plot(t_arr, records['V'], 'b-', label='V')
    V_des_arr = np.piecewise(
        t_arr,
        [t_arr < 10.0, (t_arr >= 10.0) & (t_arr < 20.0), t_arr >= 20.0],
        [20.0, 25.0, 15.0]
    )
    axs[0].plot(t_arr, V_des_arr, 'r--', label='V_des')
    axs[0].set_ylabel('V (m/s)')
    axs[0].set_title('Forward Flight Simulation')
    axs[0].legend(loc='best')
    axs[0].axvline(10.0, color=phase_color, linestyle=':')
    axs[0].axvline(20.0, color=phase_color, linestyle=':')

    # 子图 2: 高度
    axs[1].plot(t_arr, records['h'], 'b-', label='h')
    z_des_arr = np.piecewise(
        t_arr,
        [t_arr < 10.0, (t_arr >= 10.0) & (t_arr < 20.0), t_arr >= 20.0],
        [50.0, 55.0, 45.0]
    )
    axs[1].plot(t_arr, z_des_arr, 'r--', label='z_des')
    axs[1].set_ylabel('h (m)')
    axs[1].legend(loc='best')
    axs[1].axvline(10.0, color=phase_color, linestyle=':')
    axs[1].axvline(20.0, color=phase_color, linestyle=':')

    # 子图 3: 俯仰角
    axs[2].plot(t_arr, records['theta'], 'b-', label='theta')
    axs[2].plot(t_arr, records['theta_fwd'], 'r--', label='theta_fwd')
    axs[2].axhline(trim_pitch_deg, color='gray', linestyle='--', label='trim_pitch')
    axs[2].set_ylabel('theta (deg)')
    axs[2].legend(loc='best')
    axs[2].axvline(10.0, color=phase_color, linestyle=':')
    axs[2].axvline(20.0, color=phase_color, linestyle=':')

    # 子图 4: 操纵量
    ax4_twin = axs[3].twinx()
    axs[3].plot(t_arr, records['throttle_fwd'], 'b-', label='throttle')
    ax4_twin.plot(t_arr, records['de_sym'], 'r--', label='de_sym')
    axs[3].set_ylabel('throttle', color='b')
    ax4_twin.set_ylabel('de_sym (deg)', color='r')
    axs[3].set_xlabel('Time (s)')
    axs[3].axvline(10.0, color=phase_color, linestyle=':')
    axs[3].axvline(20.0, color=phase_color, linestyle=':')

    # 合并图例
    lines1, labels1 = axs[3].get_legend_handles_labels()
    lines2, labels2 = ax4_twin.get_legend_handles_labels()
    axs[3].legend(lines1 + lines2, labels1 + labels2, loc='best')

    plt.tight_layout()
    plt.savefig('test_forward_flight.png', dpi=150)
    print("图片已保存: test_forward_flight.png")


if __name__ == '__main__':
    main()
