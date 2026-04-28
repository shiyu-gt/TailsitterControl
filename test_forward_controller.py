"""
test_forward_controller.py
Standalone test for ForwardController I/O logic without aircraft model.
"""

import numpy as np
import matplotlib
matplotlib.rcParams['font.family'] = 'DejaVu Sans'
import matplotlib.pyplot as plt
from forward_controller import ForwardController


def euler_to_quaternion(phi, theta, psi):
    """roll, pitch, yaw -> quaternion [w, x, y, z]"""
    cr, sr = np.cos(phi * 0.5), np.sin(phi * 0.5)
    cp, sp = np.cos(theta * 0.5), np.sin(theta * 0.5)
    cy, sy = np.cos(psi * 0.5), np.sin(psi * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y, z])


def make_state(V, pz, theta_deg=0.0):
    """
    Build a simplified 13-D state vector compatible with aircraft_6dof.
    Only fields used by ForwardController are filled:
      u,v,w (0:3) and pz (12).
    """
    q = euler_to_quaternion(0.0, np.radians(theta_deg), 0.0)  # [w,x,y,z]
    state = np.zeros(13)
    state[0] = V          # u
    state[1] = 0.0        # v
    state[2] = 0.0        # w
    state[6:10] = [q[1], q[2], q[3], q[0]]  # qx, qy, qz, qw
    state[12] = pz        # pz (NED: positive down)
    return state


def run_case(case_name, state0, V_des, z_des, dt, n_steps,
             tau_v=3.0, tau_z=5.0, step_perturb=None):
    """
    Open-loop test: V and pz follow first-order convergence toward targets.
    Records ForwardController outputs.

    step_perturb: optional velocity step at given step, format (step_idx, dV)
    """
    trim_pitch = np.radians(5.0)
    cruise_throttle = 0.5

    ctrl = ForwardController(trim_pitch, cruise_throttle)
    ctrl.reset(theta_trim=trim_pitch)

    t_hist = np.zeros(n_steps)
    theta_hist = np.zeros(n_steps)
    throttle_hist = np.zeros(n_steps)
    V_hist = np.zeros(n_steps)
    z_hist = np.zeros(n_steps)

    state = state0.copy()

    print(f"\n{'='*60}")
    print(f"  {case_name}")
    print(f"{'='*60}")
    print(f"  Params: trim_pitch={np.degrees(trim_pitch):.1f}deg, "
          f"cruise_throttle={cruise_throttle}, dt={dt}, n={n_steps}")
    print(f"  Target: V_des={V_des} m/s, z_des={z_des} m")

    for i in range(n_steps):
        t = i * dt
        t_hist[i] = t

        # ---- simplified state update (first-order inertia) ----
        V_target = V_des
        pz_target = -z_des

        state[0] += (V_target - state[0]) / tau_v * dt
        state[12] += (pz_target - state[12]) / tau_z * dt

        # optional step perturbation (Case 2 demo)
        if step_perturb and i == step_perturb[0]:
            state[0] += step_perturb[1]

        V = np.sqrt(state[0]**2 + state[1]**2 + state[2]**2)
        V_hist[i] = V
        z_hist[i] = -state[12]

        theta_des, phi_des, throttle_fwd = ctrl.compute(state, V_des, z_des, dt)
        theta_hist[i] = np.degrees(theta_des)
        throttle_hist[i] = throttle_fwd

        # print first 3, middle 3, last 3 steps
        if i < 3 or (n_steps // 2 - 1 <= i <= n_steps // 2 + 1) or i >= n_steps - 3:
            print(f"  step {i:3d}  t={t:.2f}s  V={V:6.2f} m/s  "
                  f"theta_des={np.degrees(theta_des):7.2f}deg  "
                  f"throttle={throttle_fwd:.4f}")

    return t_hist, theta_hist, throttle_hist, V_hist, z_hist


def plot_case(t, theta, throttle, V, z, case_name, filename,
              trim_pitch_deg=5.0):
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    ax = axes[0]
    ax.plot(t, theta, 'b-', linewidth=1.5, label=r'$\theta_{des}$')
    ax.axhline(trim_pitch_deg, color='r', linestyle='--', alpha=0.7,
               label=f'trim_pitch = {trim_pitch_deg:.1f}deg')
    ax.set_ylabel(r'$\theta_{des}$ (deg)')
    ax.set_title(f'{case_name} — ForwardController Output')
    ax.legend(loc='best')
    ax.grid(True)

    ax = axes[1]
    ax.plot(t, throttle, 'g-', linewidth=1.5, label='throttle_fwd')
    ax.axhline(0.5, color='r', linestyle='--', alpha=0.7,
               label='cruise_throttle = 0.5')
    ax.set_ylabel('throttle_fwd')
    ax.set_xlabel('Time (s)')
    ax.legend(loc='best')
    ax.grid(True)

    plt.tight_layout()
    fig.savefig(filename, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Plot saved: {filename}")


# =============================================================================
# Main
# =============================================================================
if __name__ == "__main__":
    dt = 0.01
    n_steps = 200  # 2 seconds

    # ---- Case 1: forward transition (hover -> cruise) ----
    state0 = make_state(V=0.0, pz=-20.0, theta_deg=90.0)
    t, theta, throttle, V, z = run_case(
        "Case 1 (Forward Transition: Hover -> Cruise)",
        state0, V_des=20.0, z_des=50.0, dt=dt, n_steps=n_steps,
        tau_v=3.0, tau_z=5.0
    )
    plot_case(t, theta, throttle, V, z,
              "Case 1: Forward Transition", "test_case1.png")

    # ---- Case 2: cruise hold with velocity step perturbation ----
    state0 = make_state(V=20.0, pz=-50.0, theta_deg=5.0)
    t, theta, throttle, V, z = run_case(
        "Case 2 (Cruise Hold: Velocity Step Perturbation)",
        state0, V_des=20.0, z_des=50.0, dt=dt, n_steps=n_steps,
        tau_v=2.0, tau_z=5.0, step_perturb=(100, -3.0)
    )
    plot_case(t, theta, throttle, V, z,
              "Case 2: Cruise Hold", "test_case2.png")

    # ---- Case 3: recovery deceleration (cruise -> hover) ----
    state0 = make_state(V=20.0, pz=-50.0, theta_deg=5.0)
    t, theta, throttle, V, z = run_case(
        "Case 3 (Recovery Deceleration: Cruise -> Hover)",
        state0, V_des=0.0, z_des=20.0, dt=dt, n_steps=n_steps,
        tau_v=3.0, tau_z=5.0
    )
    plot_case(t, theta, throttle, V, z,
              "Case 3: Recovery Deceleration", "test_case3.png")

    print("\nAll tests completed.")
