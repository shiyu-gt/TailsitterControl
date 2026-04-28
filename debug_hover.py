"""
Debug hover simulation - print first 1s of state and control
"""
import numpy as np
from aircraft_6dof import Aircraft6DOF, euler_to_quaternion
from hover_pid_controller import HoverPID
from hover_simulation import integrate_6dof_quaternion

ac = Aircraft6DOF()
rho = 1.225
pid = HoverPID()

# Get trim state
x0_trim, u0_trim, alpha_t, theta_t, rho = ac.trim(V_trim=0.0, h_trim=0.0)
hover_throttle = u0_trim[0]
pid.set_hover_point(pos=[0.0, 0.0, 0.0], att=None, throttle=hover_throttle)

class DebugController:
    def __init__(self, pid):
        self.pid = pid
        self.last_t = -1.0
    def __call__(self, t, x, dt):
        u = self.pid.compute(t, x, dt)
        if t - self.last_t >= 0.5:
            px, py, pz = x[10:13]
            qx, qy, qz, qw = x[6:10]
            phi = np.arctan2(2*(qw*qx+qy*qz), 1-2*(qx**2+qy**2))
            sin_theta = 2*(qw*qy - qz*qx)
            sin_theta = np.clip(sin_theta, -1.0, 1.0)
            theta = np.arcsin(sin_theta)
            psi = np.arctan2(2*(qw*qz+qx*qy), 1-2*(qy**2+qz**2))
            print(f"t={t:.3f} pos=({px:.3f},{py:.3f},{pz:.3f}) "
                  f"q=({qw:.4f},{qx:.4f},{qy:.4f},{qz:.4f}) "
                  f"att=({np.degrees(phi):.2f},{np.degrees(theta):.2f},{np.degrees(psi):.2f}) "
                  f"u=({u[0]:.4f},{u[1]:.4f},{np.degrees(u[2]):.3f},{np.degrees(u[3]):.3f})")
            self.last_t = t
        return u

pid.reset()
ctrl = DebugController(pid)
t, x_hist, u_hist = integrate_6dof_quaternion(ac, x0_trim, u0_trim, rho, ctrl, (0, 20.0), dt=0.01)

# Check state over time
for i in [0, 100, 500, 1000, 1500, 2000]:
    if i < len(t):
        px, py, pz = x_hist[i, 10:13]
        print(f"t={t[i]:.2f}: pos=({px:.3f},{py:.3f},{pz:.3f})")
