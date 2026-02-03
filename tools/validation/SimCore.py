import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, Tuple
from scipy.spatial.transform import Rotation
import pandas as pd

# --- [Core Utility Functions - Retained from Original] ---
def skew_symmetric(array):
    return np.array([[0,-array[2], array[1]],
                     [array[2],0,-array[0]],
                     [-array[1],array[0],0]])

def quaternion_conjugate(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])

def quaternion_product(p, q):
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    return np.array([
        pw*qw - px*qx - py*qy - pz*qz,
        pw*qx + px*qw + py*qz - pz*qy,
        pw*qy - px*qz + py*qw + pz*qx,
        pw*qz + px*qy - py*qx + pz*qw
    ])

# --- [Motor Dynamics from SimCore] ---
@dataclass
class MotorParams:
    tau: float = 0.025        # 25ms time constant [cite: 78, 296]
    omega_max: float = 30000  # RPM [cite: 166]
    kappa: float = 0.5        # Non-linearity [cite: 77, 166]
    omega_idle: float = 1500  # Armed idle RPM [cite: 164, 166]
    k_thrust: float = 1.3e-7  # Thrust coefficient (T = k*w^2) [cite: 78]
    k_torque: float = 1.0e-8  # Drag/Reaction torque coefficient [cite: 78]
    j_rotor: float = 1e-5     # Rotor inertia for B2 reaction torque [cite: 66]

class MotorPropellerStack:
    def __init__(self, params: MotorParams):
        self.p = params
        self.omega = np.full(4, self.p.omega_idle) # [cite: 164]
        self.omega_dot = np.zeros(4)               # [cite: 172]

    def step(self, esc_commands: np.ndarray, dt):
        # Ensure commands are clipped to [0, 1] for sqrt safety
        esc_clipped = np.clip(esc_commands, 0, 1)
        # Steady state target (Eq 14) [cite: 166]
        omega_s = self.p.omega_max * (self.p.kappa * esc_clipped + 
                  (1 - self.p.kappa) * np.sqrt(esc_clipped)) + self.p.omega_idle
        
        # First order lag dynamics (Eq 3) [cite: 78]
        new_omega_dot = (omega_s - self.omega) / self.p.tau
        alpha = np.exp(-dt / self.p.tau)
        self.omega = omega_s * (1 - alpha) + self.omega * alpha
        self.omega_dot = (omega_s - self.omega) / self.p.tau
        return self.omega, self.omega_dot

# --- [High-Fidelity QuadPlant] ---
class Quadcopter:
    """Replaces the original Quadcopter class with SimCore dynamics."""
    def __init__(self, initial_state: np.ndarray,use_motors=False):
        self.state = initial_state # [pos(3), vel(3), quat(4), rates(3)]
        self.g = 9.81
        self.mass = 1.0
        self.J = np.diag([0.1, 0.1, 0.2])
        self.J_inv = np.linalg.inv(self.J)
        self.mag_earth = np.array([1.0, 0.0, 0.0])
        
        # Geometry and Motors
        self.arm_length = 0.15
        self.use_motors=use_motors
        self.motors = MotorPropellerStack(MotorParams())
        
    def rotate_vector(self, q: np.ndarray, v: np.ndarray) -> np.ndarray:
        """Body -> Earth rotation using Rotation matrix."""
        R = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix() # To SciPy [x,y,z,w]
        return R @ v

    def get_forces_and_moments(self):
        """Calculates B1 and B2 effects[cite: 69, 88]."""
        omega=self.motors.omega
        omega_dot = self.motors.omega_dot
        
        # 1. Aerodynamic Forces (T = k*w^2) [cite: 78]
        thrusts = self.motors.p.k_thrust * (omega**2)
        f_body = np.array([0, 0, np.sum(thrusts)])
        
        # 2. Moments (B1 and B2)
        # B1: Standard Roll/Pitch control effectiveness
        m_b1 = np.array([
            (thrusts[0] - thrusts[1] - thrusts[2] + thrusts[3]) * self.arm_length,
            (thrusts[0] + thrusts[1] - thrusts[2] - thrusts[3]) * self.arm_length,
            0.0
        ])
        
        # B2: Reaction Torque from rotor acceleration 
        # Yaw = Aerodynamic drag + Rotor Inertia kick
        yaw_drag = self.motors.p.k_torque * (omega**2)
        yaw_accel = self.motors.p.j_rotor * omega_dot
        m_yaw = np.sum((yaw_drag + yaw_accel) * np.array([1, -1, 1, -1]))
        
        m_body = m_b1 + np.array([0, 0, m_yaw])
        return f_body, m_body, omega, omega_dot

    def get_true_acceleration_earth(self, state: np.ndarray, ) -> np.ndarray:
        q = state[6:10]
        f_earth = self.rotate_vector(q, self.f_body)
        return np.array([0, 0, -self.g]) + (f_earth / self.mass)

    def get_state_derivative(self, state: np.ndarray, f_body: np.ndarray, m_body: np.ndarray):
        vel = state[3:6]
        q = state[6:10]
        w = state[10:13]

        # Kinematics
        pos_dot = vel
        self.f_body=f_body
        vel_dot = self.get_true_acceleration_earth(state)
        
        # Quaternion rates
        qw, qx, qy, qz = q
        q_dot = 0.5 * np.array([
            [-qx, -qy, -qz],
            [ qw, -qz,  qy],
            [ qz,  qw, -qx],
            [-qy,  qx,  qw]
        ]) @ w

        # Dynamics (Euler Equations)
        w_dot = self.J_inv @ (m_body - np.cross(w, self.J @ w))

        return np.concatenate([pos_dot, vel_dot, q_dot, w_dot])

    def step(self, dt: float, inputs: np.ndarray):
        """Integration step matching the original Model.py logic."""
        if self.use_motors:
            self.motors.step(inputs,dt)
            f_body, m_body, rpm, rpm_dot = self.get_forces_and_moments()
            # RK1 (Euler) integration
            k1 = self.get_state_derivative(self.state, f_body, m_body)
        else:
            k1 = self.get_state_derivative(self.state, inputs[:3], inputs[3:])
        self.state = self.state + k1 * dt
        
        # Normalize Quat
        self.state[6:10] /= np.linalg.norm(self.state[6:10])
        
        return self.state, self.motors.omega

# --- [Sensor and Framework Classes - Updated for QuadPlant] ---
# Note: Sensor code is truncated but updated to accept 'QuadPlant' as quad_model