# Standard Library Imports
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
def generate_data(num_steps, imu_rate, gps_rate, accel_bias, gyro_bias, accel_noise_std, gyro_noise_std, gps_noise_std, seed=None):
    """
    Generates simulated IMU and GPS data for a stationary object with sensor biases.
    A seed can be provided to ensure reproducible data generation.
    """
    if seed is not None:
        np.random.seed(seed)
    
    dt_imu = 1.0 / imu_rate
    dt_gps = 1.0 / gps_rate
    
    imu_accel_data = []
    imu_gyro_data = []
    gps_data = []

    true_pos = np.zeros(3)
    
    for i in range(num_steps):
        t = i * dt_imu
        
        # Simulate IMU (accelerometer & gyro) data
        true_accel = np.zeros(3)  # Stationary, no true accel
        true_gyro = np.zeros(3)   # Not rotating, no true gyro
        
        accel_measurement = true_accel + accel_bias + np.random.normal(0, accel_noise_std, 3)
        gyro_measurement = true_gyro + gyro_bias + np.random.normal(0, gyro_noise_std, 3)
        
        imu_accel_data.append(accel_measurement)
        imu_gyro_data.append(gyro_measurement)
        
        # Simulate GPS (position) data at a lower rate
        if i % (imu_rate // gps_rate) == 0:
            gps_measurement = true_pos + np.random.normal(0, gps_noise_std, 3)
            gps_data.append(gps_measurement)

    return imu_accel_data, imu_gyro_data, gps_data, dt_imu

class Quaternion:
    def __init__(self, w, x, y, z):
        self.q = np.array([w, x, y, z], dtype=float)

    @property
    def w(self):
        return self.q[0]

    @property
    def x(self):
        return self.q[1]

    @property
    def y(self):
        return self.q[2]

    @property
    def z(self):
        return self.q[3]

    def normalize(self):
        norm = np.linalg.norm(self.q)
        if norm > 0:
            self.q /= norm
        return self

    def conjugate(self):
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def __mul__(self, other):
        # Hamiltonian product
        w1, x1, y1, z1 = self.q
        w2, x2, y2, z2 = other.q
        
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        
        return Quaternion(w, x, y, z)

    def __repr__(self):
        return f"Quaternion(w={self.w:.4f}, x={self.x:.4f}, y={self.y:.4f}, z={self.z:.4f})"


def integrate_quaternion(current_quaternion, angular_velocity, dt):
    """
    Integrates an angular velocity vector over a time step to update a quaternion.
    
    Args:
        current_quaternion (Quaternion): The current orientation quaternion.
        angular_velocity (np.array): A 3D numpy array [x, y, z] of angular velocity.
        dt (float): The time step in seconds.
    
    Returns:
        Quaternion: The updated quaternion.
    """
    
    # Calculate the magnitude of the angular velocity
    omega_mag = np.linalg.norm(angular_velocity)
    
    # Handle the case of zero angular velocity
    if omega_mag == 0:
        return current_quaternion
    
    # Calculate the rotation angle and axis
    theta = omega_mag * dt
    axis = angular_velocity / omega_mag
    
    # Construct the quaternion increment from the axis-angle representation
    sin_half_theta = np.sin(theta / 2.0)
    w_inc = np.cos(theta / 2.0)
    x_inc = axis[0] * sin_half_theta
    y_inc = axis[1] * sin_half_theta
    z_inc = axis[2] * sin_half_theta
    
    increment_q = Quaternion(w_inc, x_inc, y_inc, z_inc)
    
    # Multiply the current quaternion by the increment and normalize
    # The order of multiplication depends on the frame of reference of the angular velocity
    # For angular velocity in the body frame, the correct order is increment_q * current_quaternion
    new_quaternion = increment_q * current_quaternion
    
    return new_quaternion.normalize()

def rotate_vector_by_quaternion(vector, rotation_quaternion):
    """
    Rotates a 3D vector using a rotation quaternion via the sandwich product.

    Args:
        vector (np.array): A 3D numpy array [x, y, z] to be rotated.
        rotation_quaternion (Quaternion): The quaternion representing the rotation.

    Returns:
        np.array: The rotated 3D vector.
    """
    # 1. Represent the vector as a pure quaternion (w=0)
    vector_q = Quaternion(0.0, vector[0], vector[1], vector[2])
    
    # 2. Get the inverse (conjugate) of the rotation quaternion
    # Assumes the quaternion is a unit quaternion
    q_inv = rotation_quaternion.conjugate()
    
    # 3. Perform the sandwich product: q * v_q * q_inv
    rotated_q = rotation_quaternion * vector_q * q_inv
    
    # 4. Extract the vector part from the result
    return np.array([rotated_q.x, rotated_q.y, rotated_q.z])

def run_imu_frame_ekf(imu_accel_data, imu_gyro_data, gps_data, dt_imu, imu_rate, gps_rate,
                             accel_noise_std, gyro_noise_std, gps_noise_std, true_accel_bias, true_gyro_bias):
    """
    
    State: [px_global, py_global, pz_global, vx_global, vy_global, vz_global, ax_imu, ay_imu, az_imu,
            qx, qy, qz, qw, gx, gy, gz, b_ax, b_ay, b_az, b_gx, b_gy, b_gz]
    """
    num_steps = len(imu_accel_data)
    
    # State Vector: Position (3), Velocity (3), Accel (3), Orientation (4), Angular Vel (3), Accel Bias (3), Gyro Bias (3)
    x = np.zeros(22)
    x[9] = 1.0 # Initialize quaternion with [1,0,0,0] for identity rotation
    
    # Error Covariance Matrix (P)
    P = np.eye(22) * 100
    P[9:13, 9:13] = np.eye(4) * 0.1
    
    # Process Noise Covariance Matrix (Q)
    Q = np.diag([
        0.001, 0.001, 0.001,    # Position
        0.01, 0.01, 0.01,       # Velocity
        0.1, 0.1, 0.1,          # Acceleration
        0.001, 0.001, 0.001, 0.001, # Orientation
        0.01, 0.01, 0.01,       # Angular Velocity
        0.0001, 0.0001, 0.0001,     # Accel Bias
        0.0001, 0.0001, 0.0001      # Gyro Bias
    ])
    
    # Measurement Noise Covariance Matrix (R) for GPS, Gyro and Accel
    R_gps = np.diag([gps_noise_std**2, gps_noise_std**2, gps_noise_std**2])
    R_gyro = np.diag([gyro_noise_std**2, gyro_noise_std**2, gyro_noise_std**2])
    R_accel = np.diag([accel_noise_std**2, accel_noise_std**2, accel_noise_std**2])
    
    # Storage for results
    estimated_pos = []
    estimated_vel = []
    estimated_acc = []
    estimated_accel_bias = []
    estimated_gyro_bias = []
    estimated_orientation = []
    
    gps_data_idx = 0
    
    for i in range(num_steps):
        dt = dt_imu
        
        # Prediction step.
        x_pred = np.zeros(22)
        
        x_pred[0:3] = x[0:3] + x[3:6]*dt
        
        global_rot = Quaternion(*x[9:13])
        global_rot = R.from_quat(x[9:13]).as_matrix()
        global_acc = x[6:9]@global_rot
        x_pred[3:6] = x[3:6] + (global_acc+np.array([0,0,-9.8]))*dt
        
        imu_rot = integrate_quaternion(Quaternion(1,0,0,0), x[13:16], dt)
        x_pred[6:9] = rotate_vector_by_quaternion(x_pred[6:9], imu_rot)
        
        x_pred[9:13] = integrate_quaternion(Quaternion(*x[9:13]), x[13:16], dt).q
        x_pred[16:22] = x[16:22]
        F = np.eye(22)
        F[0:3, 3:6] = np.eye(3) * dt
        
        # Jacobian with respect to accel and gyro biases.
        F[3:6, 16:19] = -global_rot * dt
        #19:22
        omega_cross = np.array([[0, -x[21], x[20]],
                                [x[21], 0, -x[19]],
                                [-x[20], x[19], 0]])
        
        F[9:12, 19:22] = -0.5 * global_rot @ omega_cross * dt
        
        P_pred = F@P@F.T + Q * dt
        
        P=P_pred
        x=x_pred
        
        #Update step
        # 1. Gyroscope Update Step (Always available)
        gyro_measurement = imu_gyro_data[i]
        
        # Measurement matrix for gyroscope
        H_gyro = np.zeros((3, 22))
        H_gyro[0:3, 13:16] = np.eye(3) 
        H_gyro[0:3, 19:22] = np.eye(3) 
        
        # Predicted measurement
        h_gyro = x[13:16] + x[19:22]
        
        y_gyro = gyro_measurement - h_gyro
        S_gyro = H_gyro @ P @ H_gyro.T + R_gyro
        K_gyro = P @ H_gyro.T @ inv(S_gyro)
        x = x + K_gyro @ y_gyro
        P = (np.eye(22) - K_gyro @ H_gyro) @ P
        
        # 2. Accelerometer Update Step (Always available)
        accel_measurement = imu_accel_data[i]
        global_rot = Quaternion(*x[9:13])
        
        gravity_in_imu_frame = rotate_vector_by_quaternion(np.array([0, 0, 9.81]),global_rot.conjugate())
        
        # Measurement matrix for accelerometer
        H_accel = np.zeros((3, 22))
        H_accel[0:3, 6:9] = np.eye(3) # Measuring true acceleration
        H_accel[0:3, 16:19] = np.eye(3) # Measuring accel bias
        
        # Predicted measurement
        h_accel = x[6:9] + gravity_in_imu_frame + x[16:19] 
        
        y_accel = accel_measurement - h_accel
        S_accel = H_accel @ P @ H_accel.T + R_accel
        K_accel = P @ H_accel.T @ inv(S_accel)
        x = x + K_accel @ y_accel
        P = (np.eye(22) - K_accel @ H_accel) @ P
        
        # 3. GPS Update Step (if data is available)
        if i % (imu_rate // gps_rate) == 0:
            gps_measurement = gps_data[gps_data_idx]
            gps_data_idx += 1

            # Measurement Matrix (H)
            H_gps = np.zeros((3, 22))
            H_gps[0:3, 0:3] = np.eye(3) # Measure position
            
            # Innovation
            y_gps = gps_measurement - H_gps @ x
            
            # Innovation Covariance
            S_gps = H_gps @ P @ H_gps.T + R_gps
            
            # Kalman Gain
            K_gps = P @ H_gps.T @ inv(S_gps)
            
            # Updated State
            x = x + K_gps @ y_gps
            
            # Updated Error Covariance
            P = (np.eye(22) - K_gps @ H_gps) @ P
            
        estimated_pos.append(x[0:3])
        estimated_vel.append(x[3:6])
        estimated_acc.append(x[6:9])
        estimated_accel_bias.append(x[16:19])
        estimated_gyro_bias.append(x[19:22])
        estimated_orientation.append(x[9:13])
            
    return np.array(estimated_pos), np.array(estimated_vel),np.array(estimated_acc), np.array(estimated_accel_bias), np.array(estimated_gyro_bias), np.array(estimated_orientation)


def plot_results(estimated_pos, estimated_vel, estimated_accel_bias, estimated_gyro_bias, estimated_orientation, true_accel_bias, true_gyro_bias, title):
    """Helper function to plot the results."""
    plt.style.use('dark_background')
    fig, axs = plt.subplots(5, 1, figsize=(10, 20))
    fig.suptitle(title, fontsize=16)

    # Plot Position
    axs[0].plot(estimated_pos[:, 0], label='Estimated X')
    axs[0].plot(estimated_pos[:, 1], label='Estimated Y')
    axs[0].plot(estimated_pos[:, 2], label='Estimated Z')
    axs[0].plot(np.zeros_like(estimated_pos[:, 0]), '--', color='white', label='True Position (0)')
    axs[0].set_title('Position Estimate (meters)')
    axs[0].set_xlabel('IMU Step')
    axs[0].set_ylabel('Position (m)')
    axs[0].legend()
    axs[0].grid(True)
    
    # Plot Velocity
    axs[1].plot(estimated_vel[:, 0], label='Estimated X Vel')
    axs[1].plot(estimated_vel[:, 1], label='Estimated Y Vel')
    axs[1].plot(estimated_vel[:, 2], label='Estimated Z Vel')
    axs[1].plot(np.zeros_like(estimated_vel[:, 0]), '--', color='white', label='True Velocity (0)')
    axs[1].set_title('Velocity Estimate (m/s)')
    axs[1].set_xlabel('IMU Step')
    axs[1].set_ylabel('Velocity (m/s)')
    axs[1].legend()
    axs[1].grid(True)
    
    # Plot Accel Bias
    axs[2].plot(estimated_accel_bias[:, 0], label='Estimated X Accel Bias')
    axs[2].plot(estimated_accel_bias[:, 1], label='Estimated Y Accel Bias')
    axs[2].plot(estimated_accel_bias[:, 2], label='Estimated Z Accel Bias')
    axs[2].plot(np.full_like(estimated_accel_bias[:, 0], true_accel_bias[0]), '--', color='red', label='True Accel Bias')
    axs[2].plot(np.full_like(estimated_accel_bias[:, 1], true_accel_bias[1]), '--', color='green')
    axs[2].plot(np.full_like(estimated_accel_bias[:, 2], true_accel_bias[2]), '--', color='blue')
    axs[2].set_title('Accelerometer Bias Estimate')
    axs[2].set_xlabel('IMU Step')
    axs[2].set_ylabel('Bias')
    axs[2].legend()
    axs[2].grid(True)
    
    # Plot Gyro Bias
    axs[3].plot(estimated_gyro_bias[:, 0], label='Estimated X Gyro Bias')
    axs[3].plot(estimated_gyro_bias[:, 1], label='Estimated Y Gyro Bias')
    axs[3].plot(estimated_gyro_bias[:, 2], label='Estimated Z Gyro Bias')
    axs[3].plot(np.full_like(estimated_gyro_bias[:, 0], true_gyro_bias[0]), '--', color='red', label='True Gyro Bias')
    axs[3].plot(np.full_like(estimated_gyro_bias[:, 1], true_gyro_bias[1]), '--', color='green')
    axs[3].plot(np.full_like(estimated_gyro_bias[:, 2], true_gyro_bias[2]), '--', color='blue')
    axs[3].set_title('Gyroscope Bias Estimate')
    axs[3].set_xlabel('IMU Step')
    axs[3].set_ylabel('Bias')
    axs[3].legend()
    axs[3].grid(True)

    # Plot Orientation (Euler angles)
    # Convert quaternions to Euler angles for plotting
    estimated_euler = R.from_quat(estimated_orientation[:, [1, 2, 3, 0]]).as_euler('xyz')
    axs[4].plot(estimated_euler[:, 0], label='Estimated Roll')
    axs[4].plot(estimated_euler[:, 1], label='Estimated Pitch')
    axs[4].plot(estimated_euler[:, 2], label='Estimated Yaw')
    axs[4].plot(np.zeros_like(estimated_euler[:, 0]), '--', color='white', label='True Orientation (0)')
    axs[4].set_title('Orientation Estimate (Euler Angles)')
    axs[4].set_xlabel('IMU Step')
    axs[4].set_ylabel('Angle (rad)')
    axs[4].legend()
    axs[4].grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    # Simulation Parameters
    NUM_STEPS = 100
    IMU_RATE = 100 # Hz
    GPS_RATE = 1 # Hz
    ACCEL_BIAS = np.array([0.1, 0.2, 0.3]) # m/s^2
    GYRO_BIAS = np.array([0.05, -0.05, -0.05]) # rad/s
    ACCEL_NOISE_STD = 0.05
    GYRO_NOISE_STD = 0.01
    GPS_NOISE_STD = 0.5
    SEED = 420 
    
    print("Running EKF with IMU Frame...")
    imu_accel_data, imu_gyro_data, gps_data, dt_imu = generate_data(
        NUM_STEPS, IMU_RATE, GPS_RATE, ACCEL_BIAS, GYRO_BIAS, ACCEL_NOISE_STD, GYRO_NOISE_STD, GPS_NOISE_STD, seed=SEED
    )
    
    est_pos, est_vel,est_acc, est_accel_bias, est_gyro_bias, est_orientation = run_imu_frame_ekf(
        imu_accel_data, imu_gyro_data, gps_data, dt_imu, IMU_RATE, GPS_RATE,
        ACCEL_NOISE_STD, GYRO_NOISE_STD, GPS_NOISE_STD, ACCEL_BIAS, GYRO_BIAS
    )
    
    plot_results(est_pos, est_vel, est_accel_bias, est_gyro_bias, est_orientation, ACCEL_BIAS, GYRO_BIAS, "EKF: Position Estimate in IMU Frame")

