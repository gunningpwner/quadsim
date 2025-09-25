# Standard Library Imports
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
def generate_data(num_steps, imu_rate, gps_rate, accel_bias, gyro_bias, accel_noise_std, gyro_noise_std, gps_noise_std, true_angular_velocity, true_velocity=None, seed=None):
    """
    Generates simulated IMU and GPS data for an object with constant angular velocity,
    constant velocity, and sensor biases.
    A seed can be provided to ensure reproducible data generation.
    """
    if true_velocity is None:
        true_velocity = np.zeros(3)

    if seed is not None:
        np.random.seed(seed)
    
    dt_imu = 1.0 / imu_rate
    dt_gps = 1.0 / gps_rate
    
    imu_accel_data = []
    imu_gyro_data = []
    gps_data = []
    true_orientation_data = []

    # Initial state
    true_pos = np.zeros(3)
    # Start with an identity orientation
    true_orientation = R.from_quat([0, 0, 0, 1]) 
    
    for i in range(num_steps):
        t = i * dt_imu
        
        # Store the true orientation at the beginning of the step
        true_orientation_data.append(true_orientation.as_quat()) # Stored as [x, y, z, w]

        # Simulate IMU (accelerometer & gyro) data
        true_accel = true_orientation.apply(np.array([0, 0, 9.81]), inverse=True) # Gravity in body frame
        true_gyro = true_angular_velocity
        
        accel_measurement = true_accel + accel_bias + np.random.normal(0, accel_noise_std, 3)
        gyro_measurement = true_gyro + gyro_bias + np.random.normal(0, gyro_noise_std, 3)
        
        imu_accel_data.append(accel_measurement)
        imu_gyro_data.append(gyro_measurement)
        
        # Update orientation based on true angular velocity
        rotation_increment = R.from_rotvec(true_angular_velocity * dt_imu)
        true_orientation = true_orientation * rotation_increment

        # Update position based on true velocity
        true_pos += true_velocity * dt_imu

        # Simulate GPS (position) data at a lower rate
        if i % (imu_rate // gps_rate) == 0:
            gps_measurement = true_pos + np.random.normal(0, gps_noise_std, 3)
            gps_data.append(gps_measurement)

    return imu_accel_data, imu_gyro_data, gps_data, np.array(true_orientation_data), dt_imu

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
    # P[6:9,6:9] = np.eye(3)*.1
    P[9:13, 9:13] = np.eye(4) * 0.1
    # P[16:22,16:22] = np.eye(6)*10000
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
    estimated_ang_vel = []
    estimated_accel_bias = []
    estimated_gyro_bias = []
    estimated_orientation = []
    P_diag_history = []
    
    gps_data_idx = 0
    
    for i in range(num_steps):
        dt = dt_imu
        
        # Prediction step.
        x_pred = np.zeros(22)
        
        # x_pred[0:3] = x[0:3] + x[3:6]*dt
        
        global_rot = Quaternion(*x[9:13])
        
        global_acc = rotate_vector_by_quaternion(x[6:9], global_rot)
        x_pred[3:6] = x[3:6] + (global_acc)*dt
        
        # imu_rot = integrate_quaternion(Quaternion(1,0,0,0), x[13:16], dt)
        # x_pred[6:9] = rotate_vector_by_quaternion(x_pred[6:9], imu_rot)
        
        # x_pred[9:13] = integrate_quaternion(Quaternion(*x[9:13]), x[13:16], dt).q
        
        x_pred[0:3] = x[0:3] + x[3:6]*dt +.5*x[6:9]*dt**2
        # x_pred[3:6] = x[3:6] + x[6:9]*dt
        x_pred[6:9] = x[6:9]
        
        
        x_pred[9:22] = x[9:22]
        # x_pred[13:22] = x[13:22]
        
        F = np.eye(22)
        F[0:3, 3:6] = np.eye(3) * dt
        F[0:3,6:9] = 05.*np.eye(3)*dt**2
        F[3:6,6:9] = np.eye(3) * dt
        
        # Jacobian with respect to accel and gyro biases.
        # F[3:6, 16:19] = -global_rot * dt
        # omega_cross = np.array([[0, -x[21], x[20]],
        #                         [x[21], 0, -x[19]],
        #                         [-x[20], x[19], 0]])
        
        # F[9:12, 19:22] = -0.5 * global_rot @ omega_cross * dt
        
        P_pred = F@P@F.T + Q * dt
        
        P=P_pred
        x=x_pred
        
        #Update step
        # 1. Gyroscope Update Step (Always available)
        # gyro_measurement = imu_gyro_data[i]
        
        # # Measurement matrix for gyroscope
        # H_gyro = np.zeros((3, 22))
        # H_gyro[0:3, 13:16] = np.eye(3) 
        # H_gyro[0:3, 19:22] = np.eye(3) 
        
        # # Predicted measurement
        # h_gyro = x[13:16] + x[19:22]
        
        # y_gyro = gyro_measurement - h_gyro
        # S_gyro = H_gyro @ P @ H_gyro.T + R_gyro
        # K_gyro = P @ H_gyro.T @ inv(S_gyro)
        # x = x + K_gyro @ y_gyro
        # P = (np.eye(22) - K_gyro @ H_gyro) @ P
        
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
        estimated_orientation.append(x[9:13])
        estimated_ang_vel.append(x[13:16])
        estimated_accel_bias.append(x[16:19])
        estimated_gyro_bias.append(x[19:22])
        P_diag_history.append(np.diag(P))
        
            
    return np.array(estimated_pos), np.array(estimated_vel),np.array(estimated_acc),np.array(estimated_orientation),np.array(estimated_ang_vel), np.array(estimated_accel_bias), np.array(estimated_gyro_bias), np.array(P_diag_history)


def plot_results(estimated_pos, estimated_vel, estimated_acc, imu_accel_data, estimated_accel_bias, estimated_gyro_bias, estimated_orientation, estimated_ang_vel, true_orientation, true_accel_bias, true_gyro_bias, true_angular_velocity, P_diag_history, title):
    """Helper function to plot the results."""
    plt.style.use('dark_background')

    # --- Figure 1: Translational States ---
    fig1, axs1 = plt.subplots(4, 1, figsize=(12, 18))
    fig1.suptitle(f"{title} - Translational States", fontsize=16)

    # Plot Position
    axs1[0].plot(estimated_pos[:, 0], label='Estimated X')
    axs1[0].plot(estimated_pos[:, 1], label='Estimated Y')
    axs1[0].plot(estimated_pos[:, 2], label='Estimated Z')
    axs1[0].plot(np.zeros_like(estimated_pos[:, 0]), '--', color='white', label='True Position (0)')
    axs1[0].set_title('Position Estimate (meters)')
    axs1[0].set_xlabel('IMU Step')
    axs1[0].set_ylabel('Position (m)')
    axs1[0].legend()
    axs1[0].grid(True)
    
    # Plot Velocity
    axs1[1].plot(estimated_vel[:, 0], label='Estimated X Vel')
    axs1[1].plot(estimated_vel[:, 1], label='Estimated Y Vel')
    axs1[1].plot(estimated_vel[:, 2], label='Estimated Z Vel')
    axs1[1].axhline(y=true_velocity[0], linestyle='--', color='red', label='True X Vel')
    axs1[1].axhline(y=true_velocity[1], linestyle='--', color='green', label='True Y Vel')
    axs1[1].axhline(y=true_velocity[2], linestyle='--', color='blue', label='True Z Vel')
    axs1[1].set_title('Velocity Estimate (m/s)')
    axs1[1].set_xlabel('IMU Step')
    axs1[1].set_ylabel('Velocity (m/s)')
    axs1[1].legend(loc='upper right')
    axs1[1].grid(True)

    # Plot Acceleration
    imu_accel_data_np = np.array(imu_accel_data)
    axs1[2].plot(estimated_acc[:, 0], label='Estimated X Accel')
    axs1[2].plot(estimated_acc[:, 1], label='Estimated Y Accel')
    axs1[2].plot(estimated_acc[:, 2], label='Estimated Z Accel')
    axs1[2].plot(imu_accel_data_np[:, 0], '--', color='red', label='IMU X Accel')
    axs1[2].plot(imu_accel_data_np[:, 1], '--', color='green', label='IMU Y Accel')
    axs1[2].plot(imu_accel_data_np[:, 2], '--', color='blue', label='IMU Z Accel')
    axs1[2].set_title('IMU vs. Estimated Acceleration (Body Frame)')
    axs1[2].set_xlabel('IMU Step')
    axs1[2].set_ylabel('Acceleration (m/s^2)')
    axs1[2].legend()
    axs1[2].grid(True)
    
    # Plot Accel Bias
    axs1[3].plot(estimated_accel_bias[:, 0], label='Estimated X Accel Bias')
    axs1[3].plot(estimated_accel_bias[:, 1], label='Estimated Y Accel Bias')
    axs1[3].plot(estimated_accel_bias[:, 2], label='Estimated Z Accel Bias')
    axs1[3].plot(np.full_like(estimated_accel_bias[:, 0], true_accel_bias[0]), '--', color='red', label='True Accel Bias')
    axs1[3].plot(np.full_like(estimated_accel_bias[:, 1], true_accel_bias[1]), '--', color='green')
    axs1[3].plot(np.full_like(estimated_accel_bias[:, 2], true_accel_bias[2]), '--', color='blue')
    axs1[3].set_title('Accelerometer Bias Estimate')
    axs1[3].set_xlabel('IMU Step')
    axs1[3].set_ylabel('Bias')
    axs1[3].legend()
    axs1[3].grid(True)

    fig1.tight_layout(rect=[0, 0.03, 1, 0.96])

    # --- Figure 2: Rotational States ---
    fig2, axs2 = plt.subplots(3, 1, figsize=(12, 15))
    fig2.suptitle(f"{title} - Rotational States", fontsize=16)

    # Plot Orientation (Euler angles)
    # Convert quaternions to Euler angles for plotting
    true_euler = R.from_quat(true_orientation).as_euler('xyz')
    estimated_euler = R.from_quat(estimated_orientation[:, [1, 2, 3, 0]]).as_euler('xyz')
    axs2[0].plot(estimated_euler[:, 0], label='Estimated Roll')
    axs2[0].plot(estimated_euler[:, 1], label='Estimated Pitch')
    axs2[0].plot(estimated_euler[:, 2], label='Estimated Yaw')
    axs2[0].plot(true_euler[:, 0], '--', color='red', label='True Roll')
    axs2[0].plot(true_euler[:, 1], '--', color='green', label='True Pitch')
    axs2[0].plot(true_euler[:, 2], '--', color='blue', label='True Yaw')
    axs2[0].set_title('Orientation Estimate (Euler Angles)')
    axs2[0].set_xlabel('IMU Step')
    axs2[0].set_ylabel('Angle (rad)')
    axs2[0].legend()
    axs2[0].grid(True)

    # Plot Gyro Bias
    axs2[1].plot(estimated_gyro_bias[:, 0], label='Estimated X Gyro Bias')
    axs2[1].plot(estimated_gyro_bias[:, 1], label='Estimated Y Gyro Bias')
    axs2[1].plot(estimated_gyro_bias[:, 2], label='Estimated Z Gyro Bias')
    axs2[1].plot(np.full_like(estimated_gyro_bias[:, 0], true_gyro_bias[0]), '--', color='red', label='True Gyro Bias')
    axs2[1].plot(np.full_like(estimated_gyro_bias[:, 1], true_gyro_bias[1]), '--', color='green')
    axs2[1].plot(np.full_like(estimated_gyro_bias[:, 2], true_gyro_bias[2]), '--', color='blue')
    axs2[1].set_title('Gyroscope Bias Estimate')
    axs2[1].set_xlabel('IMU Step')
    axs2[1].set_ylabel('Bias')
    axs2[1].legend()
    axs2[1].grid(True)

    # Plot Angular Velocity
    axs2[2].plot(estimated_ang_vel[:, 0], label='Estimated X Ang Vel')
    axs2[2].plot(estimated_ang_vel[:, 1], label='Estimated Y Ang Vel')
    axs2[2].plot(estimated_ang_vel[:, 2], label='Estimated Z Ang Vel')
    axs2[2].plot(np.full_like(estimated_ang_vel[:, 0], true_angular_velocity[0]), '--', color='red', label='True Ang Vel')
    axs2[2].plot(np.full_like(estimated_ang_vel[:, 1], true_angular_velocity[1]), '--', color='green')
    axs2[2].plot(np.full_like(estimated_ang_vel[:, 2], true_angular_velocity[2]), '--', color='blue')
    axs2[2].set_title('Angular Velocity Estimate (rad/s)')
    axs2[2].set_xlabel('IMU Step')
    axs2[2].set_ylabel('Angular Velocity (rad/s)')
    axs2[2].legend()
    axs2[2].grid(True)

    fig2.tight_layout(rect=[0, 0.03, 1, 0.96])

    # --- Figure 3: Covariance Matrix Diagonals (Variance) ---
    fig3, axs3 = plt.subplots(4, 1, figsize=(12, 18), sharex=True)
    fig3.suptitle(f"{title} - State Variances (P Matrix Diagonals)", fontsize=16)

    # State labels for the P matrix diagonal
    state_labels = [
        'Pos X', 'Pos Y', 'Pos Z',
        'Vel X', 'Vel Y', 'Vel Z',
        'Bias Acc X', 'Bias Acc Y', 'Bias Acc Z'
    ]
    P_diag_history = np.array(P_diag_history) # Ensure it's a numpy array

    # Disable plots for non-estimated states in this simple version
    axs3[3].set_visible(False)

    # Plot Position and Velocity Variance
    axs3[0].plot(P_diag_history[:, 0:3])
    axs3[0].set_title('Position Variance')
    axs3[0].set_ylabel('Variance (m^2)')
    axs3[0].legend(state_labels[0:3])
    axs3[0].grid(True)

    # Plot Velocity Variance
    axs3[1].plot(P_diag_history[:, 3:6])
    axs3[1].set_title('Velocity Variance')
    axs3[1].set_ylabel('Variance ((m/s)^2)')
    axs3[1].legend(state_labels[3:6])
    axs3[1].grid(True)

    # Plot Accel Bias Variance
    axs3[2].plot(P_diag_history[:, 6:9])
    axs3[2].set_title('Accelerometer Bias Variance')
    axs3[2].set_xlabel('IMU Step')
    axs3[2].set_ylabel('Variance ((m/s^2)^2)')
    axs3[2].legend(state_labels[6:9])
    axs3[2].grid(True)

    fig3.tight_layout(rect=[0, 0.03, 1, 0.96])
    plt.show()

if __name__ == '__main__':
    # Simulation Parameters
    NUM_STEPS = 10000
    IMU_RATE = 100 # Hz
    GPS_RATE = 1 # Hz
    ACCEL_BIAS = np.array([0.1, 0.2, .3]) # m/s^2
    GYRO_BIAS = np.array([0.05, -0.05, -0.05]) # rad/s
    ACCEL_NOISE_STD = 0.05
    GYRO_NOISE_STD = 0.01
    GPS_NOISE_STD = 0.5
    true_angular_velocity=np.array([0,0,0])
    true_velocity = np.array([1.0, -10, 0.2]) # Constant velocity in m/s
    SEED = 420 
    
    print("Running EKF with IMU Frame...")
    imu_accel_data, imu_gyro_data, gps_data, true_orientation_data, dt_imu = generate_data(
        NUM_STEPS, IMU_RATE, GPS_RATE, ACCEL_BIAS, GYRO_BIAS, ACCEL_NOISE_STD, GYRO_NOISE_STD, GPS_NOISE_STD, true_angular_velocity, true_velocity=true_velocity, seed=SEED
    )
    
    est_pos, est_vel,est_acc, est_orientation,est_ang_vel, est_accel_bias, est_gyro_bias, P_diag = run_imu_frame_ekf(
        imu_accel_data, imu_gyro_data, gps_data, dt_imu, IMU_RATE, GPS_RATE,
        ACCEL_NOISE_STD, GYRO_NOISE_STD, GPS_NOISE_STD, ACCEL_BIAS, GYRO_BIAS
    )
    
    plot_results(est_pos, est_vel, est_acc, imu_accel_data, est_accel_bias, est_gyro_bias, est_orientation, est_ang_vel, true_orientation_data, ACCEL_BIAS, GYRO_BIAS, true_velocity, P_diag, "EKF: Position Estimate in IMU Frame")
