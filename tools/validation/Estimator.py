import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, Tuple
from scipy.spatial.transform import Rotation
import pandas as pd
from SimCore import Quadcopter
from DataLogger import DataLogger
# %matplotlib qt
# --- Configuration Classes ---
@dataclass
class SensorConfig:
    update_rate_hz: float
    noise_std: np.ndarray  # Standard deviation vector
    bias: np.ndarray       # Constant bias vector
    name: str

# --- Data Logger ---

def skew_symmetric(array):
    
    return np.array([[0,-array[2], array[1]],
                         [array[2],0,-array[0]],
                         [-array[1],array[0],0]])

def quaternion_conjugate(q):
    """
    Returns the conjugate (inverse for unit quaternions).
    q = [w, x, y, z] -> q* = [w, -x, -y, -z]
    """
    w, x, y, z = q
    return np.array([w, -x, -y, -z])

def euler_to_quaternion(roll, pitch, yaw):
    """
    Converts Euler angles to a Quaternion (Hamilton Convention: [w, x, y, z]).
    
    Args:
        roll (float): Rotation around X-axis in radians.
        pitch (float): Rotation around Y-axis in radians.
        yaw (float): Rotation around Z-axis in radians.
        
    Returns:
        np.array: A 4-element unit quaternion [w, x, y, z]
    """
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)

    q_w = cr * cp * cy + sr * sp * sy
    q_x = sr * cp * cy - cr * sp * sy
    q_y = cr * sp * cy + sr * cp * sy
    q_z = cr * cp * sy - sr * sp * cy

    return np.array([q_w, q_x, q_y, q_z])

def quaternion_to_euler(quaternions):
    """
    Converts a Quaternion (Hamilton Convention: [w, x, y, z]) or an 
    nx4 array of Quaternions to Euler angles (roll, pitch, yaw) 
    (intrinsic Z-Y-X sequence).

    Args:
        quaternions (np.array): A 4-element unit quaternion [w, x, y, z] 
                                or an nx4 array of quaternions.

    Returns:
        np.array: A 3-element array [roll, pitch, yaw] in radians, 
                  or an nx3 array of [roll, pitch, yaw] for array input.
    """
    # Ensure input is a NumPy array and determine if it's a single quaternion or an array
    quaternions = np.asarray(quaternions)
    
    # Check if the input is a single 4-element quaternion
    is_single_quaternion = quaternions.ndim == 1 or quaternions.shape[-1] != 4
    if is_single_quaternion:
        # Reshape to a 1x4 array for unified processing
        q = quaternions.reshape(1, 4)
        was_single = True
    else:
        # Input is an nx4 array
        q = quaternions
        was_single = False

    # Separate quaternion components
    # The convention is [w, x, y, z]
    q_w = q[:, 0]
    q_x = q[:, 1]
    q_y = q[:, 2]
    q_z = q[:, 3]
    
    # --- Calculations for Z-Y-X (Roll-Pitch-Yaw) sequence ---
    
    # Roll (Rotation around X-axis)
    # roll = atan2(2*(q_w*q_x + q_y*q_z), 1 - 2*(q_x**2 + q_y**2))
    sinr_cosp = 2.0 * (q_w * q_x + q_y * q_z)
    cosr_cosp = 1.0 - 2.0 * (q_x**2 + q_y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (Rotation around Y-axis)
    # pitch = asin(2*(q_w*q_y - q_z*q_x))
    sinp = 2.0 * (q_w * q_y - q_z * q_x)
    
    # Clamping the argument of arcsin to the range [-1, 1] to prevent
    # domain errors due to floating point inaccuracies
    # The max value for sinp is +/-1.0, but float precision can cause values
    # slightly outside this range (e.g., 1.0000000000000002)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)
    
    # Yaw (Rotation around Z-axis)
    # yaw = atan2(2*(q_w*q_z + q_x*q_y), 1 - 2*(q_y**2 + q_z**2))
    siny_cosp = 2.0 * (q_w * q_z + q_x * q_y)
    cosy_cosp = 1.0 - 2.0 * (q_y**2 + q_z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    # Combine the results into an Nx3 array
    eulers = np.stack((roll, pitch, yaw), axis=-1)
    
    # Return the correct shape based on the input
    if was_single:
        # Return a 1D array if a single quaternion was input
        return eulers.flatten()
    else:
        # Return the Nx3 array for array input
        return eulers
    
def axis_angle_to_quaternion(omega_x, omega_y, omega_z, dt):
    """
    Converts an angular velocity vector over a time step into a delta quaternion.
    Implements Equation 101 from the paper.
    
    Args:
        omega_x, omega_y, omega_z (float): Angular rates in rad/s (gyro - bias).
        dt (float): Time step in seconds.
        
    Returns:
        np.array: A 4-element unit quaternion [w, x, y, z] representing the small rotation.
    """
    # 1. Calculate the rotation vector magnitude (angle theta)
    # This corresponds to ||v|| in the paper
    theta = np.sqrt(omega_x**2 + omega_y**2 + omega_z**2) * dt
    
    # Handle the zero-rotation edge case to avoid division by zero
    if theta < 1e-6:
        return np.array([1.0, 0.0, 0.0, 0.0])
    
    # 2. Calculate the unit axis u = v / ||v||
    # We multiply by sin(theta/2) directly to match Eq 101: [cos(phi/2), u*sin(phi/2)]
    half_theta = 0.5 * theta
    k = np.sin(half_theta) / theta # Scaling factor
    
    dq_w = np.cos(half_theta)
    dq_x = omega_x * dt * k
    dq_y = omega_y * dt * k
    dq_z = omega_z * dt * k
    
    return np.array([dq_w, dq_x, dq_y, dq_z])

def quaternion_product(p, q):
    """
    Computes the Hamilton product of two quaternions p and q.
    Implements Equation 12 from "Quaternion kinematics for the error-state Kalman filter".
    
    Args:
        p (np.array): Left quaternion [w, x, y, z]
        q (np.array): Right quaternion [w, x, y, z]
        
    Returns:
        np.array: The resulting quaternion p (x) q
    """
    pw, px, py, pz = p
    qw, qx, qy, qz = q
    
    # Equation 12 matrix expansion
    w = pw * qw - px * qx - py * qy - pz * qz
    x = pw * qx + px * qw + py * qz - pz * qy
    y = pw * qy - px * qz + py * qw + pz * qx
    z = pw * qz + px * qy - py * qx + pz * qw
    
    return np.array([w, x, y, z])

# --- Sensor Base Class ---
class Sensor:
    def __init__(self, config: SensorConfig):
        self.config = config
        self.last_update_time = 0.0
        self.dt = 1.0 / config.update_rate_hz

    def needs_update(self, current_time: float) -> bool:
        return current_time >= self.last_update_time + self.dt

    def add_noise_and_bias(self, true_value: np.ndarray) -> np.ndarray:
        noise = np.random.normal(0, self.config.noise_std)
        return true_value + self.config.bias + noise

class GPS(Sensor): # Base class Sensor is unchanged from previous message
    def __init__(self, config): # Re-init to access Base
        self.config = config
        self.last_update_time = 0.0
        self.dt = 1.0 / config.update_rate_hz

    def needs_update(self, current_time: float) -> bool:
        return current_time >= self.last_update_time + self.dt

    def add_noise_and_bias(self, true_value: np.ndarray) -> np.ndarray:
        noise = np.random.normal(0, self.config.noise_std)
        return true_value + self.config.bias + noise
        
    def get_measurement(self, current_time: float, true_state: np.ndarray) -> np.ndarray:
        if self.needs_update(current_time):
            self.last_update_time = current_time
            # true_state: [pos(3), vel(3), quat(4), rates(3)]
            true_pos_vel = true_state[0:6]
            return self.add_noise_and_bias(true_pos_vel)
        return None

class IMU(Sensor): # Assumes 'Sensor' base class from previous code exists
    def __init__(self, config):
        self.config = config
        self.last_update_time = 0.0
        self.dt = 1.0 / config.update_rate_hz
        # We need access to the Quadcopter class logic for rotation
        # Ideally, we pass the quad object or use static methods, 
        # but for this snippet I will instantiate a dummy quad for math 
        # or rely on the caller passing the quad instance. 
        # Let's assume the caller passes the Quadcopter instance.

    def needs_update(self, current_time: float) -> bool:
        return current_time >= self.last_update_time + self.dt

    def add_noise_and_bias(self, true_value: np.ndarray) -> np.ndarray:
        noise = np.random.normal(0, self.config.noise_std)
        return true_value + self.config.bias + noise

    def get_measurement(self, current_time: float, true_state: np.ndarray, control_input: np.ndarray, quad_model: Quadcopter) -> np.ndarray:
        """
        Calculates measurement using: R_earth2body * (a_earth - g_earth)
        """
        if self.needs_update(current_time):
            self.last_update_time = current_time
            
            # 1. Get True Kinematics
            q_body_to_earth = true_state[6:10]
            true_rates = true_state[10:13]
            
            # 2. Get Coordinate Acceleration in Earth Frame (Includes Gravity acting on object)
            accel_earth = quad_model.get_true_acceleration_earth(true_state, control_input[0:3])
            
            # 3. Define Gravity Vector in Earth Frame
            gravity_earth = np.array([0, 0, -9.81])
            
            # 4. Calculate Proper Acceleration (Specific Force) in Earth Frame
            # Accelerometers measure (a - g). 
            # If stationary: (0 - (-9.81)) = +9.81 UP.
            proper_accel_earth = accel_earth - gravity_earth
            
            # 5. Rotate into Body Frame
            # We need R_earth_to_body, which is the conjugate of q_body_to_earth
            q_earth_to_body = quaternion_conjugate(q_body_to_earth)
            
            # Re-use the quad's rotate function (which does q * v * q_inv)
            # By passing the conjugate q, we perform the inverse rotation.
            proper_accel_body = quad_model.rotate_vector(q_earth_to_body, proper_accel_earth)
            # print(proper_accel_body,proper_accel_earth)
            # 6. Combine with Gyro
            true_measure = np.concatenate([proper_accel_body, true_rates])
            
            return self.add_noise_and_bias(true_measure)
        return None
    
class Magnetometer(Sensor):
    def get_measurement(self, current_time: float, true_state: np.ndarray, quad_model: Quadcopter) -> np.ndarray:
        """
        Measures the Earth's magnetic field in the body frame.
        """
        if self.needs_update(current_time):
            self.last_update_time = current_time
            
            # 1. Get True Orientation
            q_body_to_earth = true_state[6:10]
            
            # 2. Get Earth's Mag Field
            mag_earth = quad_model.mag_earth
            
            # 3. Rotate into Body Frame: R_earth2body * mag_earth
            q_earth_to_body = quaternion_conjugate(q_body_to_earth)
            mag_body = quad_model.rotate_vector(q_earth_to_body, mag_earth)
            
            return self.add_noise_and_bias(mag_body)
        return None
    
# --- Simulation Orchestrator (UPDATED Init) ---
class EstimatorFramework:
    def __init__(self):
        # 1. Setup Ground Truth
        # Note: Initial state size is now 13.
        # Quat [1, 0, 0, 0] is Identity (No rotation)
        x0 = np.zeros(13)
        x0[6] = 1.0 
        # x0[6:10]= euler_to_quaternion(np.deg2rad(15),np.deg2rad(15),0)
        self.quad = Quadcopter(initial_state=x0)
        
        # 2. Setup Sensors
        self.gps = GPS(SensorConfig(
            update_rate_hz=5.0,
            noise_std=np.array([0.5, 0.5, 0.5, 0.1, 0.1, 0.1]), 
            bias=np.array([0.0, -0.0, 0.0, 0.0, 0.0, 0.0]),
            name="GPS"
        ))
        
        self.imu = IMU(SensorConfig(
            update_rate_hz=100.0,
            noise_std=np.array([0.1]*3 + [0.01]*3), 
            bias=np.array([0.2]*3+[.1]*3),
            name="IMU"
        ))
        self.mag = Magnetometer(SensorConfig(
            update_rate_hz=20.0, # Slower than IMU
            noise_std=np.array([0.05, 0.05, 0.05]), # Some noise
            bias=np.array([0.0, 0.0, 0.0]),
            name="MAG"
        ))
        # self.gps = GPS(SensorConfig(
        #     update_rate_hz=5.0,
        #     noise_std=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), 
        #     bias=np.array([0.0, -0.0, 0.0, 0.0, 0.0, 0.0]),
        #     name="GPS"
        # ))
        
        # self.imu = IMU(SensorConfig(
        #     update_rate_hz=100.0,
        #     noise_std=np.array([0.0]*3 + [0.0]*3), 
        #     bias=np.array([0.0]*6),
        #     name="IMU"
        # ))
        # self.mag = Magnetometer(SensorConfig(
        #     update_rate_hz=20.0, # Slower than IMU
        #     noise_std=np.array([0.0, 0.0, 0.0]), # Some noise
        #     bias=np.array([0.0, 0.0, 0.0]),
        #     name="MAG"
        # ))
        self.logger = DataLogger()
        self.time = 0.0
        self.dt_sim = 0.001 

    def run(self, duration_sec: float, filter_callback):
        steps = int(duration_sec / self.dt_sim)
        print(f"Starting Quaternion Sim: {duration_sec}s")
        
        for _ in range(steps):
            # Hover throttle to counter gravity (approx mass=1.0, g=9.81)
            if self.time<5:
                u_control = np.array([0, 0, 10.8, .02, 0, 0])
            elif self.time<10:
                u_control = np.array([0, 3, 8.8, -.02, 0, 0])
            else:
                u_control = np.array([0, 0, 9.8, 0, 0, 0])
            # Physics Step
            true_state,_ = self.quad.step(self.dt_sim, u_control)
            
            # Sensor Updates
            imu_data = self.imu.get_measurement(self.time, true_state, u_control, self.quad)
            
            if imu_data is not None:
                self.logger.log(self.time,imu_acc=imu_data[:3],imu_gyr=imu_data[3:])
                filter_callback(self.time, "IMU", imu_data, self.logger)
            
            gps_data = self.gps.get_measurement(self.time, true_state)
            if gps_data is not None:
                filter_callback(self.time, "GPS", gps_data, self.logger)
            
            mag_data = self.mag.get_measurement(self.time, true_state, self.quad)
            if mag_data is not None:
                self.logger.log(self.time, mag_meas=mag_data)
                filter_callback(self.time, "MAG", mag_data, self.logger)
                
            # Logging Ground Truth
            # Log Quaternion components
            self.logger.log(self.time, 
                            truth_pos=true_state[:3],truth_vel=true_state[3:6],
                            truth_quat=true_state[6:10])
            
            self.time += self.dt_sim
        print("Done.")
# --- USER LAND: How you implement your logic ---

# A simple dummy state estimator (Just trusts the sensors directly)
class SimpleEstimator:
    def __init__(self):
        # State estimate vector consists of 
        # Position, Velocity, Quaternion, Accelerometer Bias, Gyroscope Bias and Gravity Vector
        self._nominal_state=np.zeros(19)
        self._nominal_state[16:19] = [0,0,-9.8]
        # Error state has angular error while nominal state has quaternion
        # hence 18 vs 19
        # self._error_state_covariance=np.eye(18)*1
        self._error_state_covariance=np.diag([1,1,1, 5,5,5, 1,1,1, .1,.1,.1, .1,.1,.1, .05,.05,.05])
        self._nominal_state[6]=1
        self._last_time=0
        #idk man
        self._acc_variance_est=.1**2
        self._acc_bias_est=.01**2
        self._gyro_variance_est=.01**2
        self._gyro_bias_est=.01**2
        
        self._gps_pos_variance = .5**2
        self._gps_vel_variance = .1**2
        self._mag_variance=.05**2
        self.mag_earth = np.array([1.0, 0.0, 0.0]) 
        self.mag_earth = self.mag_earth / np.linalg.norm(self.mag_earth)
        
    def update(self, t, sensor_type, data, logger):
        dt = t-self._last_time
        pos = self._nominal_state[:3]
        vel = self._nominal_state[3:6]
        quat = self._nominal_state[6:10]
        acc_bias = self._nominal_state[10:13]
        gyr_bias = self._nominal_state[13:16]
        grav_vec = self._nominal_state[16:19]
        
        if sensor_type == "GPS":
            H_x = np.zeros((6,19))
            H_x[:6,:6] = np.eye(6)
            X_deltax = np.zeros((19,18))
            X_deltax[:6,:6] = np.eye(6)
            
            Q_deltatheta = 1/2*np.array([[ -quat[1], -quat[2], -quat[3] ],
                                         [ quat[0], -quat[3], quat[2] ],
                                         [ quat[3], quat[0], -quat[1] ],
                                         [ -quat[2], quat[1], quat[0] ]])
            X_deltax[6:10,6:9] = Q_deltatheta
            X_deltax[10:19,9:18] = np.eye(9)
            H = H_x@X_deltax
            V=np.eye(6)
            V[:3,:3]*=self._gps_pos_variance
            V[3:6,3:6]*=self._gps_vel_variance
            self.correction_step(H, V, data, self._nominal_state[:6])
            
            logger.log(t,gps_k=self.K)
        elif sensor_type=="MAG":
            rot_mat = Rotation.from_quat(quat,scalar_first=True).as_matrix()
            pred_mag = rot_mat.T@self.mag_earth
            
            H = np.zeros((3,18))
            H[:,6:9] = skew_symmetric(pred_mag)
            
            V = np.eye(3)*self._mag_variance
            self.correction_step(H, V, data, pred_mag)
            
        elif sensor_type == "IMU":
            
            # IMU measurement: [ax, ay, az, gx, gy, gz]
            acc_meas = data[:3]
            gyr_meas = data[3:]

            rot_mat = Rotation.from_quat(quat,scalar_first=True).as_matrix()
            
            acc_cor=acc_meas-acc_bias

            gyr_cor=gyr_meas-gyr_bias
            logger.log(t,gyr_cor=gyr_cor)
            delta_quat = axis_angle_to_quaternion(*gyr_cor,dt)
            
            pos = pos+vel*dt+1/2*(rot_mat@acc_cor+grav_vec)*dt**2
            vel = vel + (rot_mat@acc_cor+grav_vec)*dt
            quat = quaternion_product(quat,delta_quat)
            
            self._nominal_state = np.concatenate([pos,vel,quat,acc_bias,gyr_bias,grav_vec])
            
            
            
            Fx = np.zeros((18,18))
            Fx[:3,:3] = np.eye(3)
            Fx[:3,3:6] = np.eye(3)*dt
            Fx[3:6,3:6]=np.eye(3)
            
            acc_skew = skew_symmetric(acc_cor)
            
            Fx[3:6,6:9] = -rot_mat@acc_skew*dt
            Fx[3:6,9:12] = -rot_mat*dt
            Fx[3:6,15:18] = np.eye(3)*dt
            # delta_rot = Rotation.from_euler("XYZ",gyr_cor).as_matrix()
            delta_rot = Rotation.from_rotvec(gyr_cor * dt).as_matrix()
            Fx[6:9,6:9]=delta_rot.T
            Fx[6:9,12:15] = -np.eye(3)*dt
            Fx[9:12,9:12] = np.eye(3)
            Fx[12:15,12:15] = np.eye(3)
            Fx[15:18,15:18] = np.eye(3)
            
            Fi = np.zeros((18,12))
            Fi[3:6,0:3] = np.eye(3)
            Fi[6:9,3:6]= np.eye(3)
            Fi[9:12,6:9]= np.eye(3)
            Fi[12:15,9:12]= np.eye(3)
            
            Q_i=np.zeros((12,12))
            Q_i[:3,:3]=np.eye(3)*self._acc_variance_est*dt**2
            Q_i[3:6,3:6]=np.eye(3)*self._gyro_variance_est*dt**2
            Q_i[6:9,6:9]=np.eye(3)*self._acc_bias_est*dt
            Q_i[9:12,9:12]=np.eye(3)*self._gyro_bias_est*dt
            self._error_state_covariance=Fx@self._error_state_covariance@Fx.T + Fi@Q_i@Fi.T
            self._last_time=t
            
            
        # Log the estimate
        logger.log(t,est_pos=self._nominal_state[:3], est_vel=self._nominal_state[3:6],
                   est_acc_bias=self._nominal_state[10:13],est_gyr_bias=self._nominal_state[13:16],est_grav_vec=self._nominal_state[16:19],
                   est_quat=self._nominal_state[6:10],est_covariances=self._error_state_covariance.flatten() )
        
        
    def correction_step(self,H,V,measurement,prediction):
        pos = self._nominal_state[:3]
        vel = self._nominal_state[3:6]
        quat = self._nominal_state[6:10]
        acc_bias = self._nominal_state[10:13]
        gyr_bias = self._nominal_state[13:16]
        grav_vec = self._nominal_state[16:19]
        
        K = self._error_state_covariance@H.T@np.linalg.inv(H@self._error_state_covariance@H.T+V)
        self.K=K
        error_state_mean = K@(measurement-prediction)

        self._error_state_covariance = (np.eye(18)-K@H)@self._error_state_covariance
        
        # inject error state mean into nominal state
        pos=pos+error_state_mean[:3]
        vel=vel+error_state_mean[3:6]
        delta_quat = axis_angle_to_quaternion(*error_state_mean[6:9],1)
        quat=quaternion_product(quat, delta_quat)
        quat = quat / np.linalg.norm(quat)
        acc_bias = acc_bias + error_state_mean[9:12]
        gyr_bias = gyr_bias+error_state_mean[12:15]
        grav_vec = grav_vec+error_state_mean[15:18]
        self._nominal_state = np.concatenate([pos,vel,quat,acc_bias,gyr_bias,grav_vec])
        
        #reset eskf 
        G = np.eye(18)
        G[6:9,6:9] -= skew_symmetric(1/2*error_state_mean[6:9])
        
        self._error_state_covariance=G@self._error_state_covariance@G.T

def plot_interactive_heatmap(data_tensor, time_array, labels, text_data_tensor=None, title_prefix="Data", cmap='coolwarm', vmin=-1, vmax=1):
    """
    Args:
        data_tensor: Used for the heatmap COLORS (usually Correlation).
        text_data_tensor: (Optional) Used for the TEXT values (usually Covariance).
                          If None, defaults to using data_tensor.
    """
    rows, cols = data_tensor.shape[1], data_tensor.shape[2]
    
    # If no specific text data is provided, use the color data for text
    if text_data_tensor is None:
        text_data_tensor = data_tensor

    # 1. Setup the Plot
    fig, ax = plt.subplots(figsize=(14, 10)) # Made slightly wider for scientific notation
    plt.subplots_adjust(bottom=0.25) 
    
    # Initial Image (Colors based on data_tensor)
    im = ax.imshow(data_tensor[0], cmap=cmap, vmin=vmin, vmax=vmax)
    
    ax.set_xticks(np.arange(cols))
    ax.set_yticks(np.arange(rows))
    ax.set_xticklabels(labels, rotation=90)
    ax.set_yticklabels(labels)
    ax.set_title(f"{title_prefix} at t={time_array[0]:.2f}s")
    fig.colorbar(im, ax=ax)

    # 2. Initialize Text Annotations
    text_annotations = []
    
    # Contrast threshold based on the COLOR data (heatmap intensity)
    range_span = max(abs(vmax), abs(vmin))
    contrast_threshold = range_span * 0.6 

    for i in range(rows):
        row_texts = []
        for j in range(cols):
            # Value for Color (Correlation)
            val_color = data_tensor[0, i, j]
            # Value for Text (Covariance)
            val_text = text_data_tensor[0, i, j]
            
            # Determine text color based on the BACKGROUND intensity
            text_color = "white" if abs(val_color) > contrast_threshold else "black"
            
            # Create text using scientific notation for covariance
            text = ax.text(j, i, f"{val_text:.2e}", 
                           ha="center", va="center", 
                           color=text_color, 
                           fontsize=6) 
            row_texts.append(text)
        text_annotations.append(row_texts)

    # 3. Create the Slider
    ax_slider = plt.axes([0.2, 0.1, 0.6, 0.03], facecolor='lightgoldenrodyellow')
    slider = Slider(
        ax=ax_slider,
        label='Time (s)',
        valmin=time_array[0],
        valmax=time_array[-1],
        valinit=time_array[0],
    )

    # 4. Update Function
    def update(val):
        t_selected = slider.val
        idx = (np.abs(time_array - t_selected)).argmin()
        
        # Get matrices for this timestep
        matrix_color = data_tensor[idx]
        matrix_text = text_data_tensor[idx]
        
        # Update heatmap colors
        im.set_data(matrix_color)
        
        # Update text values and text colors
        for i in range(rows):
            for j in range(cols):
                val_c = matrix_color[i, j]
                val_t = matrix_text[i, j]
                
                # Update number (Un-normalized Covariance)
                text_annotations[i][j].set_text(f"{val_t:.2e}")
                
                # Update visibility (based on Correlation background)
                text_annotations[i][j].set_color("white" if abs(val_c) > contrast_threshold else "black")

        ax.set_title(f"{title_prefix} at t={time_array[idx]:.2f}s")
        fig.canvas.draw_idle()

    slider.on_changed(update)
    plt.tight_layout()
    print(f"Plotting {title_prefix}...")
    plt.show()

def plot_covariance_heatmap(data_dict):
    # 1. Extract and reshape data
    raw_cov = data_dict['est_covariances']
    times = raw_cov[:, 0]
    
    # Reshape to (N, 18, 18) -> This is the RAW COVARIANCE
    cov_matrices = raw_cov[:, 1:].reshape(-1, 18, 18)
    
    # 2. Pre-compute Correlation Matrices (for COLORS)
    corr_matrices = []
    epsilon = 1e-9 
    
    print("Pre-computing correlation matrices...")
    for P in cov_matrices:
        std_dev = np.sqrt(np.diag(P))
        denominator = np.outer(std_dev, std_dev) + epsilon
        rho = P / denominator
        rho = np.clip(rho, -1, 1)
        corr_matrices.append(rho)
        
    corr_matrices = np.array(corr_matrices)

    labels = ['Px', 'Py', 'Pz', 'Vx', 'Vy', 'Vz', 
              'Roll', 'Pitch', 'Yaw', 
              'Ba_x', 'Ba_y', 'Ba_z', 'Bg_x', 'Bg_y', 'Bg_z', 
              'Gx', 'Gy', 'Gz']

    # 3. Call Generic Plotter
    plot_interactive_heatmap(
        data_tensor=corr_matrices,      # Colors = Correlation
        text_data_tensor=cov_matrices,  # Text = Raw Covariance
        time_array=times,
        labels=labels,
        title_prefix="Covariance Analysis",
        cmap='coolwarm',
        vmin=-1,
        vmax=1
    )
# --- Execution ---
if __name__ == "__main__":
    # Initialize Framework
    def calc_error(truth_in,est_in):
        truth = pd.DataFrame(truth_in[:,1:4],truth_in[:,0])
        est=pd.DataFrame(est_in[:,1:4],est_in[:,0])
        diff=(truth-est).dropna()
        diff.reset_index(inplace=True)
        return diff.values
    
    def plot_three(data,label,times=None,components=None,**kwargs):
        if components is None:
            components = 'XYZ'
        if times is None:
            plt.plot(data[:,0],data[:,1],label=f'{label} {components[0]}',**kwargs)
            plt.plot(data[:,0],data[:,2],label=f'{label} {components[1]}',**kwargs)
            plt.plot(data[:,0],data[:,3],label=f'{label} {components[2]}',**kwargs)            
        else:
            plt.plot(times,data[:,0],label=f'{label} {components[0]}',**kwargs)
            plt.plot(times,data[:,1],label=f'{label} {components[1]}',**kwargs)
            plt.plot(times,data[:,2],label=f'{label} {components[2]}',**kwargs)           
            
    np.random.seed(1234)
    
    sim = EstimatorFramework()
    
    # Initialize your Custom Filter
    my_filter = SimpleEstimator()
    
    # Run Simulation
    sim.run(duration_sec=60.0, filter_callback=my_filter.update)
    
    # Plotting Results
    data = sim.logger.get_arrays()
    
    plt.close('all')
    
    plt.figure(figsize=(10, 6))
    plt.title("Position")
    plot_three(data['truth_pos'],'Truth',ls='-')
    plot_three(data['est_pos'],'Estimated',ls='--')
    plt.legend()
    plt.twinx()
    err=calc_error(data['truth_pos'],data['est_pos'])
    plt.plot(err[:,0], np.linalg.norm(err[:,1:4],axis=1), '-', label='Error')
    plt.grid()

    
    plt.figure(figsize=(10, 6))
    plt.title("Velocity Estimation")
    plot_three(data['truth_vel'],'Truth',ls='-')
    plot_three(data['est_vel'],'Estimated',ls='--')
    plt.legend()
    plt.twinx()
    err=calc_error(data['truth_vel'],data['est_vel'])
    plt.plot(err[:,0], np.linalg.norm(err[:,1:4],axis=1), '-', label='Error')
    plt.grid()
    
    plt.figure(figsize=(10, 6))
    plt.title("orientation")
    est_ori = np.rad2deg(quaternion_to_euler(data['est_quat'][:,1:5]))
    plot_three(est_ori,'Estimated',times=data['est_quat'][:,0],ls='--')
    truth_ori = np.rad2deg(quaternion_to_euler(data['truth_quat'][:,1:5]))
    plot_three(truth_ori,'Truth',times=data['truth_quat'][:,0],ls='-')
    plt.legend()
    plt.grid()
    
    plt.figure(figsize=(10, 6))
    plt.title("Biases")
    plot_three(data['est_acc_bias'],'Acc',ls='-')
    plot_three(data['est_gyr_bias'],'Gyro',ls='--')
    plt.legend()
    plt.grid()
    
    plt.figure(figsize=(10, 6))
    plt.title("Gravity")
    plot_three(data['est_grav_vec'],'Grav',ls='-')
    plt.legend()
    plt.grid()
    
    plot_covariance_heatmap(data)