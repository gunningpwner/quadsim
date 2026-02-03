import numpy as np
from dataclasses import dataclass, field
from SimCore import Quadcopter
from utils import quaternion_conjugate
@dataclass
class SensorConfig:
    update_rate_hz: float
    noise_std: np.ndarray  # Standard deviation vector
    bias: np.ndarray       # Constant bias vector
    name: str
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

    def get_measurement(self, current_time: float, true_state: np.ndarray, quad_model: Quadcopter) -> np.ndarray:
        """
        Calculates measurement using: R_earth2body * (a_earth - g_earth)
        """
        if self.needs_update(current_time):
            self.last_update_time = current_time
            
            # 1. Get True Kinematics
            q_body_to_earth = true_state[6:10]
            true_rates = true_state[10:13]
            
            # 2. Get Coordinate Acceleration in Earth Frame (Includes Gravity acting on object)
            accel_earth = quad_model.get_true_acceleration_earth(true_state)
            
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