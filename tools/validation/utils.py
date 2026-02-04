import numpy as np
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

def quaternion_from_axis_angle(axis, angle):
    """
    Creates a unit quaternion from a rotation axis and angle.
    Args:
        axis: np.array of shape (3,) (should be normalized)
        angle: float (radians)
    Returns:
        np.array: [w, x, y, z]
    """
    half_angle = angle * 0.5
    sin_half = np.sin(half_angle)
    return np.array([
        np.cos(half_angle),
        axis[0] * sin_half,
        axis[1] * sin_half,
        axis[2] * sin_half
    ])

def quaternion_get_z_axis(q):
    """
    Extracts the Z-axis (3rd column) of the rotation matrix represented by q.
    Equivalent to rotating the vector [0, 0, 1] by the quaternion.
    """
    w, x, y, z = q
    return np.array([
        2.0 * (x*z + w*y),
        2.0 * (y*z - w*x),
        1.0 - 2.0 * (x*x + y*y)
    ])