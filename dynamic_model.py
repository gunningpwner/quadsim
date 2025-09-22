import sympy as sp
from sympy.physics.mechanics import dynamicsymbols, init_vprinting
from sympy import Quaternion
sp.init_session()
from sympy import *
init_vprinting()

p = symbols('p_x p_y p_z')
v = symbols('v_x v_y v_z')
a = symbols ('a_x a_y a_z')
theta = symbols('theta_w theta_x theta_y theta_z')
omega = symbols('omega_x omega_y omega_z')
imu_bias = symbols('imu_bx imu_by imu_bz')
gyro_bias = symbols('gyro_bx gyro_by gyro_bz')

dt = symbols('dt')
# state is [px_global, py_global, pz_global, vx_global, vy_global, vz_global, ax_imu, ay_imu, az_imu,
#        qx, qy, qz, qw, gx, gy, gz, b_ax, b_ay, b_az, b_gx, b_gy, b_gz]
def integrate_angular_velocity_to_matrix(angular_velocity, dt):
    """
    Integrates an angular velocity vector over a time step to get a rotation matrix.
    
    Args:
        angular_velocity (list or tuple): A 3D list/tuple [x, y, z] of angular velocity.
        dt (float): The time step in seconds.
    
    Returns:
        sympy.Matrix: The 3x3 rotation matrix.
    """
    omega_x, omega_y, omega_z = angular_velocity
    
    # Calculate the magnitude of the angular velocity
    omega_mag = sqrt(omega_x**2 + omega_y**2 + omega_z**2)
    
    # Handle the case of zero angular velocity
    if omega_mag == 0:
        return Matrix.eye(3)
    
    # Calculate the rotation angle and axis
    theta = omega_mag * dt
    
    # Construct the unit axis vector
    k = Matrix([omega_x, omega_y, omega_z]) / omega_mag
    
    # Construct the skew-symmetric matrix K
    K = Matrix([
        [0, -k[2], k[1]],
        [k[2], 0, -k[0]],
        [-k[1], k[0], 0]
    ])
    
    # Apply Rodrigues' formula
    I = Matrix.eye(3)
    R = I + sin(theta) * K + (1 - cos(theta)) * K**2
    
    return R

def apply_angular_velocity_sympy(current_quaternion, angular_velocity, dt):
    """
    Applies an angular velocity vector over a time step to a quaternion
    using SymPy's symbolic capabilities.

    Args:
        current_quaternion (Quaternion): The current orientation quaternion.
        angular_velocity (list or tuple): A 3D list/tuple [x, y, z] of angular velocity.
        dt (float): The time step in seconds.

    Returns:
        Quaternion: The updated quaternion.
    """
    omega_x, omega_y, omega_z = angular_velocity

    # Calculate the magnitude of the angular velocity
    omega_mag = sqrt(omega_x**2 + omega_y**2 + omega_z**2)

    # Handle the case of zero angular velocity
    if omega_mag == 0:
        return current_quaternion

    # Calculate the rotation angle and the normalized axis of rotation
    theta = omega_mag * dt
    
    # Construct a pure quaternion from the angular velocity for normalization
    omega_q = Quaternion(0, omega_x, omega_y, omega_z)

    # Calculate the quaternion increment
    # SymPy's Quaternion class handles the normalization for us here.
    increment_q = cos(theta / 2.0) + sin(theta / 2.0) * omega_q.normalize()

    # The new quaternion is the product of the increment and the current quaternion.
    # The order of multiplication (increment * current_quaternion) is standard for
    # angular velocities defined in the body frame.
    new_quaternion = increment_q * current_quaternion

    # SymPy's quaternion multiplication automatically produces a new quaternion object.
    # Normalization might not be strictly necessary for symbolic results, but for
    # numerical evaluations, it's good practice.
    return new_quaternion.normalize()

state=[*p,*v,*a,*theta,*omega,*imu_bias,*gyro_bias]

ori = Quaternion(*theta,norm=1)


pmat=Matrix(p)
vmat=Matrix(v)
amat=Matrix(a)
omat = Matrix(omega)
# position, velocity are all in global frame
# acceleration measured in imu frame.
p_pred = pmat+vmat*dt


# rotate acceleration to global frame
a_rot=(ori*Quaternion(0,*a)*ori.inverse()).to_Matrix(vector_only=True)

#need to normalize a_rot and p_mat in code, but for jacobian, omit
v_pred = vmat+(a_rot+sp.Matrix([0,0,-9.8]))*dt


# need to rotate from last measurement frame to new measurement frame
imu_rot = integrate_angular_velocity_to_matrix(omat,dt)
a_pred = imu_rot@amat

ori_pred = apply_angular_velocity_sympy(ori,omat,dt)

new_state = Matrix([*p_pred,*v_pred,*a_pred,*ori_pred.to_Matrix(),*omega,*imu_bias,*gyro_bias])

new_state.jacobian(state)