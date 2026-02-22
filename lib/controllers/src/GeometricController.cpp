#include "GeometricController.h"
#include "Logger.h"
#include "timing.h"

using Vec3Map = Eigen::Map<const Eigen::Vector3f>;
using QuatMap = Eigen::Map<const Eigen::Quaternionf>;

template <typename Derived>
Eigen::Matrix3f skew(const Eigen::MatrixBase<Derived> &v)
{
    Eigen::Matrix3f m;
    m << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return m;
}

template <typename Derived>
Eigen::Vector3f skewnt(const Eigen::MatrixBase<Derived> &m)
{
    return Eigen::Vector3f(m(2, 1), m(0, 2), m(1, 0));
}

void axis_deriv(Vector3f &v_in, Vector3f &vd_in, Vector3f &vdd_in, Vector3f &v_out, Vector3f &vd_out, Vector3f &vdd_out)
{
    float norm = v_in.norm();
    float norm3 = pow(norm, 3);
    float norm5 = pow(norm, 5);
    float norm2_vd = pow(vd_in.norm(), 2);

    v_out = -v_in / norm;

    vd_out = -vd_in / norm + v_in.dot(vd_in) / norm3 * v_in;

    vdd_out = -vdd_in / norm + 2 * v_in.dot(vd_in) / norm3 * vd_in + (norm2_vd + v_in.dot(vdd_in)) / norm3 * v_in - 3 * v_in.dot(vd_in) / norm5 * vd_in;
}

GeometricController::GeometricController(DataManager::StateConsumer m_state_consumer)
    : m_state_consumer(m_state_consumer),
      kx(1.0f),
      kv(1.0f),
      komega(5.0f),
      kr(10.0f)
{
    pos_desired.setZero();
    vel_desired.setZero();
    acc_desired.setZero();
    jerk_desired.setZero();
    snap_desired.setZero();

    vel_cmd.setZero();
    acc_cmd.setZero();
    jerk_cmd.setZero();
    snap_cmd.setZero();

    rot_desired.setIdentity();
    ang_vel_desired.setZero();
    ang_acc_desired.setZero();
    front_dir_desired << 1.0f, 0.0f, 0.0f; // Adjusted for FRD (Forward is X)
    linear_z_accel_cmd = 0.0f;
}

void GeometricController::run()
{
    state_data = m_state_consumer.readLatest();
    updatePositionControl();
    updateVelocityControl();
    updateRotationControl();
}

void GeometricController::updatePositionControl()
{
    // Note: Assuming state_data has been updated to use NED arrays.
    Vector3f pos_err = Vec3Map(state_data->position_ned.data()) - pos_desired;
    
    // Position control outputs a commanded velocity for the velocity loop
    vel_cmd = vel_desired - kx * pos_err;

    // Feedforward derivatives pass through
    acc_cmd = acc_desired;
    jerk_cmd = jerk_desired;
    snap_cmd = snap_desired;
}

void GeometricController::updateVelocityControl()
{
    Vector3f vel_err = Vec3Map(state_data->velocity_ned.data()) - vel_cmd;

    // A is the desired kinematic acceleration of the drone
    Vector3f A = acc_cmd - kv * vel_err;
    
    // In NED, gravity acts in +Z. The drone produces thrust acceleration 'c' in the -b3 direction.
    // Equations of motion: a = g * UnitZ - c * b3
    // We want to track a = A, therefore: c * b3 = g * UnitZ - A
    Vector3f gravity = 9.81f * Vector3f::UnitZ();
    Vector3f v_in = A - gravity;

    Matrix3f rot_mat = QuatMap(state_data->orientation.data()).toRotationMatrix();
    Vector3f b3 = rot_mat * Vector3f::UnitZ();
    Vector3f b3_dot = rot_mat * skew(Vec3Map(state_data->angular_vel.data())) * Vector3f::UnitZ();

    // Thrust acceleration magnitude needed from INDI (c)
    linear_z_accel_cmd = v_in.dot(b3);
    if (linear_z_accel_cmd>0.0f) linear_z_accel_cmd = 0.0f;

    // Differentiate actual acceleration to find velocity tracking error derivatives
    Vector3f current_accel = gravity - linear_z_accel_cmd * b3;
    Vector3f vel_err_dot = current_accel - acc_cmd; 
    
    Vector3f A_dot = jerk_cmd - kv * vel_err_dot;
    Vector3f vd_in = A_dot; // gravity is constant, so derivative of v_in is A_dot

    float c_dot = -vd_in.dot(b3) - v_in.dot(b3_dot);
    Vector3f current_jerk = -c_dot * b3 - linear_z_accel_cmd * b3_dot;
    Vector3f vel_err_ddot = current_jerk - jerk_cmd;

    Vector3f A_ddot = snap_cmd - kv * vel_err_ddot;
    Vector3f vdd_in = A_ddot;

    // Compute desired geometric axes
    Vector3f b3c, b3c_dot, b3c_ddot;
    axis_deriv(v_in, vd_in, vdd_in, b3c, b3c_dot, b3c_ddot);

    Vector3f &b1d = front_dir_desired;
    Vector3f C = b1d.cross(b3c);
    Vector3f C_dot = b1d.cross(b3c_dot);
    Vector3f C_ddot = b1d.cross(b3c_ddot);
    
    Vector3f b2c, b2c_dot, b2c_ddot;
    axis_deriv(C, C_dot, C_ddot, b2c, b2c_dot, b2c_ddot);

    Vector3f b1c = b2c.cross(b3c);
    Vector3f b1c_dot = b2c_dot.cross(b3c) + b2c.cross(b3c_dot);
    Vector3f b1c_ddot = b2c_ddot.cross(b3c) + 2 * b2c_dot.cross(b3c_dot) + b2c.cross(b3c_ddot);

    Matrix3f rot_cmd, rotd_cmd, rotdd_cmd;
    rot_cmd << b1c, b2c, b3c;
    rotd_cmd << b1c_dot, b2c_dot, b3c_dot;
    rotdd_cmd << b1c_ddot, b2c_ddot, b3c_ddot;

    Matrix3f omega_hat = rot_cmd.transpose() * rotd_cmd;
    Vector3f omega_cmd = skewnt(omega_hat);
    Vector3f omegad_cmd = skewnt(rot_cmd.transpose() * rotdd_cmd - omega_hat * omega_hat);

    rot_desired = rot_cmd;
    ang_vel_desired = omega_cmd;
    ang_acc_desired = omegad_cmd;

    // Logger::getInstance().log("rot_desired", rot_desired, getCurrentTimeUs());
    // Logger::getInstance().log("ang_vel_desired", ang_vel_desired, getCurrentTimeUs());
    // Logger::getInstance().log("ang_acc_desired", ang_acc_desired, getCurrentTimeUs());
    // Logger::getInstance().log("rot_mat", rot_mat, getCurrentTimeUs());
    // Logger::getInstance().log("omega_hat", omega_hat, getCurrentTimeUs());
    // Logger::getInstance().log("linear_z_accel_cmd", linear_z_accel_cmd, getCurrentTimeUs());
    Logger::getInstance().log("vel_err", vel_err, getCurrentTimeUs());
    Logger::getInstance().log("A", A, getCurrentTimeUs());
    Logger::getInstance().log("b3", b3, getCurrentTimeUs());

    // Logger::getInstance().log("rot_cmd", rot_cmd, getCurrentTimeUs());
    // Logger::getInstance().log("b2c", b2c, getCurrentTimeUs());
}

void GeometricController::updateRotationControl()
{
    Vector3f omega = Vec3Map(state_data->angular_vel.data());
    Matrix3f rot_mat = QuatMap(state_data->orientation.data()).toRotationMatrix();
    Vector3f rot_err = skewnt(rot_desired.transpose() * rot_mat - rot_mat.transpose() * rot_desired) / 2.0f;
    Vector3f omega_err = omega - rot_mat.transpose() * rot_desired * ang_vel_desired;
    
    // Output angular acceleration directly for the INDI controller
    ang_acc_cmd = -kr * rot_err - komega * omega_err + ang_acc_desired;
    
    Logger::getInstance().log("rot_err", rot_err, getCurrentTimeUs());
    Logger::getInstance().log("omega_err", omega_err, getCurrentTimeUs());
    // Logger::getInstance().log("rot_mat", rot_mat, getCurrentTimeUs());
    // Logger::getInstance().log("rot_desired", rot_desired, getCurrentTimeUs());
}