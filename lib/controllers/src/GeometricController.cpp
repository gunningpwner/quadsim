#include "GeometricController.h"

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
    // Extracts the vector components from the skew-symmetric matrix
    // consistent with the provided skew() function.
    // v(0) corresponds to m(2, 1)
    // v(1) corresponds to m(0, 2)
    // v(2) corresponds to m(1, 0)

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
      kv(1.0f)
{
}
void GeometricController::run()
{
    state_data = m_state_consumer.readLatest();
}

void GeometricController::updatePositionControl()
{

    Matrix3f rot_mat = QuatMap(state_data->orientation.data()).toRotationMatrix();
    Vector3f axis_down = rot_mat * Vector3f::UnitZ();
    Vector3f axis_ddown = rot_mat * skew(Vec3Map(state_data->angular_vel.data())) * Vector3f::UnitZ();

    Vector3f pos_err = Vec3Map(state_data->position_enu.data()) - pos_desired;
    Vector3f vel_err = Vec3Map(state_data->velocity_enu.data()) - vel_desired;

    Vector3f A = -kx * pos_err - kv * vel_err - mass * 9.81f * Vector3f::UnitZ() + mass * acc_desired;

    float thrust = -A.dot(axis_down);

    Vector3f acc_err = 9.81f * Vector3f::UnitZ() - thrust / mass * axis_down - acc_desired;
    Vector3f A_dot = -kx * vel_err - kv * acc_err + mass * jerk_desired;

    float thrust_dot = -A_dot.dot(axis_down) - A_dot.dot(axis_ddown);
    Vector3f jerk_err = -thrust_dot / mass * axis_down - thrust / mass * axis_ddown - jerk_desired;
    Vector3f A_ddot = -kx * acc_err - kv * jerk_err + mass * snap_desired;

    // Can probably make this way more efficient by initializing the rot_cmd matrices here and
    // then just using .row to do stuff in place

    Vector3f b3c, b3c_dot, b3c_ddot;
    axis_deriv(A, A_dot, A_ddot, b3c, b3c_dot, b3c_ddot);

    Vector3f &b1d = front_dir_desired;
    Vector3f C = b1d.cross(b3c);
    // I'm just gonna assume front_dir_desired does not change
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
}

void GeometricController::updateRotationControl()
{
    Vector3f omega = Vec3Map(state_data->angular_vel.data());
    Matrix3f rot_mat = QuatMap(state_data->orientation.data()).toRotationMatrix();
    Vector3f rot_err = skewnt(rot_desired.transpose() * rot_mat - rot_mat.transpose() * rot_desired) / 2.0f;
    Vector3f omega_err = omega - rot_mat.transpose() * rot_desired * ang_vel_desired;
    // Vector3f moment = -kr * rot_err - komega * omega_err + omega.cross(inertia_mat * omega) - inertia_mat * (skew(omega) * rot_mat.transpose() * rot_desired * ang_vel_desired - rot_mat.transpose() * rot_desired * ang_acc_desired);
    Vector3f ang_acc_cmd = -kr * rot_err - komega * omega_err +  inertia_mat * (skew(omega) * rot_mat.transpose() * rot_desired * ang_vel_desired - rot_mat.transpose() * rot_desired * ang_acc_desired);
}
