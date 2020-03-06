#pragma once

#include "ekf_types.h"

namespace ekf
{

class EKF
{
private:
    State m_x;
    Rotation m_R_ref;
    Covariance m_P;

    static const Vec3 s_gravity;

public:
    EKF() = default;
    EKF(const Vec3& initial_pos, const Vec3& initial_vel, const Rotation& initial_rot, const Covariance& initial_covar);

    Vec3 get_pos() const { return m_x.segment<3>(0); }
    Vec3 get_vel() const { return m_x.segment<3>(3); }
    Rotation get_rot() const { return m_R_ref; }
    Quaternion get_quat() const { return Quaternion(m_R_ref); }

    void predict(double dt, const Vec3& acc, const Vec3& ang_vel);

    void update_with_position(const Position& measurement);

private:
    void reset_attitude_error();

    static State state_transition_model(const State& x, const Rotation& R_ref, double dt, const Vec3& acc, const Vec3& ang_vel);
    static Jacobian state_transition_jac(const Rotation& R_ref, double dt, const Vec3& acc, const Vec3& ang_vel);
    static Covariance state_transition_var(double dt);

    static Position position_measurement_model(const State& x);
    static PositionJacobian position_measurement_jac();
    static PositionCovariance position_measurement_var();
};

}