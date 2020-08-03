#pragma once

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>

#include "mekf_types.h"

namespace mekf
{

class MEKF
{
private:
    State m_x;
    ugl::Rotation m_R_ref;
    Covariance m_P;

    static const ugl::Vector3 s_gravity;

public:
    MEKF() = default;
    MEKF(const ugl::Vector3& initial_pos, const ugl::Vector3& initial_vel, const ugl::Rotation& initial_rot, const Covariance& initial_covar);

    ugl::Vector3 get_pos() const { return m_x.segment<3>(0); }
    ugl::Vector3 get_vel() const { return m_x.segment<3>(3); }
    ugl::Rotation get_rot() const { return m_R_ref; }
    ugl::UnitQuaternion get_quat() const { return ugl::UnitQuaternion(m_R_ref); }

    void predict(double dt, const ugl::Vector3& acc, const ugl::Vector3& ang_vel);

    void update_with_position(const Position& measurement);

private:
    void reset_attitude_error();

    static State state_transition_model(const State& x, const ugl::Rotation& R_ref, double dt, const ugl::Vector3& acc, const ugl::Vector3& ang_vel);
    static Jacobian state_transition_jac(const ugl::Rotation& R_ref, double dt, const ugl::Vector3& acc, const ugl::Vector3& ang_vel);
    static Covariance state_transition_var(double dt);

    static Position position_measurement_model(const State& x);
    static PositionJacobian position_measurement_jac();
    static PositionCovariance position_measurement_var();
};

}