#pragma once

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>
#include <ugl/lie_group/rotation.h>

#include "mekf_types.h"

namespace mekf
{

class MEKF
{
public:
    MEKF() = default;
    MEKF(const ugl::lie::Rotation& R0, const ugl::Vector3& p0, const ugl::Vector3& v0, const Covariance<9>& P0);

    ugl::Vector3 get_pos() const { return m_x.segment<3>(0); }
    ugl::Vector3 get_vel() const { return m_x.segment<3>(3); }
    ugl::lie::Rotation get_rot() const { return m_R_ref; }
    ugl::UnitQuaternion get_quat() const { return m_R_ref.to_quaternion(); }

    void set_pos(const ugl::Vector3& pos) { m_x.segment<3>(0) = pos; }
    void set_vel(const ugl::Vector3& vel) { m_x.segment<3>(3) = vel; }
    void set_rot(const ugl::lie::Rotation& rot) { m_R_ref = rot; }
    void set_quat(const ugl::UnitQuaternion& quat) { m_R_ref = ugl::lie::Rotation{quat}; }

    void predict(double dt, const ugl::Vector3& acc, const ugl::Vector3& ang_vel);
    void mocap_update(const ugl::lie::Rotation& R_measured, const ugl::Vector3& pos_measured);
    void position_update(const Position& measurement);

private:
    void reset_attitude_error();

    static State state_transition_model(const State& x, const ugl::lie::Rotation& R_ref, double dt, const ugl::Vector3& acc, const ugl::Vector3& ang_vel);
    static Jacobian state_transition_jac(const ugl::lie::Rotation& R_ref, double dt, const ugl::Vector3& acc, const ugl::Vector3& ang_vel);
    static Covariance<9> state_transition_var(double dt);

    static Position position_measurement_model(const State& x);
    static PositionJacobian position_measurement_jac();
    static Covariance<3> position_measurement_var();

private:
    State m_x;
    ugl::lie::Rotation m_R_ref;
    Covariance<9> m_P;

    static const ugl::Vector3 s_gravity;
};

}