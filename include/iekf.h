#pragma once

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>

#include "iekf_types.h"

namespace invariant
{

class IEKF
{
public:
    IEKF() = default;
    IEKF(const ugl::Rotation& R0, const ugl::Vector3& p0, const ugl::Vector3& v0, const Covariance<9>& P0);

    ugl::Vector3 get_pos() const { return m_X.get_pos(); }
    ugl::Vector3 get_vel() const { return m_X.get_vel(); }
    ugl::Rotation get_rot() const { return m_X.get_rot(); }
    ugl::UnitQuaternion get_quat() const { return ugl::UnitQuaternion(m_X.get_rot()); }

    void set_pos(const ugl::Vector3& pos) { m_X.set_pos(pos); }
    void set_vel(const ugl::Vector3& vel) { m_X.set_vel(vel); }
    void set_rot(const ugl::Rotation& rot) { m_X.set_rot(rot); }
    void set_quat(const ugl::UnitQuaternion& quat) { m_X.set_rot(ugl::Rotation(quat)); }

    void predict(double dt, const ugl::Vector3& acc, const ugl::Vector3& ang_vel);
    void mocap_update(const ugl::Rotation& R_measured, const ugl::Vector3& pos_measured);

private:
    State m_X = State::Identity();
    Covariance<9> m_P = Covariance<9>::Identity();

    static const ugl::Vector3 s_gravity;
};

}