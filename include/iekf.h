#pragma once

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>

#include "iekf_types.h"

namespace invariant
{

class IEKF
{
private:
    State m_X;
    Covariance<9> m_P;

    static const ugl::Vector3 s_gravity;

public:
    IEKF() = default;
    IEKF(const IEKF&) = default;
    IEKF(IEKF&&) = default;
    IEKF& operator=(const IEKF&) = default;
    IEKF& operator=(IEKF&&) = default;
    ~IEKF() = default;
    
    IEKF(const ugl::Rotation& R0, const ugl::Vector3& p0, const ugl::Vector3& v0, const Covariance<9>& P0);

    ugl::Vector3 get_pos() const { return m_X.get_pos(); }
    ugl::Vector3 get_vel() const { return m_X.get_vel(); }
    ugl::Rotation get_rot() const { return m_X.get_rot(); }
    ugl::UnitQuaternion get_quat() const { return ugl::UnitQuaternion(m_X.get_rot()); }

    void predict(double dt, const ugl::Vector3& acc, const ugl::Vector3& ang_vel);
    void mocap_update(const ugl::Rotation& R_measured, const ugl::Vector3& pos_measured);

private:
};

}