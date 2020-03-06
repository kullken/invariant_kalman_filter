#pragma once

#include "iekf_types.h"

namespace iekf
{

class IEKF
{
private:
    State m_X;
    Covariance<9> m_P;

    static const Vector3 s_gravity;

public:
    IEKF() = default;
    IEKF(const Rotation& R0, const Vector3& p0, const Vector3& v0, const Covariance<9>& P0);

    Vector3 get_pos() const { return m_X.get_pos(); }
    Vector3 get_vel() const { return m_X.get_vel(); }
    Rotation get_rot() const { return m_X.get_rot(); }
    Quaternion get_quat() const { return Quaternion(m_X.get_rot()); }

    void predict(double dt, const Vector3& acc, const Vector3& ang_vel);
    void mocap_update(const Rotation& R_measured, const Vector3& pos_measured);

private:
};

}