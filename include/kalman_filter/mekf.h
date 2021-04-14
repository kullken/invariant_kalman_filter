#ifndef INVARIANT_MEKF_H
#define INVARIANT_MEKF_H

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>

#include <ugl/lie_group/euclidean.h>
#include <ugl/lie_group/rotation.h>
#include <ugl/lie_group/pose.h>
#include <ugl/lie_group/extended_pose.h>

#include "gps_model.h"
#include "imu_model.h"
#include "mocap_model.h"

namespace invariant
{

class MEKF
{
public:
    using State = ugl::Vector<9>;

    template<int n>
    using Covariance = ugl::Matrix<n, n>;

    template<int rows, int cols>
    using Jacobian = ugl::Matrix<rows, cols>;

public:
    MEKF() = default;
    MEKF(const ugl::lie::ExtendedPose& X0, const Covariance<9>& P0);
    MEKF(const ugl::lie::Rotation& R0, const ugl::Vector3& p0, const ugl::Vector3& v0, const Covariance<9>& P0);

    ugl::Vector3 get_pos() const { return m_x.segment<3>(kPosIndex); }
    ugl::Vector3 get_vel() const { return m_x.segment<3>(kVelIndex); }
    ugl::lie::Rotation get_rot() const { return m_R_ref; }
    ugl::UnitQuaternion get_quat() const { return get_rot().to_quaternion(); }
    ugl::lie::ExtendedPose get_state() const;
    const Covariance<9>& get_covariance() const { return m_P; }

    void set_pos(const ugl::Vector3& pos) { m_x.segment<3>(kPosIndex) = pos; }
    void set_vel(const ugl::Vector3& vel) { m_x.segment<3>(kVelIndex) = vel; }
    void set_rot(const ugl::lie::Rotation& rot) { m_R_ref = rot; }
    void set_quat(const ugl::UnitQuaternion& quat) { m_R_ref = ugl::lie::Rotation{quat}; }
    void set_state(const ugl::lie::ExtendedPose& state);
    void set_covariance(const Covariance<9>& P) { m_P = P; }

    void predict(double dt, const ugl::Vector3& acc, const ugl::Vector3& ang_vel, const ImuModel& imu_model);

    /// @return NIS - Normalized Innovation Squared
    double update(const ugl::lie::Pose& y, const MocapModel& sensor_model);

    /// @return NIS - Normalized Innovation Squared
    double update(const ugl::lie::Euclidean<3>& y, const GpsModel& sensor_model);

private:
    void reset_attitude_error();

    static State state_transition_model(const State& x, const ugl::lie::Rotation& R_ref, double dt, const ugl::Vector3& acc, const ugl::Vector3& ang_vel);

    /// @brief Error jacobian of the process model
    static Jacobian<9,9> process_error_jacobian(const ugl::lie::Rotation& R_ref, const ugl::Vector3& acc, const ugl::Vector3& ang_vel);

private:
    State m_x = State::Zero();
    ugl::lie::Rotation m_R_ref = ugl::lie::Rotation::Identity();
    Covariance<9> m_P = Covariance<9>::Identity();

    static constexpr int kPosIndex = 6;
    static constexpr int kVelIndex = 3;
    static constexpr int kRotIndex = 0;

    static const ugl::Vector3 s_gravity;
};

} // namespace invariant

#endif // INVARIANT_MEKF_H
