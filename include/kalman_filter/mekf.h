#ifndef INVARIANT_MEKF_H
#define INVARIANT_MEKF_H

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>
#include <ugl/lie_group/rotation.h>
#include <ugl/lie_group/pose.h>
#include <ugl/lie_group/extended_pose.h>

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
    MEKF(const ugl::lie::Rotation& R0, const ugl::Vector3& p0, const ugl::Vector3& v0, const Covariance<9>& P0);

    ugl::Vector3 get_pos() const { return m_x.segment<3>(kPosIndex); }
    ugl::Vector3 get_vel() const { return m_x.segment<3>(kVelIndex); }
    ugl::lie::Rotation get_rot() const { return m_R_ref; }
    ugl::UnitQuaternion get_quat() const { return get_rot().to_quaternion(); }
    ugl::lie::ExtendedPose get_state() const;

    void set_pos(const ugl::Vector3& pos) { m_x.segment<3>(kPosIndex) = pos; }
    void set_vel(const ugl::Vector3& vel) { m_x.segment<3>(kVelIndex) = vel; }
    void set_rot(const ugl::lie::Rotation& rot) { m_R_ref = rot; }
    void set_quat(const ugl::UnitQuaternion& quat) { m_R_ref = ugl::lie::Rotation{quat}; }
    void set_state(const ugl::lie::ExtendedPose& state);

    void predict(double dt, const ugl::Vector3& acc, const ugl::Vector3& ang_vel);
    void mocap_update(const ugl::lie::Pose&);
    void gps_update(const ugl::Vector3& y);

private:
    void reset_attitude_error();

    static State state_transition_model(const State& x, const ugl::lie::Rotation& R_ref, double dt, const ugl::Vector3& acc, const ugl::Vector3& ang_vel);
    static Jacobian<9,9> state_transition_jac(const ugl::lie::Rotation& R_ref, double dt, const ugl::Vector3& acc, const ugl::Vector3& ang_vel);
    static Covariance<9> state_transition_var(double dt);

    /// @brief Error jacobian of the process model
    static Jacobian<9,9> process_error_jacobian(const ugl::lie::Rotation& R_ref, const ugl::Vector3& acc, const ugl::Vector3& ang_vel);

    /// @brief Noise jacobian of the process model
    static Jacobian<9,6> process_noise_jacobian();

    /// @brief Noise covariance of the process model
    static Covariance<6> process_noise_covariance();

private:
    State m_x = State::Zero();
    ugl::lie::Rotation m_R_ref = ugl::lie::Rotation::Identity();
    Covariance<9> m_P = s_default_covariance;

    static const ugl::Matrix<9,9> s_default_covariance;

    static constexpr int kPosIndex = 6;
    static constexpr int kVelIndex = 3;
    static constexpr int kRotIndex = 0;

    static const ugl::Vector3 s_gravity;
};

} // namespace invariant

#endif // INVARIANT_MEKF_H
