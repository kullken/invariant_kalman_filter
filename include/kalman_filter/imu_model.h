#ifndef INVARIANT_IMU_MODEL_H
#define INVARIANT_IMU_MODEL_H

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>

namespace invariant
{

class ImuModel
{
public:
    explicit ImuModel(const ugl::Matrix<6,6>& noise_covariance);

    /// @brief Error jacobian of system dynamics.
    ugl::Matrix<9,9> error_jacobian(const ugl::Vector3& acc, const ugl::Vector3& ang_vel) const;

    /// @brief Q_hat = D*Q*D^T
    const ugl::Matrix<9,9>& modified_noise_covariance() const;

private:
    ugl::Matrix<9,9> m_modified_noise_covariance;
};

} // namespace invariant

#endif // INVARIANT_IMU_MODEL_H
