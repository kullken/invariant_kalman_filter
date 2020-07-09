#include "imu_sensor_model.h"

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>

namespace invariant::test
{

const ugl::Vector3 ImuSensorModel::s_gravity{0.0, 0.0, -9.82};

ugl::Vector3 ImuSensorModel::get_accel_reading(double t) const
{
    ugl::Vector3 acc = trajectory_.get_acceleration(t);
    ugl::Rotation R_inv = trajectory_.get_rotation(t).inverse();
    return R_inv * (acc - s_gravity);
}

ugl::Vector3 ImuSensorModel::get_gyro_reading(double t) const
{
    ugl::Vector3 gyro = trajectory_.get_angular_velocity(t);
    ugl::Rotation R_inv = trajectory_.get_rotation(t).inverse();
    return R_inv * gyro;
}

}