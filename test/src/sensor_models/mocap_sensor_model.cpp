#include "mocap_sensor_model.h"

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>

namespace invariant::test
{

ugl::Vector3 MocapSensorModel::get_pos_reading(double t) const
{
    return trajectory_.get_position(t);
}

ugl::Rotation MocapSensorModel::get_rot_reading(double t) const
{
    return trajectory_.get_rotation(t);
}

}