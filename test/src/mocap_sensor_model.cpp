#include "mocap_sensor_model.h"

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>

namespace invariant::test
{

ugl::Vector3 MocapSensorModel::getPosReading(double t) const
{
    return trajectory_.get_position(t);
}

ugl::Rotation MocapSensorModel::getRotReading(double t) const
{
    return trajectory_.get_rotation(t);
}

}