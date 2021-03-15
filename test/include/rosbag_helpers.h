#ifndef INVARIANT_TEST_ROSBAG_HELPERS_H
#define INVARIANT_TEST_ROSBAG_HELPERS_H

#include <vector>

#include "accuracy_test.h"
#include "sensor_event.h"

namespace invariant::test
{

void save_to_rosbag(const std::vector<Result>& iekf_results, const std::vector<Result>& mekf_results, const std::vector<SensorEvent>& events);

} // namespace invariant::test

#endif // INVARIANT_TEST_ROSBAG_HELPERS_H