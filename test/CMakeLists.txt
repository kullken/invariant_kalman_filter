#######################
## Testing Libraries ##
#######################

## Sensor Models
add_library(test_sensor_models SHARED)
add_library(invariant::test_sensor_models ALIAS test_sensor_models)
target_sources(test_sensor_models
  PRIVATE
    src/sensor_models/virtual_sensor.cpp
    src/sensor_models/sensor_event.cpp
    src/sensor_models/imu_sensor_model.cpp
    src/sensor_models/mocap_sensor_model.cpp
    src/sensor_models/gps_sensor_model.cpp
)
target_include_directories(test_sensor_models
  PUBLIC
    include/sensor_models
)
target_link_libraries(test_sensor_models
  PUBLIC
    ugl::math
    ugl::trajectory
    ugl::lie_group
    ugl::random
    invariant::sensor_models
  PRIVATE
    project_options
    project_warnings
)

## Accuracy Test
add_library(accuracy_test SHARED)
add_library(invariant::accuracy_test ALIAS accuracy_test)
target_sources(accuracy_test
  PRIVATE
    src/accuracy_test.cpp
    src/csv_helpers.cpp
    src/offset_generator.cpp
    src/rosbag_helpers.cpp
    src/test_trajectories.cpp
)
target_include_directories(accuracy_test
  PUBLIC
    include
)
target_link_libraries(accuracy_test
  PUBLIC
    ugl::math
    ugl::trajectory
    ugl::lie_group
    ugl::random
    invariant::test_sensor_models
    ${catkin_LIBRARIES}
  PRIVATE
    project_options
    project_warnings
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  catkin_add_executable_with_gtest(generate_data src/generate_data.cpp)
  target_include_directories(generate_data
    PRIVATE
      include
  )
  target_link_libraries(generate_data
      invariant::kalman_filter
      invariant::accuracy_test
      invariant::test_sensor_models
      project_options
      project_warnings
  )

  catkin_add_executable_with_gtest(regression_tests src/regression_tests.cpp)
  target_include_directories(regression_tests
    PRIVATE
      include
  )
  target_link_libraries(regression_tests
      invariant::kalman_filter
      invariant::accuracy_test
      invariant::test_sensor_models
      project_options
      project_warnings
  )
endif()
