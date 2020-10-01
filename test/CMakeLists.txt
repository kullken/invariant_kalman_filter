#######################
## Testing Libraries ##
#######################

## Sensor Models
add_library(sensor_models SHARED
  src/sensor_models/sensor_event.cpp
  src/sensor_models/imu_sensor_model.cpp
  src/sensor_models/mocap_sensor_model.cpp
)

target_include_directories(sensor_models
  PUBLIC
    include/sensor_models
)

target_link_libraries(sensor_models
  PUBLIC
    ugl::math
    ugl::trajectory
    ugl::lie_group
    ugl::random
  PRIVATE
    project_options
    project_warnings
)

## Accuracy Test
add_library(accuracy_test SHARED
  src/accuracy_test.cpp
  src/accuracy_test_config.cpp
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
    ugl::random
    iekf
    mekf
    sensor_models
  PRIVATE
    ugl::lie_group
    ${catkin_LIBRARIES}
    project_options
    project_warnings
)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  catkin_add_executable_with_gtest(create_csv_data src/create_csv_data.cpp)
  target_include_directories(create_csv_data
    PRIVATE
      include
  )
  target_link_libraries(create_csv_data
      accuracy_test
      project_options
      project_warnings
  )

  catkin_add_executable_with_gtest(regression_tests src/regression_tests.cpp)
  target_include_directories(regression_tests
    PRIVATE
      include
  )
  target_link_libraries(regression_tests
      accuracy_test
      project_options
      project_warnings
  )

  catkin_add_executable_with_gtest(ros_trajectory_test src/ros_trajectory_test.cpp)
  target_include_directories(ros_trajectory_test
    PRIVATE
      include
  )
  target_sources(ros_trajectory_test
    PRIVATE
      src/mock_trajectory_node.cpp
  )
  target_link_libraries(ros_trajectory_test
      accuracy_test
      ${catkin_LIBRARIES}
      project_options
      project_warnings
  )
endif()
