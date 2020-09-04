cmake_minimum_required(VERSION 3.10.2)
project(invariant_kalman_filter)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_ros
    ugl_ros
)

find_package(ugl REQUIRED)

add_library(project_options INTERFACE)
target_compile_features(project_options INTERFACE cxx_std_17)

add_library(project_warnings INTERFACE)
target_compile_options(project_warnings
  INTERFACE
    -Wall -Wextra -Wpedantic
    -Wnon-virtual-dtor
    -Wcast-align
    -Wunused
    -Woverloaded-virtual
    -Wnull-dereference
    -Wmisleading-indentation
    -Wno-deprecated-copy
)

include(cmake/StaticAnalyzers.cmake)
include(cmake/Sanitizers.cmake)

enable_sanitizers(project_options)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
#  LIBRARIES localisation
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## IEKF ##
add_library(iekf SHARED
  src/iekf.cpp
)

target_link_libraries(iekf
  PUBLIC
    ugl::math
    ugl::lie_group
  PRIVATE
    project_options
    project_warnings
)

## MEKF ##
add_library(mekf SHARED
  src/mekf.cpp
)

target_link_libraries(mekf
  PUBLIC
    ugl::math
    ugl::lie_group
  PRIVATE
    project_options
    project_warnings
)

## Kalman ROS-node ##
add_executable(kalman_node
  src/kalman_node.cpp
  src/measurement.cpp
)

target_link_libraries(kalman_node
  PUBLIC
    iekf
    ${catkin_LIBRARIES}
  PRIVATE
    ugl::math
    ugl::lie_group
    project_options
    project_warnings
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#######################
## Testing Libraries ##
#######################

## Sensor Models
add_library(sensor_models SHARED
  test/src/sensor_models/imu_sensor_model.cpp
  test/src/sensor_models/mocap_sensor_model.cpp
)

target_include_directories(sensor_models
  PUBLIC
    test/include/sensor_models
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

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  catkin_add_executable_with_gtest(create_csv_data test/src/create_csv_data.cpp)

  target_include_directories(create_csv_data
    PRIVATE
      test/include
  )

  target_sources(create_csv_data
    PRIVATE
      test/src/accuracy_test.cpp
      test/src/accuracy_test_config.cpp
      test/src/test_trajectories.cpp
  )

  target_link_libraries(create_csv_data
      project_options
      project_warnings
      iekf
      mekf
      sensor_models
      ugl::math
      ugl::trajectory
  )

  catkin_add_executable_with_gtest(regression_tests test/src/regression_tests.cpp)

  target_include_directories(regression_tests
    PRIVATE
      test/include
  )

  target_sources(regression_tests
    PRIVATE
      test/src/accuracy_test.cpp
      test/src/accuracy_test_config.cpp
      test/src/test_trajectories.cpp
  )

  target_link_libraries(regression_tests
      project_options
      project_warnings
      iekf
      mekf
      sensor_models
      ugl::math
      ugl::trajectory
  )
endif()
