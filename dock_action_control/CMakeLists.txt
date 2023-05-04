cmake_minimum_required(VERSION 3.8)
project(dock_action_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
    #${PROJECT_SOURCE_DIR}/launch
    )
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depthai REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(dock_action_control src/dock_action_control.cpp)
ament_target_dependencies(dock_action_control geometry_msgs control_toolbox rclcpp rclcpp_action rclcpp_components action_interfaces sensor_msgs irobot_create_msgs cv_bridge depthai OpenCV)


include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries(dock_action_control ${OpenCV_LIBS} )

target_include_directories(dock_action_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(dock_action_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS dock_action_control
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  dock_action_control
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
#install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}/
#)

ament_package()
