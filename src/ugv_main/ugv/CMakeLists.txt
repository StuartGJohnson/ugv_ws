cmake_minimum_required(VERSION 3.8)
project(ugv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(GTest REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(UDEV REQUIRED libudev)

# UGV library
add_library(ugv_lib src/robot_tools.cpp src/base_controller.cpp src/ugv_bringup_node.cpp src/ugv_driver_estop_node.cpp)
target_include_directories(ugv_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${UDEV_INCLUDE_DIRS})
ament_target_dependencies(ugv_lib rclcpp std_msgs geometry_msgs sensor_msgs)
target_link_libraries(ugv_lib ${UDEV_LIBRARIES})


# ugv_bringup executable
add_executable(ugv_bringup src/ugv_bringup_main.cpp)
ament_target_dependencies(ugv_bringup rclcpp std_msgs sensor_msgs)
target_link_libraries(ugv_bringup ugv_lib)

# ugv_driver_estop executable
add_executable(ugv_driver_estop src/ugv_driver_estop_main.cpp)
ament_target_dependencies(ugv_driver_estop rclcpp std_msgs geometry_msgs sensor_msgs)
target_link_libraries(ugv_driver_estop ugv_lib)


install(TARGETS
  ugv_bringup
  ugv_driver_estop
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_executable(test_robot_functional test/test_robot_functional.cpp)
  target_link_libraries(test_robot_functional GTest::gtest ugv_lib)

  add_executable(test_robot_tools test/test_robot_tools.cpp)
  target_link_libraries(test_robot_tools GTest::gtest ugv_lib)

  add_test(
    NAME robot_tools_test
    COMMAND test_robot_tools
  )
endif()

ament_package()
