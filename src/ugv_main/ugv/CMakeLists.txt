cmake_minimum_required(VERSION 3.8)
project(ugv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(GTest REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(UDEV REQUIRED libudev)

# UGV library
add_library(ugv_lib src/ekf1.cpp src/robot_tools.cpp src/base_controller.cpp src/ugv_bringup_node.cpp src/ugv_driver_estop_node.cpp src/ugv_node.cpp)
target_include_directories(ugv_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${UDEV_INCLUDE_DIRS})
ament_target_dependencies(ugv_lib rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros rosbag2_cpp)
target_link_libraries(ugv_lib Eigen3::Eigen ${UDEV_LIBRARIES})

# ugv executable
add_executable(ugv src/ugv_main.cpp)
ament_target_dependencies(ugv rclcpp) # rclcpp is the base dependency
target_link_libraries(ugv ugv_lib)


install(TARGETS
  ugv
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}
)

# why do I need an extra PROJECT_NAME here?
install(DIRECTORY test/data/
  DESTINATION share/${PROJECT_NAME}/test/data)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_executable(test_robot_functional test/test_robot_functional.cpp)
  target_link_libraries(test_robot_functional GTest::gtest ugv_lib)

  add_executable(test_robot_tools test/test_robot_tools.cpp)
  target_link_libraries(test_robot_tools GTest::gtest ugv_lib)
  
  add_executable(test_ekf test/test_ekf.cpp)
  target_link_libraries(test_ekf GTest::gtest ugv_lib)

  add_test(
    NAME robot_tools_test
    COMMAND test_robot_tools
  )
endif()

ament_package()
