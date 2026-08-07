cmake_minimum_required(VERSION 3.22.1)
project(robross_painter)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)

add_compile_options(-Wall -Wextra)

add_library(cartesian_failure_diagnostics STATIC
  src/cartesian_failure_diagnostics.cpp)
target_include_directories(cartesian_failure_diagnostics PUBLIC src)
ament_target_dependencies(cartesian_failure_diagnostics
  rclcpp
  moveit_msgs
  geometry_msgs
  shape_msgs
)
target_link_libraries(cartesian_failure_diagnostics jsoncpp_lib)

add_executable(painting_executor src/painting_executor.cpp)
ament_target_dependencies(painting_executor
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  trajectory_msgs
)
target_link_libraries(painting_executor
  cartesian_failure_diagnostics
  jsoncpp_lib)

add_executable(cartesian_path_probe src/cartesian_path_probe.cpp)
ament_target_dependencies(cartesian_path_probe
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
)
target_link_libraries(cartesian_path_probe
  cartesian_failure_diagnostics
  jsoncpp_lib)

add_executable(teach_nudge src/teach_nudge.cpp)
ament_target_dependencies(teach_nudge
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  geometry_msgs
  std_srvs
  tf2
)

install(TARGETS painting_executor cartesian_path_probe teach_nudge
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/teach_canvas.py scripts/teach_tcp.py
  scripts/extract_cartesian_probe_seeds.py
  scripts/analyze_tracking_bag.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cartesian_postprocess
    test/test_cartesian_postprocess.cpp)
  target_include_directories(test_cartesian_postprocess PRIVATE src)
  ament_target_dependencies(test_cartesian_postprocess trajectory_msgs)
  target_link_libraries(test_cartesian_postprocess Eigen3::Eigen)
  ament_add_gtest(test_endpoint_settling
    test/test_endpoint_settling.cpp)
  target_include_directories(test_endpoint_settling PRIVATE src)
  ament_add_gtest(test_cartesian_failure_diagnostics
    test/test_cartesian_failure_diagnostics.cpp)
  target_include_directories(test_cartesian_failure_diagnostics PRIVATE src)
  target_link_libraries(test_cartesian_failure_diagnostics
    cartesian_failure_diagnostics)
  ament_add_pytest_test(test_paint_launch test/test_paint_launch.py)
  ament_add_pytest_test(test_cartesian_probe_launch
    test/test_cartesian_probe_launch.py)
  ament_add_pytest_test(test_extract_cartesian_probe_seeds
    test/test_extract_cartesian_probe_seeds.py)
  ament_add_pytest_test(test_teach_canvas_math test/test_teach_canvas_math.py)
  ament_add_pytest_test(test_teach_tcp_math test/test_teach_tcp_math.py)
  ament_add_pytest_test(test_analyze_tracking_bag
    test/test_analyze_tracking_bag.py)
endif()

ament_package()
