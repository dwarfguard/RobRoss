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
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(jsoncpp REQUIRED)

add_compile_options(-Wall -Wextra)

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
)
target_link_libraries(painting_executor jsoncpp_lib)

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

install(TARGETS painting_executor teach_nudge
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/teach_canvas.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_paint_launch test/test_paint_launch.py)
  ament_add_pytest_test(test_teach_canvas_math test/test_teach_canvas_math.py)
endif()

ament_package()
