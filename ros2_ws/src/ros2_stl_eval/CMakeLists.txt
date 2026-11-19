cmake_minimum_required(VERSION 3.8)
project(ros2_stl_eval)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

add_executable(generic_stl_monitor src/generic_stl_monitor.cpp)
ament_target_dependencies(generic_stl_monitor rclcpp std_msgs)

add_executable(event_listener src/event_listener.cpp)
ament_target_dependencies(event_listener rclcpp std_msgs)

add_executable(turtlesim_adapter src/turtlesim_adapter.cpp)
ament_target_dependencies(turtlesim_adapter rclcpp std_msgs turtlesim geometry_msgs)

add_executable(odom_adapter src/odom_adapter.cpp)
ament_target_dependencies(odom_adapter rclcpp std_msgs nav_msgs geometry_msgs)

# Link the external STLRom Engine using dynamic relative paths
target_include_directories(generic_stl_monitor SYSTEM PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/STLRom/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/STLRom/build"
)

target_link_directories(generic_stl_monitor PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/STLRom/build"
)

target_link_libraries(generic_stl_monitor stlromlib)
set_target_properties(generic_stl_monitor PROPERTIES INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/STLRom/build")

# Install rules
install(TARGETS 
  generic_stl_monitor
  turtlesim_adapter
  odom_adapter
  event_listener
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  config 
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
