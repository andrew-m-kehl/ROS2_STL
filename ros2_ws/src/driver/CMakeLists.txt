cmake_minimum_required(VERSION 3.8)
project(driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(point_driver src/point_driver.cpp)
ament_target_dependencies(point_driver rclcpp geometry_msgs turtlesim)

add_executable(odom_driver src/odom_driver.cpp)
ament_target_dependencies(odom_driver rclcpp geometry_msgs turtlesim)

add_executable(manual_driver src/manual_driver.cpp)
ament_target_dependencies(manual_driver rclcpp geometry_msgs)

install(TARGETS
  point_driver
  odom_driver
  manual_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()