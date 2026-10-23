cmake_minimum_required(VERSION 3.8)
project(turtle_square_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(square_driver src/square_driver.cpp)
ament_target_dependencies(square_driver rclcpp geometry_msgs turtlesim std_msgs)

add_executable(manual_driver src/manual_driver.cpp)
ament_target_dependencies(manual_driver rclcpp geometry_msgs std_msgs)

add_executable(stl_processor src/stl_processor.cpp)
ament_target_dependencies(stl_processor rclcpp turtlesim std_msgs)

target_include_directories(stl_processor SYSTEM PUBLIC
  "/home/Andy/Desktop/CodingFiles/ROS2_STL/STLRom/include"
  "/home/Andy/Desktop/CodingFiles/ROS2_STL/STLRom/build"
)

target_link_directories(stl_processor PUBLIC
  "/home/Andy/Desktop/CodingFiles/ROS2_STL/STLRom/build"
)

target_link_libraries(stl_processor stlromlib)
set_target_properties(stl_processor PROPERTIES INSTALL_RPATH "/home/Andy/Desktop/CodingFiles/ROS2_STL/STLRom/build")

install(TARGETS
  square_driver
  manual_driver
  stl_processor
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/stl_plotter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})
  
ament_package()
