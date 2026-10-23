# ROS 2 Temporal Logic (STL) Waypoint Driver

A modular ROS 2 architecture integrating the STLRom C++ engine to evaluate Signal Temporal Logic (STL) formulas in real-time against dynamic waypoints in TurtleSim.

## 1. Build the STLRom Library
Navigate to the STLRom directory and compile the engine from source:
` ``bash
cd STLRom
mkdir build && cd build
cmake ..
make
` ``

## 2. Update CMake Paths
The ROS 2 `CMakeLists.txt` requires absolute paths to link the compiled STLRom library. Before building the workspace, you must update the absolute paths in `ros2_ws/src/turtle_square_driver/CMakeLists.txt` to match your local clone directory for the following properties:
* `target_include_directories`
* `target_link_directories`
* `INSTALL_RPATH`

## 3. Build the ROS 2 Workspace
Navigate to the workspace and build the ROS 2 packages:
` ``bash
cd ros2_ws
colcon build
source install/setup.bash
` ``

## 4. Launch the System
Execute the multi-node launch file to start the driver, STLRom processor, manual override terminal, and real-time Python plotter:
` ``bash
ros2 launch turtle_square_driver square_driver.launch.py
` ``