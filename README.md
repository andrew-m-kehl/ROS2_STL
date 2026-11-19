# Online STL Monitoring and Benchmarking in ROS 2

A modular ROS 2 architecture integrating the STLRom C++ engine to evaluate Signal Temporal Logic (STL) formulas in real-time. This repository contains launch files for turtlesim as well as the Create3 Robot

## File system archetecture:
```text
ros2_ws/
└── src/
    ├── driver/
    │   ├── include/
    │   ├── launch/
    │   │   ├── odom_driver.launch.py
    │   │   └── point_driver.launch.py
    │   ├── src/
    │   │   ├── odom_driver.cpp
    │   │   └── point_driver.cpp
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── ros2_stl_eval/
    │   ├── config/
    │   │   ├── stl_params.yaml
    │   │   └── formulas.stl
    │   ├── launch/
    │   │   ├── odom_eval.launch.py
    │   │   └── point_eval.launch.py
    │   ├── src/
    │   │   ├── odom_adapter.cpp
    │   │   ├── turtlesim_adapter.cpp 
    │   │   └── generic_stl_monitor.cpp
    │   ├── STLRom/
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── .gitignore
    └── README.md
```

```text

       iRobot Create 3                 Turtlesim
       nav_msgs/Odometry              turtlesim/Pose
              |                            |
              v                            v
       +--------------+             +------------------+
       | Odom Adapter |             | Turtlesim Adapter|
       +--------------+             +------------------+
              |                            |
              |                            |
              +------------+---------------+
                           |
                           v
                      /stl_input
                    Float64MultiArray
                           |
                           v
                +----------------------+   +----------------------+ 
                |  Generic Monitor     |   |    STL Evaluator     |
                |        (C++)         |   |       (C++)          |
                +----------------------+   +----------------------+
                           |
                           v
                     STLRom C++ API
                           |
                           v
                   Robustness Results
                           |
                           v
                   /stl_robustness

```
## 1. Build the STLRom Library
Navigate to the STLRom directory and compile the engine from source:
```bash
cd STLRom
mkdir build && cd build
cmake ..
make
```

## 2. Build the ROS 2 Workspace
Navigate to the workspace and build the ROS 2 packages:

```bash
cd ros2_ws
```

For a new install:

```bash
colcon build
```

or use this command if you are just adjusting the stl_params 

```bash
colcon build --symlink-install
```

then

```bash
source install/setup.bash
```

## 3. Launch the system.

Launch the evaluator and plotter:
In Terminal 1:

```bash
ros2 launch ros2_stl_eval turtlesim_eval.launch.py
```
Launch the driver:
In Terminal 2:
```bash
ros2 launch driver point_driver.launch.py
```

Launch the Monitor:
In Terminal 3:
```bash
ros2 run ros2_stl_eval stl_monitor --ros-args -p formula_file:="/home/Andy/Desktop/CodingFiles/ROS2_STL/ros2_ws/src/ros2_stl_eval/config/formulas.stl" -p formula_name:="in_bounds" -p signal_names:="x,y,v" -p window:=2.0
```

### 3.1a. Launch the evaluator.

In terminal 1:

```bash
ros2 run ros2_stl_eval stl_processor --ros-args -p waypoints_x:=[2.0, 8.0, 8.0, 2.0]"
                                                -p waypoints_y:="[2.0, 2.0, 8.0, 8.0]" 
                                                -p zone_radius:=0.5 
                                                -p input_topic:="/stl_input" 
                                                -p metrics_topic:="/stl_metrics" 
                                                -p signal_names:="['x', 'y', 'v']" 
                                                -p phi_names:="['phi_vel']" 
                                                -p phi_formulas:="['phi_vel := alw_[0.0, 2.0] (v[t] < 3.0)\n']" 
                                                -p phi_lookaheads:="[2.0]"
```

### 3.1b. Launch the plotter.
In terminal 2:

```bash
ros2 run ros2_stl_eval_plotter stl_plotter --ros-args -p metrics_topic:="/stl_metrics" 
                                                      -p phi_names:="['phi1', 'phi2', 'phi_vel']"
```

### 3.2a. Launch a turtle.

In terminal 3:

```bash
ros2 run turtlesim turtlesim_node
```

### 3.2b. Launch the driver program (point-based or odometry-based).

In terminal 4:

```bash
ros2 run driver point_driver --ros-args -p waypoints_x:="[2.0, 8.0, 8.0, 2.0]" 
                                        -p waypoints_y:="[2.0, 2.0, 8.0, 8.0]"
```
or
```bash
ros2 run driver odom_driver --ros-args -p waypoints_x:="[2.0, 8.0, 8.0, 2.0]" 
                                       -p waypoints_y:="[2.0, 2.0, 8.0, 8.0]"
```
### 3.2c. Launch the manual driver

## Using Launch Files

### Alternativly for 3.1a & 3.1b, use the Launch File for evaluator and plotter.
In terminal 1:
```bash
ros2 launch ros2_stl_eval turtlesim_eval.launch.py
```

### 5.ab. Alternativly for 4.2a and 4.2b, use the launch file for turtle and driver 
#### Point-based 

In terminal 2:
```bash
ros2 launch driver point_driver.launch.py
```
or
#### Odometry-based
In terminal 2:
```bash
ros2 launch driver odom_driver.launch.py
```

If the plotter file somehow gets lost:
```bash
chmod +x src/turtle_square_driver/src/stl_plotter.py
```