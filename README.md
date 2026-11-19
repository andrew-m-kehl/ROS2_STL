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
    │   │   └── stl_params.yaml
    │   ├── launch/
    │   │   ├── odom_eval.launch.py
    │   │   └── point_eval.launch.py
    │   ├── src/
    │   │   ├── turtlesim_adapter.cpp
    │   │   ├── odom_adapter.cpp
    │   │   ├── generic_stl_monitor.cpp
    │   │   └── event_listener.cpp
    │   ├── STLRom/
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── ros2_stl_eval_plotter/
    │   ├── resource/
    │   ├── ros2_stl_eval_plotter/
    │   │   ├── __pycache__/
    │   │   ├── __init__.py
    │   │   └── stl_plotter.py
    │   ├── test/
    │   ├── package.xml
    │   ├── setup.cfg
    │   └── setup.py
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
                +----------------------+ 
                |  Generic Monitor     |
                |        (C++)         |   
                +----------------------+   
                           |
                           v
                     STLRom C++ API
                           |
                           v
                   Robustness Results
                           |
                           v
                   /stl_robustness
                           |
                           v
                +----------------------+ 
                |    Event Listener    |
                |       (C++)          |
                +----------------------+
                           |
                           v
                +----------------------+ 
                |    STL Plotter       |
                |       (C++)          |
                +----------------------+


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

## 3. Launch the system using Launch files.

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



### 3.1a. Launch the system individually.

In terminal 1:
```bash
ros2 run ros2_stl_eval turtlesim_adapter
```

In terminal 2:
```bash
ros2 run ros2_stl_eval generic_stl_monitor --ros-args \
       --params-file ~/Desktop/CodingFiles/ROS2_STL/ros2_ws/src/ros2_stl_eval/config/stl_params.yaml \
       -p formula_file:=/home/Andy/Desktop/CodingFiles/ROS2_STL/ros2_ws/src/ros2_stl_eval/config/formulas.stl
```
In Terminal 3:
```bash
ros2 run ros2_stl_eval event_listener
```

In Terminal 4:
```bash
ros2 run ros2_stl_eval_plotter stl_plotter
```

In Terminal 5:
```bash
ros2 launch driver point_driver.launch.py
```




### If the plotter file somehow gets lost:
```bash
chmod +x src/turtle_square_driver/src/stl_plotter.py
```