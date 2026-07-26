
# ugv_ws

This repo includes a rewrite of certain components of <a href="https://github.com/waveshareteam/ugv_ws">Waveshare ugv_ws</a>, in particular the source code in ```src/ugv_main/ugv```. This repo contains instructions for installation and execution of various components. The repo is currently heavily in flux. A running explanation of projects using this repo (which is also a ROS2 workspace) is <a href="https://stuartgjohnson.github.io/ugv_ws">UGV02xProjects</a> 

## Project Indoor Explorer

### Installation

ROS2 packages are needed both from git and from apt install.

You will need - or may already have (via sudo apt install):
- ros-humble-rtabmap
- ros-humble-navigation2

For the robot, you should have (in ugv_ws/src):

- [frontier_explorer](https://github.com/StuartGJohnson/frontier_explorer.git)
- [ldlidar_stl_ros2](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git)
- [rf2o_laser_odometry](https://github.com/MAPIRlab/rf2o_laser_odometry.git)
- [safety](https://github.com/StuartGJohnson/safety.git)

For the manager/simulation node (these should really be split - I'll get to it):
- [frontier_explorer](https://github.com/StuartGJohnson/frontier_explorer.git)
- [point_cloud_tools](https://github.com/StuartGJohnson/point_cloud_tools.git)
- [gazebo_differential_drive_robot_4wheel](https://github.com/StuartGJohnson/gazebo_differential_drive_robot_4wheel.git)
- [rviz_record](https://github.com/StuartGJohnson/rviz_record.git)

You will also need to install Gazebo Garden and rviz2.

### Build - simulation/manager computer

```colcon build --packages-select ugv ugv_description differential_drive_test ldlidar_stl_ros2 rf2o_laser_odometry safety gazebo_differential_drive_robot_4wheel point_cloud_tools frontier_explorer rviz_record --cmake-args -DCMAKE_BUILD_TYPE=Release```

Note: this list will be pruned.

```source install/setup.bash```

### Build - robot computer

```colcon build --packages-select ugv ugv_description ldlidar_stl_ros2 rf2o_laser_odometry safety point_cloud_tools frontier_explorer --cmake-args -DCMAKE_BUILD_TYPE=Release```

```source install/setup.bash```

### bringup and operation : mobile mode

In all cases, run from this repository.

For this project, rtabmap is used as the SLAM component. On the robot:

```ros2 launch ugv_bringup bringup_rtabmap.launch.py```

```ros2 run frontier_explorer frontier_explorer_node```

On the manager computer:

```rviz2 -d rviz_frontier_explorer.rviz```

```python3 exploration_orchestrator.py --config exploration_orchestrator.yml```

This will write results to a directory called

```explore_YYYYMMDD_hhmmss```

### bringup and operation : simulation mode

In all cases, run from this repository.

For this project, rtabmap is used as the SLAM component. On the manager computer:

```ros2 launch ugv_bringup bringup_rtabmap_sim.launch.py```

```ros2 run frontier_explorer frontier_explorer_node```

```rviz2 -d rviz_frontier_explorer.rviz```

```python3 exploration_orchestrator.py --config exploration_orchestrator_sim.yml```

This will write results to a directory called

```explore_YYYYMMDD_hhmmss```

### misc tools usage

Several useful tools are included to help post-process various files into nice plots.

<code>bag_to_map.py</code> converts occupancy grids in a bag snapshot to png files. See:

```python3 bag_to_map.py --help```

<code>bag_to_map.py</code> converts PointCloud2 items in a bag snapshot to pcd files. See:

```python3 bag_to_pcd.py --help```

<code>map_convert.py</code> puts nice axes on your occupancy grids and outputs to png. See:

```python3 map_convert.py --help```

## Credits

Code herein benefited from the extensive contributions of GPT4o and GPT5 (OpenAI, May-September 2025).

