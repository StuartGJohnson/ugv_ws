# Example:
#
#   Bringup turtlebot3:
#     $ export TURTLEBOT3_MODEL=waffle
#     $ export LDS_MODEL=LDS-01
#     $ ros2 launch turtlebot3_bringup robot.launch.py
#
#   SLAM:
#     $ ros2 launch rtabmap_demos turtlebot3_rgbd_scan.launch.py
#
#   Navigation (install nav2_bringup package):
#     $ ros2 launch nav2_bringup navigation_launch.py
#     $ ros2 launch nav2_bringup rviz_launch.py
#
#   Teleop:
#     $ ros2 run turtlebot3_teleop teleop_keyboard

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
import os

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')

    # todo: on a real robot, we want this false - until
    # we get around to fixing proprioceptive odometry
    pub_odom_tf_arg = DeclareLaunchArgument(
        'pub_odom_tf', default_value='true',
        description='Whether to publish the tf from the original odom to the base_footprint'
    )

    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_description'), 'launch', 'bringup.launch.py')
        ),
        condition=UnlessCondition(use_sim_time)
    )

    # lidar launch
    # TODO: move the launch file to a ros2 package, so the launch works from anywhere
    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ldlidar_stl_ros2'), 'launch', 'ld19.launch.py')
        ),
        condition=UnlessCondition(use_sim_time)
    )

    # realsense launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'pointcloud.enable': 'false',
            'align_depth.enable': 'true',
            'depth_module.depth_profile': '480x270x6',
            'rgb_camera.color_profile': '424x240x6',
            'camera_namespace': '/',
        }.items(),
        condition=UnlessCondition(use_sim_time)
    )

    # nav2 launch
    nav2_launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': 'nav2_slam_sim_params.yaml',
            'use_sim_time': 'true'
        }.items(),
        condition=IfCondition(use_sim_time)
    )

    slam_launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': 'mapper_params_online_async_sim.yaml',
            'use_sim_time': 'true'
        }.items(),
        condition=IfCondition(use_sim_time)
    )

    # Include laser odometry launch file
    # TODO: move the launch file to a ros2 package, so the launch works from anywhere
    rf2o_laser_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            'rf2o_laser_odometry.launch.py'
        ),
        condition=UnlessCondition(use_sim_time)
    )

    bringup_node = Node(
        package='ugv_bringup',
        executable='ugv_bringup',
        condition=UnlessCondition(use_sim_time)
    )

    dyn_tf = Node(
        package='ugv_bringup',
        executable='dynamic_tf_publisher',
        condition=UnlessCondition(use_sim_time)
    )


    driver_node = Node(
        package='ugv_bringup',
        executable='ugv_driver_estop',
        condition=UnlessCondition(use_sim_time)
    )

    # Define the base node with parameters
    base_node = Node(
        package='ugv_base_node',
        executable='base_node_no_imu',
        parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}],
        condition=UnlessCondition(use_sim_time)
    )


    # simulated robot launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          os.path.join(get_package_share_directory('gazebo_differential_drive_robot_4wheel'), 'launch', 'ugv.launch.py')
        ),
        launch_arguments={
          'world': '/home/sjohnson/WorldGeneration/scene_stuff2.sdf',
        }.items(),
        condition=IfCondition(use_sim_time)
    )

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        # robot base nodes
        pub_odom_tf_arg,
        bringup_node,
        driver_node,
        dyn_tf,
        base_node,
        gazebo_launch,
        robot_state_launch,
        realsense_launch,
        laser_bringup_launch,
        rf2o_laser_odometry_launch,
        nav2_launch_sim,
        slam_launch_sim,

        # Nodes to launch

    ])
