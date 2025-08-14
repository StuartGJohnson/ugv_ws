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
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')

    pub_odom_tf_arg = DeclareLaunchArgument(
        'pub_odom_tf', default_value='false',
        description='Whether to publish the tf from the original odom to the base_footprint'
    )

    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'subscribe_rgbd':True,
          'subscribe_scan':True,
          'use_action_for_goal':True,
          # RTAB-Map's parameters should be strings:
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'RGBD/NeighborLinkRefining':'true',
          'Grid/FromDepth': 'true',
          'GridGlobal/FullUpdate': 'true',
          'Grid/RayTracing':'true', # Fill empty space
          'Grid/3D':'true', # Use 2D occupancy
          'Grid/RangeMax':'3',
          'Grid/NormalsSegmentation':'true', # Use passthrough filter to detect obstacles
          'Grid/Sensor':'2', # Use both laser scan and camera for obstacle detection in global map
          'Grid/MaxGroundHeight':'0.015', # All points above 5 cm are obstacles
          'Grid/MaxGroundAngle': '10',  # All points above 5 cm are obstacles
          'Grid/MaxObstacleHeight':'0.5',  # All points over 1 meter are ignored
          'Grid/MinPlaneMinInliers': 100,
          'Grid/RangeMin':'0.010', # ignore laser scan points on the robot itself
          'Grid/CellSize':'.02',
          'Grid/Voxel': '.02',
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }

    remappings=[
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/aligned_depth_to_color/image_raw')]

    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_description'), 'launch', 'bringup.launch.py')
        )
    )

    # lidar launch
    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ldlidar_stl_ros2'), 'launch', 'ld19.launch.py')
        )
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
        }.items()
    )

    # nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': 'nav2_rtabmap_params.yaml',
        }.items()
    )

    # Include laser odometry launch file
    rf2o_laser_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rf2o_laser_odometry'), 'launch', 'rf2o_laser_odometry.launch.py')
        )
    )

    bringup_node = Node(
        package='ugv_bringup',
        executable='ugv_bringup',
    )

    dyn_tf = Node(
        package='ugv_bringup',
        executable='dynamic_tf_publisher'
    )


    driver_node = Node(
        package='ugv_bringup',
        executable='ugv_driver_estop',
    )

    # Define the base node with parameters
    base_node = Node(
        package='ugv_base_node',
        executable='base_node_no_imu',
        parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}]
    )

    return LaunchDescription([

        # robot base nodes
        pub_odom_tf_arg,
        bringup_node,
        driver_node,
        dyn_tf,
        base_node,
        robot_state_launch,
        realsense_launch,
        laser_bringup_launch,
        rf2o_laser_odometry_launch,
        nav2_launch,

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        # Nodes to launch

        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
            remappings=remappings),

        # SLAM Mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),

        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=[parameters],
        #     remappings=remappings),
        
        # Obstacle detection with the camera for nav2 local costmap:
        # Node(
        #     package='rtabmap_util', executable='obstacles_detection', output='screen',
        #     parameters=[parameters],
        #     remappings=[('cloud', '/camera/depth/color/points'),
        #                 ('obstacles', '/camera/obstacles'),
        #                 ('ground', '/camera/ground')]),

        # Obstacle detection with the camera for nav2 local costmap.
        # First, we need to convert depth image to a point cloud.
        # Second, we segment the floor from the obstacles.
        Node(
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            parameters=[{'decimation': 2,
                         'max_depth': 3.0,
                         'voxel_size': 0.02}],
            remappings=[('depth/image', '/camera/aligned_depth_to_color/image_raw'),
                        ('depth/camera_info', '/camera/aligned_depth_to_color/camera_info'),
                        ('cloud', '/camera/cloud')]),
        Node(
            package='rtabmap_util', executable='obstacles_detection', output='screen',
            parameters=[parameters],
            remappings=[('cloud', '/camera/cloud'),
                        ('obstacles', '/camera/obstacles'),
                        ('ground', '/camera/ground')]),
    ])
