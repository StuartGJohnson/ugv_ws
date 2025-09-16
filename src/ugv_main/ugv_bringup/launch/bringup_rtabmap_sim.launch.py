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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
import os

def generate_launch_description():

    localization = LaunchConfiguration('localization')

    # todo: on a real robot, we want this false - until
    # we get around to fixing proprioceptive odometry
    pub_odom_tf_arg = DeclareLaunchArgument(
        'pub_odom_tf', default_value='true',
        description='Whether to publish the tf from the original odom to the base_footprint'
    )

    parameters={
          'frame_id':'base_footprint',
          'use_sim_time': True,
          'subscribe_rgbd': False,
          'subscribe_rgb': True,
          'subscribe_depth': False,
          'subscribe_scan': False,
          'subscribe_scan_cloud':True,
          'use_action_for_goal':True,
          # RTAB-Map's parameters should be strings:
          'Reg/Strategy':'2', # for cloud only
          #'Reg/Strategy': '1',
          'RGBD/LinearUpdate' : '0.10',
          'RGBD/AngularUpdate' : '0.03',
          'Mem/STMSize':'0',
          'Reg/Force3DoF':'true',
          'RGBD/NeighborLinkRefining':'true',
          'Grid/FromDepth': 'false',
          # not real, apparently:
          #'Grid/AlwaysUpdate': 'true',
          'GridGlobal/FullUpdate': 'true',
          'Grid/RayTracing':'true', # Fill empty space
          'Grid/3D':'true', # Use 3D occupancy
          #'Grid/3D': 'false',  # Use 2D occupancy
          'Grid/RangeMax':'3',
          'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
          'Grid/Sensor':'0', # scan_cloud
          #'Grid/Sensor': '1',  # laser scan
          'Grid/MaxGroundHeight':'0.015', # All points above 1.5 cm are obstacles
          'Grid/MaxGroundAngle': '10',  # All ground tilted more than 10 degrees is an obstacle
          'Grid/MaxObstacleHeight':'0.5',  # All points over 0.5 meter are ignored
          'Grid/MinPlaneMinInliers': '100',
          'Grid/RangeMin':'0.010', # ignore laser scan points on the robot itself
          'Grid/CellSize':'.02',
          'Grid/Voxel': '.02',
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }

    remappings=[
          ('rgb/image', '/camera/image'),
          ('rgb/camera_info', '/camera/camera_info'),
          ('depth/image', '/camera/depth'),
          ('scan_cloud', '/fusion/points')]
          #('octomap_grid', '/map')]

    # nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': 'nav2_rtabmap_sim_params.yaml',
            'use_sim_time': 'true'
        }.items()
    )

    # Include laser odometry launch file
    # this generates a lot of laser scan messages and
    # causes jerky placement of the laser scans in the map
    # rf2o_laser_odometry_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         'rf2o_laser_odometry_sim.launch.py'
    #     )
    # )

    # point cloud/scan fusion
    fuser_node = Node(
        package='point_cloud_tools',
        executable='cloud_scan_fuser_node',
        parameters=[
            {'cloud_topic': '/camera/cloud',
             'cloud_stride_u': 1,
             'cloud_stride_v': 1
             }
        ]
    )

    # simulated robot launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
          os.path.join(get_package_share_directory('gazebo_differential_drive_robot_4wheel'), 'launch', 'ugv.launch.py')
        ),
        launch_arguments={
          'world': '/home/sjohnson/WorldGeneration/scene_stuffx.sdf',
        }.items()
    )

    # register_node = Node(
    #     package='depth_image_proc',
    #     executable='register_node',
    #     remappings=[('depth/image_rect', '/camera/depth/image_raw'),
    #                 ('depth/camera_info', '/camera/depth/camera_info'),
    #                 ('rgb/camera_info', '/camera/color/camera_info'),
    #                 ('depth_registered/image_rect', '/camera/aligned_depth_to_color/image_raw'),
    #                 ('depth_registered/camera_info', '/camera/aligned_depth_to_color/camera_info')
    #                 ]
    # )

    # # voxel grid downsample filter launch (autoware)
    # downsample_launch = IncludeLaunchDescription(
    #     FrontendLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('autoware_downsample_filters'), 'launch', 'voxel_grid_downsample_filter_node.launch.xml')
    #     ),
    #     launch_arguments={
    #         'input_topic_name': '/camera_points',
    #         'output_topic_name': "/cloud",
    #         'input_frame': "camera_link",
    #         'output_frame': "camera_link",
    #         'voxel_grid_downsample_filter_param_file': "voxel_grid_downsample_filter_node.param.yaml",
    #     }.items()
    # )

    # voxel grid downsample node
    pc_node = Node(
        package='point_cloud_tools',
        executable='voxel_grid_filter_node',
        parameters=[{
            'leaf_size': .02,
            'input_topic': '/camera/points',
            'output_topic': '/camera/cloud'
        }]
    )

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        # robot base nodes
        pub_odom_tf_arg,
        fuser_node,
        gazebo_launch,
        nav2_launch,
        pc_node,
        #rf2o_laser_odometry_launch,

        # Nodes to launch

        # Node(
        #     package='rtabmap_sync', executable='rgbd_sync', output='screen',
        #     parameters=[{'approx_sync':False, 'use_sim_time':use_sim_time}],
        #     remappings=remappings),

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
        # Node(
        #     package='rtabmap_util', executable='point_cloud_xyz', output='screen',
        #     parameters=[{'decimation': 2,
        #                  'max_depth': 3.0,
        #                  'voxel_size': 0.02}],
        #     remappings=[('depth/image', '/camera/depth_image'),
        #                 ('depth/camera_info', '/camera/camera_info'),
        #                 ('cloud', '/camera/cloud')]),

        # Node(
        #     package='rtabmap_util', executable='obstacles_detection', output='screen',
        #     parameters=[parameters],
        #     remappings=[('cloud', '/camera/cloud'),
        #                 ('obstacles', '/camera/obstacles'),
        #                 ('ground', '/camera/ground')]),

        # Node(
        #     package='rtabmap_util', executable='obstacles_detection', output='screen',
        #     parameters=[parameters],
        #     remappings=[('cloud', '/camera/cloud'),
        #                 ('obstacles', '/camera/obstacles'),
        #                 ('ground', '/camera/ground')]),
    ])
