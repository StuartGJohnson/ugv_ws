from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
import os


def generate_launch_description():

    # todo: on a real robot, we want this false - until
    # we get around to fixing proprioceptive odometry
    pub_odom_tf_arg = DeclareLaunchArgument(
        'pub_odom_tf', default_value='true',
        description='Whether to publish the tf from the original odom to the base_footprint'
    )

    parameters = {
        'frame_id': 'base_footprint',
        'use_sim_time': True,
        'subscribe_rgbd': False,
        'subscribe_rgb': True,
        'subscribe_depth': False,
        'subscribe_scan': False,
        'subscribe_scan_cloud': True,
        'use_action_for_goal': True,
        # RTAB-Map's parameters should be strings:
        'Reg/Strategy': '2',  # for cloud only
        # 'Reg/Strategy': '1',
        'RGBD/LinearUpdate': '0.10',
        'RGBD/AngularUpdate': '0.03',
        'Mem/STMSize': '0',
        'Reg/Force3DoF': 'true',
        'RGBD/NeighborLinkRefining': 'true',
        'Grid/FromDepth': 'false',
        # not real, apparently:
        # 'Grid/AlwaysUpdate': 'true',
        'GridGlobal/FullUpdate': 'true',
        'Grid/RayTracing': 'true',  # Fill empty space
        'Grid/3D': 'true',  # Use 3D occupancy
        # 'Grid/3D': 'false',  # Use 2D occupancy
        'Grid/RangeMax': '3',
        'Grid/NormalsSegmentation': 'false',  # Use passthrough filter to detect obstacles
        'Grid/Sensor': '0',  # scan_cloud
        # 'Grid/Sensor': '1',  # laser scan
        'Grid/MaxGroundHeight': '0.015',  # All points above 1.5 cm are obstacles
        'Grid/MaxGroundAngle': '10',  # All ground tilted more than 10 degrees is an obstacle
        'Grid/MaxObstacleHeight': '0.5',  # All points over 0.5 meter are ignored
        'Grid/MinPlaneMinInliers': '100',
        'Grid/RangeMin': '0.010',  # ignore laser scan points on the robot itself
        'Grid/CellSize': '.02',
        'Grid/Voxel': '.02',
        'Optimizer/GravitySigma': '0'  # Disable imu constraints (we are already in 2D)
    }

    remappings = [
        ('rgb/image', '/camera/image'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('depth/image', '/camera/depth'),
        ('scan_cloud', '/fusion/points'),
        ('map', 'rtabmap/map') ]
    # ('octomap_grid', '/map')]

    # nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': 'nav2_rtabmap_sim_localization_params.yaml',
            'use_sim_time': 'true'
        }.items()
    )

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
            os.path.join(get_package_share_directory('gazebo_differential_drive_robot_4wheel'), 'launch',
                         'ugv.launch.py')
        ),
        launch_arguments={
            'world': '/home/sjohnson/WorldGeneration/scene_stuffx3.sdf',
        }.items()
    )

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

    # map server and friends (this is localization, not SLAM!)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': "scene_stuffx3_ros2.yaml",
            'topic_name': 'map',
            'frame_id': 'map',
        }],
    )

    map_server_lifetime = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    # global localizer - this emits /initialpose
    glob_loc = Node(
        package='global_robot_localization',
        executable='global_robot_localization_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'occupied_threshold': 50,
            'unknown_is_occupied': False,
            'base_frame': "base_link",
            'num_scans': 1,
            'scan_collection_timeout_sec': 3.0,
            'transform_timeout_sec': 0.2,
            'publish_initialpose': True,
            'publish_markers': True,
            'coarse_xy_step': 0.1,
            'coarse_yaw_step_deg': 5.0,
            'top_k': 2,
            'candidate_min_xy_separation': 0.45,
            'candidate_min_yaw_separation_deg': 8.0,
            'refine_levels': 4,
            'refine_xy_step': 0.15,
            'refine_yaw_step_deg': 4.0,
            'max_range': 5.5,
            'scan_stride': 1,
            'off_map_distance': 3.0,
            'map_padding_xy': 1.0,
            'free_space_weight': 0.4,
            'free_space_sample_step': 0.2,
            'min_endpoint_count': 80,
            'covariance_regularization': 1.0e-2,
            'sigma_r': 0.03,
            'sigma_theta': 0.005,
            'alpha_m': 1.0
        }],
    )

    return LaunchDescription([

        # robot base nodes
        pub_odom_tf_arg,
        fuser_node,
        gazebo_launch,
        nav2_launch,
        pc_node,
        map_server,
        map_server_lifetime,
        glob_loc,
        # rf2o_laser_odometry_launch,

        # other nodes to launch
        # Localization mode:
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
                        {'Mem/IncrementalMemory': 'False',
                         'Mem/InitWMWithAllNodes': 'True'}],
            remappings=remappings),


    ])
