from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
import os


def generate_launch_description():

    # nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': 'nav2_rtabmap_sim_no_rot_localization_params.yaml',
            'use_sim_time': 'true'
        }.items()
    )

    # ground truth republish
    gt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ground_truth_republish'), 'launch', 'ground_truth_republish.launch.py')
        ),
        launch_arguments={
            'params_file': 'ground_truth_republish_simx_params.yaml',
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
                         'ugv_gt.launch.py')
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

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        output='screen',
        parameters=[{
            'min_particles': 1000,
            'max_particles': 4000,
            'alpha1': 0.05,
            'alpha2': 0.02,
            'alpha3': 0.03,
            'alpha4': 0.05,
            'alpha5': 0.01,
            'update_min_d': 0.001,
            'update_min_a': 0.001,
            'laser_model_type': "likelihood_field",
            'max_beams': 180,
            'sigma_hit': 0.01,
            'z_hit': 0.9,
            'z_rand': 0.05,
            'z_short': 0.03,
            'z_max': 0.02,
            'laser_likelihood_max_dist': 1.0,
            'base_frame_id': 'base_footprint',
            'global_frame_id': 'map',
            'odom_frame_id': 'odom',
            'use_sim_time': True,
            'scan_topic': '/scan'
        }],
    )

    lifetime = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'smoother_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother']
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
        fuser_node,
        gazebo_launch,
        nav2_launch,
        pc_node,
        map_server,
        amcl,
        lifetime,
        glob_loc,
        gt_launch,
        # rf2o_laser_odometry_launch,
    ])
