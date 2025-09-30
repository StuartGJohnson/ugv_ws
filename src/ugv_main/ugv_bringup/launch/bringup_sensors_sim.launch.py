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
          'world': '/home/sjohnson/WorldGeneration/scene_cal.sdf',
        }.items()
    )


    # voxel grid downsample node
    pc_node = Node(
        package='point_cloud_tools',
        executable='voxel_grid_filter_node',
        parameters=[{
            'leaf_size': .002,
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
        pc_node,
        #rf2o_laser_odometry_launch,
    ])
