import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    pub_odom_tf_arg = DeclareLaunchArgument(
        'pub_odom_tf', default_value='false',
        description='Whether to publish the tf from the original odom to the base_footprint'
    )


    # Define the nodes to be launched
    bringup_node = Node(
        package='ugv',
        executable='ugv',
    )


    # Return the launch description with all defined actions
    return LaunchDescription([
        bringup_node,
    ])
