import os
import xacro
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition

# Function to set up and launch ROS 2 nodes based on the given context
def launch_setup(context, *args, **kwargs):
  
    # Set paths to Xacro model and configuration files
    robot_model_path = os.path.join(
            get_package_share_directory('ugv_description'),
            'xacro',
            'robot.xacro'
        )
    # Process the Xacro file to generate the URDF representation of the robot
    robot_description = xacro.process_file(robot_model_path).toxml()
    
    # Define the robot_state_publisher node to publish the robot's URDF model
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        #namespace='ugv',
        parameters=[
            {'robot_description': robot_description, 'use_tf_static': False, 'use_sim_time': False}
            ],
    )
    
    # Define the robot_state_publisher node to publish the robot's URDF model
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        #namespace='ugv',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': False}
            ],
    )

    # Return a list of nodes to launch
    return [
        robot_state_publisher_node,
        joint_state_publisher_node
    ]

# Function to generate the launch description with configurable arguments
def generate_launch_description():
    return LaunchDescription([
        # Opaque function to execute the setup
        OpaqueFunction(function=launch_setup)
    ])

# Main entry point for launching the description
if __name__ == '__main__':
    generate_launch_description()
