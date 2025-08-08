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

    # Include the robot state launch from the ugv_description package
    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_description'), 'launch', 'bringup.launch.py')
        )
    )

    # Define the nodes to be launched
    # imu_complementary_filter_node = Node(
    #         package='imu_complementary_filter',
    #         executable='complementary_filter_node',
    #         name='complementary_filter_gain_node',
    #         output='screen',
    #         parameters=[
    #             {'do_bias_estimation': True},
    #             {'do_adaptive_gain': True},
    #             {'use_mag': False},
    #             {'gain_acc': 0.01},
    #             {'gain_mag': 0.01},
    #         ]
    # )

    # Include laser odometry launch file
    rf2o_laser_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rf2o_laser_odometry'), 'launch', 'rf2o_laser_odometry.launch.py')
        )
    )

    # Define the nodes to be launched
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

    # Include laser lidar launch file
    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ldlidar_stl_ros2'), 'launch', 'ld19.launch.py')
        )
    )

    # Define the base node with parameters
    base_node = Node(
        package='ugv_base_node',
        executable='base_node_no_imu',
        parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}]
    )

    # Return the launch description with all defined actions
    return LaunchDescription([
        pub_odom_tf_arg,
        laser_bringup_launch,
        robot_state_launch,
        bringup_node,
        dyn_tf,
        driver_node,
        base_node,
        rf2o_laser_odometry_launch
        #imu_complementary_filter_node
    ])
