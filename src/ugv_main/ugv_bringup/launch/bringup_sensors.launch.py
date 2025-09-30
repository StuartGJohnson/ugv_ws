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
        'pub_odom_tf', default_value='false',
        description='Whether to publish the tf from the original odom to the base_footprint',
    )

    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_description'), 'launch', 'bringup_ugv.launch.py')
        )
    )

    # lidar launch
    # TODO: move the launch file to a ros2 package, so the launch works from anywhere
    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            'ld19.launch.py'
        )
    )

    # realsense launch
    realsense_launch = GroupAction(
        actions=[
            # these are handled by pointcloud_node and camera_sync
            # SetRemap(src='/camera/color/image_raw',dst='/camera/image'),
            # SetRemap(src='/camera/color/camera_info', dst='/camera/camera_info'),
            # SetRemap(src='/camera/aligned_depth_to_color/image_raw', dst='/camera/depth'),
            # SetRemap(src='/camera/depth/color/points', dst='/camera/points'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
                ),
                launch_arguments={
                    'pointcloud.enable': 'false',
                    'align_depth.enable': 'true',
                    'depth_module.depth_profile': '480x270x15',
                    'rgb_camera.color_profile': '424x240x15',
                    'camera_namespace': '/'
                }.items()
            )
        ],
    )

    # well, the realsense ros package has broken point cloud generation
    # actually, this node is faster, and the point cloud is thinned
    pointcloud_node = Node(
        package='rtabmap_util', executable='point_cloud_xyz', output='screen',
        parameters=[{'decimation': 1,
                     'max_depth': 1.5,
                     'voxel_size': 0.002}],
        remappings=[('depth/image', '/camera/aligned_depth_to_color/image_raw'),
                    ('depth/camera_info', '/camera/aligned_depth_to_color/camera_info'),
                    ('cloud', '/camera/cloud')]
    )

    camera_sync = Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        parameters=[{'approx_sync':False}],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw')
        ]
    )

    # Include laser odometry launch file
    # TODO: move the launch file to a ros2 package, so the launch works from anywhere
    rf2o_laser_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            'rf2o_laser_odometry.launch.py'
        )
    )

    bringup_node = Node(
        package='ugv_bringup',
        executable='ugv_bringup',
    )

    dyn_tf = Node(
        package='ugv_bringup',
        executable='dynamic_tf_publisher',
    )


    driver_node = Node(
        package='ugv_bringup',
        executable='ugv_driver_estop',
    )

    # Define the base node with parameters
    base_node = Node(
        package='ugv_base_node',
        executable='base_node_no_imu',
        # TODO: when the base node has odom, might bring this back
        # right now, laser odometry is doing this
        # parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}],
        parameters=[{'pub_odom_tf': False}],
    )


    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        # robot base nodes
        pub_odom_tf_arg,
        bringup_node,
        driver_node,
        dyn_tf,
        base_node,
        #fuser_node,
        robot_state_launch,
        realsense_launch,
        laser_bringup_launch,
        rf2o_laser_odometry_launch,
        pointcloud_node,
        camera_sync,

    ])
