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

    # /scan to /scan_cloud
    scan_to_pc_node = Node(
        package='point_cloud_tools',
        executable='scan_to_point_cloud',
        parameters=[
            {'scan_topic': '/scan',
             'output_topic': '/scan_cloud',
             'min_range': 0.05,
             'max_range': 0.6
             }
        ]
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
                    'camera_namespace': '/',
                    'publish_tf': 'true'
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
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/odom_rf2o',
            'publish_tf' : False,
            'base_frame_id' : 'base_footprint',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 10.0}],
    )

    ugv_node = Node(
        package='ugv',
        executable='ugv',
        parameters=[{'pub_odom_tf': True}]
    )

    # TODO: this is annoying, try to get rid of it
    dyn_tf = Node(
        package='ugv',
        executable='static_tf_to_dynamic_publisher',
    )


    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),
        # robot base nodes
        pub_odom_tf_arg,
        ugv_node,
        #dyn_tf,
        robot_state_launch,
        realsense_launch,
        laser_bringup_launch,
        scan_to_pc_node,
        #rf2o_node,
        pointcloud_node,
        camera_sync,
    ])
