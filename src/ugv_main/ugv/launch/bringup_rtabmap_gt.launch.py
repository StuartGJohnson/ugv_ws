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

    # this is published by ground_truth_republish
    pub_odom_tf_arg = DeclareLaunchArgument(
        'pub_odom_tf', default_value='false',
        description='Whether to publish the tf from the original odom to the base_footprint',
    )

    parameters={
          'frame_id':'base_footprint',
          'use_sim_time': False,
          'publish_tf': False,  # ground_truth_republish does this
          'subscribe_rgbd': True,
          'subscribe_scan': True,
          'use_action_for_goal':True,
          # RTAB-Map's parameters should be strings:
          #'Reg/Strategy':'2', # for cloud only
          'Reg/Strategy': '1',
          'RGBD/LinearUpdate' : '0.10',
          'RGBD/AngularUpdate' : '0.10',
          'Mem/STMSize':'0',
          'Reg/Force3DoF':'true',
          'RGBD/NeighborLinkRefining':'true',
          'Grid/FromDepth': 'false',
          'GridGlobal/FullUpdate': 'true',
          'Grid/RayTracing':'true', # Fill empty space
          'Grid/3D':'true', # Use 3D occupancy
          #'Grid/3D': 'false',  # Use 2D occupancy
          'Grid/RangeMax':'3',
          'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
          #'Grid/Sensor':'0', # scan_cloud
          'Grid/Sensor': '2',  # laser scan and camera
          'Grid/MaxGroundHeight':'0.04', # All points above 4 cm are obstacles
          'Grid/MaxGroundAngle': '10',  # All ground tilted more than 10 degrees is an obstacle
          'Grid/MaxObstacleHeight':'0.5',  # All points over 0.5 meter are ignored
          'Grid/MinPlaneMinInliers': '100',
          'Grid/RangeMin':'0.10', # ignore laser scan points on the robot itself
          'Grid/CellSize':'.02',
          'Grid/Voxel': '.02',
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }
    # remappings=[
    #       ('rgb/image', '/camera/image'),
    #       ('rgb/camera_info', '/camera/camera_info'),
    #       ('depth/image', '/camera/depth'),
    #       ('scan_cloud', '/fusion/points')]
    #       #('octomap_grid', '/map')]

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
        parameters=[{'decimation': 2,
                     'max_depth': 3.0,
                     'voxel_size': 0.02}],
        remappings=[('depth/image', '/camera/aligned_depth_to_color/image_raw'),
                    ('depth/camera_info', '/camera/aligned_depth_to_color/camera_info'),
                    ('cloud', '/camera/cloud')]
    )

    # obstacles_node = Node(
    #     package='rtabmap_util', executable='obstacles_detection', output='screen',
    #     parameters=[parameters],
    #     remappings=[('cloud', '/camera/cloud'),
    #                 ('obstacles', '/camera/obstacles'),
    #                 ('ground', '/camera/ground')])

    camera_sync = Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        parameters=[{'approx_sync':True}],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/aligned_depth_to_color/image_raw')
        ]
    )

    # nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': 'nav2_rtabmap_gt_params.yaml',
        }.items(),
    )

    # ground truth republish - the central point of this launch file
    # is that the ground_truth_republish node publishes the IPS system's odom as
    # the odom->base_footprint transform
    gt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ground_truth_republish'), 'launch', 'ground_truth_republish.launch.py')
        ),
        launch_arguments={
            'params_file': 'ground_truth_republish_ips_params.yaml',
            'use_sim_time': 'false'
        }.items()
    )

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

    # should not need this
    dyn_tf = Node(
        package='ugv',
        executable='static_tf_to_dynamic_publisher',
    )

    # note that /odom is reserved for the ground truth odom, so
    # we reconfigure for the ugv ekf /odom
    ugv_node = Node(
        package='ugv',
        executable='ugv',
        parameters=[{'pub_odom_tf': False, 'odom_topic': '/odom_ugv'}]
    )

    # point cloud/scan fusion
    # fuser_node = Node(
    #     package='point_cloud_tools',
    #     executable='cloud_scan_fuser_node',
    #     parameters=[{'cloud_stride_u': 8, 'cloud_stride_v': 8}]
    # )

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
    # pc_node = Node(
    #     package='point_cloud_tools',
    #     executable='voxel_grid_filter_node',
    #     parameters=[{
    #         'leaf_size': .01,
    #         'input_topic': '/camera_points',
    #         'output_topic': 'cloud'
    #     }]
    # )

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),

        # robot base nodes
        pub_odom_tf_arg,
        gt_launch,
        ugv_node,
        robot_state_launch,
        realsense_launch,
        laser_bringup_launch,
        rf2o_node,
        nav2_launch,
        pointcloud_node,
        camera_sync,

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
            #remappings=remappings,
            arguments=['-d']),
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            #remappings=remappings
        ),

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
