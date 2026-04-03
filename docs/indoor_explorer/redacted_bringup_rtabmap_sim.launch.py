
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
