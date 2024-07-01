from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    queue_size = LaunchConfiguration('queue_size')    
    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_launch'), 'launch', 'config', 'rgbd.rviz'
    )

    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'queue_size':queue_size,
          'subscribe_depth':True,
          'subscribe_rgb':False,
          'subscribe_scan':True,
          "approx_sync": True,
          'use_action_for_goal':True,
          'qos_scan':qos,
          'qos_image':qos,
          'qos_imu':qos,
          # RTAB-Map's parameters should be strings:
          'Reg/Strategy':'1',
          'Reg/Force3DoF':'true',
          'RGBD/NeighborLinkRefining':'True',
          'Grid/RangeMin':'0.2', # ignore laser scan points on the robot itself
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }

    remappings=[
          ('rgb/image', '/color/preview/image'),
          ('rgb/camera_info', '/color/preview/camera_info'),
          ('depth/image', '/stereo/depth'),
          ('odom', '/odom')
        ]
        
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),
            
        DeclareLaunchArgument(
            'localization', default_value='true',
            description='Launch in localization mode.'),
        
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz, 
                              description='Configuration path of rviz2.'),
        
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RVIZ (optional).'),

        DeclareLaunchArgument(
            'queue_size', default_value='10',
            description='queue_size'),

        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':True, 'approx_sync_max_interval':0.01, 'use_sim_time':use_sim_time, 'qos':qos}],
            remappings=remappings),

        # SLAM Mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', 
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),

    ])