import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def get_localplan_config_file(context):
    use_localplan = context.launch_configurations['use_localplan']

    ugv_nav_dir = get_package_share_directory('ugv_nav')

    teb_param_path = os.path.join(ugv_nav_dir, 'param', 'teb.yaml')
    dwa_param_path = os.path.join(ugv_nav_dir, 'param', 'dwa.yaml')

    config_map = {
        'teb': teb_param_path,
        'dwa': dwa_param_path
    }

    return config_map.get(use_localplan, teb_param_path)

def launch_setup(context, *args, **kwargs):

    use_localplan = context.launch_configurations['use_localplan']
    
    param_file = get_localplan_config_file(context)
    
    ugv_nav_dir = get_package_share_directory('ugv_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 定义启动参数
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(ugv_nav_dir, 'maps', 'map.yaml'))
                            
    # 包含Lidar启动文件
    bringup_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ugv_bringup'), 'launch', 'bringup_lidar.launch.py')),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': 'nav_2d',  # 可能需要根据实际情况调整
        }.items()
    )

    # 包含AMCL启动文件
    nav2_bringup_amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_path,
            'params_file': param_file
        }.items(),
        condition=LaunchConfigurationEquals('use_localization', 'amcl')
    )

    # 包含Cartographer启动文件
    nav2_bringup_cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ugv_nav'), 'launch', 'bringup_launch_cartographer.launch.py')),
        launch_arguments={
            'params_file': param_file
        }.items(),
        condition=LaunchConfigurationEquals('use_localization', 'cartographer')
    )

    return [
        bringup_lidar_launch,
        nav2_bringup_amcl_launch,
        nav2_bringup_cartographer_launch
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_localplan', default_value='teb', description='Choose which localplan to use: dwa,teb'),
        DeclareLaunchArgument('use_localization', default_value='amcl', description='Choose which use_localization to use: amcl,cartographer'),
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == '__main__':
    generate_launch_description()
