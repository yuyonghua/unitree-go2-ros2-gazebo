import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('go2_nav')
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'map', default_value=os.path.join(pkg, 'maps', 'warehouse_l1.yaml')))
    ld.add_action(DeclareLaunchArgument(
        'params_file', default_value=os.path.join(pkg, 'param', 'nav2_l1.yaml')))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={'map': LaunchConfiguration('map'),
                          'use_sim_time': LaunchConfiguration('use_sim_time'),
                          'params_file': LaunchConfiguration('params_file'),
                          'namespace': 'robot1',
                          'use_namespace': 'true'}.items()))
    ld.add_action(Node(package='rviz2', executable='rviz2', name='rviz2',
                       namespace='/robot1', output='screen',
                       arguments=['-d', os.path.join(pkg, 'rviz', 'go2_nav.rviz')],
                       parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                       remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]))
    return ld
