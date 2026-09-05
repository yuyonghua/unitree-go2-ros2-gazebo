import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('go2_slam')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', namespace='robot1', output='screen',
             parameters=[{'use_sim_time': use_sim_time},
                         os.path.join(pkg, 'config', 'mapper_params_l1.yaml'),
                         {'scan_topic': 'scan',      # 原 velodyne → 改 scan
                          'odom_topic': 'odom',      # 经 remap 实为 odometry/filtered
                          'base_frame': 'base_link', 'odom_frame': 'odom',
                          'map_frame': 'map'}],
             remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'),
                         ('/scan', '/robot1/scan'),
                         ('/odom', '/robot1/odometry/filtered')]),
    ])
