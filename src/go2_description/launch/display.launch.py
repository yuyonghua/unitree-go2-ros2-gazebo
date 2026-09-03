import os
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def _nodes(context: LaunchContext, *args, **kwargs):
    # 注意：xacro.process_file 在 launch 文件求值时立即执行，LaunchConfiguration
    # 必须先 perform 成字符串再传 mappings，不能直接传对象。
    use_ext = LaunchConfiguration('use_external_lidar').perform(context)
    pkg = get_package_share_directory('go2_description')
    robot_desc = xacro.process_file(
        os.path.join(pkg, 'xacro', 'robot.xacro'),
        mappings={'robot_name': 'robot1',
                  'use_external_lidar': use_ext}).toxml()
    use_gui = LaunchConfiguration('joint_state_gui')
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': robot_desc, 'use_sim_time': False}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             output='screen', condition=IfCondition(use_gui)),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(pkg, 'rviz', 'display.rviz')]),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('joint_state_gui', default_value='true'),
        DeclareLaunchArgument('use_external_lidar', default_value='false',
                              description='是否加载外置 360° 雷达预览（默认 false）'),
        OpaqueFunction(function=_nodes),
    ])
