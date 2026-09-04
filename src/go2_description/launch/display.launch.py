# 看模型（静态）：只起 robot_state_publisher + 关节滑杆 + RViz，不碰 Gazebo
#
# 用法：
#   cd ~/git/yyh/go2_ws && colcon build --packages-select go2_description && source install/setup.bash
#   ros2 launch go2_description display.launch.py                      # 只看内置 L1
#   ros2 launch go2_description display.launch.py use_external_lidar:=true  # 加挂外置 360° 雷达
#
# RViz 配置 rviz/display.rviz 已删掉：不带配置文件打开，
# 手动 Add -> RobotModel，Fixed Frame 选 base_link 即可。
# 想在 Gazebo 里看模型，等 P2 写 go2_gazebo_bringup 的 launch（那才是仿真的地盘）。

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context: LaunchContext, *args, **kwargs):
    # xacro.process_file 在求值时立即执行，LaunchConfiguration 必须先
    # perform 成普通字符串才能进 mappings，不能直接传对象。
    use_ext = LaunchConfiguration('use_external_lidar').perform(context)
    pkg = get_package_share_directory('go2_description')
    robot_desc = xacro.process_file(
        os.path.join(pkg, 'xacro', 'robot.xacro'),
        mappings={'robot_name': 'robot1',
                  'use_external_lidar': use_ext}).toxml()
    return [
        # 发布 TF（base_link -> trunk -> lidar ... 全靠它）
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_desc}]),
        # 关节滑杆：RViz 里拖动可看腿能不能动（零位 -> 任意角度）
        # 需要 sudo apt install ros-humble-joint-state-publisher-gui
        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui', output='screen'),
        # RViz：不带配置文件打开，手动 Add -> RobotModel，Fixed Frame 选 base_link
        Node(package='rviz2', executable='rviz2', output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_external_lidar', default_value='false',
            description='是否加挂外置 360° 雷达一起预览（默认 false，只看内置 L1）'),
        OpaqueFunction(function=_nodes),
    ])
