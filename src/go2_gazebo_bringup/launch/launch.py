# L1 仿真外层（总入口）：选世界 → 起 Gazebo → 延迟 6 秒 → 调起 l1.launch.py
#
# 用法：
#   ros2 launch go2_gazebo_bringup launch.py                           # 默认 warehouse.sdf，只用 L1
#   ros2 launch go2_gazebo_bringup launch.py world:=rmuc_2025_world.sdf
#   ros2 launch go2_gazebo_bringup launch.py use_external_lidar:=true  # 加挂外置 360° 雷达
#   ros2 launch go2_gazebo_bringup launch.py rviz:=true                 # 顺手开 RViz（默认不开）
#   ros2 launch go2_gazebo_bringup launch.py x:=1.0 y:=2.0 z:=1.0 yaw:=1.57  # 指定出生位姿
#
# 延迟 6 秒是等 Gazebo 世界加载完再 spawn，原项目也是这个手法（launch.py）。

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler,
                            OpaqueFunction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter


def _gz(context: LaunchContext, *args, **kwargs):
    # world 参数是文件名（如 warehouse.sdf），拼成 worlds/ 下的完整路径
    world = LaunchConfiguration('world').perform(context)
    wf = os.path.join(get_package_share_directory('go2_gazebo_bringup'), 'worlds', world)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v4 {wf}', 'on_exit_shutdown': 'true'}.items())]


def generate_launch_description():
    ld = LaunchDescription()
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_ext = LaunchConfiguration('use_external_lidar', default='false')
    use_rviz = LaunchConfiguration('rviz', default='false')
    spawn_x = LaunchConfiguration('x', default='0.0')
    spawn_y = LaunchConfiguration('y', default='0.0')
    spawn_z = LaunchConfiguration('z', default='0.8')
    spawn_yaw = LaunchConfiguration('yaw', default='0.0')

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument(
        'use_external_lidar', default_value='false',
        description='是否加载外置 360° 激光雷达，透传给 l1.launch.py'))
    ld.add_action(DeclareLaunchArgument(
        'rviz', default_value='false',
        description='是否启动 RViz，透传给 l1.launch.py（配置文件按 use_external_lidar 自动选）'))
    # 出生位姿：不同 world 地面高度不一样，z 宁高勿低（狗掉下去再站起来）；
    # 默认 0.8 三个 world 通用，rmuc 场地表面偏高可给到 1.0。
    ld.add_action(DeclareLaunchArgument('x', default_value='0.0', description='出生点 x（米）'))
    ld.add_action(DeclareLaunchArgument('y', default_value='0.0', description='出生点 y（米）'))
    ld.add_action(DeclareLaunchArgument('z', default_value='0.8', description='出生点 z（米）'))
    ld.add_action(DeclareLaunchArgument('yaw', default_value='0.0', description='出生朝向 yaw（弧度）'))
    ld.add_action(DeclareLaunchArgument('world', default_value='warehouse.sdf',
                                        description='worlds/ 目录下的世界文件名'))
    ld.add_action(SetParameter(name='use_sim_time', value=use_sim_time))
    ld.add_action(OpaqueFunction(function=_gz))

    # 等 6 秒再 spawn：Gazebo 没起来就 spawn 会失败
    pause = ExecuteProcess(cmd=['sleep', '6'], output='screen')
    ld.add_action(pause)
    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=pause,
        on_exit=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('go2_gazebo_bringup'), 'launch', 'l1.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'use_external_lidar': use_ext,
                              'rviz': use_rviz,
                              'x': spawn_x, 'y': spawn_y,
                              'z': spawn_z, 'yaw': spawn_yaw}.items())])))
    return ld
