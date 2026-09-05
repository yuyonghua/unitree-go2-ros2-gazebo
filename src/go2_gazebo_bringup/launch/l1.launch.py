# L1 仿真内层：Spawn 狗 + 桥接 + 控制器 + 里程计 + EKF（单机器人版）
#
# 一般不直接跑它，由外层 launch.py 起完 Gazebo 世界后再调起：
#   ros2 launch go2_gazebo_bringup launch.py
#   ros2 launch go2_gazebo_bringup launch.py use_external_lidar:=true  # 加挂外置 360° 雷达
#   ros2 launch go2_gazebo_bringup launch.py rviz:=true                 # 顺手开 RViz（默认不开）
#
# 节点顺序不能乱：robot_state_publisher → spawn → bridge → spawner → 控制器/odom/ekf

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _robot_nodes(context: LaunchContext, *args, **kwargs):
    # xacro.process_file 在求值时立即执行，LaunchConfiguration 必须先
    # perform 成普通字符串才能进 mappings，不能直接传对象，所以包一层 OpaqueFunction。
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ext = LaunchConfiguration('use_external_lidar').perform(context)
    ns = 'robot1'  # 命名空间：话题、TF、控制器全挂在它下面
    pkg = get_package_share_directory('go2_gazebo_bringup')

    # 多机器人时代留下的 remap 习惯：/tf、/scan、/odom 转成命名空间内的相对名，
    # 这样 slam_toolbox / Nav2 配相对话题名就能直接复用。
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static'),
                  ('/scan', 'scan'), ('/odom', 'odometry/filtered')]

    # --- 1. 机器人描述：xacro 转 URDF，use_external_lidar 决定展不展开外置雷达 ---
    desc_pkg = get_package_share_directory('go2_description')
    robot_desc = xacro.process_file(
        os.path.join(desc_pkg, 'xacro', 'robot.xacro'),
        mappings={'robot_name': ns,
                  'use_external_lidar': use_ext}).toxml()

    # -- 1.1 robot_state_publisher：发 TF、robot_description ---
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               namespace=ns, output='screen',
               parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}],
               remappings=remappings)

    # --- 2. 把狗扔进 Gazebo：-topic 必须等于 rsp 发出来的话题（namespace + robot_description）---
    # 上次 Gazebo 里没模型的教训：这里一旦写错（比如少了 /robot1 前缀），
    # create 会因等不到话题而失败，表现为有世界、无机器人。
    # spawn 的 -name 是 Gazebo 里模型的名字，和 namespace 没关系，随便写。
    # spawn 是 ros_gz_sim 包里的 create，可在 Gazebo 里创建模型，等价于 gz sim -r -m <model>.\
    # 这个 create 节点本身并不负责发布模型话题，也不是给 RViz 接收数据的；
    # 它的角色是一个“搬运工”——负责从 ROS 2 的话题中读取机器人模型，并调用 Gazebo 的内部服务在仿真世界里把机器人“生成（Spawn）”出来。
    spawn = Node(package='ros_gz_sim', executable='create', namespace=ns, output='screen',
                 arguments=['-topic', f'/{ns}/robot_description',
                            '-name', f'{ns}_my_bot', '-allow_renaming', 'true',
                            '-x', '0.0', '-y', '0.0', '-z', '0.5'])

    # --- 3. 桥接：Gazebo 话题 → ROS 2 话题。外置雷达开启时才追加它的三个话题，
    # 未开启时 Gazebo 侧根本没有这些话题，加了反而报错 ---
    # 话题类型对应关系：
    #   sensor_msgs/msg/Imu <-> gz.msgs.IMU
    #   sensor_msgs/msg/LaserScan <-> gz.msgs.LaserScan
    #   sensor_msgs/msg/PointCloud2 <-> gz.msgs.PointCloudPacked
    #   tf2_msgs/msg/TFMessage <-> gz.msgs.Pose_V
    #   sensor_msgs/msg/JointState <-> gz.msgs.Model
    bridge_args = [
        f'/{ns}/imu_plugin/out@sensor_msgs/msg/Imu@gz.msgs.IMU',
        f'/{ns}/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        f'/{ns}/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        f'/{ns}/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        f'/{ns}/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model']
    if use_ext == 'true':
        bridge_args += [
            f'/{ns}/velodyne@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            f'/{ns}/velodyne/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            f'/{ns}/velodyne_imu@sensor_msgs/msg/Imu@gz.msgs.IMU']
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  namespace=ns, output='screen', arguments=bridge_args)

    # --- 4. 全局时钟桥：仿真时间同步，没它全链路 tf 会报 timeout ---
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                        arguments=['--ros-args', '-p',
                                   f'config_file:={os.path.join(pkg, "config", "gz_bridge.yaml")}'])

    # --- 5. 控制器：joint_state_broadcaster（发 joint_states）+ joint_group_controller（收指令）---
    # 前提是 URDF 里 ros2_control 段有完整的 12 个关节定义，否则 spawner 直接 exit 1。
    # 血泪教训（2026-09-04/05/06）：spawner 必须等 Gazebo 里的 controller_manager
    # 真正就绪后再起，且两个 spawner 串行。原因有两层：
    #  1) 上游 bug（ros-humble-controller-manager 2.54，spawner.py）：load_controller
    #     调用没传 service_call_timeout，永远用硬编码 10 秒（源码 controller_manager_services.py
    #     默认 call_timeout=10.0）。所以 --service-call-timeout 60 只保得住 configure/
    #     activate，保不住 load。本机实测 controller_manager 约在 Gazebo 启动后 17 秒
    #     才响应 load，第一次调用必 10 秒超时、服务端随后成功、重试撞 already loaded FATAL。
    #  2) 两个 spawner 同时起会一起 hammer 还没起来的 controller_manager。
    # 解法：TimerAction 延迟 20 秒再起 broadcaster（此时 load 一次即中），broadcaster
    # 退出后再起 group controller（OnProcessExit 串行）。--controller-manager-timeout /
    # --service-call-timeout 60 照留，保 configure/activate 阶段。机器慢导致还是 FATAL
    # 的话，把下面的 20.0 调大。注意 OnProcessExit 不看退出码：若 jsb 死了 jgc 照样会起，
    # 此时用 list_controllers 检查，手动 configure + activate（见指南 §5.4 排错）。
    cm = f'/{ns}/controller_manager'
    jsb = Node(package='controller_manager', executable='spawner',
               namespace=ns, output='screen',
               arguments=['joint_state_broadcaster',
                          '--controller-manager', cm,
                          '--controller-manager-timeout', '60',
                          '--service-call-timeout', '60'],
               remappings=remappings)
    jgc = Node(package='controller_manager', executable='spawner',
               namespace=ns, output='screen',
               arguments=['joint_group_controller',
                          '--controller-manager', cm,
                          '--controller-manager-timeout', '60',
                          '--service-call-timeout', '60'],
               remappings=remappings)
    # 延迟 20 秒：等 Gazebo 内 controller_manager 就绪再办 load，一次即中
    jsb_delayed = TimerAction(period=20.0, actions=[jsb])
    # broadcaster 退出后才起 group controller，避免同时 hammer controller_manager
    jgc_after_jsb = RegisterEventHandler(OnProcessExit(
        target_action=jsb, on_exit=[jgc]))

    # --- 6. 四足业务节点（整体复用 quadropted_controller，不手写）---
    controller = Node(package='quadropted_controller', executable='robot_controller_gazebo.py',
                      name='quadruped_controller', namespace=ns,
                      output='screen', remappings=remappings)
    cmd_vel_pub = Node(package='quadropted_controller', executable='cmd_vel_pub.py',
                       name='cmd_vel_pub', namespace=ns,
                       output='screen', remappings=remappings)
    odom = Node(package='quadropted_controller', executable='QuadrupedOdometryNode.py',
                name='odom', namespace=ns, output='screen',
                parameters=[{'verbose': False, 'publish_rate': 50, 'open_loop': False,
                             'has_imu_heading': True, 'is_gazebo': True,
                             'imu_topic': f'/{ns}/imu', 'base_frame_id': 'base_link',
                             'odom_frame_id': 'odom', 'clock_topic': '/clock',
                             'enable_odom_tf': True}],
                remappings=remappings)

    # --- 7. EKF：融合 odom + IMU，输出 odometry/filtered（SLAM/Nav2 吃这个）---
    ekf = Node(package='robot_localization', executable='ekf_node',
               name='ekf_filter_node', namespace=ns, output='screen',
               parameters=[os.path.join(pkg, 'config', 'ekf.yaml'),
                           {'use_sim_time': use_sim_time}],
               remappings=remappings)

    # --- 8. RViz：默认不开，需要看时 launch.py 加 rviz:=true ---
    # 配置按 use_external_lidar 自动二选一（外置雷达开才用 external 版，否则 RViz 里
    # velodyne 那两个 display 因无话题而报红）。namespace + remappings 写法与其他节点
    # 一致，这样 RViz 订阅的是 /robot1/tf；Fixed Frame=odom（P2 还没有 map）。
    rviz_cfg = os.path.join(
        pkg, 'rviz',
        'go2_l1_external.rviz' if use_ext == 'true' else 'go2_l1.rviz')
    rviz = Node(package='rviz2', executable='rviz2', name='rviz2',
                namespace=ns, output='screen',
                arguments=['-d', rviz_cfg],
                parameters=[{'use_sim_time': use_sim_time}],
                remappings=remappings,
                condition=IfCondition(LaunchConfiguration('rviz')))

    return [clock_bridge, rsp, spawn, bridge, jsb_delayed, jgc_after_jsb,
            controller, cmd_vel_pub, odom, ekf, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_external_lidar', default_value='false',
                              description='是否加载外置 360° 激光雷达（默认 false，只用内置 L1）'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='是否启动 RViz（默认 false；为 true 时按 use_external_lidar 自动选配置文件）'),
        OpaqueFunction(function=_robot_nodes),
    ])
