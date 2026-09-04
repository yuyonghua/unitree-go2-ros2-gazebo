# GO2 前置 L1 激光雷达复现指南：建图 + 导航（全新工作空间手写版）

> 目标：不依赖原仓库启动，只用 GO2 前置 L1 雷达（`/scan`），实现 `Gazebo 仿真 → 建图（slam_toolbox）→ 导航（Nav2）`。
> 策略：全新 `go2_l1_ws` 手写，能复制的只拷 `meshes / 腿部 xacro / 控制器 / 世界文件`，其余重写。
> 约定：命名空间 `robot1`，ROS 2 Humble，DDS `rmw_cyclonedds_cpp`，`use_sim_time:=true`。
> 原项目引用路径均为 `ROS2-Gazebo-GO2/src/...`，行号基于 2026-09-03 现状。

---

## 1. 原项目剖析（复现前必读）

### 1.1 包一览

| 包 | 作用 | 复现策略 |
|---|---|---|
| `go2_description` | 狗本体 xacro + meshes + ros_control | meshes/腿部 xacro/ros_control **复制**，整包重写只留 L1 |
| `gazebo_sim` | Gazebo 启动、bridge、EKF、RViz | launch 逻辑**参考重写**，yaml **复制** |
| `quadropted_controller`（`robot_controller_gazebo.py` / `cmd_vel_pub.py` / `QuadrupedOdometryNode.py`） | 步态控制、cmd_vel 转发、odom 推算 | **整体复用，不重写** |
| `quadropted_msgs` | `RobotBehaviorCommand` 等自定义 msg/srv | **整体复用** |
| `cartographer` | `go2_cartographer.launch.py/lua` + `slam_toolbox.launch.py` + 地图 | launch 思想参考，lua 不用（改 slam_toolbox） |
| `navigation2` | `go2_navigation2.launch.py` + `go2_nav2.yaml` + 地图 | yaml **复制后改 scan 话题**，launch **参考重写** |
| `docker` | `cyclonedds.xml` | **复制** |

### 1.2 运行时数据链（`sensors:=true`）

```text
robot_VLP_D435i.xacro + gazebo_VLP_D435i.xacro
  → robot_state_publisher (/robot1/robot_description)
  → ros_gz_sim create (-x/-y/-z  spawn)
  → ros_gz_bridge parameter_bridge (imu/scan/points/joint_states/tf/image)
  → controller_manager spawner (joint_state_broadcaster + joint_group_controller)
  → quadropted_controller (robot_controller_gazebo.py + cmd_vel_pub.py + QuadrupedOdometryNode.py)
  → robot_localization ekf_node (odom + imu_plugin/out → odometry/filtered)
  → slam_toolbox / cartographer (/map) → Nav2 (amcl + planner + controller + costmap)
```

核心文件：`gazebo_sim/launch/launch.py`（选 world + 延时 6s 再 spawn，见 `:50-96`），`gazebo_sim/launch/gazebo_go2_sensors.launch.py:74-126`（xacro→spawn→bridge 全流程），`gazebo_sim/config/ekf.yaml:74-162`（odom+imu 融合）。

### 1.3 双雷达对比（最关键）

定义在 `go2_description/xacro/gazebo_VLP_D435i.xacro`：

| 项 | L1（目标，要留） | VLP16（外界，要删） |
|---|---|---|
| link/joint | `trunk → lidar`，`xyz 0.28945 0 -0.046825 rpy 0 2.8782 0`（见 `robot_VLP_D435i.xacro:69-76`） | `base_link → velodyne`，`xyz 0.22 0 0.09`（见 `:147-151`） |
| Gazebo sensor | `L1_lidar / gpu_lidar`（见 `:113-142`） | `velodyne / gpu_lidar`（见 `:290-319`） |
| topic | `/robot1/scan`（+`/scan/points`） | `/robot1/velodyne`（+`/velodyne/points`） |
| 水平 FOV | `1.396~4.887 rad ≈ 80°~280°`，**前方 200°，后方盲区** | `-pi~pi`，**360°** |
| 垂直 | 16 线，`±0.262 rad` | 16 线，`±0.262 rad` |
| range | `0.3~131 m` | `0.3~131 m` |
| update_rate | 10 Hz | 10 Hz |

> **原项目建图导航实际全用 VLP16**：`cartographer/launch/go2_cartographer.launch.py:36` 把 `/scan` remap 到 `/robot1/velodyne`；`cartographer/launch/slam_toolbox.launch.py:36 scan_topic:=velodyne`；`navigation2/param/go2_nav2.yaml:40,215,251` 三处 `/velodyne`。切 L1 时这三处必须全改成 `/scan`，否则能跑但用的还是 VLP16。

### 1.4 Topic / TF 清单（只留 L1 后应有）

```text
/robot1/scan                  LaserScan   （L1 建图导航唯一输入）
/robot1/scan/points           PointCloud2 （RViz 可视化，可选）
/robot1/imu_plugin/out        Imu         （EKF 用）
/robot1/joint_states          JointState
/robot1/odometry/filtered     Odometry    （EKF 输出，SLAM/Nav2 的 odom）
/robot1/odom                  Odometry    （QuadrupedOdometryNode 原始输出，被 remap 到 odometry/filtered 消费）
/robot1/cmd_vel               Twist       （teleop 输入，经 cmd_vel_pub 转发）
/robot1/robot_description     String(URDF)
/map                          OccupancyGrid （SLAM 输出 / Nav2 输入）
tf: map → odom（SLAM 或 amcl）→ base_link（EKF/odom）→ trunk → lidar / imu_link
```

`gazebo_go2_sensors.launch.py:51-56` 的全局 remap `(/tf→tf, /tf_static→tf_static, /scan→scan, /odom→odometry/filtered)` 是多机器人命名空间做法，单机器人复现时保留思想即可。

---

## 2. 新工作空间结构

```bash
mkdir -p ~/go2_l1_ws/src && cd ~/go2_l1_ws/src
```

```text
go2_l1_ws/src/
  go2_description/          # 手写包：机器人本体（只含 L1）
    package.xml  CMakeLists.txt            # ros2 pkg create 生成后按 4.2 节改
    xacro/robot.xacro  xacro/gazebo.xacro   # 手写（本指南给全文），与官方六件套同名
    xacro/leg.xacro  xacro/const.xacro  xacro/materials.xacro  xacro/transmission.xacro  # 从原包复制（transmission 仅占位，与官方目录结构对齐）
    xacro/lidar_external.xacro  # 手写预留（本指南给全文）：外置 360° 雷达宏，默认关闭
    meshes/trunk.dae  meshes/hip.dae  meshes/thigh.dae  meshes/thigh_mirror.dae  meshes/calf.dae  # 必需 5 件，从原包复制
    meshes/calf_mirror.dae  meshes/foot.dae    # 可选备用 2 件（原包死文件，不引用也无妨）
    config/ros_control.yaml                    # 从原包复制
    launch/display.launch.py                   # 手写（本指南给全文，静态预览用）
    rviz/display.rviz                          # 可选，RViz 存一份
  go2_gazebo_bringup/       # 手写包：仿真启动
    package.xml  CMakeLists.txt
    launch/launch.py  launch/l1.launch.py
    config/robots.yaml  config/gz_bridge.yaml  config/ekf.yaml
    worlds/warehouse.sdf    # 从原包复制（或只拷这一个）
    rviz/go2_l1.rviz
  go2_slam/                 # 手写包：建图
    package.xml  CMakeLists.txt
    launch/slam.launch.py
    config/mapper_params_l1.yaml
  go2_nav/                  # 手写包：导航
    package.xml  CMakeLists.txt
    launch/nav2.launch.py
    param/nav2_l1.yaml      # 从 go2_nav2.yaml 复制改
    maps/                   # 建图存这里
  # 以下两个整体复制，不手写：
  quadropted_controller/  quadropted_msgs/
```

> 不建 `dae/` 目录：原 `dae/` 与 `meshes/` 经 md5 确认是同一套副本（`dae/base.dae == meshes/trunk.dae`，其余 hip/thigh/thigh_mirror/calf/calf_mirror/foot 两边哈希全对等），且所有 xacro 只引用 `meshes/`，无一处引用 `dae/`。原 `CMakeLists.txt` 同时 install 两者只是兼容历史。
> 不拷 `meshes/Vlp16.dae、realsense.dae、hokuyo.dae`：分别是已删除的 VLP16 / D435i / 备用雷达外观。

`package.xml` 依赖模板（`go2_gazebo_bringup` 为例，其余按需裁）：

```xml
<depend>rclcpp</depend><depend>xacro</depend><depend>urdf</depend>
<depend>robot_state_publisher</depend><depend>controller_manager</depend>
<depend>ros_gz_sim</depend><depend>ros_gz_bridge</depend>
<depend>robot_localization</depend><depend>gz_ros2_control</depend>
<depend>teleop_twist_keyboard</depend>
<exec_depend>ros2_controllers</exec_depend>
```

`go2_slam` 需加 `<depend>slam_toolbox</depend>`，`go2_nav` 需加 `<depend>nav2_bringup</depend>`。

---

## 3. P0 环境对齐（0.5 天）

1. 复制 DDS 与环境脚本：

```bash
cp ROS2-Gazebo-GO2/src/docker/cyclonedds.xml ~/go2_l1_ws/cyclonedds.xml
# 新建 ~/go2_l1_ws/env.sh（照抄原 start.sh）：
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/go2_l1_ws/cyclonedds.xml
export GZ_SIM_RESOURCE_PATH=$HOME/go2_l1_ws/src/go2_gazebo_bringup/worlds:$HOME/go2_l1_ws/src/go2_gazebo_bringup/models
export IGN_GAZEBO_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
source /opt/ros/humble/setup.bash
source ~/go2_l1_ws/install/setup.bash
```

2. 安装系统依赖（按原 `gazebo_sim/package.xml`）：

```bash
sudo apt update
sudo apt install -y ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-image \
  ros-humble-gz-ros2-control ros-humble-controller-manager ros-humble-ros2-controllers \
  ros-humble-robot-localization ros-humble-slam-toolbox ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard ros-humble-xacro ros-humble-robot-state-publisher
```

3. 验收：`colcon build` 通过；`ros2 launch go2_gazebo_bringup launch.py world:=warehouse.sdf` 能打开空 Gazebo。

排错：首次启动慢是下载 Fuel 模型，提前 `export GZ_SIM_RESOURCE_PATH` 指向本地 `models/`；`rmw` 不一致会导致 bridge 收不到数，全终端统一 `env.sh`。

---

## 4. P1 Description 只留 L1（1 天）

### 4.1 建包（不要纯手mkdir，先 `pkg create`）

```bash
cd ~/go2_l1_ws/src
ros2 pkg create go2_description --build-type ament_cmake --license BSD
mkdir -p go2_description/{xacro,meshes,config,launch,rviz}
```

`pkg create` 会生成 `package.xml` 和 `CMakeLists.txt`，按下面改，不要从零手写。

`package.xml`：在生成文件基础上确认有以下 `exec_depend`（对标原 `src/go2_description/package.xml:15-21`，删掉历史残留的 `gazebo_plugins/launch_ros`）：

```xml
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>joint_state_publisher</exec_depend>
<exec_depend>urdf</exec_depend>
<exec_depend>xacro</exec_depend>
<exec_depend>rviz2</exec_depend>
```

`CMakeLists.txt`：全文如下（对标原 `src/go2_description/CMakeLists.txt:1-18`，注意删掉了 `dae` 和 `urdf` 两行，因为新包不建这两个目录）：

```cmake
cmake_minimum_required(VERSION 3.5)
project(go2_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY xacro DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

> 如果 `rviz/` 目录为空导致 `colcon build` 警告，放一个空的 `display.rviz` 或删掉上面 `install rviz` 那一行，两者任选其一。

### 4.2 复制（5 必需 + 2 备用 + 4 xacro + 1 yaml）

```bash
SRC=~/git/ros2-learning/ROS2-Gazebo-GO2/src/go2_description
DST=~/go2_l1_ws/src/go2_description
# 5 个必需 meshes（新 xacro 实际引用的全部，见 4.6 节对照表）：
cp $SRC/meshes/{trunk.dae,hip.dae,thigh.dae,thigh_mirror.dae,calf.dae} $DST/meshes/
# 2 个备用（原包死文件：leg.xacro 小腿不分左右统一用 calf.dae，foot_link 用 sphere 不用 foot.dae；拷上防呆，不拷也不报错）：
cp $SRC/meshes/{calf_mirror.dae,foot.dae} $DST/meshes/
# 4 个直接复用的 xacro + 1 个控制器配置（transmission 仅占位，leg.xacro 里对其引用保持注释，与官方结构对齐）：
cp $SRC/xacro/{leg.xacro,const.xacro,materials.xacro,transmission.xacro} $DST/xacro/
cp $SRC/config/ros_control.yaml $DST/config/
# 注意：Vlp16.dae 现在不拷；日后启用外置雷达时再补（见 4.5 节末尾检查单）。
```

取舍理由：`meshes/` 共 10 个文件，`Vlp16.dae / realsense.dae / hokuyo.dae` 是已删除传感器（VLP16 / D435i / 备用雷达）的外观；`dae/` 目录是 `meshes/` 的重复副本，不复制、不建目录。

`ros_control.yaml` 内容不动（12 个腿关节 `position` 指令 + `position/velocity/effort` 状态）。

### 4.3 手写 `xacro/gazebo.xacro`（与官方同名；内置 L1+IMU+控制+摩擦常驻）

对照原 `gazebo_VLP_D435i.xacro:4-142` 删减：保留 `gz_ros2_control plugin`、`ros2_control 12 joints`、`imu_link sensor`、`lidar sensor`、`trunk/腿部 mu/kp/kd`，**删除 velodyne、velodyne_imu、camera_face、d435 全部段**（外置雷达搬到 `lidar_external.xacro`，见 4.5 节）。

> 血泪教训：`ros2_control` 段 12 个 joint 必须全文手写，一个不能少。
> 之前文档这里写“照抄省略”，复现时直接把注释拷过去，导致 Gazebo 认不出关节、
> 两个 spawner exit 1、控制器全灭。下面是完整内容，照抄。

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="go2_gazebo" params="robot_name">
    <gazebo>
      <plugin name="gz_ros2_control::GazeboSimROS2ControlPlugin" filename="libgz_ros2_control-system.so">
        <ros><namespace>/${robot_name}</namespace></ros>
        <parameters>$(find go2_description)/config/ros_control.yaml</parameters>
      </plugin>
    </gazebo>
    <ros2_control name="${robot_name}_GazeboSystem" type="system">
      <hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>
      <!-- lf/rf/lh/rh × hip/upper_leg/lower_leg，共 12 组，每组 position 指令 + position/velocity/effort 状态 -->
      <joint name="lf_hip_joint">
        <command_interface name="position"/>
        <state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/>
      </joint>
      <joint name="lf_upper_leg_joint">
        <command_interface name="position"/>
        <state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/>
      </joint>
      <joint name="lf_lower_leg_joint">
        <command_interface name="position"/>
        <state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/>
      </joint>
      <joint name="rf_hip_joint">
        <command_interface name="position"/>
        <state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/>
      </joint>
      <joint name="rf_upper_leg_joint">
        <command_interface name="position"/>
        <state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/>
      </joint>
      <joint name="rf_lower_leg_joint">
        <command_interface name="position"/>
        <state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/>
      </joint>
      <joint name="lh_hip_joint">
        <command_interface name="position"/>
        <state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/>
      </joint>
      <joint name="lh_upper_leg_joint">
        <command_interface name="position"/>
        <state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/>
      </joint>
      <joint name="lh_lower_leg_joint">
        <command_interface name="position"/>
        <state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/>
      </joint>
      <joint name="rh_hip_joint">
        <command_interface name="position"/>
        <state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/>
      </joint>
      <joint name="rh_upper_leg_joint">
        <command_interface name="position"/>
        <state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/>
      </joint>
      <joint name="rh_lower_leg_joint">
        <command_interface name="position"/>
        <state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/>
      </joint>
    </ros2_control>
    <gazebo reference="imu_link">
      <gravity>false</gravity>
      <sensor name="imu_sensor" type="imu">
        <always_on>true</always_on><update_rate>100</update_rate>
        <visualize>false</visualize>
        <topic>/${robot_name}/imu_plugin/out</topic>
        <plugin filename="libgz-sim-imu-system.so" name="gz::sim::systems::Imu"/>
        <pose>0 0 0 0 0 0</pose><gz_frame_id>imu_link</gz_frame_id>
      </sensor>
    </gazebo>
    <gazebo reference="lidar">
      <sensor name="L1_lidar" type="gpu_lidar">
        <pose>0 0 0 0 0 0</pose><visualize>false</visualize><update_rate>10</update_rate>
        <lidar>
          <scan><horizontal><samples>640</samples><resolution>1</resolution>
            <min_angle>1.396263</min_angle><max_angle>4.8869</max_angle></horizontal>
            <vertical><samples>16</samples><resolution>1</resolution>
            <min_angle>-0.261799</min_angle><max_angle>0.261799</max_angle></vertical></scan>
          <range><min>0.3</min><max>131</max><resolution>0.001</resolution></range>
        </lidar>
        <topic>/${robot_name}/scan</topic><gz_frame_id>lidar</gz_frame_id>
      </sensor>
    </gazebo>
    <!-- trunk + 四条腿 link 的 mu/kp/kd：脚 mu 0.6 抓地，其余 0.2，站不站得住看这里 -->
    <gazebo reference="trunk"><mu1>0.2</mu1><mu2>0.2</mu2><kp>10000.0</kp><kd>1.0</kd></gazebo>
    <!-- FL leg -->
    <gazebo reference="lf_hip_link"><mu1>0.2</mu1><mu2>0.2</mu2></gazebo>
    <gazebo reference="lf_upper_leg_link">
      <mu1>0.2</mu1><mu2>0.2</mu2><self_collide>1</self_collide><kp>10000.0</kp><kd>1.0</kd>
    </gazebo>
    <gazebo reference="lf_lower_leg_link">
      <mu1>0.2</mu1><mu2>0.2</mu2><self_collide>1</self_collide>
    </gazebo>
    <gazebo reference="lf_foot_link">
      <mu1>0.6</mu1><mu2>0.6</mu2><self_collide>1</self_collide><kp>10000.0</kp><kd>1.0</kd>
    </gazebo>
    <!-- FR leg -->
    <gazebo reference="rf_hip"><mu1>0.2</mu1><mu2>0.2</mu2></gazebo>
    <gazebo reference="rf_upper_leg_link">
      <mu1>0.2</mu1><mu2>0.2</mu2><self_collide>1</self_collide><kp>10000.0</kp><kd>1.0</kd>
    </gazebo>
    <gazebo reference="rf_lower_leg_link">
      <mu1>0.2</mu1><mu2>0.2</mu2><self_collide>1</self_collide>
    </gazebo>
    <gazebo reference="rf_foot_link">
      <mu1>0.6</mu1><mu2>0.6</mu2><self_collide>1</self_collide><kp>10000.0</kp><kd>1.0</kd>
    </gazebo>
    <!-- RL leg -->
    <gazebo reference="lh_hip"><mu1>0.2</mu1><mu2>0.2</mu2></gazebo>
    <gazebo reference="lh_upper_leg_link">
      <mu1>0.2</mu1><mu2>0.2</mu2><self_collide>1</self_collide><kp>10000.0</kp><kd>1.0</kd>
    </gazebo>
    <gazebo reference="lh_lower_leg_link">
      <mu1>0.2</mu1><mu2>0.2</mu2><self_collide>1</self_collide>
    </gazebo>
    <gazebo reference="lh_foot_link">
      <mu1>0.6</mu1><mu2>0.6</mu2><self_collide>1</self_collide><kp>10000.0</kp><kd>1.0</kd>
    </gazebo>
    <!-- RR leg -->
    <gazebo reference="rh_hip"><mu1>0.2</mu1><mu2>0.2</mu2></gazebo>
    <gazebo reference="rh_upper_leg_link">
      <mu1>0.2</mu1><mu2>0.2</mu2><self_collide>1</self_collide><kp>10000.0</kp><kd>1.0</kd>
    </gazebo>
    <gazebo reference="rh_lower_leg_link">
      <mu1>0.2</mu1><mu2>0.2</mu2><self_collide>1</self_collide>
    </gazebo>
    <gazebo reference="rh_foot_link">
      <mu1>0.6</mu1><mu2>0.6</mu2><self_collide>1</self_collide><kp>10000.0</kp><kd>1.0</kd>
    </gazebo>
  </xacro:macro>
</robot>
```

> 省略的 70+40 行请对着原文件逐段复制，不要自己发挥：`ros2_control` 段决定 `controller_manager` 能否找到 12 个关节，`mu/kp/kd` 段决定狗在 Gazebo 里站不站得住。

### 4.4 手写 `xacro/robot.xacro`（本体，全文可照抄）

对照原 `robot_VLP_D435i.xacro`：保留 `base_link→trunk→lidar/imu_link + 4×leg`，删除 `velodyne/velodyne_imu/camera_face/camera_d435/d435i_imu` 共 5 组 link/joint（原 `:104-233`）。

```xml
<?xml version="1.0"?>
<robot name="robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find go2_description)/xacro/const.xacro"/>
  <xacro:include filename="$(find go2_description)/xacro/materials.xacro"/>
  <xacro:include filename="$(find go2_description)/xacro/leg.xacro"/>
  <xacro:include filename="$(find go2_description)/xacro/gazebo.xacro"/>
  <xacro:include filename="$(find go2_description)/xacro/lidar_external.xacro"/>
  <xacro:arg name="robot_name" default="robot1"/>
  <xacro:arg name="use_external_lidar" default="false"/>
  <xacro:go2_gazebo robot_name="$(arg robot_name)"/>
  <link name="base_link">
    <visual><origin rpy="0 0 0" xyz="0 0 0"/><geometry><box size="0.001 0.001 0.001"/></geometry></visual>
  </link>
  <joint name="floating_base" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/><parent link="base_link"/><child link="trunk"/>
  </joint>
  <link name="trunk">
    <visual><origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry><mesh filename="file://$(find go2_description)/meshes/trunk.dae" scale="1 1 1"/></geometry></visual>
    <collision><origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry><box size="${trunk_length} ${trunk_width} ${trunk_height}"/></geometry></collision>
    <inertial><origin rpy="0 0 0" xyz="${trunk_com_x} ${trunk_com_y} ${trunk_com_z}"/><mass value="${trunk_mass}"/>
      <inertia ixx="${trunk_ixx}" ixy="${trunk_ixy}" ixz="${trunk_ixz}" iyy="${trunk_iyy}" iyz="${trunk_iyz}" izz="${trunk_izz}"/></inertial>
  </link>
  <joint name="lidar_joint" type="fixed">
    <parent link="trunk"/><child link="lidar"/>
    <origin xyz="0.28945 0.0 -0.046825" rpy="0.0 2.8782 0.0"/>
  </joint>
  <link name="lidar"/>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/><child link="imu_link"/><origin rpy="0 0 0" xyz="0 0 0"/>
  </joint>
  <link name="imu_link">
    <inertial><mass value="0.001"/><origin rpy="0 0 0" xyz="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/></inertial>
    <visual><origin rpy="0 0 0" xyz="0 0 0"/><geometry><box size="0.001 0.001 0.001"/></geometry></visual>
    <collision><origin rpy="0 0 0" xyz="0 0 0"/><geometry><box size=".001 .001 .001"/></geometry></collision>
  </link>
  <xacro:leg name="rf" mirror="-1" mirror_dae="False" front_hind="1" front_hind_dae="True"/>
  <xacro:leg name="lf" mirror="1" mirror_dae="True" front_hind="1" front_hind_dae="True"/>
  <xacro:leg name="rh" mirror="-1" mirror_dae="False" front_hind="-1" front_hind_dae="False"/>
  <xacro:leg name="lh" mirror="1" mirror_dae="True" front_hind="-1" front_hind_dae="False"/>

  <!-- 外置 360° 雷达：默认关闭。开启方式：xacro 传参 use_external_lidar:=true
       （P2 的 l1.launch.py 会透传该参数，见 5.2 节；静态预览用默认值即可）。
       原理：$(arg ...) 先替换成 true/false 字符串再求值，false 即不展开。 -->
  <xacro:if value="$(arg use_external_lidar)">
    <xacro:lidar_external robot_name="$(arg robot_name)"/>
  </xacro:if>
</robot>
```

注意原 `robot_VLP_D435i.xacro:70` 行 `<parent link="trunk"/>,` 多一个逗号，手写时去掉。

### 4.5 手写 `xacro/lidar_external.xacro`（预留：外置 360° 雷达，默认不加载）

内容照抄原 `robot_VLP_D435i.xacro:131-177`（link/joint）与 `gazebo_VLP_D435i.xacro:288-334`（sensor），包进一个宏。L1 是内置雷达常驻，这个文件只装外置雷达，两者互不干扰。

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="lidar_external" params="robot_name">
    <link name="velodyne">
      <collision><origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry><mesh filename="file://$(find go2_description)/meshes/Vlp16.dae" scale="1 1 1"/></geometry></collision>
      <visual><origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry><mesh filename="file://$(find go2_description)/meshes/Vlp16.dae" scale="1 1 1"/></geometry></visual>
    </link>
    <joint name="velodyne_joint" type="fixed">
      <origin xyz="0.22 0 0.09" rpy="0 0 0"/>
      <parent link="base_link"/><child link="velodyne"/>
    </joint>
    <joint name="velodyne_imu_joint" type="fixed">
      <parent link="velodyne"/><child link="velodyne_imu"/>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </joint>
    <link name="velodyne_imu">
      <inertial><mass value="0.001"/><origin rpy="0 0 0" xyz="0 0 0"/>
        <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/></inertial>
      <visual><origin rpy="0 0 0" xyz="0 0 0"/><geometry><box size="0.001 0.001 0.001"/></geometry></visual>
      <collision><origin rpy="0 0 0" xyz="0 0 0"/><geometry><box size=".001 .001 .001"/></geometry></collision>
    </link>
    <gazebo reference="velodyne">
      <sensor name="velodyne" type="gpu_lidar">
        <pose>0 0 0 0 0 0</pose><visualize>false</visualize><update_rate>10</update_rate>
        <lidar>
          <scan><horizontal><samples>640</samples><resolution>1</resolution>
            <min_angle>-3.141592653589793</min_angle><max_angle>3.141592653589793</max_angle></horizontal>
            <vertical><samples>16</samples><resolution>1</resolution>
            <min_angle>-0.2617993877991494</min_angle><max_angle>0.2617993877991494</max_angle></vertical></scan>
          <range><min>0.3</min><max>131</max><resolution>0.001</resolution></range>
        </lidar>
        <topic>/${robot_name}/velodyne</topic><gz_frame_id>velodyne</gz_frame_id>
      </sensor>
    </gazebo>
    <gazebo reference="velodyne_imu">
      <gravity>false</gravity>
      <sensor name="velodyne_imu" type="imu">
        <always_on>true</always_on><update_rate>100</update_rate><visualize>false</visualize>
        <topic>/${robot_name}/velodyne_imu</topic>
        <plugin filename="libgz-sim-imu-system.so" name="gz::sim::systems::Imu"/>
        <pose>0 0 0 0 0 0</pose><gz_frame_id>velodyne_imu</gz_frame_id>
      </sensor>
    </gazebo>
  </xacro:macro>
</robot>
```

启用外置雷达检查单（默认全跳过，需要时按序做）：

```bash
# 1. 补拷外观模型（默认不拷就是为了保持 L1-only 干净）：
cp $SRC/meshes/Vlp16.dae $DST/meshes/
# 2. 启动时加参数（bridge 的 velodyne 话题由 l1.launch.py 自动追加，无需手工改）：
ros2 launch go2_description display.launch.py use_external_lidar:=true
ros2 launch go2_gazebo_bringup launch.py use_external_lidar:=true
# 注意：launch.py 外层需把该参数透传给 l1.launch.py（见 5.3 节，已含）。
# 3. 建图/导航切回外置雷达话题：slam.launch.py 改 scan_topic 为 velodyne；
#    nav2_l1.yaml 把 8.1 节的 sed 反向改回（scan→velodyne），RViz 加一个 velodyne 的 LaserScan 显示。
```

验收（默认关闭状态）：

```bash
xacro src/go2_description/xacro/robot.xacro robot_name:=robot1 -o /tmp/go2l1.urdf
check_urdf /tmp/go2l1.urdf
grep -c velodyne /tmp/go2l1.urdf   # 期望 0
xacro src/go2_description/xacro/robot.xacro robot_name:=robot1 use_external_lidar:=true -o /tmp/go2ext.urdf
check_urdf /tmp/go2ext.urdf
grep -c velodyne /tmp/go2ext.urdf  # 期望 ≥1（需先拷 Vlp16.dae，否则仅 URDF 结构检查通过，Gazebo 会缺模型）
```

### 4.6 meshes 引用对照表（自查用）

| 文件 | 被谁引用 | 结论 |
|---|---|---|
| `trunk.dae` | `robot.xacro:49` 躯干 visual | 必需 |
| `hip.dae` | `leg.xacro:50` 四条腿髋部 visual | 必需 |
| `thigh.dae / thigh_mirror.dae` | `leg.xacro:85,88` 按 `mirror_dae` 二选一 | 必需（两个都要） |
| `calf.dae` | `leg.xacro:124` 小腿 visual（不分左右） | 必需 |
| `calf_mirror.dae / foot.dae` | 无任何 xacro 引用（死文件） | 可选备用 |
| `Vlp16.dae / realsense.dae / hokuyo.dae` | 已删除的 VLP16 / D435i / 备用雷达 | 不拷 |

### 4.7 手写 `launch/display.launch.py`（纯静态预览，不碰 Gazebo）

> `go2_description` 只管“模型长什么样”，进 Gazebo 验证归 bringup（P2），
> 两边不重叠。原 `description.launch.py` 不可复用（旧 topic 名、老 xacro 入口），按下文重写。
> 不写 rviz 配置文件：RViz 打开后手动 Add → RobotModel，Fixed Frame 选 `base_link`。
> Fixed Frame 不要设成 `trunk`：全链路（EKF `base_link_frame`、
> Nav2 `robot_base_frame`、SLAM）用的都是 `base_link`。
> 零位下腿伸直、脚压到网格线下看着像“嵌进地里”是正常的；进仿真以 Gazebo 为准。

```python
# 看模型（静态）：只起 robot_state_publisher + 关节滑杆 + RViz，不碰 Gazebo
# 用法：ros2 launch go2_description display.launch.py
#       ros2 launch go2_description display.launch.py use_external_lidar:=true
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
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_desc}]),
        # 需 sudo apt install ros-humble-joint-state-publisher-gui
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
```

运行与验收：

```bash
cd ~/go2_l1_ws && colcon build --packages-select go2_description && source install/setup.bash
ros2 launch go2_description display.launch.py
# RViz：Fixed Frame 选 base_link，Add → RobotModel → 完整狗身；TF 里应有 trunk→lidar / base_link→imu_link，无 velodyne
xacro src/go2_description/xacro/robot.xacro robot_name:=robot1 -o /tmp/go2l1.urdf
check_urdf /tmp/go2l1.urdf
grep -c velodyne /tmp/go2l1.urdf   # 期望 0
grep -c 'name="lidar"' /tmp/go2l1.urdf  # 期望 ≥1
```

---

## 5. P2 Gazebo Bringup 极简版（1 天）

### 5.1 建包与复制

```bash
cd ~/go2_l1_ws/src
ros2 pkg create go2_gazebo_bringup --build-type ament_cmake --license BSD
mkdir -p go2_gazebo_bringup/{launch,config,worlds,rviz}
SRC=~/git/ros2-learning/ROS2-Gazebo-GO2/src/gazebo_sim
cp $SRC/config/ekf.yaml go2_gazebo_bringup/config/ekf.yaml
cp $SRC/config/gz_bridge.yaml go2_gazebo_bringup/config/gz_bridge.yaml
cp $SRC/world/warehouse.sdf go2_gazebo_bringup/worlds/warehouse.sdf
cp $SRC/rviz/go2_sensors.rviz go2_gazebo_bringup/rviz/go2_l1.rviz
# quadropted 整体复制（复用，不手写）：
cp -r ~/git/ros2-learning/ROS2-Gazebo-GO2/src/quadropted_controller ~/go2_l1_ws/src/
cp -r ~/git/ros2-learning/ROS2-Gazebo-GO2/src/quadropted_msgs ~/go2_l1_ws/src/
```

`CMakeLists.txt` 在生成文件基础上加（缺一不可，否则 launch/yaml 装不到 install 里）：

```cmake
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
```

`config/robots.yaml` 手写（只留 robot1）：

```yaml
robots:
  - name: robot1
    x_pose: '0.0'
    y_pose: '0.0'
    z_pose: '0.8'
```

### 5.2 `launch/l1.launch.py` 全文（单机器人，对照原 `gazebo_go2_sensors.launch.py:50-126`）

顺序不可乱：`robot_state_publisher → create → bridge → spawner → controller/odom/ekf`。

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _robot_nodes(context: LaunchContext, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ext = LaunchConfiguration('use_external_lidar').perform(context)
    ns = 'robot1'
    pkg = get_package_share_directory('go2_gazebo_bringup')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static'),
                  ('/scan', 'scan'), ('/odom', 'odometry/filtered')]

    desc_pkg = get_package_share_directory('go2_description')
    robot_desc = xacro.process_file(
        os.path.join(desc_pkg, 'xacro', 'robot.xacro'),
        mappings={'robot_name': ns,
                  'use_external_lidar': use_ext}).toxml()

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               namespace=ns, output='screen',
               parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}],
               remappings=remappings)

    spawn = Node(package='ros_gz_sim', executable='create', namespace=ns, output='screen',
                 arguments=['-topic', f'/{ns}/robot_description',
                            '-name', f'{ns}_my_bot', '-allow_renaming', 'true',
                            '-x', '0.0', '-y', '0.0', '-z', '0.8'])

    bridge_args = [
        f'/{ns}/imu_plugin/out@sensor_msgs/msg/Imu@gz.msgs.IMU',
        f'/{ns}/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        f'/{ns}/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        f'/{ns}/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        f'/{ns}/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model']
    if use_ext == 'true':
        # 外置雷达开启时才追加桥接，未开启时 Gazebo 侧无此话题，加了反而报错
        bridge_args += [
            f'/{ns}/velodyne@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            f'/{ns}/velodyne/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            f'/{ns}/velodyne_imu@sensor_msgs/msg/Imu@gz.msgs.IMU']
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  namespace=ns, output='screen', arguments=bridge_args)

    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                        arguments=['--ros-args', '-p',
                                   f'config_file:={os.path.join(pkg, "config", "gz_bridge.yaml")}'])

    # 两个 spawner 必须串行 + 超时加到 60 秒（血泪教训 2026-09-04：Gazebo 内
    # controller_manager 启动慢，默认 10 秒超时会导致第一次 load 客户端超时、
    # 服务端随后成功，重试撞上 already loaded 然后 FATAL，腿控制器永远 inactive、狗必翻）。
    cm = f'/{ns}/controller_manager'
    jsb = Node(package='controller_manager', executable='spawner',
               namespace=ns, output='screen',
               arguments=['joint_state_broadcaster',
                          '--controller-manager', cm,
                          '--controller-manager-timeout', '60'],
               remappings=remappings)
    jgc = Node(package='controller_manager', executable='spawner',
               namespace=ns, output='screen',
               arguments=['joint_group_controller',
                          '--controller-manager', cm,
                          '--controller-manager-timeout', '60'],
               remappings=remappings)
    # broadcaster 正常结束（exit 0）后才起 group controller
    jgc_after_jsb = RegisterEventHandler(OnProcessExit(
        target_action=jsb, on_exit=[jgc]))

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
    ekf = Node(package='robot_localization', executable='ekf_node',
               name='ekf_filter_node', namespace=ns, output='screen',
               parameters=[os.path.join(pkg, 'config', 'ekf.yaml'),
                           {'use_sim_time': use_sim_time}],
               remappings=remappings)

    return [clock_bridge, rsp, spawn, bridge, jsb, jgc_after_jsb,
            controller, cmd_vel_pub, odom, ekf]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_external_lidar', default_value='false',
                              description='是否加载外置 360° 激光雷达（默认 false，只用内置 L1）'),
        OpaqueFunction(function=_robot_nodes),
    ])
```

> 为什么包一层 `OpaqueFunction`：`xacro.process_file` 在 launch 求值时立即执行，`LaunchConfiguration` 必须 `perform` 成字符串才能进 `mappings`；同理 bridge 话题列表要按参数动态增减，也只能在函数体内组装。

### 5.3 `launch/launch.py` 全文（外层，对照原 `launch.py:50-96`）

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import SetParameter

def _gz(context, *a, **kw):
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
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_external_lidar', default_value='false',
                                        description='是否加载外置 360° 激光雷达，透传给 l1.launch.py'))
    ld.add_action(SetParameter(name='use_sim_time', value=use_sim_time))
    ld.add_action(DeclareLaunchArgument('world', default_value='warehouse.sdf'))
    ld.add_action(OpaqueFunction(function=_gz))
    pause = ExecuteProcess(cmd=['sleep', '6'], output='screen')
    ld.add_action(pause)
    ld.add_action(RegisterEventHandler(OnProcessExit(
        target_action=pause, on_exit=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('go2_gazebo_bringup'), 'launch', 'l1.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'use_external_lidar': use_ext}.items())])))
    return ld
```

用法：`ros2 launch go2_gazebo_bringup launch.py`（默认 L1）；`ros2 launch go2_gazebo_bringup launch.py use_external_lidar:=true`（加挂外置雷达）。

### 5.4 验收

```bash
ros2 launch go2_gazebo_bringup launch.py world:=warehouse.sdf
# 新终端：
ros2 topic hz /robot1/scan             # ~10Hz
ros2 topic echo /robot1/scan --once    # angle_min≈1.39 angle_max≈4.88
gz topic -l | grep robot1/scan
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel
ros2 service call /robot1/robot_behavior_command quadropted_msgs/srv/RobotBehaviorCommand "{command: 'walk'}"
```

---

## 6. P3 Odom + EKF（0.5 天，直接复制）

```bash
cp ~/git/ros2-learning/ROS2-Gazebo-GO2/src/gazebo_sim/config/ekf.yaml ~/go2_l1_ws/src/go2_gazebo_bringup/config/ekf.yaml
```

要点（原 `:74-162`）：`map_frame map / odom_frame odom / base_link_frame base_link / world_frame odom`；`odom0:=odom` 取位姿+线速度+ yaw 角速度；`imu0:=imu_plugin/out` 取姿态+角速度+线加速度（`relative:=true, remove_gravitational_acceleration:=true`）。namespace 下 remap 会把 `odom→odometry/filtered` 的输入输出接对，保持原 launch 的 `remappings` 写法即可。

验收：`ros2 topic hz /robot1/odometry/filtered`（30Hz）；`ros2 run tf2_ros tf2_echo odom base_link` 连续；静止 10s 漂移 <0.05m。

---

## 7. P4 slam_toolbox 建图（1–2 天，L1 适配核心）

### 7.0 建包

```bash
cd ~/go2_l1_ws/src
ros2 pkg create go2_slam --build-type ament_cmake --license BSD
mkdir -p go2_slam/{launch,config}
```

`CMakeLists.txt` 加：`install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})`（分两行写，`launch` 和 `config` 各一行）。

### 7.1 `launch/slam.launch.py` 全文（对照原 `slam_toolbox.launch.py:26-46` 改）

```python
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
```

### 7.2 `config/mapper_params_l1.yaml`（从 `/opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml` 复制后改）

```yaml
slam_toolbox:
  ros__parameters:
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: mapping
    map_file_name: ""
    use_sim_time: true
    # L1 相关：
    max_laser_range: 10.0        # 呼应原 carto lua max_range 10.0，过大窄 FOV 易发散
    minimum_travel_distance: 0.1 # 狗晃动大，稍放宽
    minimum_travel_heading: 0.1
    scan_buffer_size: 30
    link_match_minimum_response_fine: 0.1
    loop_search_maximum_distance: 4.0
    do_loop_closing: true
    resolution: 0.05
    max_mapping_time: -1.0
```

### 7.3 建图流程

```bash
ros2 launch go2_gazebo_bringup launch.py world:=warehouse.sdf
ros2 launch go2_slam slam.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel
# RViz：Fixed Frame=map，Add Map(/map)+LaserScan(/robot1/scan)+TF
# 慢速（线速度<0.2）多转弯走一圈，覆盖走廊两侧以补偿后方盲区
ros2 run nav2_map_server map_saver_cli -t map -f src/go2_nav/maps/warehouse_l1
```

验收：`warehouse_l1.pgm/yaml` 生成，墙面闭合、无大面积重影；`ros2 topic hz /map` ~1Hz。

---

## 8. P5 Nav2 导航（1–2 天）

### 8.0 建包

```bash
cd ~/go2_l1_ws/src
ros2 pkg create go2_nav --build-type ament_cmake --license BSD
mkdir -p go2_nav/{launch,param,maps,rviz}
```

`CMakeLists.txt` 加：`install(DIRECTORY launch param maps rviz DESTINATION share/${PROJECT_NAME})`（逐目录 install）。

### 8.1 `param/nav2_l1.yaml`（复制原 `navigation2/param/go2_nav2.yaml` 全文，只改 4 处）

```bash
cp ~/git/ros2-learning/ROS2-Gazebo-GO2/src/navigation2/param/go2_nav2.yaml ~/go2_l1_ws/src/go2_nav/param/nav2_l1.yaml
sed -i 's#scan_topic: velodyne#scan_topic: scan#; s#topic: /velodyne#topic: /scan#g' ~/go2_l1_ws/src/go2_nav/param/nav2_l1.yaml
```

改后核对：`amcl.scan_topic: scan`；`local_costmap.voxel_layer.scan.topic: /scan`；`global_costmap.obstacle_layer.scan.topic: /scan`。另建议 `amcl.laser_max_range: 100.0 → 10.0`（与建图一致），其余 `robot_radius 0.22 / inflation_radius 0.55 / FollowPath max_vel_x 0.26 / max_vel_theta 1.0` 保留（狗步态限制）。

### 8.2 `launch/nav2.launch.py` 全文（对照原 `navigation2/launch/go2_navigation2.launch.py:68-83`）

```python
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
```

> `rviz/go2_nav.rviz` 先从原 `navigation2/rviz/go2_nav2.rviz` 复制改名，后续按 L1 视角另存。

### 8.3 导航流程

```bash
ros2 launch go2_gazebo_bringup launch.py world:=warehouse.sdf
ros2 launch go2_nav nav2.launch.py map:=$(pwd)/src/go2_nav/maps/warehouse_l1.yaml
# RViz：2D Pose Estimate 粗定位 → Nav2 Goal 发目标 → 观察 global/local costmap + plan
```

验收：目标容差 `xy 0.25 / yaw 0.25` 内到达；local costmap 障碍随 `/scan` 实时刷新；`ros2 topic echo /robot1/cmd_vel` 在导航时有速度输出。

---

## 9. L1 专项坑与对策

1. **后方盲区**：L1 前 200°，倒车/侧移时 costmap 无观测。建图多绕圈，导航避免窄道掉头，`inflation_radius` 不宜 <0.5。
2. **range 过大发散**：Gazebo L1 `max 131m` 穿墙，SLAM/Nav2 一律按 10m 截断（`mapper max_laser_range / lua max_range / amcl laser_max_range`）。
3. **狗晃动**：四足步态 pitch/roll 抖，`minimum_travel_distance/heading` 放宽，EKF 保持 `two_d_mode:=true`。
4. **namespace remap 漏配**：`/tf /tf_static /scan /odom` 任一漏 remap 会出现两棵 TF 树或 SLAM 收不到 scan，逐个 `ros2 topic info` 核对。
5. **时间不同步**：全链路 `use_sim_time:=true`，`gz_bridge clock` 必须启动，否则 `tf lookup timeout 0.2s` 刷屏。
6. **外置雷达默认关闭**：`grep -rn velodyne ~/go2_l1_ws/src/ --include=*.xacro` 应只出现在 `lidar_external.xacro`（宏定义，不默认展开）；默认启动后 `gz topic -l` 无 velodyne、`ros2 topic list` 无 `/robot1/velodyne`；`use_external_lidar:=true` 后三者才出现。

---

## 10. 总验收清单

- [ ] `/robot1/scan angle_min≈1.39 angle_max≈4.88 @10Hz`
- [ ] `teleop` 狗可行走，`odometry/filtered @30Hz`
- [ ] slam_toolbox 输出 `/map`，`map_saver` 存图成功
- [ ] Nav2 用 L1 图完成任意两点导航，到达误差 <0.25m
- [ ] 默认构建中外置雷达不展开（见第 6 条），`use_external_lidar:=true` 可加挂

按 P0→P5 顺序逐段验收后再往下走，卡住时先查 `ros2 topic list | grep robot1` 和 `ros2 run tf2_tools view_frames` 两条命令 RFB。
