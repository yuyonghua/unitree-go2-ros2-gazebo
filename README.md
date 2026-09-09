# Unitree GO2 前置 L1 激光雷达：Gazebo 仿真 + 建图 + 导航

用宇树 GO2 四足机器人**前置 L1 激光雷达**（`/robot1/scan`，前方 ~200°，后方盲区），在 Gazebo 里完成
`仿真 → slam_toolbox 建图 → Nav2 导航` 全流程。ROS 2 Humble，命名空间 `robot1`，DDS `rmw_cyclonedds_cpp`，
全链路 `use_sim_time:=true`。

参考项目（只读）：`/home/user/git/ros2-learning/ROS2-Gazebo-GO2`（原项目建图导航实际用的是外置 360° VLP16，
本项目切到内置 L1）。手写复现过程见 `GO2-L1-复现指南.md`（最详细），给 AI 用的速查见 `AGENTS.md`。

## 包一览（`src/`）

| 包 | 作用 | 说明 |
|---|---|---|
| `go2_description` | 狗本体模型（xacro + meshes），**只管长相、不碰 Gazebo** | 入口 `xacro/robot.xacro`；静态预览 `launch/display.launch.py` |
| `go2_gazebo_bringup` | 仿真总入口：Gazebo 世界 + spawn + 桥接 + 控制器 + 里程计 + EKF + 可选 RViz | 入口 `launch/launch.py` |
| `go2_slam` | slam_toolbox 建图 | `launch/slam.launch.py` + `config/mapper_params_l1.yaml` |
| `go2_nav` | Nav2 导航 | `launch/nav2.launch.py` + `param/nav2_l1.yaml` + `maps/` |
| `quadropted_controller` / `quadropted_msgs` | 步态控制、cmd_vel 转发、odom 推算、自定义消息 | **整体复用，不重写** |

## 环境准备（每个终端都要做）

```bash
source env.sh
```

`env.sh` 干了四件事：source ROS 2 + 本工作空间；`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`；
`CYCLONEDDS_URI` 指向 `src/cyclonedds.xml`；`GZ_SIM_RESOURCE_PATH` 追加本地 `worlds/` + `models/`
（否则首次启动会卡在 Fuel 模型下载）。**所有终端必须 source 同一个 `env.sh`**，否则 DDS
实现不一致会导致 bridge 收不到数据。

系统依赖（按需补装）：

```bash
sudo apt install -y ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-image \
  ros-humble-gz-ros2-control ros-humble-controller-manager ros-humble-ros2-controllers \
  ros-humble-robot-localization ros-humble-slam-toolbox ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui
```

## 构建

```bash
colcon build --packages-select go2_description go2_gazebo_bringup go2_slam go2_nav \
  quadropted_controller quadropted_msgs \
  && source install/setup.bash
```

注意：`install/` 在删除文件后会残留旧产物，行为异常时手动删对应子目录再重编。

## P1 静态看模型（不碰 Gazebo）

```bash
ros2 launch go2_description display.launch.py
# 可选参数：use_external_lidar:=true（预览外置 360° 雷达，默认 false 只看内置 L1）
```

RViz 里手动 Add → RobotModel，Fixed Frame 选 `base_link`（全链路统一用它，不要选 `velodyne`）。
零位下腿伸直、脚压到网格线下看着像"嵌进地里"是正常的，进仿真以 Gazebo 为准。

结构自查：

```bash
xacro src/go2_description/xacro/robot.xacro robot_name:=robot1 -o /tmp/go2l1.urdf && check_urdf /tmp/go2l1.urdf
grep -c velodyne /tmp/go2l1.urdf   # 期望 0（默认 L1-only）
```

## P2 仿真（Gazebo + 狗 + RViz 可选）

```bash
ros2 launch go2_gazebo_bringup launch.py                           # 默认 warehouse.sdf，只用 L1
ros2 launch go2_gazebo_bringup launch.py rviz:=true                # 顺手开 RViz（配置文件自动选）
ros2 launch go2_gazebo_bringup launch.py world:=rmuc_2025_world.sdf
ros2 launch go2_gazebo_bringup launch.py use_external_lidar:=true  # 加挂外置 360° 雷达
ros2 launch go2_gazebo_bringup launch.py x:=1.0 y:=2.0 z:=1.0 yaw:=1.57  # 指定出生位姿
```

### `launch.py` 参数表

| 参数 | 默认值 | 说明 |
|---|---|---|
| `world` | `warehouse.sdf` | `worlds/` 下的世界文件名（`cafe.world` / `rmuc_2025_world.sdf` / `warehouse.sdf`） |
| `use_sim_time` | `true` | 全链路仿真时间，不要改 |
| `use_external_lidar` | `false` | `true` 时加载外置 360° 雷达（URDF 展开 + bridge 追加 `velodyne` 三个话题 + RViz 用 external 版配置） |
| `rviz` | `false` | `true` 时启动 RViz；配置文件按 `use_external_lidar` 自动选 `rviz/go2_l1.rviz` 或 `rviz/go2_l1_external.rviz`（Fixed Frame=`odom`，P2 还没有 map） |
| `x` / `y` / `z` | `0.0` / `0.0` / `0.8` | 出生点位姿（米）。**z 宁高勿低**：狗是掉下去再站起来的，给小了会卡进地里；0.8 三个 world 通用，rmuc 场地表面偏高可给到 1.0 |
| `yaw` | `0.0` | 出生朝向（弧度） |

启动顺序（`launch.py` → 6 秒 → `l1.launch.py`）：Gazebo 世界 → rsp → spawn →
bridge（含全局时钟桥）→ spawner（broadcaster 先、group controller 后，延迟串行）→
步态控制/cmd_vel转发/odom → EKF。控制器全部 `active` 约需 26 秒，可用下面命令确认：

```bash
ros2 service call /robot1/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers
# joint_state_broadcaster + joint_group_controller 都要是 active
ros2 topic hz /robot1/scan               # ~10Hz，angle_min≈1.39 angle_max≈4.88（前方~200°）
ros2 topic hz /robot1/joint_states       # ~90Hz
ros2 topic hz /robot1/odometry/filtered  # ~30Hz（EKF 输出，SLAM/Nav2 吃这个）
```

### 让狗动起来

狗初始是 `REST` 状态（腿不发力，掉下来正常），先站再走：

```bash
ros2 service call /robot1/robot_behavior_command quadropted_msgs/srv/RobotBehaviorCommand "{command: 'up'}"    # 站起来
ros2 service call /robot1/robot_behavior_command quadropted_msgs/srv/RobotBehaviorCommand "{command: 'walk'}"  # 走（sit/up/walk 三选一）
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel   # 键盘给速度
```

## P4 建图（slam_toolbox，L1 适配核心）

```bash
# 终端1：仿真（RViz 可开可不开）
ros2 launch go2_gazebo_bringup launch.py world:=warehouse.sdf
# 终端2：建图
ros2 launch go2_slam slam.launch.py
# 终端3：遥控（先发 walk，见上节）
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel
```

`slam.launch.py` 参数：只有 `use_sim_time`（默认 `true`）。关键 L1 适配在
`config/mapper_params_l1.yaml`：`max_laser_range: 10.0`（Gazebo L1 最远 131m 且穿墙，
不截断会发散）、`minimum_travel_distance/heading: 0.1`（狗晃动大放宽）、`resolution: 0.05`。

走法：**线速度 <0.2，路口/转角处原地转一圈**（补后方盲区 + 给回环机会）。RViz（Fixed Frame=`map`）
看 scan 点云和墙面是否贴合；糊了就地慢转等回环拉回。存图：

```bash
ros2 run nav2_map_server map_saver_cli -t map -f src/go2_nav/maps/warehouse_l1
# 得到 warehouse_l1.pgm/.yaml；验收：墙面闭合、无大面积重影，ros2 topic hz /map ~1Hz
```

## P5 导航（Nav2 + amcl + 静态图）

```bash
# 终端1：仿真（不要开 slam！map→odom 只能有一个发布者，slam 和 amcl 不能同开）
ros2 launch go2_gazebo_bringup launch.py world:=warehouse.sdf
# 终端2：导航（自带 RViz，Fixed Frame=map）
ros2 launch go2_nav nav2.launch.py map:=$(pwd)/src/go2_nav/maps/warehouse_l1.yaml
```

`nav2.launch.py` 参数表：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `map` | 包内 `maps/warehouse_l1.yaml` | 地图 yaml 路径（用绝对路径或 `$(pwd)/src/...` 指向刚建的图） |
| `params_file` | 包内 `param/nav2_l1.yaml` | Nav2 全参数 |
| `use_sim_time` | `true` | 不要改 |

`nav2_l1.yaml` 相对原版只改 L1 相关：三处 scan 话题（amcl + 两处 costmap）用**相对名** `scan`
（随 `robot1` 命名空间解析到 `/robot1/scan`；写绝对 `/scan` 会订阅空话题，costmap 全空），
`laser_max_range` 截到 10m，`inflation_radius: 0.55`（别 <0.5，后方盲区），其余步态限制保留
（`robot_radius 0.22`、`max_vel_x 0.26`、`max_vel_theta 1.0`）。

流程：RViz 里 `2D Pose Estimate` 粗定位（刚加载地图是斜的/偏的属正常，点一下对齐）→ `Nav2 Goal`
发目标 → 看 global/local costmap + plan。RViz Displays 里 `Controller → Local Costmap`
就是狗周围那圈粉红膨胀圈。验收：xy/yaw 0.25 容差内到达，`/robot1/cmd_vel` 导航时有速度输出。

## 话题 / TF 一览（只留 L1）

```text
/robot1/scan                LaserScan    L1 建图导航唯一输入（+ /scan/points 可视化）
/robot1/imu_plugin/out      Imu          EKF 用
/robot1/joint_states        JointState   broadcaster 发，rsp 解 TF 用
/robot1/odometry/filtered   Odometry     EKF 输出（30Hz），SLAM/Nav2 的 odom 输入
/robot1/cmd_vel             Twist        teleop 输入
/robot1/robot_behavior_command           sit/up/walk 服务
/map                        OccupancyGrid（SLAM 输出 / Nav2 输入）
TF: map → odom（slam_toolbox 或 amcl，二选一）→ base_link（EKF）→ trunk → lidar / imu_link
```

## 排错（都是实测踩过的）

- **所有节点起不来、报 `can't open configuration file`**：`env.sh` 没 source，或各终端 DDS 不一致。每个终端 `source env.sh`。
- **Gazebo 有世界没狗**：spawn `-topic` 必须等于 `/robot1/robot_description`（rsp 带命名空间）。
- **spawner `already loaded` FATAL / 腿 TF 缺 / 狗翻**：Gazebo 内 controller_manager 启动慢
  （约 17 秒才响应 load），且 Humble spawner 的 `load` 步是硬编码 10 秒超时。launch 里已用
  `TimerAction(20s)` 延迟串行绕开；还死就把 20.0 调大。免重启救活：`list_controllers` 看谁是
  `unconfigured`，手动 `configure_controller` + `switch_controller`（`activate_asap: true`）。
- **RViz 里 robot model/TF 闪烁**：TF 一条边只能有一个发布者。`odom→base_link` 只留 EKF
  （odom 节点 `enable_odom_tf` 必须 false）；`map→odom` 建图和导航二选一，不许 slam+amcl 同开。
- **建图偏移**：`max_laser_range` 是否 10m（参数文件是否真加载了）；速度 <0.2；多原地旋转。
- **Local Costmap 全空、RViz 无粉红圈**：costmap 的 `scan.topic` 必须是相对名 `scan`。
- **RViz 手动启动别忘 remap**：`rviz2 --ros-args -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static`，
  Fixed Frame=`base_link`（P2）/`map`（SLAM/Nav2）。

## 给后来者（仓库约定）

- `go2_description` 只管模型，`go2_gazebo_bringup` 管一切仿真，`quadropted_*` 复用不重写，
  SLAM 只用 slam_toolbox。`go2_slam`/`go2_nav` 建图导航已就绪。
- 新 launch/xacro 代码重注释中文。`rviz/` 只允许 `go2_l1*.rviz`（bringup）和 `go2_nav.rviz`。
- 改文件必同步 `GO2-L1-复现指南.md`（贴全文）+ `AGENTS.md`（精简条目）。
- 一次只干一段；别无提示跑长仿真；杀仿真 `pkill -f '[g]z sim'`（括号防杀自己 shell）。
