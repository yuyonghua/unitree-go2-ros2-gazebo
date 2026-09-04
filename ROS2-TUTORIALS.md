# 1 launch
## 1.1 Node
package     # 功能包名
executable  # 节点的可执行文件名
namespace   # 节点所在命名空间
name        # 节点重命名
remappings  # 资源重映射列表
parameters  # 加载参数列表或文件
## 1.2 DeclareLaunchArgument    # 创建launch内参数
## 1.3 IncludeLaunchDescription # 包含另一个launch文件
## 1.4 GroupAction              # 对指定launch文件启动的功能附上操作
## 1.5 LaunchDescription        # 返回launch描述信息

# 2 URDF
## meshes   # 视觉模型
## dae      # 碰撞模型
## urdf     # 最终URDF文件
### link
刚体部分,描述连杆尺寸(size)、颜色(color)、形状(shape)、惯性矩阵(inertial matrix)、碰撞参数(collision properties)等。
### joint
link间的关节部分,分为六种类型:continuous(无限旋转)、revolute(限制角度旋转)、prismatic(滑动关节)、fixed(固定关节)、floating(浮动)、planar(平面)
### robot
完整机器人的最顶层标签
## xacro    # 机器人结构的模块化宏定义
### const.xacro
定义常量，例如：机器人尺寸、关节长度、材质参数、传感器位置
### materials.xacro
定义材质颜色，例如：trunk_color、leg_color、sensor_color
### leg.xacro
定义一条腿的结构：hip → thigh → calf → foot、关节类型（revolute）、关节轴、link 的 mesh 和 collision
### transmission.xacro
定义 ros2_control 的传动结构：joint hardware interface、controller mapping
### gazebo.xacro
定义 Gazebo 插件：ros2_control、IMU 插件、L1 雷达插件、camera 插件
### robot.xacro
主文件，组合所有部件：trunk、四条腿、传感器、gazebo 插件、常量、材质、传动



