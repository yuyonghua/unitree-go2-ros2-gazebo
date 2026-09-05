# RobotGaitCommand.msg
# 机器人 ID
uint16 robot_id
# 步态类型
uint16 gait_type
# 机身高度
float32 body_height
# 步高
float32 leg_height


# RobotGaitConstants.msg
uint16 GAIT_TYPE_IDLE = 0
uint16 GAIT_TYPE_TROT = 1
uint16 GAIT_TYPE_TROT_RUN = 2
uint16 GAIT_TYPE_CLIMB_STAIR = 3
uint16 GAIT_TYPE_TROT_OBST = 4