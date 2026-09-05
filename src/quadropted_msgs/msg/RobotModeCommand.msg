# RobotModeCommand.msg
# 机器人工作模式说明

# 可选的工作模式
string mode  # 工作模式: REST, TROT, CRAWL, STAND 等

# 可选: 附加参数
uint16 robot_id  # 机器人标识符 (多机器人时使用)