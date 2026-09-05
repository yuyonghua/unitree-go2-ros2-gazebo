#!/usr/bin/env python3
# Author: lnotspotl, abutalipovvv
import numpy as np
import tf_transformations  # 使用 tf2 代替 tf
from .StateCommand import State, Command, BehaviorState
from .RestController import RestController
from .TrotGaitController import TrotGaitController
from .CrawlGaitController import CrawlGaitController
from .StandController import StandController
from geometry_msgs.msg import Twist
from quadropted_msgs.msg import RobotModeCommand, RobotVelocity
from quadropted_msgs.srv import RobotBehaviorCommand
class Robot:
    def __init__(self, node, body, legs, imu, robot_id):
        self.body = body
        self.legs = legs
        self.node = node  
        self.robot_id = robot_id  

        self.delta_x = self.body[0] * 0.5
        self.delta_y = self.body[1] * 0.5 + self.legs[1]
        self.x_shift_front = 0.02
        self.x_shift_back = -0.0
        self.default_height = 0.25

        self.trotGaitController = TrotGaitController(
            self.node, self.default_stance,
            stance_time=0.04, swing_time=0.18, time_step=0.02,
            use_imu=imu
        )

        self.crawlGaitController = CrawlGaitController(
            self.default_stance,
            stance_time=0.55, swing_time=0.45, time_step=0.02
        )

        self.standController = StandController(self.node, self.default_stance)  # 把 node 传给控制器
        self.restController = RestController(self.default_stance)
        self.currentController = self.restController

        self.state = State(self.default_height)
        self.state.foot_locations = self.default_stance
        self.command = Command(self.default_height)

        # 将 TROT 模式设为默认
        self.command.trot_event = True
        self.command.rest_event = False
        self.command.crawl_event = False
        self.command.stand_event = False

        # 切换至 TROT
        self.change_controller()
        
        self.node.create_subscription(RobotModeCommand, 'robot_mode', self.mode_callback, 10)
        self.node.create_subscription(RobotVelocity, 'robot_velocity', self.velocity_callback, 10)

        self.behavior_service = self.node.create_service(
            RobotBehaviorCommand,
            'robot_behavior_command',
            self.handle_behavior_command
        )

    def mode_callback(self, msg):
        
        if msg.robot_id == self.robot_id:
            if self.node.verbose:
                self.node.get_logger().info(f"Received mode command: {msg.mode} for robot_id: {msg.robot_id}")
            if msg.mode == "REST":
                self.command.rest_event = True
                self.command.trot_event = False
                self.command.crawl_event = False
                self.command.stand_event = False
            elif msg.mode == "TROT":
                self.command.rest_event = False
                self.command.trot_event = True
                self.command.crawl_event = False
                self.command.stand_event = False
            elif msg.mode == "CRAWL":
                self.command.rest_event = False
                self.command.trot_event = False
                self.command.crawl_event = True
                self.command.stand_event = False
            elif msg.mode == "STAND":
                self.command.rest_event = False
                self.command.trot_event = False
                self.command.crawl_event = False
                self.command.stand_event = True
            
            self.change_controller()

    def velocity_callback(self, msg):
        
        if msg.robot_id == self.robot_id:
            self.command.velocity = np.array([
                msg.cmd_vel.linear.x,
                msg.cmd_vel.linear.y,
                msg.cmd_vel.linear.z
            ])  #  [x, y, z]
            
            self.command.yaw_rate = np.array([
                msg.cmd_vel.angular.x,
                msg.cmd_vel.angular.y,
                msg.cmd_vel.angular.z
            ])  # numpy  [roll, pitch, yaw]
            
            if self.node.verbose:
                self.node.get_logger().info(
                    f"Velocity command updated: linear={self.command.velocity}, angular={self.command.yaw_rate}"
                )

    def handle_behavior_command(self, request, response):
        command = request.command.lower()
        self.node.get_logger().info(f"收到行为指令: {command}")

        if command == 'sit':
            self.command.stand_event = True
            self.command.rest_event = False
            self.command.trot_event = False
            self.command.crawl_event = False

            self.change_controller()
            self.state.body_local_position[2] = -0.15

            response.success = True
            response.message = "机器人坐了下来。"
        
        elif command == 'up':
            self.command.rest_event = True
            self.command.stand_event = False
            self.command.trot_event = False
            self.command.crawl_event = False

            self.change_controller()
            self.state.body_local_position[2] = 0.0

            response.success = True
            response.message = "机器人站了起来。"
        
        elif command == 'walk':
            
            self.command.rest_event = True
            self.command.trot_event = True
            self.command.stand_event = False
            self.command.crawl_event = False

            self.change_controller()
            self.state.body_local_position[2] = 0.0

            response.success = True
            response.message = "机器人开始行走。"
        
        else:
            response.success = False
            response.message = f"未知命令: {command}"

        return response

    def change_controller(self):
        if self.command.trot_event and self.command.rest_event:
            # 'walk' REST, to TROT
            self.state.behavior_state = BehaviorState.REST
            self.currentController = self.restController
            self.currentController.pid_controller.reset()
            self.command.rest_event = False
            self.node.get_logger().info("切换到 REST 控制器")
            
            #  TROT
            self.state.behavior_state = BehaviorState.TROT
            self.currentController = self.trotGaitController
            self.currentController.pid_controller.reset()
            self.state.ticks = 0
            self.node.get_logger().info("切换到 TROT 控制器")
            self.command.trot_event = False

        elif self.command.trot_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.TROT
                self.currentController = self.trotGaitController
                self.currentController.pid_controller.reset()
                self.state.ticks = 0
            self.command.trot_event = False
            self.node.get_logger().info("切换到 TROT 控制器")
        
        elif self.command.crawl_event:
            if self.state.behavior_state == BehaviorState.REST:
                self.state.behavior_state = BehaviorState.CRAWL
                self.currentController = self.crawlGaitController
                self.currentController.first_cycle = True
                self.state.ticks = 0
            self.command.crawl_event = False
            self.node.get_logger().info("切换到 CRAWL 控制器")
        
        elif self.command.stand_event:
            if self.state.behavior_state != BehaviorState.STAND:
                self.state.behavior_state = BehaviorState.STAND
                self.currentController = self.standController
                self.state.body_local_position[2] = 0.005 
                self.node.get_logger().info("切换到 STAND 控制器")
            self.command.stand_event = False
        
        elif self.command.rest_event:
            self.state.behavior_state = BehaviorState.REST
            self.currentController = self.restController
            self.currentController.pid_controller.reset()
            self.command.rest_event = False
            self.node.get_logger().info("切换到 REST 控制器")


    def run(self):
        # 返回当前控制器的数据
        return self.currentController.run(self.state, self.command)

    @property
    def default_stance(self):
        # FR, FL, RR, RL
        return np.array([
            [self.delta_x + self.x_shift_front, self.delta_x + self.x_shift_front, -self.delta_x + self.x_shift_back, -self.delta_x + self.x_shift_back],
            [-self.delta_y, self.delta_y, -self.delta_y, self.delta_y],
            [0, 0, 0, 0]
        ])

