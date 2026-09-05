#!/usr/bin/env python3
import numpy as np
from math import sin, cos

class ForwardKinematics:
    def __init__(self, body_dimensions, leg_dimensions):
        """
        初始化机器人参数。
        :param body_dimensions: 身体尺寸 [长度, 宽度]
        :param leg_dimensions: 腿部尺寸 [l1, l2, l3, l4]
        """
        self.body_length = body_dimensions[0]
        self.body_width = body_dimensions[1]

        self.l1 = leg_dimensions[0]  # 躯干高度（从质心到腿部基座）
        self.l2 = leg_dimensions[1]  # 大腿长度
        self.l3 = leg_dimensions[2]  # 小腿长度
        self.l4 = leg_dimensions[3]  # 足端长度

    def homog_transform(self, dx, dy, dz, alpha, beta, gamma):
        """
        创建一个4x4的齐次变换矩阵。
        :param dx, dy, dz: 沿x、y、z轴的平移
        :param alpha, beta, gamma: 绕x、y、z轴的旋转角度（以弧度表示）
        :return: 变换矩阵 4x4
        """
        # 绕X轴的旋转
        rx = np.array([
            [1, 0, 0, 0],
            [0, cos(alpha), -sin(alpha), 0],
            [0, sin(alpha), cos(alpha), 0],
            [0, 0, 0, 1]
        ])

        # 绕Y轴的旋转
        ry = np.array([
            [cos(beta), 0, sin(beta), 0],
            [0, 1, 0, 0],
            [-sin(beta), 0, cos(beta), 0],
            [0, 0, 0, 1]
        ])

        # 绕Z轴的旋转
        rz = np.array([
            [cos(gamma), -sin(gamma), 0, 0],
            [sin(gamma), cos(gamma), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # 平移
        trans = np.array([
            [1, 0, 0, dx],
            [0, 1, 0, dy],
            [0, 0, 1, dz],
            [0, 0, 0, 1]
        ])

        # 最终变换矩阵：先旋转，后平移。
        return trans @ rz @ ry @ rx

    def forward_kinematics_per_leg(self, theta_hip, theta_thigh, theta_calf, leg_index):
        """
        计算基于关节角度的足端位置。
        :param theta_hip: hip_joint的角度（以弧度表示）
        :param theta_thigh: thigh_joint的角度（以弧度表示）
        :param theta_calf: calf_joint的角度（以弧度表示）
        :param leg_index: 腿部索引（0: FR, 1: FL, 2: RR, 3: RL）
        :return: 足端位置（x, y, z）相对于身体
        """
        # 确定每条腿基座连杆的位置。
        if leg_index == 0:  # FR
            base_x = self.body_length / 2
            base_y = self.body_width / 2
        elif leg_index == 1:  # FL
            base_x = self.body_length / 2
            base_y = -self.body_width / 2
        elif leg_index == 2:  # RR
            base_x = -self.body_length / 2
            base_y = self.body_width / 2
        elif leg_index == 3:  # RL
            base_x = -self.body_length / 2
            base_y = -self.body_width / 2
        else:
            raise ValueError("Invalid leg_index. Must be 0 (FR), 1 (FL), 2 (RR), or 3 (RL).")

        # 初始转换：调整腿部位置与躯干高度。
        T_base = self.homog_transform(base_x, base_y, -self.l1, 0, 0, 0)

        # 绕Z轴旋转hip_joint（外展/内收）
        T_hip_abd = self.homog_transform(0, 0, 0, 0, 0, theta_hip)

        # 绕 Y 轴（俯仰角）旋转大腿关节
        T_thigh_pitch = self.homog_transform(0, 0, 0, 0, theta_thigh, 0)

        # X 轴偏移量为大腿长度
        T_thigh = self.homog_transform(self.l2, 0, 0, 0, 0, 0)

        # 小腿关节绕 Y 轴旋转（俯仰）
        T_calf_pitch = self.homog_transform(0, 0, 0, 0, theta_calf, 0)

        # X 轴偏移量为小腿长度
        T_calf = self.homog_transform(self.l3, 0, 0, 0, 0, 0)

        # X 轴偏移量为足端长度
        T_foot = self.homog_transform(self.l4, 0, 0, 0, 0, 0)

        # 最终变换矩阵
        T_total = T_base @ T_hip_abd @ T_thigh_pitch @ T_thigh @ T_calf_pitch @ T_calf @ T_foot

        # 局部坐标系中的足端位置
        foot_position = T_total @ np.array([0, 0, 0, 1])

        return foot_position[:3]  # 仅返回 x、y、z

    def forward_kinematics_all_legs(self, joint_angles):
        """
            计算所有腿部的足端位置。 
            :param joint_angles: 12个关节角度的列表 [FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf,
            RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf]
            :return: 4个足端位置的列表 [(x_FR, y_FR, z_FR), ..., (x_RL, y_RL, z_RL)]
        """
        if len(joint_angles) != 12:
            raise ValueError("Expected 12 joint angles.")

        foot_positions = []
        for leg in range(4):
            idx = leg * 3
            theta_hip = joint_angles[idx]
            theta_thigh = joint_angles[idx + 1]
            theta_calf = joint_angles[idx + 2]

            foot_pos = self.forward_kinematics_per_leg(theta_hip, theta_thigh, theta_calf, leg)
            foot_positions.append(foot_pos)

        return foot_positions
