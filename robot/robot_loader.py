#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
机器人加载模块

提供统一的机器人模型加载接口，支持：
- URDF 格式
- MJCF (MuJoCo XML) 格式
"""

import os

from urdf_parser import URDFParser
from mjcf_parser import MJCFParser


class RobotLoader:
    """统一的机器人模型加载器"""

    @staticmethod
    def detect_format(filepath):
        """检测文件格式

        Args:
            filepath: 文件路径

        Returns:
            'urdf', 'mjcf', 或 None (未知格式)
        """
        if not filepath or not os.path.exists(filepath):
            return None

        ext = os.path.splitext(filepath)[1].lower()

        if ext == '.urdf':
            return 'urdf'
        elif ext == '.xml':
            # 检查是否是有效的 MJCF 文件
            if MJCFParser.is_valid_mjcf(filepath):
                return 'mjcf'
            return None
        else:
            return None

    @staticmethod
    def create_parser(filepath):
        """根据文件格式创建解析器

        Args:
            filepath: 文件路径

        Returns:
            URDFParser 或 MJCFParser 实例，如果格式未知则返回 None
        """
        fmt = RobotLoader.detect_format(filepath)

        if fmt == 'urdf':
            return URDFParser(filepath)
        elif fmt == 'mjcf':
            return MJCFParser(filepath)
        else:
            return None

    @staticmethod
    def load(filepath):
        """加载机器人模型

        Args:
            filepath: 文件路径

        Returns:
            dict 包含:
                - 'parser': 解析器实例
                - 'format': 文件格式 ('urdf' 或 'mjcf')
                - 'robot_info': get_robot_info() 的结果
                - 'chains': 运动链信息 (仅 URDF)
                - 'visual_geometries': 视觉几何体信息 (仅 MJCF)
            如果加载失败则返回 None
        """
        fmt = RobotLoader.detect_format(filepath)
        if fmt is None:
            return None

        try:
            parser = RobotLoader.create_parser(filepath)
            if parser is None:
                return None

            robot_info = parser.get_robot_info()

            result = {
                'parser': parser,
                'format': fmt,
                'robot_info': robot_info,
                'chains': [],
                'visual_geometries': [],
            }

            # URDF 特有信息
            if fmt == 'urdf':
                chains, _ = parser.get_chain_info()
                result['chains'] = chains

            # MJCF 特有信息
            if fmt == 'mjcf':
                result['visual_geometries'] = parser.get_visual_geometries()

            return result

        except Exception as e:
            print(f"Error loading robot from {filepath}: {e}")
            return None

    @staticmethod
    def get_revolute_joints(robot_info):
        """从 robot_info 提取 revolute 关节信息

        Args:
            robot_info: get_robot_info() 的返回值

        Returns:
            [{'name', 'index', 'parent', 'child', 'axis', 'lower', 'upper'}, ...] 列表
        """
        (_, _, _, _, _, joint_names, _, joint_types, joint_axes,
         joint_parent_links, joint_child_links, joint_limits,
         _, _, _, _) = robot_info

        revolute_joints = []
        for i, joint_type in enumerate(joint_types):
            if joint_type == 'revolute':
                revolute_joints.append({
                    'name': joint_names[i],
                    'index': i,
                    'parent': joint_parent_links[i],
                    'child': joint_child_links[i],
                    'axis': joint_axes[i],
                    'lower': joint_limits[i]['lower'],
                    'upper': joint_limits[i]['upper']
                })

        return revolute_joints

    @staticmethod
    def get_link_names(robot_info):
        """从 robot_info 提取连杆名称

        Args:
            robot_info: get_robot_info() 的返回值

        Returns:
            连杆名称列表
        """
        return robot_info[0]

    @staticmethod
    def get_joint_names(robot_info):
        """从 robot_info 提取关节名称

        Args:
            robot_info: get_robot_info() 的返回值

        Returns:
            关节名称列表
        """
        return robot_info[5]
