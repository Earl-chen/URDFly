#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
坐标系管理模块

提供各种坐标系的可视化功能，包括：
- 连杆坐标系 (Link Frames)
- MDH 坐标系 (MDH Frames)
- 关节轴 (Joint Axes)

参考 robot_viewer/CoordinateAxesManager.js
"""

import math
import numpy as np
import vtk

from .geometry_factory import GeometryFactory


class AxesManager:
    """坐标系管理器"""

    def __init__(self, renderer):
        """初始化坐标系管理器

        Args:
            renderer: VTK renderer
        """
        self.renderer = renderer

        # MDH 坐标系
        self.mdh_frames_actors = []
        self.mdh_text_actors = []

        # 关节轴
        self.joint_axis_actors = []
        self.joint_axis_info = []  # {'actor', 'joint_name', 'axis'}

        # 可见性状态
        self.show_mdh_frames = False
        self.show_joint_axes = False

    def clear(self):
        """清除所有坐标系可视化元素"""
        # 清除 MDH 坐标系
        for actor in self.mdh_frames_actors:
            self.renderer.RemoveActor(actor)
        self.mdh_frames_actors = []

        for text_actor in self.mdh_text_actors:
            self.renderer.RemoveActor(text_actor)
        self.mdh_text_actors = []

        # 清除关节轴
        for actor in self.joint_axis_actors:
            self.renderer.RemoveActor(actor)
        self.joint_axis_actors = []
        self.joint_axis_info = []

    def create_mdh_frames(self, mdh_frames, visible=True):
        """创建 MDH 坐标系可视化

        Args:
            mdh_frames: MDH 坐标系变换矩阵列表
            visible: 初始可见性
        """
        # 清除现有 MDH 坐标系
        for actor in self.mdh_frames_actors:
            self.renderer.RemoveActor(actor)
        self.mdh_frames_actors = []

        for text_actor in self.mdh_text_actors:
            self.renderer.RemoveActor(text_actor)
        self.mdh_text_actors = []

        # 创建每个 MDH 坐标系
        for i, frame in enumerate(mdh_frames):
            # 创建坐标轴
            axes = GeometryFactory.create_axes(
                length=0.05,
                shaft_type=0,
                show_labels=False,
                cylinder_radius=0.01
            )

            # 应用变换
            vtk_transform = vtk.vtkTransform()
            vtk_transform.SetMatrix(frame.flatten())
            axes.SetUserTransform(vtk_transform)

            # 添加到渲染器
            self.renderer.AddActor(axes)
            self.mdh_frames_actors.append(axes)
            axes.SetVisibility(visible)

            # 创建文本标签
            z_endpoint = [0, 0, 0.05, 1]
            transformed_point = vtk_transform.TransformPoint(
                z_endpoint[0], z_endpoint[1], z_endpoint[2]
            )

            text_actor = GeometryFactory.create_text_caption(
                f"MDH{i}",
                transformed_point,
                color=(0, 0, 1),  # 蓝色
                font_size=14
            )

            self.renderer.AddActor(text_actor)
            self.mdh_text_actors.append(text_actor)
            text_actor.SetVisibility(visible)

        self.show_mdh_frames = visible

    def create_joint_axes(self, joint_names, joint_frames, joint_types, joint_axes, visible=True):
        """创建关节轴可视化

        Args:
            joint_names: 关节名称列表
            joint_frames: 关节坐标系变换矩阵列表
            joint_types: 关节类型列表
            joint_axes: 关节轴方向列表
            visible: 初始可见性
        """
        # 清除现有关节轴
        for actor in self.joint_axis_actors:
            self.renderer.RemoveActor(actor)
        self.joint_axis_actors = []
        self.joint_axis_info = []

        for joint_name, joint_frame, joint_type, axis in zip(
            joint_names, joint_frames, joint_types, joint_axes
        ):
            # 转换关节名称为字符串
            joint_name_str = str(joint_name) if hasattr(joint_name, '__iter__') and not isinstance(joint_name, str) else joint_name

            # 只为 revolute 和 continuous 关节创建轴
            if joint_type not in ['revolute', 'continuous']:
                continue

            # 创建关节轴圆柱体
            actor = GeometryFactory.create_joint_axis_cylinder(
                joint_frame, axis,
                axis_length=0.3,
                radius=0.01,
                color=(1, 0, 1)  # 洋红色
            )
            actor.SetVisibility(visible)

            self.renderer.AddActor(actor)
            self.joint_axis_actors.append(actor)
            self.joint_axis_info.append({
                'actor': actor,
                'joint_name': joint_name_str,
                'axis': list(axis)
            })

        self.show_joint_axes = visible

    def update_joint_axes_transforms(self, joint_names, joint_frames, joint_axes):
        """更新关节轴的变换

        Args:
            joint_names: 关节名称列表
            joint_frames: 关节坐标系变换矩阵列表
            joint_axes: 关节轴方向列表
        """
        # 构建关节名称到坐标系的映射
        joint_name_to_frame = {name: frame for name, frame in zip(joint_names, joint_frames)}
        joint_name_to_axis = {name: axis for name, axis in zip(joint_names, joint_axes)}

        for info in self.joint_axis_info:
            actor = info['actor']
            joint_name = info['joint_name']
            axis = info['axis']

            if joint_name in joint_name_to_frame:
                joint_frame = joint_name_to_frame[joint_name]

                # 重新创建变换
                vtk_transform = vtk.vtkTransform()

                # 提取关节位置
                joint_position = joint_frame[:3, 3]

                # 将轴从局部坐标系变换到世界坐标系
                R_joint = joint_frame[:3, :3]
                joint_axis_local = np.array(axis)
                joint_axis_local = joint_axis_local / (np.linalg.norm(joint_axis_local) + 1e-10)
                joint_axis_world = R_joint @ joint_axis_local

                # 定位到关节原点
                vtk_transform.Translate(joint_position[0], joint_position[1], joint_position[2])

                # 将圆柱体（默认 Y 轴）旋转到关节轴方向
                y_axis = np.array([0, 1, 0])

                if not np.allclose(y_axis, joint_axis_world, atol=0.01):
                    cross = np.cross(y_axis, joint_axis_world)
                    dot = np.dot(y_axis, joint_axis_world)
                    angle = math.acos(np.clip(dot, -1, 1))
                    if np.linalg.norm(cross) > 1e-6:
                        axis_rot = cross / np.linalg.norm(cross)
                        vtk_transform.RotateWXYZ(
                            math.degrees(angle),
                            axis_rot[0], axis_rot[1], axis_rot[2]
                        )

                actor.SetUserTransform(vtk_transform)

    def set_mdh_frames_visibility(self, visible):
        """设置 MDH 坐标系可见性

        Args:
            visible: 是否可见
        """
        self.show_mdh_frames = visible
        for actor in self.mdh_frames_actors:
            actor.SetVisibility(visible)
        for text_actor in self.mdh_text_actors:
            text_actor.SetVisibility(visible)

    def set_joint_axes_visibility(self, visible):
        """设置关节轴可见性

        Args:
            visible: 是否可见
        """
        self.show_joint_axes = visible
        for actor in self.joint_axis_actors:
            actor.SetVisibility(visible)

    def has_joint_axes(self):
        """检查是否已创建关节轴"""
        return len(self.joint_axis_actors) > 0

    def has_mdh_frames(self):
        """检查是否已创建 MDH 坐标系"""
        return len(self.mdh_frames_actors) > 0
