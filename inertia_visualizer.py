#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
惯量可视化模块

提供质心(COM)和惯量盒的可视化功能，包括：
- 质心标记创建
- 惯量盒创建（支持非对角惯性张量的主轴分解）
- 特征值分解算法

参考 robot_viewer/InertialVisualization.js
"""

import math
import numpy as np
import vtk

from geometry_factory import GeometryFactory


class InertiaVisualizer:
    """惯量可视化管理器"""

    def __init__(self, renderer):
        """初始化惯量可视化器

        Args:
            renderer: VTK renderer
        """
        self.renderer = renderer

        # 质心标记
        self.com_actors = []
        self.com_actor_info = []  # {'actor', 'link_name', 'T_com_rel'}

        # 惯量盒
        self.inertia_actors = []
        self.inertia_actor_info = []  # {'actor', 'link_name', 'T_com_rel'}

        # 可见性状态
        self.show_com = False
        self.show_inertia = False

    def clear(self):
        """清除所有惯量可视化元素"""
        for actor in self.com_actors:
            self.renderer.RemoveActor(actor)
        self.com_actors = []
        self.com_actor_info = []

        for actor in self.inertia_actors:
            self.renderer.RemoveActor(actor)
        self.inertia_actors = []
        self.inertia_actor_info = []

    def create_com_markers(self, parser, link_names, link_frames, joint_values=None):
        """为所有连杆创建质心标记

        Args:
            parser: URDF 解析器
            link_names: 连杆名称列表
            link_frames: 连杆坐标系变换矩阵列表
            joint_values: 关节角度值（用于更新位置）
        """
        # 清除现有标记
        for actor in self.com_actors:
            self.renderer.RemoveActor(actor)
        self.com_actors = []
        self.com_actor_info = []

        # 跟踪已处理的连杆
        processed_links = set()

        for link_name, link_frame in zip(link_names, link_frames):
            if link_name in processed_links:
                continue
            processed_links.add(link_name)

            # 获取惯性数据
            link_info = parser.links.get(link_name)
            if link_info is None or link_info.get('inertial') is None:
                continue

            inertial = link_info['inertial']
            if inertial is None or inertial.get('mass') is None:
                continue

            # 获取质心相对于连杆坐标系的位置
            com_origin = inertial.get('origin', {})
            com_xyz = np.array(com_origin.get('xyz', [0, 0, 0]))
            com_rpy = np.array(com_origin.get('rpy', [0, 0, 0]))

            # 计算质心变换矩阵
            T_com = self.compute_transformation(com_rpy, com_xyz)
            T_world = link_frame @ T_com

            # 创建质心标记（双层球体）
            actor = GeometryFactory.create_com_marker_styled(T_world)
            actor.SetVisibility(self.show_com)

            self.renderer.AddActor(actor)
            self.com_actors.append(actor)
            self.com_actor_info.append({
                'actor': actor,
                'link_name': link_name,
                'T_com_rel': T_com
            })

    def create_inertia_boxes(self, parser, link_names, link_frames, joint_values=None):
        """为所有连杆创建惯量盒

        Args:
            parser: URDF 解析器
            link_names: 连杆名称列表
            link_frames: 连杆坐标系变换矩阵列表
            joint_values: 关节角度值（用于更新位置）
        """
        # 清除现有惯量盒
        for actor in self.inertia_actors:
            self.renderer.RemoveActor(actor)
        self.inertia_actors = []
        self.inertia_actor_info = []

        # 跟踪已处理的连杆
        processed_links = set()

        for link_name, link_frame in zip(link_names, link_frames):
            if link_name in processed_links:
                continue
            processed_links.add(link_name)

            # 获取惯性数据
            link_info = parser.links.get(link_name)
            if link_info is None or link_info.get('inertial') is None:
                continue

            inertial = link_info['inertial']
            if inertial is None or inertial.get('inertia') is None:
                continue

            inertia = inertial['inertia']
            mass = inertial.get('mass', 1.0)

            if mass <= 0:
                continue

            # 计算惯量盒参数
            box_data = self.compute_inertia_box(inertia, mass)
            if box_data is None:
                continue

            box_x = box_data['box_x']
            box_y = box_data['box_y']
            box_z = box_data['box_z']
            R_principal = box_data['R_principal']

            # 获取质心位置
            com_origin = inertial.get('origin', {})
            com_xyz = np.array(com_origin.get('xyz', [0, 0, 0]))
            com_rpy = np.array(com_origin.get('rpy', [0, 0, 0]))

            # 计算变换：link_frame @ T_com @ R_principal
            T_com = self.compute_transformation(com_rpy, com_xyz)

            # 构建主轴旋转的 4x4 齐次矩阵
            T_principal = np.eye(4)
            T_principal[:3, :3] = R_principal

            T_world = link_frame @ T_com @ T_principal

            # 创建惯量盒
            actor = GeometryFactory.create_inertia_box(
                (box_x, box_y, box_z), T_world
            )
            actor.SetVisibility(self.show_inertia)

            self.renderer.AddActor(actor)
            self.inertia_actors.append(actor)

            # 存储信息用于更新
            T_com_with_principal = T_com @ T_principal
            self.inertia_actor_info.append({
                'actor': actor,
                'link_name': link_name,
                'T_com_rel': T_com_with_principal
            })

    def update_transforms(self, link_names, link_frames):
        """更新所有惯量可视化元素的变换

        Args:
            link_names: 连杆名称列表
            link_frames: 连杆坐标系变换矩阵列表
        """
        # 构建连杆名称到坐标系的映射
        link_name_to_frame = {name: frame for name, frame in zip(link_names, link_frames)}

        # 更新质心标记
        for info in self.com_actor_info:
            actor = info['actor']
            link_name = info['link_name']
            T_com_rel = info['T_com_rel']

            if link_name in link_name_to_frame:
                link_frame = link_name_to_frame[link_name]
                T_world = link_frame @ T_com_rel

                vtk_transform = vtk.vtkTransform()
                vtk_transform.SetMatrix(T_world.flatten())
                actor.SetUserTransform(vtk_transform)

        # 更新惯量盒
        for info in self.inertia_actor_info:
            actor = info['actor']
            link_name = info['link_name']
            T_com_rel = info['T_com_rel']

            if link_name in link_name_to_frame:
                link_frame = link_name_to_frame[link_name]
                T_world = link_frame @ T_com_rel

                vtk_transform = vtk.vtkTransform()
                vtk_transform.SetMatrix(T_world.flatten())
                actor.SetUserTransform(vtk_transform)

    def set_com_visibility(self, visible):
        """设置质心标记可见性"""
        self.show_com = visible
        for actor in self.com_actors:
            actor.SetVisibility(visible)

    def set_inertia_visibility(self, visible):
        """设置惯量盒可见性"""
        self.show_inertia = visible
        for actor in self.inertia_actors:
            actor.SetVisibility(visible)

    @staticmethod
    def compute_transformation(rpy, xyz):
        """从 RPY 和 XYZ 计算 4x4 变换矩阵"""
        roll, pitch, yaw = rpy

        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)],
        ])

        Ry = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)],
        ])

        Rz = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1],
        ])

        R = Rz @ Ry @ Rx

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = xyz

        return T

    @staticmethod
    def compute_eigen_decomposition_3x3(matrix_3x3):
        """对 3x3 对称矩阵进行特征值分解（Jacobi 迭代法）"""
        a00, a01, a02 = matrix_3x3[0, 0], matrix_3x3[0, 1], matrix_3x3[0, 2]
        a11, a12 = matrix_3x3[1, 1], matrix_3x3[1, 2]
        a22 = matrix_3x3[2, 2]

        v = np.eye(3)

        max_iterations = 50
        for _ in range(max_iterations):
            abs_01 = abs(a01)
            abs_02 = abs(a02)
            abs_12 = abs(a12)
            max_val = abs_01
            p, q = 0, 1

            if abs_02 > max_val:
                max_val = abs_02
                p, q = 0, 2
            if abs_12 > max_val:
                max_val = abs_12
                p, q = 1, 2

            if max_val < 1e-10:
                break

            if p == 0 and q == 1:
                apq, app, aqq = a01, a00, a11
            elif p == 0 and q == 2:
                apq, app, aqq = a02, a00, a22
            else:
                apq, app, aqq = a12, a11, a22

            tau = (aqq - app) / (2.0 * apq)
            t = math.copysign(1.0, tau) / (abs(tau) + math.sqrt(1.0 + tau * tau))
            c = 1.0 / math.sqrt(1.0 + t * t)
            s = t * c

            if p == 0 and q == 1:
                t00, t01, t02 = a00, a01, a02
                t11, t12 = a11, a12
                a00 = c*c*t00 - 2*c*s*t01 + s*s*t11
                a11 = s*s*t00 + 2*c*s*t01 + c*c*t11
                a01 = 0.0
                a02 = c*t02 - s*t12
                a12 = s*t02 + c*t12
                for row in range(3):
                    tv0 = v[row, 0]
                    tv1 = v[row, 1]
                    v[row, 0] = c * tv0 - s * tv1
                    v[row, 1] = s * tv0 + c * tv1

            elif p == 0 and q == 2:
                t00, t01, t02 = a00, a01, a02
                t12, t22 = a12, a22
                a00 = c*c*t00 - 2*c*s*t02 + s*s*t22
                a22 = s*s*t00 + 2*c*s*t02 + c*c*t22
                a02 = 0.0
                a01 = c*t01 - s*t12
                a12 = s*t01 + c*t12
                for row in range(3):
                    tv0 = v[row, 0]
                    tv2 = v[row, 2]
                    v[row, 0] = c * tv0 - s * tv2
                    v[row, 2] = s * tv0 + c * tv2

            else:  # p == 1, q == 2
                t11, t01, t12 = a11, a01, a12
                t02, t22 = a02, a22
                a11 = c*c*t11 - 2*c*s*t12 + s*s*t22
                a22 = s*s*t11 + 2*c*s*t12 + c*c*t22
                a12 = 0.0
                a01 = c*t01 - s*t02
                a02 = s*t01 + c*t02
                for row in range(3):
                    tv1 = v[row, 1]
                    tv2 = v[row, 2]
                    v[row, 1] = c * tv1 - s * tv2
                    v[row, 2] = s * tv1 + c * tv2

        return [a00, a11, a22], v

    @staticmethod
    def compute_inertia_box(inertia, mass):
        """根据惯性张量计算等效长方体的尺寸和主轴旋转"""
        Ixx = inertia.get('ixx', 0)
        Iyy = inertia.get('iyy', 0)
        Izz = inertia.get('izz', 0)
        Ixy = inertia.get('ixy', 0)
        Ixz = inertia.get('ixz', 0)
        Iyz = inertia.get('iyz', 0)

        min_mass_threshold = 0.01
        if mass < min_mass_threshold:
            avg_inertia = (abs(Ixx) + abs(Iyy) + abs(Izz)) / 3.0
            if avg_inertia > 0:
                inertia_radius = math.sqrt(avg_inertia / mass)
                if inertia_radius > 0.05:
                    return None

        inertia_threshold = 1e-9
        if abs(Ixx) < inertia_threshold and abs(Iyy) < inertia_threshold and abs(Izz) < inertia_threshold:
            return None

        has_off_diagonal = abs(Ixy) > 1e-10 or abs(Ixz) > 1e-10 or abs(Iyz) > 1e-10

        R_principal = np.eye(3)

        if has_off_diagonal:
            inertia_matrix = np.array([
                [Ixx, Ixy, Ixz],
                [Ixy, Iyy, Iyz],
                [Ixz, Iyz, Izz]
            ])
            principal_inertias, eigenvectors = InertiaVisualizer.compute_eigen_decomposition_3x3(inertia_matrix)
            R_principal = eigenvectors
        else:
            principal_inertias = [Ixx, Iyy, Izz]

        factor = 6.0 / mass
        width_sq = factor * (principal_inertias[1] + principal_inertias[2] - principal_inertias[0])
        height_sq = factor * (principal_inertias[0] + principal_inertias[2] - principal_inertias[1])
        depth_sq = factor * (principal_inertias[0] + principal_inertias[1] - principal_inertias[2])

        box_x = math.sqrt(abs(width_sq))
        box_y = math.sqrt(abs(height_sq))
        box_z = math.sqrt(abs(depth_sq))

        avg_inertia = (abs(principal_inertias[0]) + abs(principal_inertias[1]) + abs(principal_inertias[2])) / 3.0
        inertia_radius = math.sqrt(avg_inertia / mass) if mass > 0 else 0
        avg_box_size = (box_x + box_y + box_z) / 3.0
        if avg_box_size > 0 and inertia_radius > avg_box_size:
            return None

        volume = box_x * box_y * box_z
        if volume > 0:
            equivalent_density = mass / volume
            if equivalent_density < 0.0001:
                return None

        min_size = 0.01
        box_x = max(box_x, min_size)
        box_y = max(box_y, min_size)
        box_z = max(box_z, min_size)

        return {
            'box_x': box_x,
            'box_y': box_y,
            'box_z': box_z,
            'R_principal': R_principal
        }
