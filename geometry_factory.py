#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
几何体创建工厂模块

提供各种 VTK 几何体的创建方法，包括：
- 基本几何体：box, sphere, cylinder
- 坐标轴
- 变换应用
"""

import vtk
import numpy as np


def update_transform_for_actor(actor, transform_matrix):
    """更新 actor 的变换矩阵

    Args:
        actor: VTK actor
        transform_matrix: 4x4 numpy 变换矩阵
    """
    vtk_transform = vtk.vtkTransform()
    vtk_transform.SetMatrix(transform_matrix.flatten())
    actor.SetUserTransform(vtk_transform)


class GeometryFactory:
    """几何体创建工厂类"""

    # 默认颜色定义
    DEFAULT_COLLISION_COLOR = (0.2, 0.7, 0.3)  # 绿色
    DEFAULT_COLLISION_OPACITY = 0.5
    DEFAULT_INERTIA_COLOR = (0.9, 0.4, 0.1)    # 暖橙色
    DEFAULT_INERTIA_OPACITY = 0.5
    DEFAULT_VISUAL_OPACITY = 1.0

    @staticmethod
    def create_box(size, color=None, opacity=1.0):
        """创建长方体几何体

        Args:
            size: (x, y, z) 尺寸
            color: (r, g, b) 颜色，0-1 范围
            opacity: 透明度，0-1 范围

        Returns:
            vtkActor
        """
        cube = vtk.vtkCubeSource()
        cube.SetXLength(size[0])
        cube.SetYLength(size[1])
        cube.SetZLength(size[2])
        cube.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cube.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        if color:
            actor.GetProperty().SetColor(color[0], color[1], color[2])
        actor.GetProperty().SetOpacity(opacity)

        return actor

    @staticmethod
    def create_sphere(radius, color=None, opacity=1.0, resolution=24):
        """创建球体几何体

        Args:
            radius: 半径
            color: (r, g, b) 颜色
            opacity: 透明度
            resolution: 分辨率

        Returns:
            vtkActor
        """
        sphere = vtk.vtkSphereSource()
        sphere.SetRadius(radius)
        sphere.SetPhiResolution(resolution)
        sphere.SetThetaResolution(resolution)
        sphere.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        if color:
            actor.GetProperty().SetColor(color[0], color[1], color[2])
        actor.GetProperty().SetOpacity(opacity)

        return actor

    @staticmethod
    def create_cylinder(radius, length, color=None, opacity=1.0,
                        resolution=24, align_to_z=False):
        """创建圆柱体几何体

        VTK 默认圆柱体沿 Y 轴，可选旋转到 Z 轴

        Args:
            radius: 半径
            length: 长度（全长）
            color: (r, g, b) 颜色
            opacity: 透明度
            resolution: 分辨率
            align_to_z: 是否将圆柱体对齐到 Z 轴（默认 Y 轴）

        Returns:
            vtkActor
        """
        cylinder = vtk.vtkCylinderSource()
        cylinder.SetRadius(radius)
        cylinder.SetHeight(length)
        cylinder.SetResolution(resolution)
        cylinder.Update()

        if align_to_z:
            # 将 Y 轴对齐的圆柱体旋转到 Z 轴
            rotate_transform = vtk.vtkTransform()
            rotate_transform.RotateX(90)

            transform_filter = vtk.vtkTransformPolyDataFilter()
            transform_filter.SetInputConnection(cylinder.GetOutputPort())
            transform_filter.SetTransform(rotate_transform)
            transform_filter.Update()

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(transform_filter.GetOutputPort())
        else:
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(cylinder.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        if color:
            actor.GetProperty().SetColor(color[0], color[1], color[2])
        actor.GetProperty().SetOpacity(opacity)

        return actor

    @staticmethod
    def apply_transform(actor, transform_matrix):
        """应用变换矩阵到 actor

        Args:
            actor: VTK actor
            transform_matrix: 4x4 numpy 变换矩阵
        """
        vtk_transform = vtk.vtkTransform()
        vtk_transform.SetMatrix(transform_matrix.flatten())
        actor.SetUserTransform(vtk_transform)

    @staticmethod
    def apply_color_and_opacity(actor, color, opacity=1.0):
        """应用颜色和透明度到 actor

        Args:
            actor: VTK actor
            color: (r, g, b) 或 (r, g, b, a) 颜色
            opacity: 透明度（如果 color 没有 alpha 分量）
        """
        if color:
            actor.GetProperty().SetColor(color[0], color[1], color[2])
            if len(color) > 3:
                actor.GetProperty().SetOpacity(color[3])
            else:
                actor.GetProperty().SetOpacity(opacity)

    @staticmethod
    def create_capsule(radius, half_length, color=None, opacity=1.0, resolution=24):
        """创建胶囊体几何体

        胶囊体 = 圆柱体 + 两个半球端盖

        Args:
            radius: 半径
            half_length: 圆柱部分的半长度（MuJoCo 风格）
            color: (r, g, b) 颜色
            opacity: 透明度
            resolution: 分辨率

        Returns:
            vtkActor
        """
        try:
            # VTK 9+ 有 vtkCapsuleSource
            capsule = vtk.vtkCapsuleSource()
            capsule.SetRadius(radius)
            capsule.SetCylinderLength(half_length * 2)
            capsule.SetPhiResolution(resolution)
            capsule.SetThetaResolution(resolution)
            capsule.Update()

            # VTK 胶囊沿 Y 轴，旋转到 Z 轴
            rotate_transform = vtk.vtkTransform()
            rotate_transform.RotateX(90)

            transform_filter = vtk.vtkTransformPolyDataFilter()
            transform_filter.SetInputConnection(capsule.GetOutputPort())
            transform_filter.SetTransform(rotate_transform)
            transform_filter.Update()

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(transform_filter.GetOutputPort())

        except AttributeError:
            # 回退：使用圆柱体近似
            cylinder = vtk.vtkCylinderSource()
            cylinder.SetRadius(radius)
            cylinder.SetHeight(half_length * 2 + radius * 2)
            cylinder.SetResolution(resolution)
            cylinder.Update()

            rotate_transform = vtk.vtkTransform()
            rotate_transform.RotateX(90)

            transform_filter = vtk.vtkTransformPolyDataFilter()
            transform_filter.SetInputConnection(cylinder.GetOutputPort())
            transform_filter.SetTransform(rotate_transform)
            transform_filter.Update()

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(transform_filter.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        if color:
            actor.GetProperty().SetColor(color[0], color[1], color[2])
        actor.GetProperty().SetOpacity(opacity)

        return actor

    @staticmethod
    def create_axes(length=0.05, shaft_type=0, show_labels=False, cylinder_radius=0.01):
        """创建坐标轴 actor

        Args:
            length: 轴长度
            shaft_type: 轴形状类型
            show_labels: 是否显示轴标签
            cylinder_radius: 圆柱半径

        Returns:
            vtkAxesActor
        """
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(length, length, length)
        axes.SetShaftType(shaft_type)
        axes.SetAxisLabels(1 if show_labels else 0)
        axes.SetCylinderRadius(cylinder_radius)
        return axes

    @staticmethod
    def create_text_caption(text, position, color=(0, 0, 0), font_size=14, bold=False):
        """创建文本标签 actor

        Args:
            text: 文本内容
            position: (x, y, z) 位置
            color: (r, g, b) 颜色
            font_size: 字体大小
            bold: 是否加粗

        Returns:
            vtkCaptionActor2D
        """
        text_actor = vtk.vtkCaptionActor2D()
        text_actor.SetCaption(text)
        text_actor.GetTextActor().SetTextScaleModeToNone()
        text_actor.GetCaptionTextProperty().SetFontSize(font_size)
        text_actor.GetCaptionTextProperty().SetColor(color[0], color[1], color[2])
        text_actor.GetCaptionTextProperty().SetBold(bold)
        text_actor.SetAttachmentPoint(position[0], position[1], position[2])
        text_actor.BorderOff()
        text_actor.LeaderOff()
        text_actor.ThreeDimensionalLeaderOff()
        text_actor.SetPadding(2)
        return text_actor

    @staticmethod
    def create_collision_box(size, transform_matrix, color=None, opacity=None):
        """创建碰撞体长方体

        Args:
            size: (x, y, z) 尺寸
            transform_matrix: 4x4 变换矩阵
            color: (r, g, b) 颜色，默认 DEFAULT_COLLISION_COLOR
            opacity: 透明度，默认 DEFAULT_COLLISION_OPACITY

        Returns:
            vtkActor
        """
        c = color if color is not None else GeometryFactory.DEFAULT_COLLISION_COLOR
        o = opacity if opacity is not None else GeometryFactory.DEFAULT_COLLISION_OPACITY
        actor = GeometryFactory.create_box(size, c, o)
        GeometryFactory.apply_transform(actor, transform_matrix)
        return actor

    @staticmethod
    def create_collision_sphere(radius, transform_matrix, color=None, opacity=None):
        """创建碰撞体球体

        Args:
            radius: 半径
            transform_matrix: 4x4 变换矩阵
            color: (r, g, b) 颜色，默认 DEFAULT_COLLISION_COLOR
            opacity: 透明度，默认 DEFAULT_COLLISION_OPACITY

        Returns:
            vtkActor
        """
        c = color if color is not None else GeometryFactory.DEFAULT_COLLISION_COLOR
        o = opacity if opacity is not None else GeometryFactory.DEFAULT_COLLISION_OPACITY
        actor = GeometryFactory.create_sphere(radius, c, o, resolution=16)
        GeometryFactory.apply_transform(actor, transform_matrix)
        return actor

    @staticmethod
    def create_collision_cylinder(radius, length, transform_matrix, color=None, opacity=None):
        """创建碰撞体圆柱体

        Args:
            radius: 半径
            length: 长度
            transform_matrix: 4x4 变换矩阵
            color: (r, g, b) 颜色，默认 DEFAULT_COLLISION_COLOR
            opacity: 透明度，默认 DEFAULT_COLLISION_OPACITY

        Returns:
            vtkActor
        """
        c = color if color is not None else GeometryFactory.DEFAULT_COLLISION_COLOR
        o = opacity if opacity is not None else GeometryFactory.DEFAULT_COLLISION_OPACITY
        actor = GeometryFactory.create_cylinder(radius, length, c, o, resolution=16)
        GeometryFactory.apply_transform(actor, transform_matrix)
        return actor

    @staticmethod
    def create_visual_box(size, transform_matrix, color=None):
        """创建视觉体长方体（MuJoCo 风格，size 是半尺寸）

        Args:
            size: (half_x, half_y, half_z) 半尺寸
            transform_matrix: 4x4 变换矩阵
            color: (r, g, b) 或 (r, g, b, a) 颜色

        Returns:
            vtkActor
        """
        # MuJoCo size 是半尺寸，需要转换为全尺寸
        full_size = (size[0] * 2, size[1] * 2, size[2] * 2)
        actor = GeometryFactory.create_box(full_size)
        GeometryFactory.apply_color_and_opacity(actor, color)
        GeometryFactory.apply_transform(actor, transform_matrix)
        return actor

    @staticmethod
    def create_visual_sphere(radius, transform_matrix, color=None):
        """创建视觉体球体

        Args:
            radius: 半径
            transform_matrix: 4x4 变换矩阵
            color: (r, g, b) 或 (r, g, b, a) 颜色

        Returns:
            vtkActor
        """
        actor = GeometryFactory.create_sphere(radius)
        GeometryFactory.apply_color_and_opacity(actor, color)
        GeometryFactory.apply_transform(actor, transform_matrix)
        return actor

    @staticmethod
    def create_visual_cylinder(radius, half_length, transform_matrix, color=None):
        """创建视觉体圆柱体（MuJoCo 风格，length 是半长度）

        Args:
            radius: 半径
            half_length: 半长度
            transform_matrix: 4x4 变换矩阵
            color: (r, g, b) 或 (r, g, b, a) 颜色

        Returns:
            vtkActor
        """
        actor = GeometryFactory.create_cylinder(
            radius, half_length * 2,
            align_to_z=True
        )
        GeometryFactory.apply_color_and_opacity(actor, color)
        GeometryFactory.apply_transform(actor, transform_matrix)
        return actor

    @staticmethod
    def create_visual_capsule(radius, half_length, transform_matrix, color=None):
        """创建视觉体胶囊体

        Args:
            radius: 半径
            half_length: 圆柱部分半长度
            transform_matrix: 4x4 变换矩阵
            color: (r, g, b) 或 (r, g, b, a) 颜色

        Returns:
            vtkActor
        """
        actor = GeometryFactory.create_capsule(radius, half_length)
        GeometryFactory.apply_color_and_opacity(actor, color)
        GeometryFactory.apply_transform(actor, transform_matrix)
        return actor

    @staticmethod
    def create_com_marker(position, radius=0.02, color=(0.5, 0.5, 0.5)):
        """创建质心标记（小球体）

        Args:
            position: (x, y, z) 位置或 4x4 变换矩阵
            radius: 半径
            color: 颜色

        Returns:
            vtkActor
        """
        actor = GeometryFactory.create_sphere(radius, color, 1.0, resolution=16)

        vtk_transform = vtk.vtkTransform()
        if isinstance(position, np.ndarray) and position.shape == (4, 4):
            # 是变换矩阵
            vtk_transform.SetMatrix(position.flatten())
        else:
            # 是位置向量
            vtk_transform.Translate(position[0], position[1], position[2])
        actor.SetUserTransform(vtk_transform)

        return actor

    @staticmethod
    def create_joint_axis_cylinder(joint_frame, joint_axis_local,
                                   axis_length=0.3, radius=0.01,
                                   color=(1, 0, 1)):
        """创建关节轴可视化圆柱体

        Args:
            joint_frame: 4x4 关节坐标系变换矩阵
            joint_axis_local: 局部坐标系中的轴方向 [x, y, z]
            axis_length: 轴长度
            radius: 圆柱半径
            color: 颜色

        Returns:
            vtkActor
        """
        import math

        cylinder = vtk.vtkCylinderSource()
        cylinder.SetRadius(radius)
        cylinder.SetHeight(axis_length)
        cylinder.SetResolution(16)
        cylinder.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cylinder.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(color[0], color[1], color[2])
        actor.GetProperty().SetOpacity(1.0)

        # 创建变换
        vtk_transform = vtk.vtkTransform()

        # 提取关节位置
        joint_position = joint_frame[:3, 3]

        # 将轴从局部坐标系变换到世界坐标系
        R_joint = joint_frame[:3, :3]
        joint_axis_local = np.array(joint_axis_local)
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
                vtk_transform.RotateWXYZ(math.degrees(angle),
                                         axis_rot[0], axis_rot[1], axis_rot[2])

        actor.SetUserTransform(vtk_transform)
        return actor

    @staticmethod
    def create_com_marker_styled(transform_matrix, radius=0.012):
        """创建质心标记（双层球体风格）

        简洁的双层设计：
        - 外层：橙色半透明球体
        - 内核：白色小实心球

        Args:
            transform_matrix: 4x4 变换矩阵
            radius: 外层球体半径

        Returns:
            vtkAssembly: 组合 actor
        """
        assembly = vtk.vtkAssembly()

        # 1. 外层球体 - 橙色半透明
        outer_sphere = GeometryFactory.create_sphere(
            radius, color=(1.0, 0.5, 0.0), opacity=0.6, resolution=16
        )
        assembly.AddPart(outer_sphere)

        # 2. 内核 - 白色小实心球
        inner_sphere = GeometryFactory.create_sphere(
            radius * 0.4, color=(1.0, 1.0, 1.0), opacity=1.0, resolution=12
        )
        assembly.AddPart(inner_sphere)

        # 应用世界坐标变换
        vtk_transform = vtk.vtkTransform()
        vtk_transform.SetMatrix(transform_matrix.flatten())
        assembly.SetUserTransform(vtk_transform)

        return assembly

    @staticmethod
    def create_inertia_box(box_size, transform_matrix,
                           color=None, opacity=None):
        """创建惯量盒

        Args:
            box_size: (x, y, z) 尺寸
            transform_matrix: 4x4 变换矩阵
            color: 颜色，默认 DEFAULT_INERTIA_COLOR
            opacity: 透明度，默认 DEFAULT_INERTIA_OPACITY

        Returns:
            vtkActor
        """
        c = color if color is not None else GeometryFactory.DEFAULT_INERTIA_COLOR
        o = opacity if opacity is not None else GeometryFactory.DEFAULT_INERTIA_OPACITY
        actor = GeometryFactory.create_box(box_size, c, o)
        GeometryFactory.apply_transform(actor, transform_matrix)
        return actor

    @staticmethod
    def create_joint_axis_arrow(joint_frame, joint_axis_local,
                                axis_length=0.06, radius=0.004):
        """创建带箭头和圆环的关节轴

        设计：
        - 主体：细圆柱
        - 顶端：箭头圆锥
        - 中部：旋转指示圆环 (torus)

        Args:
            joint_frame: 4x4 关节坐标系变换矩阵
            joint_axis_local: 局部坐标系中的轴方向 [x, y, z]
            axis_length: 轴总长度
            radius: 圆柱半径

        Returns:
            vtkAssembly: 组合 actor
        """
        import math

        assembly = vtk.vtkAssembly()

        # 1. 主体圆柱 - 洋红色
        cylinder_length = axis_length * 0.80
        cylinder = vtk.vtkCylinderSource()
        cylinder.SetRadius(radius)
        cylinder.SetHeight(cylinder_length)
        cylinder.SetResolution(16)
        cylinder.Update()

        cylinder_mapper = vtk.vtkPolyDataMapper()
        cylinder_mapper.SetInputConnection(cylinder.GetOutputPort())
        cylinder_actor = vtk.vtkActor()
        cylinder_actor.SetMapper(cylinder_mapper)
        cylinder_actor.GetProperty().SetColor(0.8, 0.2, 0.8)

        # 平移圆柱使底部在原点
        cylinder_transform = vtk.vtkTransform()
        cylinder_transform.Translate(0, cylinder_length / 2, 0)
        cylinder_actor.SetUserTransform(cylinder_transform)
        assembly.AddPart(cylinder_actor)

        # 2. 箭头圆锥 - 洋红色
        cone_height = axis_length * 0.20
        cone = vtk.vtkConeSource()
        cone.SetRadius(radius * 2.5)
        cone.SetHeight(cone_height)
        cone.SetResolution(16)
        cone.SetDirection(0, 1, 0)
        cone.Update()

        cone_mapper = vtk.vtkPolyDataMapper()
        cone_mapper.SetInputConnection(cone.GetOutputPort())
        cone_actor = vtk.vtkActor()
        cone_actor.SetMapper(cone_mapper)
        cone_actor.GetProperty().SetColor(0.8, 0.2, 0.8)

        cone_transform = vtk.vtkTransform()
        cone_transform.Translate(0, cylinder_length + cone_height / 2, 0)
        cone_actor.SetUserTransform(cone_transform)
        assembly.AddPart(cone_actor)

        # 3. 旋转指示圆环 (torus) - 半透明青色
        torus = vtk.vtkParametricTorus()
        torus.SetRingRadius(radius * 4)
        torus.SetCrossSectionRadius(radius * 0.8)

        torus_source = vtk.vtkParametricFunctionSource()
        torus_source.SetParametricFunction(torus)
        torus_source.SetUResolution(32)
        torus_source.SetVResolution(12)
        torus_source.Update()

        torus_mapper = vtk.vtkPolyDataMapper()
        torus_mapper.SetInputConnection(torus_source.GetOutputPort())
        torus_actor = vtk.vtkActor()
        torus_actor.SetMapper(torus_mapper)
        torus_actor.GetProperty().SetColor(0.0, 0.8, 0.8)
        torus_actor.GetProperty().SetOpacity(0.7)

        # 圆环放在轴的 1/4 处，并旋转使其垂直于 Y 轴
        torus_transform = vtk.vtkTransform()
        torus_transform.Translate(0, cylinder_length * 0.25, 0)
        torus_transform.RotateX(90)
        torus_actor.SetUserTransform(torus_transform)
        assembly.AddPart(torus_actor)

        # 创建世界坐标变换
        vtk_transform = vtk.vtkTransform()
        joint_position = joint_frame[:3, 3]

        R_joint = joint_frame[:3, :3]
        joint_axis_local = np.array(joint_axis_local)
        joint_axis_local = joint_axis_local / (np.linalg.norm(joint_axis_local) + 1e-10)
        joint_axis_world = R_joint @ joint_axis_local

        vtk_transform.Translate(joint_position[0], joint_position[1], joint_position[2])

        y_axis = np.array([0, 1, 0])
        if not np.allclose(y_axis, joint_axis_world, atol=0.01):
            cross = np.cross(y_axis, joint_axis_world)
            dot = np.dot(y_axis, joint_axis_world)
            angle = math.acos(np.clip(dot, -1, 1))
            if np.linalg.norm(cross) > 1e-6:
                axis_rot = cross / np.linalg.norm(cross)
                vtk_transform.RotateWXYZ(math.degrees(angle),
                                         axis_rot[0], axis_rot[1], axis_rot[2])

        assembly.SetUserTransform(vtk_transform)
        return assembly
