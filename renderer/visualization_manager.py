#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
可视化管理模块

管理视觉模型和碰撞体的显示，包括：
- 视觉模型 (Visual Models)
- 碰撞体模型 (Collision Models)
- 透明度控制
- 可见性控制

参考 robot_viewer/VisualizationManager.js
"""

import vtk
import numpy as np

from .geometry_factory import GeometryFactory, update_transform_for_actor


class SimpleModel:
    """简单模型对象，用于存储基本几何体信息"""

    def __init__(self, name, actor):
        self.name = name
        self.actor = actor
        self.axes_actor = None
        self.text_actor = None

        # 存储原始颜色以便高亮后恢复
        self.original_color = self.actor.GetProperty().GetColor()

        # 高亮状态标记
        self.is_highlighted = False

    def apply_transform(self, transform_matrix):
        """应用变换矩阵"""
        update_transform_for_actor(self.actor, transform_matrix)

    def set_transparency(self, opacity):
        """设置透明度"""
        self.actor.GetProperty().SetOpacity(opacity)

    def highlight(self):
        """高亮显示（与 URDFModel 保持一致的视觉效果）"""
        if not self.is_highlighted:
            # 存储当前颜色（如果尚未存储）
            if not hasattr(self, "original_color"):
                self.original_color = self.actor.GetProperty().GetColor()

            # 设置高亮属性
            self.actor.GetProperty().SetColor(1.0, 1.0, 0.0)  # 黄色高亮
            self.actor.GetProperty().SetEdgeVisibility(True)
            self.actor.GetProperty().SetEdgeColor(1.0, 1.0, 1.0)  # 白色边界
            self.actor.GetProperty().SetLineWidth(2.0)
            self.actor.GetProperty().SetSpecular(0.6)  # 镜面反射增强
            self.actor.GetProperty().SetSpecularPower(30)
            self.is_highlighted = True

    def unhighlight(self):
        """取消高亮"""
        if self.is_highlighted:
            # 恢复原始颜色和属性
            self.actor.GetProperty().SetColor(self.original_color)
            self.actor.GetProperty().SetEdgeVisibility(False)
            self.actor.GetProperty().SetSpecular(0.0)
            self.is_highlighted = False


class VisualizationManager:
    """可视化管理器"""

    def __init__(self, renderer):
        """初始化可视化管理器

        Args:
            renderer: VTK renderer
        """
        self.renderer = renderer

        # 视觉模型列表
        self.visual_models = []

        # 碰撞体模型列表
        self.collision_models = []

        # 可见性状态
        self.show_visual = True
        self.show_collision = True

        # 当前透明度
        self.transparency = 1.0

    def clear(self):
        """清除所有模型"""
        # 清除视觉模型
        for model in self.visual_models:
            self.renderer.RemoveActor(model.actor)
            if model.axes_actor is not None:
                self.renderer.RemoveActor(model.axes_actor)
            if hasattr(model, 'text_actor') and model.text_actor is not None:
                self.renderer.RemoveActor(model.text_actor)
        self.visual_models = []

        # 清除碰撞体模型
        for model in self.collision_models:
            self.renderer.RemoveActor(model.actor)
            if model.axes_actor is not None:
                self.renderer.RemoveActor(model.axes_actor)
            if hasattr(model, 'text_actor') and model.text_actor is not None:
                self.renderer.RemoveActor(model.text_actor)
        self.collision_models = []

    def add_visual_model(self, model):
        """添加视觉模型

        Args:
            model: URDFModel 或 SimpleModel 对象
        """
        self.visual_models.append(model)
        self.renderer.AddActor(model.actor)

        if model.axes_actor is not None:
            self.renderer.AddActor(model.axes_actor)

        if hasattr(model, 'text_actor') and model.text_actor is not None:
            self.renderer.AddActor(model.text_actor)

    def add_collision_model(self, model):
        """添加碰撞体模型

        Args:
            model: URDFModel 或 SimpleModel 对象
        """
        self.collision_models.append(model)
        model.actor.SetPickable(0)  # 设置为不可拾取，避免干扰鼠标拖拽控制
        self.renderer.AddActor(model.actor)

        if model.axes_actor is not None:
            self.renderer.AddActor(model.axes_actor)

        if hasattr(model, 'text_actor') and model.text_actor is not None:
            self.renderer.AddActor(model.text_actor)

    def add_collision_box(self, name, size, transform_matrix):
        """添加碰撞体长方体

        Args:
            name: 名称
            size: (x, y, z) 尺寸
            transform_matrix: 变换矩阵
        """
        actor = GeometryFactory.create_collision_box(size, transform_matrix)
        model = SimpleModel(name, actor)
        self.add_collision_model(model)
        return model

    def add_collision_sphere(self, name, radius, transform_matrix):
        """添加碰撞体球体

        Args:
            name: 名称
            radius: 半径
            transform_matrix: 变换矩阵
        """
        actor = GeometryFactory.create_collision_sphere(radius, transform_matrix)
        model = SimpleModel(name, actor)
        self.add_collision_model(model)
        return model

    def add_collision_cylinder(self, name, radius, length, transform_matrix):
        """添加碰撞体圆柱体

        Args:
            name: 名称
            radius: 半径
            length: 长度
            transform_matrix: 变换矩阵
        """
        actor = GeometryFactory.create_collision_cylinder(radius, length, transform_matrix)
        model = SimpleModel(name, actor)
        self.add_collision_model(model)
        return model

    def add_visual_box(self, name, size, transform_matrix, color=None):
        """添加视觉体长方体（MuJoCo 风格）

        Args:
            name: 名称
            size: (half_x, half_y, half_z) 半尺寸
            transform_matrix: 变换矩阵
            color: 颜色
        """
        actor = GeometryFactory.create_visual_box(size, transform_matrix, color)
        model = SimpleModel(name, actor)
        self.add_visual_model(model)
        return model

    def add_visual_sphere(self, name, radius, transform_matrix, color=None):
        """添加视觉体球体

        Args:
            name: 名称
            radius: 半径
            transform_matrix: 变换矩阵
            color: 颜色
        """
        actor = GeometryFactory.create_visual_sphere(radius, transform_matrix, color)
        model = SimpleModel(name, actor)
        self.add_visual_model(model)
        return model

    def add_visual_cylinder(self, name, radius, half_length, transform_matrix, color=None):
        """添加视觉体圆柱体（MuJoCo 风格）

        Args:
            name: 名称
            radius: 半径
            half_length: 半长度
            transform_matrix: 变换矩阵
            color: 颜色
        """
        actor = GeometryFactory.create_visual_cylinder(radius, half_length, transform_matrix, color)
        model = SimpleModel(name, actor)
        self.add_visual_model(model)
        return model

    def add_visual_capsule(self, name, radius, half_length, transform_matrix, color=None):
        """添加视觉体胶囊体

        Args:
            name: 名称
            radius: 半径
            half_length: 半长度
            transform_matrix: 变换矩阵
            color: 颜色
        """
        actor = GeometryFactory.create_visual_capsule(radius, half_length, transform_matrix, color)
        model = SimpleModel(name, actor)
        self.add_visual_model(model)
        return model

    def set_visual_visibility(self, visible):
        """设置视觉模型可见性

        Args:
            visible: 是否可见
        """
        self.show_visual = visible
        for model in self.visual_models:
            model.actor.SetVisibility(visible)

    def set_collision_visibility(self, visible):
        """设置碰撞体模型可见性

        Args:
            visible: 是否可见
        """
        self.show_collision = visible
        for model in self.collision_models:
            model.actor.SetVisibility(visible)
            if model.axes_actor is not None:
                model.axes_actor.SetVisibility(visible)
            if hasattr(model, 'text_actor') and model.text_actor is not None:
                model.text_actor.SetVisibility(visible)

    def set_link_frames_visibility(self, visible):
        """设置连杆坐标系可见性

        Args:
            visible: 是否可见
        """
        for model in self.visual_models:
            if model.axes_actor is not None:
                model.axes_actor.SetVisibility(visible)
            if hasattr(model, 'text_actor') and model.text_actor is not None:
                model.text_actor.SetVisibility(visible)

    def set_transparency(self, opacity):
        """设置视觉模型透明度

        Args:
            opacity: 透明度 (0.0 - 1.0)
        """
        self.transparency = opacity
        for model in self.visual_models:
            if hasattr(model, 'set_transparency'):
                model.set_transparency(opacity)

    def update_visual_transforms(self, link_names, link_mesh_transformations, link_frames):
        """更新视觉模型的变换

        Args:
            link_names: 连杆名称列表
            link_mesh_transformations: mesh 变换矩阵列表
            link_frames: 连杆坐标系变换矩阵列表
        """
        for i, model in enumerate(self.visual_models):
            if i < len(link_names) and model.name == link_names[i]:
                # 更新 mesh 变换
                model.apply_transform(link_mesh_transformations[i])

                # 更新坐标轴
                if model.axes_actor is not None:
                    vtk_transform = vtk.vtkTransform()
                    vtk_transform.SetMatrix(link_frames[i].flatten())
                    model.axes_actor.SetUserTransform(vtk_transform)

                # 更新文本位置
                if hasattr(model, 'text_actor') and model.text_actor is not None:
                    z_endpoint = [0, 0, 0.05, 1]
                    vtk_transform = vtk.vtkTransform()
                    vtk_transform.SetMatrix(link_frames[i].flatten())
                    transformed_point = vtk_transform.TransformPoint(
                        z_endpoint[0], z_endpoint[1], z_endpoint[2]
                    )
                    model.text_actor.SetAttachmentPoint(
                        transformed_point[0], transformed_point[1], transformed_point[2]
                    )

    def update_collision_transforms(self, collision_mesh_transformations):
        """更新碰撞体模型的变换

        Args:
            collision_mesh_transformations: 碰撞体变换矩阵列表
        """
        for model in self.collision_models:
            # 从模型名称提取索引
            if model.name.startswith("collision_"):
                try:
                    collision_idx = int(model.name.split("_")[1])
                    if collision_idx < len(collision_mesh_transformations):
                        transform = collision_mesh_transformations[collision_idx]
                        model.apply_transform(transform)

                        # 更新坐标轴
                        if model.axes_actor is not None:
                            vtk_transform = vtk.vtkTransform()
                            vtk_transform.SetMatrix(transform.flatten())
                            model.axes_actor.SetUserTransform(vtk_transform)

                        # 更新文本位置
                        if hasattr(model, 'text_actor') and model.text_actor is not None:
                            z_endpoint = [0, 0, 0.05, 1]
                            vtk_transform = vtk.vtkTransform()
                            vtk_transform.SetMatrix(transform.flatten())
                            transformed_point = vtk_transform.TransformPoint(
                                z_endpoint[0], z_endpoint[1], z_endpoint[2]
                            )
                            model.text_actor.SetAttachmentPoint(
                                transformed_point[0], transformed_point[1], transformed_point[2]
                            )
                except (ValueError, IndexError):
                    pass

    def get_visual_model_by_name(self, name):
        """根据名称获取视觉模型

        Args:
            name: 模型名称

        Returns:
            模型对象或 None
        """
        for model in self.visual_models:
            if model.name == name:
                return model
        return None

    def get_collision_model_by_name(self, name):
        """根据名称获取碰撞体模型

        Args:
            name: 模型名称

        Returns:
            模型对象或 None
        """
        for model in self.collision_models:
            if model.name == name:
                return model
        return None

    def highlight_link(self, link_name):
        """高亮显示指定连杆

        Args:
            link_name: 连杆名称
        """
        # 先取消所有高亮
        for model in self.visual_models:
            model.unhighlight()

        # 高亮指定连杆
        model = self.get_visual_model_by_name(link_name)
        if model:
            model.highlight()

    def unhighlight_all(self):
        """取消所有高亮"""
        for model in self.visual_models:
            model.unhighlight()

    def set_link_visibility(self, link_name, visible):
        """设置单个连杆的可见性

        Args:
            link_name: 连杆名称
            visible: 是否可见
        """
        model = self.get_visual_model_by_name(link_name)
        if model:
            model.actor.SetVisibility(visible)
            if model.axes_actor is not None:
                model.axes_actor.SetVisibility(visible)
            if hasattr(model, 'text_actor') and model.text_actor is not None:
                model.text_actor.SetVisibility(visible)

    def get_link_name_by_actor(self, actor):
        """根据 actor 获取连杆名称

        Args:
            actor: VTK actor

        Returns:
            连杆名称或 None
        """
        for model in self.visual_models:
            if model.actor == actor:
                return model.name
        return None
