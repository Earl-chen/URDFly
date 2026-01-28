#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
场景管理中心模块

作为协调中心，持有并协调各个专门的 Manager：
- VisualizationManager: 视觉/碰撞体管理
- InertiaVisualizer: 质心/惯量盒可视化
- AxesManager: 坐标系/关节轴可视化

参考 robot_viewer/SceneManager.js
"""

import os
import vtk

from urdf_parser import URDFParser
from mjcf_parser import MJCFParser
from urdf_vtk_model import URDFModel

from .visualization_manager import VisualizationManager
from .inertia_visualizer import InertiaVisualizer
from .axes_manager import AxesManager


class SceneManager:
    """场景管理中心"""

    def __init__(self, renderer):
        """初始化场景管理器

        Args:
            renderer: VTK renderer
        """
        self.renderer = renderer

        # 初始化各个 Manager
        self.visualization = VisualizationManager(renderer)
        self.inertia = InertiaVisualizer(renderer)
        self.axes = AxesManager(renderer)

        # 当前机器人状态
        self.current_file = None
        self.current_parser_type = None  # 'urdf' or 'mjcf'
        self.parser = None

        # 运动链信息
        self.chains = []
        self.selected_chain = None
        self.selected_chain_index = 0

        # 关节信息
        self.revolute_joints = []
        self.joint_values = []

        # 碰撞体 mesh 文件列表（用于分解）
        self.collision_mesh_files = None

        # 缓存的机器人信息
        self._robot_info = None

    def clear_all(self):
        """清除所有场景内容"""
        self.visualization.clear()
        self.inertia.clear()
        self.axes.clear()

        self.current_file = None
        self.current_parser_type = None
        self.parser = None
        self.chains = []
        self.selected_chain = None
        self.selected_chain_index = 0
        self.revolute_joints = []
        self.joint_values = []
        self.collision_mesh_files = None
        self._robot_info = None

    def load_robot(self, filepath):
        """加载机器人模型

        Args:
            filepath: URDF 或 MJCF 文件路径

        Returns:
            bool: 加载是否成功
        """
        if not filepath or not os.path.exists(filepath):
            return False

        # 清除之前的模型
        self.clear_all()

        # 检测文件类型
        if filepath.lower().endswith('.urdf'):
            return self._load_urdf(filepath)
        elif filepath.lower().endswith('.xml'):
            return self._load_mjcf(filepath)
        else:
            return False

    def _load_urdf(self, filepath):
        """加载 URDF 文件

        Args:
            filepath: URDF 文件路径

        Returns:
            bool: 加载是否成功
        """
        try:
            parser = URDFParser(filepath)
            self._robot_info = parser.get_robot_info()

            (link_names, link_mesh_files, link_mesh_transformations,
             link_frames, link_colors, joint_names, joint_frames,
             joint_types, joint_axes, joint_parent_links, joint_child_links,
             joint_limits, collision_mesh_files, collision_mesh_transformations,
             collision_link_names, collision_geometries) = self._robot_info

            # 存储 revolute 关节信息
            self.revolute_joints = []
            for i, joint_type in enumerate(joint_types):
                if joint_type == 'revolute':
                    self.revolute_joints.append({
                        'name': joint_names[i],
                        'index': i,
                        'parent': joint_parent_links[i],
                        'child': joint_child_links[i],
                        'axis': joint_axes[i],
                        'lower': joint_limits[i]['lower'],
                        'upper': joint_limits[i]['upper']
                    })

            # 初始化关节值
            self.joint_values = [0.0] * len(self.revolute_joints)

            # 获取运动链信息
            self.chains, _ = parser.get_chain_info()

            # 创建视觉模型
            for i in range(len(link_names)):
                self._add_visual_model(
                    link_names[i],
                    link_mesh_files[i],
                    link_mesh_transformations[i],
                    link_frames[i],
                    link_colors[i]
                )

            # 创建碰撞体模型
            for i, geom in enumerate(collision_geometries):
                self._add_collision_model(
                    f"collision_{i}",
                    geom,
                    collision_mesh_files[i],
                    collision_mesh_transformations[i]
                )

            # 存储碰撞体 mesh 文件
            self.collision_mesh_files = [f for f in link_mesh_files if f is not None]

            self.current_file = filepath
            self.current_parser_type = 'urdf'
            self.parser = parser

            return True

        except Exception as e:
            print(f"Error loading URDF: {e}")
            return False

    def _load_mjcf(self, filepath):
        """加载 MJCF 文件

        Args:
            filepath: MJCF 文件路径

        Returns:
            bool: 加载是否成功
        """
        # 检查是否是有效的 MJCF 文件
        if not MJCFParser.is_valid_mjcf(filepath):
            return False

        try:
            parser = MJCFParser(filepath)
            self._robot_info = parser.get_robot_info()

            (link_names, link_mesh_files, link_mesh_transformations,
             link_frames, link_colors, joint_names, joint_frames,
             joint_types, joint_axes, joint_parent_links, joint_child_links,
             joint_limits, collision_mesh_files, collision_mesh_transformations,
             collision_link_names, collision_geometries) = self._robot_info

            # 存储 revolute 关节信息
            self.revolute_joints = []
            for i, joint_type in enumerate(joint_types):
                if joint_type == 'revolute':
                    self.revolute_joints.append({
                        'name': joint_names[i],
                        'index': i,
                        'parent': joint_parent_links[i],
                        'child': joint_child_links[i],
                        'axis': joint_axes[i],
                        'lower': joint_limits[i]['lower'],
                        'upper': joint_limits[i]['upper']
                    })

            # 初始化关节值
            self.joint_values = [0.0] * len(self.revolute_joints)

            # 获取运动链信息 (MJCF 也支持)
            self.chains, _ = parser.get_chain_info()

            # 获取视觉几何体信息
            visual_geometries = parser.get_visual_geometries()

            # 创建视觉模型
            for i in range(len(link_names)):
                vis_geom = visual_geometries[i] if i < len(visual_geometries) else None

                if link_mesh_files[i] is not None:
                    self._add_visual_model(
                        link_names[i],
                        link_mesh_files[i],
                        link_mesh_transformations[i],
                        link_frames[i],
                        link_colors[i]
                    )
                elif vis_geom is not None:
                    self._add_primitive_visual_model(
                        link_names[i],
                        vis_geom,
                        link_mesh_transformations[i],
                        link_frames[i],
                        link_colors[i]
                    )
                else:
                    # 无视觉几何体，创建占位符
                    self._add_visual_model(
                        link_names[i],
                        None,
                        link_mesh_transformations[i],
                        link_frames[i],
                        link_colors[i]
                    )

            # 创建碰撞体模型
            for i, geom in enumerate(collision_geometries):
                self._add_collision_model(
                    f"collision_{i}",
                    geom,
                    collision_mesh_files[i],
                    collision_mesh_transformations[i]
                )

            # 存储碰撞体 mesh 文件
            self.collision_mesh_files = [f for f in link_mesh_files if f is not None]

            self.current_file = filepath
            self.current_parser_type = 'mjcf'
            self.parser = parser

            return True

        except Exception as e:
            print(f"Error loading MJCF: {e}")
            return False

    def _add_visual_model(self, name, mesh_file, mesh_transform, frame, color):
        """添加视觉模型

        Args:
            name: 模型名称
            mesh_file: mesh 文件路径
            mesh_transform: mesh 变换矩阵
            frame: 连杆坐标系变换矩阵
            color: 颜色
        """
        try:
            model = URDFModel(name, mesh_file, mesh_transform, frame, color, axis_text=name)
            self.visualization.add_visual_model(model)
        except Exception as e:
            print(f"Error adding visual model {name}: {e}")

    def _add_primitive_visual_model(self, name, vis_geom, transform, frame, color):
        """添加基本几何体视觉模型

        Args:
            name: 模型名称
            vis_geom: 几何体信息 dict
            transform: 变换矩阵
            frame: 连杆坐标系变换矩阵
            color: 颜色
        """
        geom_type = vis_geom.get('type')
        geom_size = vis_geom.get('size', [0.01, 0.01, 0.01])

        if geom_type == 'box':
            self.visualization.add_visual_box(name, geom_size, transform, color)
        elif geom_type == 'sphere':
            self.visualization.add_visual_sphere(name, geom_size[0], transform, color)
        elif geom_type == 'cylinder':
            self.visualization.add_visual_cylinder(name, geom_size[0], geom_size[1], transform, color)
        elif geom_type == 'capsule':
            self.visualization.add_visual_capsule(name, geom_size[0], geom_size[1], transform, color)

    def _add_collision_model(self, name, geom, mesh_file, transform):
        """添加碰撞体模型

        Args:
            name: 模型名称
            geom: 几何体信息 dict
            mesh_file: mesh 文件路径
            transform: 变换矩阵
        """
        geom_type = geom['type']

        if geom_type == 'mesh':
            try:
                model = URDFModel(name, mesh_file, transform, None, None)
                model.actor.GetProperty().SetColor(0.3, 0.6, 1.0)
                model.actor.GetProperty().SetOpacity(0.4)
                self.visualization.add_collision_model(model)
            except Exception as e:
                print(f"Error adding collision mesh {name}: {e}")
        elif geom_type == 'box':
            self.visualization.add_collision_box(name, geom['size'], transform)
        elif geom_type == 'sphere':
            self.visualization.add_collision_sphere(name, geom['radius'], transform)
        elif geom_type == 'cylinder':
            self.visualization.add_collision_cylinder(name, geom['radius'], geom['length'], transform)

    def update_joint_angles(self, joint_values):
        """更新关节角度并刷新场景

        Args:
            joint_values: 关节角度值列表
        """
        if not self.current_file:
            return

        self.joint_values = joint_values

        # ✅ 使用缓存的解析器而非重新创建 (性能优化)
        if self.parser is None:
            self.parser = self._create_parser()

        # ✅ 调用轻量级变换更新方法 (性能优化)
        transforms = self.parser.update_transforms(joint_values)

        link_names = transforms['link_names']
        link_mesh_transformations = transforms['link_mesh_transformations']
        link_frames = transforms['link_frames']
        collision_mesh_transformations = transforms['collision_mesh_transformations']
        joint_names = transforms['joint_names']
        joint_frames = transforms['joint_frames']
        joint_axes = transforms['joint_axes']

        # 更新视觉模型变换
        self.visualization.update_visual_transforms(
            link_names, link_mesh_transformations, link_frames
        )

        # 更新碰撞体变换
        self.visualization.update_collision_transforms(collision_mesh_transformations)

        # 更新关节轴变换
        if self.axes.has_joint_axes():
            self.axes.update_joint_axes_transforms(joint_names, joint_frames, joint_axes)

        # 更新惯量可视化变换
        self.inertia.update_transforms(link_names, link_frames)

        # 更新 MDH 坐标系
        if self.axes.show_mdh_frames and self.selected_chain:
            self.update_mdh_frames()

    def update_mdh_frames(self):
        """更新 MDH 坐标系"""
        if not self.selected_chain:
            return

        parser = self._create_parser()
        chains, _ = parser.get_chain_info()

        # 找到匹配的链
        current_chain = None
        for chain in chains:
            if chain['name'] == self.selected_chain['name']:
                current_chain = chain
                break

        if current_chain is None:
            current_chain = self.selected_chain

        mdh_frames = parser.get_mdh_frames(current_chain)
        mdh_frames = parser.update_mdh_frames(mdh_frames, self.joint_values)

        self.axes.create_mdh_frames(mdh_frames, visible=True)

    def _create_parser(self):
        """创建合适的解析器"""
        if self.current_parser_type == 'mjcf':
            return MJCFParser(self.current_file)
        else:
            return URDFParser(self.current_file)

    # === 可见性控制接口 ===

    def set_visual_visibility(self, visible):
        """设置视觉模型可见性"""
        self.visualization.set_visual_visibility(visible)

    def set_collision_visibility(self, visible):
        """设置碰撞体可见性"""
        self.visualization.set_collision_visibility(visible)

    def set_link_frames_visibility(self, visible):
        """设置连杆坐标系可见性"""
        self.visualization.set_link_frames_visibility(visible)

    def set_mdh_frames_visibility(self, visible):
        """设置 MDH 坐标系可见性"""
        if visible and self.selected_chain:
            self.update_mdh_frames()
        self.axes.set_mdh_frames_visibility(visible)

    def set_joint_axes_visibility(self, visible):
        """设置关节轴可见性"""
        if visible and not self.axes.has_joint_axes():
            self._create_joint_axes()
        self.axes.set_joint_axes_visibility(visible)

    def set_com_visibility(self, visible):
        """设置质心可见性"""
        if visible and len(self.inertia.com_actors) == 0:
            self._create_com_markers()
        self.inertia.set_com_visibility(visible)

    def set_inertia_visibility(self, visible):
        """设置惯量盒可见性"""
        if visible and len(self.inertia.inertia_actors) == 0:
            self._create_inertia_boxes()
        self.inertia.set_inertia_visibility(visible)

    def set_transparency(self, opacity):
        """设置透明度"""
        self.visualization.set_transparency(opacity)

    def _create_joint_axes(self):
        """创建关节轴"""
        if not self.current_file:
            return

        parser = self._create_parser()
        robot_info = parser.get_robot_info(qs=self.joint_values)

        joint_names = robot_info[5]
        joint_frames = robot_info[6]
        joint_types = robot_info[7]
        joint_axes = robot_info[8]

        self.axes.create_joint_axes(
            joint_names, joint_frames, joint_types, joint_axes,
            visible=True
        )

    def _create_com_markers(self):
        """创建质心标记"""
        if not self.current_file:
            return

        parser = self._create_parser()
        robot_info = parser.get_robot_info(qs=self.joint_values)

        link_names = robot_info[0]
        link_frames = robot_info[3]

        self.inertia.create_com_markers(parser, link_names, link_frames, self.joint_values)

    def _create_inertia_boxes(self):
        """创建惯量盒"""
        if not self.current_file:
            return

        parser = self._create_parser()
        robot_info = parser.get_robot_info(qs=self.joint_values)

        link_names = robot_info[0]
        link_frames = robot_info[3]

        self.inertia.create_inertia_boxes(parser, link_names, link_frames, self.joint_values)

    # === 链选择接口 ===

    def select_chain(self, index):
        """选择运动链

        Args:
            index: 链索引
        """
        if 0 <= index < len(self.chains):
            self.selected_chain_index = index
            self.selected_chain = self.chains[index]

            # 如果 MDH 坐标系可见，更新它
            if self.axes.show_mdh_frames:
                self.update_mdh_frames()

    def get_chain_names(self):
        """获取所有链名称"""
        return [(i, chain['name']) for i, chain in enumerate(self.chains)]

    def get_selected_chain_links(self):
        """获取当前选中链的连杆名称列表"""
        if self.selected_chain:
            return self.selected_chain.get('link_names', [])
        return []

    # === 其他接口 ===

    def get_revolute_joints(self):
        """获取 revolute 关节列表"""
        return self.revolute_joints

    def get_parser(self):
        """获取当前解析器"""
        return self._create_parser() if self.current_file else None

    def is_urdf(self):
        """检查当前是否是 URDF 文件"""
        return self.current_parser_type == 'urdf'

    def is_mjcf(self):
        """检查当前是否是 MJCF 文件"""
        return self.current_parser_type == 'mjcf'

    def redraw(self):
        """重新渲染场景"""
        self.renderer.GetRenderWindow().Render()

    def reset_camera(self):
        """重置相机以显示所有 actor"""
        self.renderer.ResetCamera()
        self.redraw()

    def highlight_link(self, link_name):
        """高亮显示连杆"""
        self.visualization.highlight_link(link_name)

    def unhighlight_all(self):
        """取消所有高亮"""
        self.visualization.unhighlight_all()

    def set_link_visibility(self, link_name, visible):
        """设置单个连杆的可见性

        Args:
            link_name: 连杆名称
            visible: 是否可见
        """
        self.visualization.set_link_visibility(link_name, visible)

    def get_link_name_by_actor(self, actor):
        """根据 actor 获取连杆名称

        Args:
            actor: VTK actor

        Returns:
            连杆名称或 None
        """
        return self.visualization.get_link_name_by_actor(actor)

    def get_joint_for_child_link(self, link_name):
        """获取某个 link 作为 child 的关节

        当拖拽一个 link 时，控制的是该 link 作为 child 的关节

        Args:
            link_name: 连杆名称

        Returns:
            (index, joint_info) 或 (None, None)
        """
        for i, joint in enumerate(self.revolute_joints):
            if joint['child'] == link_name:
                return i, joint
        return None, None
