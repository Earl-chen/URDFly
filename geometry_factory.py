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
