#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
自定义 VTK 交互风格模块

支持拖拽 link 控制关节角度的自定义交互风格
"""

import vtk


class DragJointInteractorStyle(vtk.vtkInteractorStyleTrackballCamera):
    """支持拖拽 link 控制关节的自定义交互风格

    点击 link 并拖拽时，控制关联的关节角度；
    其他操作时保持标准的 TrackballCamera 行为。
    """

    def __init__(self, parent=None):
        super().__init__()

        # 回调函数
        self._get_link_name_callback = None
        self._get_joint_for_link_callback = None
        self._on_joint_drag_callback = None
        self._on_drag_start_callback = None
        self._on_drag_end_callback = None

        # 拖拽状态
        self.dragging = False
        self.joint_index = None
        self.joint_info = None
        self.drag_start_pos = None
        self.dragging_link_name = None  # 当前拖拽的 link 名称

        # 拖拽灵敏度（像素到弧度的转换系数）
        self.drag_sensitivity = 0.01

        # Picker
        self.picker = vtk.vtkPropPicker()

        # 添加事件观察者
        self.AddObserver("LeftButtonPressEvent", self._on_left_button_press)
        self.AddObserver("LeftButtonReleaseEvent", self._on_left_button_release)
        self.AddObserver("MouseMoveEvent", self._on_mouse_move)

    def set_callbacks(self, get_link_name, get_joint_for_link, on_joint_drag,
                      on_drag_start=None, on_drag_end=None):
        """设置回调函数

        Args:
            get_link_name: 根据 actor 获取 link 名称的回调
            get_joint_for_link: 根据 link 名称获取关联关节的回调
            on_joint_drag: 关节角度变化的回调
            on_drag_start: 拖拽开始的回调，参数为 (link_name,)
            on_drag_end: 拖拽结束的回调，参数为 (link_name,)
        """
        self._get_link_name_callback = get_link_name
        self._get_joint_for_link_callback = get_joint_for_link
        self._on_joint_drag_callback = on_joint_drag
        self._on_drag_start_callback = on_drag_start
        self._on_drag_end_callback = on_drag_end

    def _on_left_button_press(self, obj, event):
        """鼠标左键按下事件"""
        # 获取点击位置
        click_pos = self.GetInteractor().GetEventPosition()

        # 获取 renderer
        renderer = self.GetInteractor().GetRenderWindow().GetRenderers().GetFirstRenderer()

        # 执行拾取
        self.picker.Pick(click_pos[0], click_pos[1], 0, renderer)
        picked_actor = self.picker.GetActor()

        if picked_actor and self._get_link_name_callback:
            # 获取 link 名称
            link_name = self._get_link_name_callback(picked_actor)

            if link_name and self._get_joint_for_link_callback:
                # 获取关联的关节
                joint_index, joint_info = self._get_joint_for_link_callback(link_name)

                if joint_index is not None:
                    # 进入拖拽模式
                    self.dragging = True
                    self.joint_index = joint_index
                    self.joint_info = joint_info
                    self.drag_start_pos = click_pos
                    self.dragging_link_name = link_name

                    # 触发拖拽开始回调（用于高亮显示）
                    if self._on_drag_start_callback:
                        self._on_drag_start_callback(link_name)

                    return  # 不执行默认的相机旋转

        # 默认行为：相机旋转
        self.OnLeftButtonDown()

    def _on_left_button_release(self, obj, event):
        """鼠标左键释放事件"""
        if self.dragging:
            # 触发拖拽结束回调（用于取消高亮）
            if self._on_drag_end_callback and self.dragging_link_name:
                self._on_drag_end_callback(self.dragging_link_name)

            # 退出拖拽模式
            self.dragging = False
            self.joint_index = None
            self.joint_info = None
            self.drag_start_pos = None
            self.dragging_link_name = None
        else:
            # 默认行为
            self.OnLeftButtonUp()

    def _on_mouse_move(self, obj, event):
        """鼠标移动事件"""
        if self.dragging and self.joint_index is not None:
            # 获取当前鼠标位置
            current_pos = self.GetInteractor().GetEventPosition()

            # 计算水平位移（用于控制角度）
            dx = current_pos[0] - self.drag_start_pos[0]

            # 将位移转换为角度变化
            delta_angle = dx * self.drag_sensitivity

            # 调用回调
            if self._on_joint_drag_callback:
                self._on_joint_drag_callback(self.joint_index, delta_angle)

            # 更新起始位置（增量模式）
            self.drag_start_pos = current_pos
        else:
            # 默认行为：相机移动
            self.OnMouseMove()
