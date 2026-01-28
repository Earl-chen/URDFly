#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
关节面板模块

右侧关节滑块面板，包括：
- 关节角度滑块
- 重置/随机按钮
- 单位切换 (rad/deg)
"""

import math
import numpy as np

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QLabel, QGroupBox, QSlider, QComboBox, QScrollArea
)
from PyQt5.QtCore import Qt, pyqtSignal


class JointPanel(QWidget):
    """右侧关节滑块面板"""

    # 信号定义
    joint_changed = pyqtSignal(int, float)  # (index, value_in_radians)
    reset_requested = pyqtSignal()
    random_requested = pyqtSignal()
    units_changed = pyqtSignal(str)  # 'rad' or 'deg'

    def __init__(self, translation_manager, parent=None):
        super().__init__(parent)
        self.tr_mgr = translation_manager

        self.joint_sliders = []
        self.joint_value_labels = []
        self.joint_values = []
        self.display_in_degrees = False

        self.init_ui()

    def init_ui(self):
        """初始化 UI"""
        layout = QVBoxLayout(self)

        # 关节控制组
        self.joint_group = QGroupBox(self.tr_mgr.tr("joint_group"))
        joint_group_layout = QVBoxLayout(self.joint_group)

        # 按钮行
        buttons_layout = QHBoxLayout()
        self.btn_reset = QPushButton("重置")
        self.btn_reset.clicked.connect(self.reset_requested.emit)
        self.btn_random = QPushButton("随机")
        self.btn_random.clicked.connect(self.random_requested.emit)
        buttons_layout.addWidget(self.btn_reset)
        buttons_layout.addWidget(self.btn_random)

        # 单位选择
        self.units_combo = QComboBox()
        self.units_combo.addItems(["rad", "deg"])
        self.units_combo.setCurrentText("rad")
        self.units_combo.currentTextChanged.connect(self._on_units_changed)
        buttons_layout.addWidget(self.units_combo)

        joint_group_layout.addLayout(buttons_layout)

        # 滚动区域
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)

        # 关节容器
        self.joint_container = QWidget()
        self.joint_layout = QVBoxLayout(self.joint_container)

        # 标签
        self.joint_label = QLabel(self.tr_mgr.tr("joint_control"))
        self.joint_layout.addWidget(self.joint_label)

        scroll_area.setWidget(self.joint_container)
        joint_group_layout.addWidget(scroll_area)

        layout.addWidget(self.joint_group, 1)
        self.setFixedWidth(300)

    def _on_units_changed(self, text):
        """单位变更处理"""
        self.display_in_degrees = (text == "deg")
        self._update_all_labels()
        self.units_changed.emit(text)

    def _update_all_labels(self):
        """更新所有关节值标签"""
        for i, label in enumerate(self.joint_value_labels):
            if i < len(self.joint_values):
                label.setText(self._format_angle(self.joint_values[i]))

    def _format_angle(self, angle_radians):
        """格式化角度显示"""
        if self.display_in_degrees:
            return f"{angle_radians * 180.0 / math.pi:.2f}°"
        return f"{angle_radians:.2f} rad"

    def create_sliders(self, revolute_joints):
        """创建关节滑块

        Args:
            revolute_joints: [{'name', 'index', 'lower', 'upper', ...}, ...] 列表
        """
        self.clear_sliders()
        self.joint_values = [0.0] * len(revolute_joints)

        for i, joint in enumerate(revolute_joints):
            lower = joint.get('lower', -math.pi)
            upper = joint.get('upper', math.pi)

            # 关节组框
            joint_box = QGroupBox(joint['name'])
            joint_box_layout = QVBoxLayout(joint_box)

            # 范围标签
            range_text = self.tr_mgr.tr("range_label")
            range_label = QLabel(range_text.format(self._format_angle(lower), self._format_angle(upper)))
            range_label.setStyleSheet("color: gray; font-size: 10px;")
            joint_box_layout.addWidget(range_label)

            # 滑块行
            slider_layout = QHBoxLayout()

            # 最小值标签
            min_label = QLabel(f"{lower:.2f}")
            min_label.setStyleSheet("color: gray; font-size: 9px;")
            min_label.setMaximumWidth(35)
            slider_layout.addWidget(min_label)

            # 当前值标签
            value_label = QLabel(self._format_angle(0.0))
            value_label.setMinimumWidth(60)
            value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

            # 滑块
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(lower * 100))
            slider.setMaximum(int(upper * 100))
            slider.setValue(0)
            slider.setTickPosition(QSlider.TicksBelow)

            # 计算刻度间隔
            range_val = upper - lower
            if range_val >= 2 * math.pi:
                tick_interval = int(math.pi / 2 * 100)
            elif range_val >= math.pi:
                tick_interval = int(math.pi / 4 * 100)
            else:
                tick_interval = int(range_val / 4 * 100)
            slider.setTickInterval(max(1, tick_interval))

            # 连接信号
            slider.valueChanged.connect(
                lambda val, idx=i, lbl=value_label: self._on_slider_changed(val, idx, lbl)
            )

            slider_layout.addWidget(slider, 1)

            # 最大值标签
            max_label = QLabel(f"{upper:.2f}")
            max_label.setStyleSheet("color: gray; font-size: 9px;")
            max_label.setMaximumWidth(35)
            max_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            slider_layout.addWidget(max_label)

            slider_layout.addWidget(value_label)
            joint_box_layout.addLayout(slider_layout)

            self.joint_layout.addWidget(joint_box)
            self.joint_sliders.append(slider)
            self.joint_value_labels.append(value_label)

    def _on_slider_changed(self, value, index, label):
        """滑块变化处理"""
        angle = value / 100.0
        label.setText(self._format_angle(angle))
        self.joint_values[index] = angle
        self.joint_changed.emit(index, angle)

    def clear_sliders(self):
        """清除所有滑块"""
        self.joint_values = []
        self.joint_sliders = []
        self.joint_value_labels = []

        # 移除所有控件（保留第一个标签）
        while self.joint_layout.count() > 1:
            item = self.joint_layout.takeAt(1)
            widget = item.widget()
            if widget:
                widget.deleteLater()

    def set_values(self, angles):
        """设置关节值

        Args:
            angles: 弧度角度值列表
        """
        for i, angle in enumerate(angles):
            if i < len(self.joint_sliders):
                self.joint_values[i] = angle
                target = int(round(angle * 100.0))
                slider = self.joint_sliders[i]
                clamped = max(slider.minimum(), min(slider.maximum(), target))

                # 阻止信号避免重复更新
                was_blocked = slider.blockSignals(True)
                slider.setValue(clamped)
                slider.blockSignals(was_blocked)

        self._update_all_labels()

    def get_values(self):
        """获取当前关节值（弧度）"""
        return self.joint_values.copy()

    def set_slider_value(self, index, angle):
        """设置单个滑块的值

        Args:
            index: 关节索引
            angle: 弧度角度值
        """
        if 0 <= index < len(self.joint_sliders):
            self.joint_values[index] = angle
            target = int(round(angle * 100.0))
            slider = self.joint_sliders[index]
            clamped = max(slider.minimum(), min(slider.maximum(), target))

            # 阻止信号避免重复更新
            was_blocked = slider.blockSignals(True)
            slider.setValue(clamped)
            slider.blockSignals(was_blocked)

            # 更新标签
            if index < len(self.joint_value_labels):
                self.joint_value_labels[index].setText(self._format_angle(angle))

    def reset_to_zero(self):
        """重置所有关节到零位"""
        for i, slider in enumerate(self.joint_sliders):
            slider.setValue(0)

    def randomize(self):
        """随机设置所有关节"""
        for slider in self.joint_sliders:
            random_value = np.random.randint(slider.minimum(), slider.maximum())
            slider.setValue(random_value)

    def update_translations(self):
        """更新翻译文本"""
        self.joint_group.setTitle(self.tr_mgr.tr("joint_group"))
        self.joint_label.setText(self.tr_mgr.tr("joint_control"))
