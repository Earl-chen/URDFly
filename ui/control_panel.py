#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
控制面板模块

左侧控制面板 UI，包括：
- 语言选择
- 运动链选择
- 连杆列表
- 文件操作按钮
- 可见性控制
- 透明度控制
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QListWidget, QListWidgetItem, QLabel, QGroupBox,
    QCheckBox, QSlider, QComboBox
)
from PyQt5.QtCore import Qt, pyqtSignal


class ControlPanel(QWidget):
    """左侧控制面板"""

    # 信号定义
    language_changed = pyqtSignal(str)  # 语言代码
    chain_selected = pyqtSignal(int)  # 链索引
    link_selected = pyqtSignal(str)  # 连杆名称

    open_file_requested = pyqtSignal()
    edit_file_requested = pyqtSignal()
    show_mdh_requested = pyqtSignal()
    decompose_requested = pyqtSignal()
    set_joints_requested = pyqtSignal()

    visual_toggled = pyqtSignal(bool)
    collision_toggled = pyqtSignal(bool)
    link_frames_toggled = pyqtSignal(bool)
    mdh_frames_toggled = pyqtSignal(bool)
    joint_axes_toggled = pyqtSignal(bool)
    com_toggled = pyqtSignal(bool)
    inertia_toggled = pyqtSignal(bool)

    transparency_changed = pyqtSignal(int)  # 0-100

    def __init__(self, translation_manager, parent=None):
        super().__init__(parent)
        self.tr_mgr = translation_manager
        self.init_ui()

    def init_ui(self):
        """初始化 UI"""
        layout = QVBoxLayout(self)

        # 语言选择
        lang_layout = QHBoxLayout()
        lang_label = QLabel(self.tr_mgr.tr("language"))
        self.language_combo = QComboBox()
        self.language_combo.addItem(self.tr_mgr.tr("lang_zh"), "zh_CN")
        self.language_combo.addItem(self.tr_mgr.tr("lang_en"), "en")
        self.language_combo.setCurrentIndex(0 if self.tr_mgr.current_language == "zh_CN" else 1)
        self.language_combo.currentIndexChanged.connect(self._on_language_changed)
        lang_layout.addWidget(lang_label)
        lang_layout.addWidget(self.language_combo)
        lang_layout.addStretch()
        layout.addLayout(lang_layout)

        # 运动链选择
        chain_layout = QVBoxLayout()
        chain_layout.addWidget(QLabel(self.tr_mgr.tr("select_chain")))
        self.chain_combo = QComboBox()
        self.chain_combo.currentIndexChanged.connect(self._on_chain_selected)
        chain_layout.addWidget(self.chain_combo)

        # 连杆列表
        chain_layout.addWidget(QLabel(self.tr_mgr.tr("links")))
        self.link_list = QListWidget()
        self.link_list.setSelectionMode(QListWidget.SingleSelection)
        self.link_list.itemSelectionChanged.connect(self._on_link_selected)
        chain_layout.addWidget(self.link_list)

        chain_widget = QWidget()
        chain_widget.setLayout(chain_layout)
        layout.addWidget(QLabel(self.tr_mgr.tr("robot_structure")))
        layout.addWidget(chain_widget)

        # 按钮
        self.btn_open = QPushButton(self.tr_mgr.tr("open_urdf"))
        self.btn_open.clicked.connect(self.open_file_requested.emit)
        layout.addWidget(self.btn_open)

        self.btn_edit = QPushButton(self.tr_mgr.tr("edit_urdf"))
        self.btn_edit.setToolTip(self.tr_mgr.tr("edit_urdf_tooltip"))
        self.btn_edit.clicked.connect(self.edit_file_requested.emit)
        layout.addWidget(self.btn_edit)

        self.btn_mdh = QPushButton(self.tr_mgr.tr("show_mdh"))
        self.btn_mdh.clicked.connect(self.show_mdh_requested.emit)
        layout.addWidget(self.btn_mdh)

        self.btn_decomp = QPushButton(self.tr_mgr.tr("decompose_collision"))
        self.btn_decomp.clicked.connect(self.decompose_requested.emit)
        layout.addWidget(self.btn_decomp)

        self.btn_set_joints = QPushButton(self.tr_mgr.tr("set_joints"))
        self.btn_set_joints.clicked.connect(self.set_joints_requested.emit)
        layout.addWidget(self.btn_set_joints)

        # 透明度控制
        transparency_group = QGroupBox("")
        transparency_layout = QVBoxLayout()
        self.transparency_label = QLabel(self.tr_mgr.tr("transparency"))
        transparency_layout.addWidget(self.transparency_label)
        self.transparency_slider = QSlider(Qt.Horizontal)
        self.transparency_slider.setMinimum(0)
        self.transparency_slider.setMaximum(100)
        self.transparency_slider.setValue(100)
        self.transparency_slider.setTickPosition(QSlider.TicksBelow)
        self.transparency_slider.setTickInterval(10)
        self.transparency_slider.valueChanged.connect(self.transparency_changed.emit)
        transparency_layout.addWidget(self.transparency_slider)
        transparency_group.setLayout(transparency_layout)
        layout.addWidget(transparency_group)

        # 可见性控制
        self.visibility_group = QGroupBox(self.tr_mgr.tr("visibility_settings"))
        visibility_layout = QVBoxLayout()

        self.cb_visual = QCheckBox(self.tr_mgr.tr("show_visual"))
        self.cb_visual.setChecked(True)
        self.cb_visual.stateChanged.connect(lambda s: self.visual_toggled.emit(s == Qt.Checked))
        visibility_layout.addWidget(self.cb_visual)

        self.cb_link_frames = QCheckBox(self.tr_mgr.tr("show_link_frames"))
        self.cb_link_frames.setChecked(True)
        self.cb_link_frames.stateChanged.connect(lambda s: self.link_frames_toggled.emit(s == Qt.Checked))
        visibility_layout.addWidget(self.cb_link_frames)

        self.cb_mdh_frames = QCheckBox(self.tr_mgr.tr("show_mdh_frames"))
        self.cb_mdh_frames.setChecked(False)
        self.cb_mdh_frames.stateChanged.connect(lambda s: self.mdh_frames_toggled.emit(s == Qt.Checked))
        visibility_layout.addWidget(self.cb_mdh_frames)

        self.cb_joint_axes = QCheckBox(self.tr_mgr.tr("show_joint_axes"))
        self.cb_joint_axes.setChecked(False)
        self.cb_joint_axes.stateChanged.connect(lambda s: self.joint_axes_toggled.emit(s == Qt.Checked))
        visibility_layout.addWidget(self.cb_joint_axes)

        self.cb_collision = QCheckBox(self.tr_mgr.tr("show_collision"))
        self.cb_collision.setChecked(True)
        self.cb_collision.stateChanged.connect(lambda s: self.collision_toggled.emit(s == Qt.Checked))
        visibility_layout.addWidget(self.cb_collision)

        self.cb_com = QCheckBox(self.tr_mgr.tr("show_com"))
        self.cb_com.setChecked(False)
        self.cb_com.stateChanged.connect(lambda s: self.com_toggled.emit(s == Qt.Checked))
        visibility_layout.addWidget(self.cb_com)

        self.cb_inertia = QCheckBox(self.tr_mgr.tr("show_inertia"))
        self.cb_inertia.setChecked(False)
        self.cb_inertia.stateChanged.connect(lambda s: self.inertia_toggled.emit(s == Qt.Checked))
        visibility_layout.addWidget(self.cb_inertia)

        self.visibility_group.setLayout(visibility_layout)
        layout.addWidget(self.visibility_group)

        layout.addStretch()

        # 当前文件标签
        self.current_file_label = QLabel(
            self.tr_mgr.tr("current_file") + " " + self.tr_mgr.tr("current_file_none")
        )
        self.current_file_label.setWordWrap(True)
        self.current_file_label.setAlignment(Qt.AlignLeft | Qt.AlignBottom)
        layout.addWidget(self.current_file_label)

        self.setFixedWidth(300)

    def _on_language_changed(self, index):
        """语言变更处理"""
        lang_code = self.language_combo.itemData(index)
        self.language_changed.emit(lang_code)

    def _on_chain_selected(self, index):
        """运动链选择处理"""
        if index >= 0:
            self.chain_selected.emit(index)

    def _on_link_selected(self):
        """连杆选择处理"""
        selected_items = self.link_list.selectedItems()
        if selected_items:
            link_name = selected_items[0].data(Qt.UserRole)
            self.link_selected.emit(link_name)

    def populate_chains(self, chains):
        """填充运动链列表

        Args:
            chains: [(index, name), ...] 列表
        """
        self.chain_combo.clear()
        for i, name in chains:
            chain_label = self.tr_mgr.tr("chain_pattern")
            self.chain_combo.addItem(chain_label.format(i + 1, name), i)

        if chains:
            self.chain_combo.setCurrentIndex(0)

    def populate_links(self, link_names):
        """填充连杆列表

        Args:
            link_names: 连杆名称列表
        """
        self.link_list.clear()
        for link_name in link_names:
            item = QListWidgetItem(link_name)
            item.setData(Qt.UserRole, link_name)
            self.link_list.addItem(item)

    def clear_chains(self):
        """清除运动链和连杆列表"""
        self.chain_combo.clear()
        self.link_list.clear()

    def update_current_file(self, filename):
        """更新当前文件标签

        Args:
            filename: 文件名（None 表示无文件）
        """
        if filename:
            self.current_file_label.setText(
                self.tr_mgr.tr("current_file") + f" {filename}"
            )
        else:
            self.current_file_label.setText(
                self.tr_mgr.tr("current_file") + " " + self.tr_mgr.tr("current_file_none")
            )

    def set_mdh_frames_checked(self, checked):
        """设置 MDH 坐标系复选框状态"""
        self.cb_mdh_frames.setChecked(checked)

    def set_joint_axes_checked(self, checked):
        """设置关节轴复选框状态"""
        self.cb_joint_axes.setChecked(checked)

    def set_com_checked(self, checked):
        """设置质心复选框状态"""
        self.cb_com.setChecked(checked)

    def set_inertia_checked(self, checked):
        """设置惯量盒复选框状态"""
        self.cb_inertia.setChecked(checked)

    def update_translations(self):
        """更新所有翻译文本"""
        self.btn_open.setText(self.tr_mgr.tr("open_urdf"))
        self.btn_edit.setText(self.tr_mgr.tr("edit_urdf"))
        self.btn_edit.setToolTip(self.tr_mgr.tr("edit_urdf_tooltip"))
        self.btn_mdh.setText(self.tr_mgr.tr("show_mdh"))
        self.btn_decomp.setText(self.tr_mgr.tr("decompose_collision"))
        self.btn_set_joints.setText(self.tr_mgr.tr("set_joints"))
        self.transparency_label.setText(self.tr_mgr.tr("transparency"))
        self.visibility_group.setTitle(self.tr_mgr.tr("visibility_settings"))
        self.cb_visual.setText(self.tr_mgr.tr("show_visual"))
        self.cb_link_frames.setText(self.tr_mgr.tr("show_link_frames"))
        self.cb_mdh_frames.setText(self.tr_mgr.tr("show_mdh_frames"))
        self.cb_joint_axes.setText(self.tr_mgr.tr("show_joint_axes"))
        self.cb_collision.setText(self.tr_mgr.tr("show_collision"))
        self.cb_com.setText(self.tr_mgr.tr("show_com"))
        self.cb_inertia.setText(self.tr_mgr.tr("show_inertia"))
