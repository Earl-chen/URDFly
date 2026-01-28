#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
URDFly - URDF/MJCF 机器人模型查看器

主窗口框架和事件协调
"""

import sys
import os
import math

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QFileDialog, QMessageBox, QDialog, QLineEdit, QLabel, QPushButton, QComboBox,
    QDesktopWidget
)
from PyQt5.QtCore import Qt, QUrl
from PyQt5.QtGui import QDragEnterEvent, QDropEvent
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import vtk

from translations import TranslationManager
from xml_editor import XMLEditor
from mdh_dialog import MDHDialog
from decomp_dialog import DecompDialog

from renderer import SceneManager
from ui import ControlPanel, JointPanel


class DragDropVTKWidget(QVTKRenderWindowInteractor):
    """支持拖放的 VTK 窗口控件"""

    def __init__(self, parent=None):
        super().__init__()
        self.parent_viewer = parent
        self.setAcceptDrops(True)

    def dragEnterEvent(self, event: QDragEnterEvent):
        """处理拖入事件"""
        if event.mimeData().hasUrls():
            for url in event.mimeData().urls():
                if url.isLocalFile():
                    file_path = url.toLocalFile()
                    if file_path.lower().endswith('.urdf') or file_path.lower().endswith('.xml'):
                        event.acceptProposedAction()
                        return
        event.ignore()

    def dropEvent(self, event: QDropEvent):
        """处理放下事件"""
        if event.mimeData().hasUrls():
            for url in event.mimeData().urls():
                if url.isLocalFile():
                    file_path = url.toLocalFile()
                    if file_path.lower().endswith('.urdf') or file_path.lower().endswith('.xml'):
                        if self.parent_viewer:
                            self.parent_viewer.load_robot_file(file_path)
                        event.acceptProposedAction()
                        return
        event.ignore()


class URDFViewer(QMainWindow):
    """主应用窗口"""

    def __init__(self):
        super().__init__()

        # 翻译管理器
        self.translation_manager = TranslationManager()

        # XML 编辑器引用
        self.editor = None

        self.init_ui()
        self.connect_signals()

    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle(self.translation_manager.tr("window_title"))

        # 设置窗口大小和位置
        window_width, window_height = 1200, 800
        screen = QDesktopWidget().availableGeometry()
        x = (screen.width() - window_width) // 2
        y = (screen.height() - window_height) // 2
        self.setGeometry(x, y, window_width, window_height)

        # 创建中央控件和主布局
        central_widget = QWidget()
        main_layout = QHBoxLayout(central_widget)

        # 左侧控制面板
        self.control_panel = ControlPanel(self.translation_manager)
        main_layout.addWidget(self.control_panel)

        # 中间 VTK 窗口
        self.vtk_widget = DragDropVTKWidget(self)
        main_layout.addWidget(self.vtk_widget, 1)

        # 右侧关节面板
        self.joint_panel = JointPanel(self.translation_manager)
        main_layout.addWidget(self.joint_panel)

        self.setCentralWidget(central_widget)

        # 设置 VTK 渲染器
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.9, 0.9, 0.9)
        self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)

        # 设置交互器
        self.interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        style = vtk.vtkInteractorStyleTrackballCamera()
        self.interactor.SetInteractorStyle(style)
        self.interactor.Initialize()
        self.interactor.Start()

        # 初始化场景管理器
        self.scene = SceneManager(self.renderer)

    def connect_signals(self):
        """连接信号"""
        # 控制面板信号
        self.control_panel.language_changed.connect(self.on_language_changed)
        self.control_panel.chain_selected.connect(self.on_chain_selected)
        self.control_panel.link_selected.connect(self.on_link_selected)

        self.control_panel.open_file_requested.connect(self.open_file_dialog)
        self.control_panel.edit_file_requested.connect(self.edit_urdf_file)
        self.control_panel.show_mdh_requested.connect(self.show_mdh_parameters)
        self.control_panel.decompose_requested.connect(self.decompose_collision_meshes)
        self.control_panel.set_joints_requested.connect(self.open_set_joints_dialog)

        self.control_panel.visual_toggled.connect(self.on_visual_toggled)
        self.control_panel.collision_toggled.connect(self.on_collision_toggled)
        self.control_panel.link_frames_toggled.connect(self.on_link_frames_toggled)
        self.control_panel.mdh_frames_toggled.connect(self.on_mdh_frames_toggled)
        self.control_panel.joint_axes_toggled.connect(self.on_joint_axes_toggled)
        self.control_panel.com_toggled.connect(self.on_com_toggled)
        self.control_panel.inertia_toggled.connect(self.on_inertia_toggled)

        self.control_panel.transparency_changed.connect(self.on_transparency_changed)

        # 关节面板信号
        self.joint_panel.joint_changed.connect(self.on_joint_changed)
        self.joint_panel.reset_requested.connect(self.reset_joints)
        self.joint_panel.random_requested.connect(self.randomize_joints)

    # === 文件操作 ===

    def open_file_dialog(self):
        """打开文件对话框"""
        filename, _ = QFileDialog.getOpenFileName(
            self, self.translation_manager.tr("dialog_open_robot"),
            "", self.translation_manager.tr("dialog_robot_filter")
        )
        if filename:
            self.load_robot_file(filename)

    def load_robot_file(self, filepath):
        """加载机器人文件"""
        if not filepath or not os.path.exists(filepath):
            return

        # 清除之前的状态
        self.control_panel.clear_chains()
        self.joint_panel.clear_sliders()

        # 加载机器人
        success = self.scene.load_robot(filepath)

        if success:
            # 更新 UI
            self._update_ui_after_load()
            self.scene.reset_camera()
        else:
            # 根据文件类型显示错误信息
            if filepath.lower().endswith('.urdf'):
                msg = self.translation_manager.tr("load_urdf_failed")
            else:
                msg = self.translation_manager.tr("load_mjcf_failed")
            QMessageBox.critical(
                self, self.translation_manager.tr("error"),
                msg.replace("{}", "Unknown error")
            )

    def _update_ui_after_load(self):
        """加载后更新 UI"""
        # 更新当前文件标签
        if self.scene.current_file:
            filename = os.path.basename(self.scene.current_file)
            self.control_panel.update_current_file(filename)
        else:
            self.control_panel.update_current_file(None)

        # 填充运动链
        chains = self.scene.get_chain_names()
        self.control_panel.populate_chains(chains)

        # 选择第一个链
        if chains:
            self.scene.select_chain(0)
            links = self.scene.get_selected_chain_links()
            self.control_panel.populate_links(links)

        # 创建关节滑块
        revolute_joints = self.scene.get_revolute_joints()
        self.joint_panel.create_sliders(revolute_joints)

    def edit_urdf_file(self):
        """编辑 URDF 文件"""
        if not self.scene.current_file:
            QMessageBox.warning(
                self, self.translation_manager.tr("warning"),
                self.translation_manager.tr("load_urdf_edit")
            )
            return

        self.editor = XMLEditor(self.scene.current_file, self.update_model_from_xml)
        self.editor.show()

    def update_model_from_xml(self, xml_content):
        """从 XML 内容更新模型"""
        try:
            # 创建临时文件
            if '_temp.urdf' not in self.scene.current_file:
                temp_path = self.scene.current_file.lower().replace('.urdf', '_temp.urdf')
            else:
                temp_path = self.scene.current_file

            with open(temp_path, 'w', encoding='utf-8') as f:
                f.write(xml_content)

            # 重新加载
            self.load_robot_file(temp_path)

        except Exception as e:
            msg = self.translation_manager.tr("update_model_failed")
            QMessageBox.critical(
                self, self.translation_manager.tr("error"),
                msg.replace("{}", str(e))
            )

    # === 可见性控制 ===

    def on_visual_toggled(self, visible):
        """视觉模型可见性切换"""
        self.scene.set_visual_visibility(visible)
        self.scene.redraw()

    def on_collision_toggled(self, visible):
        """碰撞体可见性切换"""
        self.scene.set_collision_visibility(visible)
        self.scene.redraw()

    def on_link_frames_toggled(self, visible):
        """连杆坐标系可见性切换"""
        self.scene.set_link_frames_visibility(visible)
        self.scene.redraw()

    def on_mdh_frames_toggled(self, visible):
        """MDH 坐标系可见性切换"""
        if visible:
            if not self.scene.current_file:
                QMessageBox.warning(
                    self, self.translation_manager.tr("warning"),
                    self.translation_manager.tr("load_urdf_mdh")
                )
                self.control_panel.set_mdh_frames_checked(False)
                return

            if not self.scene.is_urdf():
                QMessageBox.warning(
                    self, self.translation_manager.tr("warning"),
                    self.translation_manager.tr("mdh_urdf_only")
                )
                self.control_panel.set_mdh_frames_checked(False)
                return

            if not self.scene.selected_chain:
                QMessageBox.warning(
                    self, self.translation_manager.tr("warning"),
                    self.translation_manager.tr("select_chain_warning")
                )
                self.control_panel.set_mdh_frames_checked(False)
                return

        self.scene.set_mdh_frames_visibility(visible)
        self.scene.redraw()

    def on_joint_axes_toggled(self, visible):
        """关节轴可见性切换"""
        if visible and not self.scene.current_file:
            QMessageBox.warning(
                self, self.translation_manager.tr("warning"),
                self.translation_manager.tr("load_urdf_joint_axes")
            )
            self.control_panel.set_joint_axes_checked(False)
            return

        self.scene.set_joint_axes_visibility(visible)
        self.scene.redraw()

    def on_com_toggled(self, visible):
        """质心可见性切换"""
        if visible and not self.scene.current_file:
            QMessageBox.warning(
                self, self.translation_manager.tr("warning"),
                self.translation_manager.tr("load_urdf_com")
            )
            self.control_panel.set_com_checked(False)
            return

        self.scene.set_com_visibility(visible)
        self.scene.redraw()

    def on_inertia_toggled(self, visible):
        """惯量盒可见性切换"""
        if visible and not self.scene.current_file:
            QMessageBox.warning(
                self, self.translation_manager.tr("warning"),
                self.translation_manager.tr("load_urdf_inertia")
            )
            self.control_panel.set_inertia_checked(False)
            return

        self.scene.set_inertia_visibility(visible)
        self.scene.redraw()

    def on_transparency_changed(self, value):
        """透明度变更"""
        opacity = value / 100.0
        self.scene.set_transparency(opacity)
        self.scene.redraw()

    # === 链和连杆选择 ===

    def on_chain_selected(self, index):
        """运动链选择"""
        self.scene.unhighlight_all()
        self.scene.select_chain(index)
        links = self.scene.get_selected_chain_links()
        self.control_panel.populate_links(links)
        self.scene.redraw()

    def on_link_selected(self, link_name):
        """连杆选择"""
        self.scene.unhighlight_all()
        self.scene.highlight_link(link_name)
        self.scene.redraw()

    # === 关节控制 ===

    def on_joint_changed(self, index, value):
        """关节角度变更"""
        joint_values = self.joint_panel.get_values()
        self.scene.update_joint_angles(joint_values)
        self.scene.redraw()

    def reset_joints(self):
        """重置关节到零位"""
        self.joint_panel.reset_to_zero()
        joint_values = self.joint_panel.get_values()
        self.scene.update_joint_angles(joint_values)
        self.scene.redraw()

    def randomize_joints(self):
        """随机设置关节"""
        self.joint_panel.randomize()
        joint_values = self.joint_panel.get_values()
        self.scene.update_joint_angles(joint_values)
        self.scene.redraw()

    def open_set_joints_dialog(self):
        """打开设置关节对话框"""
        if not self.scene.revolute_joints:
            QMessageBox.warning(
                self, self.translation_manager.tr("warning"),
                self.translation_manager.tr("no_revolute_joints")
            )
            return

        dialog = QDialog(self)
        dialog.setWindowTitle(self.translation_manager.tr("set_joints_title"))
        vbox = QVBoxLayout(dialog)

        vbox.addWidget(QLabel(self.translation_manager.tr("input_joints_prompt")))
        angles_edit = QLineEdit()

        # 预填充当前值
        current_vals = self.joint_panel.get_values()
        display_in_deg = self.joint_panel.display_in_degrees
        if display_in_deg:
            vals_str = ", ".join([f"{v * 180.0 / math.pi:.2f}" for v in current_vals])
        else:
            vals_str = ", ".join([f"{v:.2f}" for v in current_vals])
        angles_edit.setText(vals_str)
        vbox.addWidget(angles_edit)

        # 单位选择
        units_row = QHBoxLayout()
        units_row.addWidget(QLabel(self.translation_manager.tr("units_label")))
        units_combo = QComboBox()
        units_combo.addItems(["rad", "deg"])
        units_combo.setCurrentText("deg" if display_in_deg else "rad")
        units_row.addWidget(units_combo)
        units_row.addStretch(1)
        vbox.addLayout(units_row)

        # 按钮
        buttons_row = QHBoxLayout()
        btn_apply = QPushButton(self.translation_manager.tr("btn_apply"))
        btn_cancel = QPushButton(self.translation_manager.tr("btn_cancel"))
        buttons_row.addStretch(1)
        buttons_row.addWidget(btn_cancel)
        buttons_row.addWidget(btn_apply)
        vbox.addLayout(buttons_row)

        def parse_and_apply():
            raw = angles_edit.text().strip()
            if not raw:
                QMessageBox.warning(dialog, self.translation_manager.tr("warning"),
                                   self.translation_manager.tr("input_angles_warning"))
                return

            tokens = raw.replace(",", " ").split()
            try:
                vals = [float(t) for t in tokens]
            except ValueError:
                QMessageBox.critical(dialog, self.translation_manager.tr("error"),
                                    self.translation_manager.tr("input_numbers_error"))
                return

            n = len(self.scene.revolute_joints)
            if len(vals) != n:
                msg = self.translation_manager.tr("input_count_warning")
                QMessageBox.warning(dialog, self.translation_manager.tr("warning"),
                                   msg.format(n, len(vals)))
                return

            # 转换为弧度
            if units_combo.currentText() == "deg":
                vals = [v * math.pi / 180.0 for v in vals]

            self.joint_panel.set_values(vals)
            self.scene.update_joint_angles(vals)
            self.scene.redraw()
            dialog.accept()

        btn_apply.clicked.connect(parse_and_apply)
        btn_cancel.clicked.connect(dialog.reject)
        dialog.exec_()

    # === MDH 参数 ===

    def show_mdh_parameters(self):
        """显示 MDH 参数"""
        if not self.scene.selected_chain:
            QMessageBox.warning(
                self, self.translation_manager.tr("warning"),
                self.translation_manager.tr("select_chain_mdh_warning")
            )
            return

        if not self.scene.current_file:
            QMessageBox.warning(
                self, self.translation_manager.tr("warning"),
                self.translation_manager.tr("load_urdf_show_mdh")
            )
            return

        if not self.scene.is_urdf():
            QMessageBox.warning(
                self, self.translation_manager.tr("warning"),
                self.translation_manager.tr("mdh_urdf_only")
            )
            return

        parser = self.scene.get_parser()
        chains, _ = parser.get_chain_info()

        # 找到匹配的链
        current_chain = None
        for chain in chains:
            if chain['name'] == self.scene.selected_chain['name']:
                current_chain = chain
                break

        if current_chain is None:
            current_chain = self.scene.selected_chain

        _, _, _, mdh_parameters = parser.get_mdh_parameters(current_chain)
        dialog = MDHDialog(self, current_chain['name'], mdh_parameters)
        dialog.exec_()

    # === 碰撞体分解 ===

    def decompose_collision_meshes(self):
        """分解碰撞体"""
        if not self.scene.collision_mesh_files:
            QMessageBox.warning(
                self, self.translation_manager.tr("warning"),
                self.translation_manager.tr("no_collision_meshes")
            )
            return

        dialog = DecompDialog(self, self.scene.collision_mesh_files)
        result = dialog.exec_()

        if result is not None:
            # 重新加载并替换碰撞体
            self.editor = XMLEditor(self.scene.current_file, self.update_model_from_xml)
            self.editor.replace_collision()
            self.editor.show()

    # === 语言切换 ===

    def on_language_changed(self, lang_code):
        """语言变更"""
        self.translation_manager.set_language(lang_code, self)
        self.setWindowTitle(self.translation_manager.tr("window_title"))
        self.control_panel.update_translations()
        self.joint_panel.update_translations()

    # === 窗口关闭 ===

    def closeEvent(self, event):
        """窗口关闭事件"""
        if hasattr(self, 'editor') and self.editor is not None:
            self.editor.close()

        self.vtk_widget.GetRenderWindow().Finalize()
        self.vtk_widget.close()

        QApplication.quit()
        event.accept()


def main():
    """主函数"""
    app = QApplication(sys.argv)
    viewer = URDFViewer()
    viewer.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
