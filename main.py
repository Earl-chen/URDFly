#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import os
import math
import numpy as np
import tempfile
from math import pi
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QListWidget,
    QListWidgetItem,
    QFileDialog,
    QLabel,
    QLineEdit,
    QGridLayout,
    QGroupBox,
    QMessageBox,
    QSlider,
    QTreeWidget,
    QTreeWidgetItem,
    QCheckBox,
    QDialog,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QComboBox,
    QScrollArea,
    QDesktopWidget,
)
from xml_editor import XMLEditor
from mdh_dialog import MDHDialog
from decomp_dialog import DecompDialog
from translations import TranslationManager, tr as _
from PyQt5.QtCore import Qt, QUrl
from PyQt5.QtGui import QDragEnterEvent, QDropEvent
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import vtk

from urdf_parser import URDFParser
from urdf_vtk_model import URDFModel
from simplify_mesh import create_detailed_approximation


class DragDropVTKWidget(QVTKRenderWindowInteractor):
    """Custom VTK widget that supports drag-and-drop of URDF files"""
    
    def __init__(self, parent=None):
        super().__init__()
        self.parent_viewer = parent
        self.setAcceptDrops(True)
    
    def dragEnterEvent(self, event: QDragEnterEvent):
        """Handle drag enter event"""
        if event.mimeData().hasUrls():
            # Check if any of the dragged files is a URDF file
            for url in event.mimeData().urls():
                if url.isLocalFile():
                    file_path = url.toLocalFile()
                    if file_path.lower().endswith('.urdf'):
                        event.acceptProposedAction()
                        return
        event.ignore()
    
    def dropEvent(self, event: QDropEvent):
        """Handle drop event"""
        if event.mimeData().hasUrls():
            for url in event.mimeData().urls():
                if url.isLocalFile():
                    file_path = url.toLocalFile()
                    if file_path.lower().endswith('.urdf'):
                        # Load the URDF file
                        if self.parent_viewer:
                            self.parent_viewer.load_urdf_file(file_path)
                        event.acceptProposedAction()
                        return
        event.ignore()


def _update_transform_for_actor(actor, transform_matrix):
    """Helper function to update actor transform (module-level for lambda use)"""
    vtk_transform = vtk.vtkTransform()
    vtk_transform.SetMatrix(transform_matrix.flatten())
    actor.SetUserTransform(vtk_transform)


class URDFViewer(QMainWindow):
    """Main application window for URDF viewer"""

    def __init__(self):
        super().__init__()
        self.models = []  # List to store loaded URDF models
        self.models_collision = []
        self.chains = []  # List to store kinematic chains
        self.mdh_frames_actors = []  # List to store MDH frame actors
        self.mdh_text_actors = []  # List to store MDH frame text actors

        # === 新增：质心(COM)可视化相关 ===
        self.com_actors = []  # List to store center of mass actors
        # Store COM actor info: {'actor': actor, 'link_name': name, 'T_com_rel': T_com}
        # T_com_rel 是质心相对于连杆坐标系的变换矩阵
        self.com_actor_info = []

        # === 新增：惯量盒可视化相关 ===
        self.inertia_actors = []  # List to store inertia box actors
        # Store inertia actor info: {'actor': actor, 'link_name': name, 'T_com_rel': T_com}
        # T_com_rel 是惯量盒中心相对于连杆坐标系的变换矩阵
        self.inertia_actor_info = []

        # === 新增：关节轴可视化相关 ===
        self.joint_axis_actors = []  # List to store joint axis actors
        # Store joint axis info: {'actor': actor, 'joint_name': name, 'axis': axis}
        # axis 是关节的旋转轴方向（在关节局部坐标系中）
        self.joint_axis_info = []

        self.selected_chain = None  # Currently selected chain
        self.selected_chain_index = 0  # Index of the currently selected chain
        self.current_urdf_file = None  # Path to the currently loaded URDF file
        self.collision_mesh_files = None
        self.inertial_data = []  # List to store inertial data for each link

        self.joint_sliders = []  # List to store joint angle sliders
        self.joint_values = []  # List to store joint angle values
        self.revolute_joints = []  # List to store revolute joints
        self.joint_value_labels = []  # List to store joint value labels
        self.display_in_degrees = False  # False: rad, True: deg

        # === 新增：翻译管理 ===
        self.translation_manager = TranslationManager()
        self.init_ui()
        

    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle(self.translation_manager.tr("window_title"))

        # Set window size
        window_width = 1200
        window_height = 800

        # Get screen size and calculate center position
        screen = QDesktopWidget().availableGeometry()
        x = (screen.width() - window_width) // 2
        y = (screen.height() - window_height) // 2

        # Set window geometry to be centered on screen
        self.setGeometry(x, y, window_width, window_height)

        # Create central widget and main layout
        central_widget = QWidget()
        main_layout = QHBoxLayout(central_widget)

        # Create left panel for controls
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        # Create right panel for joint controls
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)

        # === 新增：语言切换控件 ===
        lang_layout = QHBoxLayout()
        lang_label = QLabel(self.translation_manager.tr("language"))
        self.language_combo = QComboBox()
        self.language_combo.addItem(self.translation_manager.tr("lang_zh"), "zh_CN")
        self.language_combo.addItem(self.translation_manager.tr("lang_en"), "en")
        self.language_combo.setCurrentIndex(0 if self.translation_manager.current_language == "zh_CN" else 1)
        self.language_combo.currentIndexChanged.connect(self.change_language)
        lang_layout.addWidget(lang_label)
        lang_layout.addWidget(self.language_combo)
        lang_layout.addStretch()
        left_layout.addLayout(lang_layout)

        # Create chain selection combo box
        chain_selection_layout = QVBoxLayout()
        chain_selection_layout.addWidget(QLabel(self.translation_manager.tr("select_chain")))
        self.chain_combo = QComboBox()
        self.chain_combo.currentIndexChanged.connect(self.on_chain_selected)
        chain_selection_layout.addWidget(self.chain_combo)

        # Create link list widget
        chain_selection_layout.addWidget(QLabel(self.translation_manager.tr("links")))
        self.link_list = QListWidget()
        self.link_list.setSelectionMode(QListWidget.SingleSelection)
        self.link_list.itemSelectionChanged.connect(self.on_link_selection_changed)
        chain_selection_layout.addWidget(self.link_list)

        # Create a widget to hold the chain selection layout
        chain_selection_widget = QWidget()
        chain_selection_widget.setLayout(chain_selection_layout)

        # Create button for opening URDF file
        btn_open = QPushButton(self.translation_manager.tr("open_urdf"))
        self.btn_open = btn_open  # 保存引用以便后续翻译
        btn_open.clicked.connect(self.open_urdf_file)

        # Create Edit button
        btn_edit = QPushButton(self.translation_manager.tr("edit_urdf"))
        self.btn_edit = btn_edit  # 保存引用
        btn_edit.clicked.connect(self.edit_urdf_file)
        btn_edit.setToolTip(self.translation_manager.tr("edit_urdf_tooltip"))

        # Create MDH button
        btn_mdh = QPushButton(self.translation_manager.tr("show_mdh"))
        btn_mdh.clicked.connect(self.show_mdh_parameters)

        btn_decomp = QPushButton(self.translation_manager.tr("decompose_collision"))
        btn_decomp.clicked.connect(self.decompose_collision_meshes)

        btn_set_joints = QPushButton(self.translation_manager.tr("set_joints"))
        btn_set_joints.clicked.connect(self.open_set_joints_dialog)

        # Create transparency controls
        transparency_group = QGroupBox("")
        transparency_layout = QVBoxLayout()

        # Transparency slider
        transparency_label = QLabel(self.translation_manager.tr("transparency"))
        transparency_layout.addWidget(transparency_label)
        self.transparency_slider = QSlider(Qt.Horizontal)
        self.transparency_slider.setMinimum(0)
        self.transparency_slider.setMaximum(100)
        self.transparency_slider.setValue(100)  # Default to fully opaque
        self.transparency_slider.setTickPosition(QSlider.TicksBelow)
        self.transparency_slider.setTickInterval(10)
        self.transparency_slider.valueChanged.connect(self.apply_transparency)
        transparency_layout.addWidget(self.transparency_slider)
        self.transparency_label = transparency_label  # 保存引用

        transparency_group.setLayout(transparency_layout)

        # Create checkboxes for frame visibility
        visibility_group = QGroupBox(self.translation_manager.tr("visibility_settings"))
        visibility_layout = QVBoxLayout()

        # === 新增：视觉显示控制 ===
        self.cb_visual = QCheckBox(self.translation_manager.tr("show_visual"))
        self.cb_visual.setChecked(True)
        self.cb_visual.stateChanged.connect(self.toggle_visual)

        # === 新增：关节轴显示控制 ===
        self.cb_joint_axes = QCheckBox(self.translation_manager.tr("show_joint_axes"))
        self.cb_joint_axes.setChecked(False)
        self.cb_joint_axes.stateChanged.connect(self.toggle_joint_axes)

        # === 新增：碰撞体显示控制 ===
        self.cb_collision = QCheckBox(self.translation_manager.tr("show_collision"))
        self.cb_collision.setChecked(True)
        self.cb_collision.stateChanged.connect(self.toggle_collision)

        # === 新增：质心显示控制 ===
        self.cb_com = QCheckBox(self.translation_manager.tr("show_com"))
        self.cb_com.setChecked(False)
        self.cb_com.stateChanged.connect(self.toggle_com)

        # === 新增：惯量显示控制 ===
        self.cb_inertia = QCheckBox(self.translation_manager.tr("show_inertia"))
        self.cb_inertia.setChecked(False)
        self.cb_inertia.stateChanged.connect(self.toggle_inertia)

        self.cb_link_frames = QCheckBox(self.translation_manager.tr("show_link_frames"))
        self.cb_link_frames.setChecked(True)
        self.cb_link_frames.stateChanged.connect(self.toggle_link_frames)

        # === 新增：MDH坐标系显示控制 ===
        self.cb_mdh_frames = QCheckBox(self.translation_manager.tr("show_mdh_frames"))
        self.cb_mdh_frames.setChecked(False)
        self.cb_mdh_frames.stateChanged.connect(self.toggle_mdh_frames)

        visibility_layout.addWidget(self.cb_visual)
        visibility_layout.addWidget(self.cb_link_frames)
        visibility_layout.addWidget(self.cb_mdh_frames)
        visibility_layout.addWidget(self.cb_joint_axes)
        visibility_layout.addWidget(self.cb_collision)
        visibility_layout.addWidget(self.cb_com)
        visibility_layout.addWidget(self.cb_inertia)

        visibility_group.setLayout(visibility_layout)

        # === 保存控件引用以便翻译 ===
        self.btn_mdh = btn_mdh
        self.btn_decomp = btn_decomp
        self.btn_set_joints = btn_set_joints
        self.visibility_group = visibility_group

        # Add widgets to left panel
        left_layout.addWidget(QLabel(self.translation_manager.tr("robot_structure")))
        left_layout.addWidget(chain_selection_widget)
        left_layout.addWidget(btn_open)
        left_layout.addWidget(btn_edit)
        left_layout.addWidget(btn_mdh)
        left_layout.addWidget(btn_decomp)
        left_layout.addWidget(btn_set_joints)

        left_layout.addWidget(transparency_group)
        left_layout.addWidget(visibility_group)
        left_layout.addStretch()

        # Add current file label at the bottom of left panel
        self.current_file_label = QLabel(self.translation_manager.tr("current_file") + " " + self.translation_manager.tr("current_file_none"))
        self.current_file_label.setWordWrap(True)
        self.current_file_label.setAlignment(Qt.AlignLeft | Qt.AlignBottom)
        left_layout.addWidget(self.current_file_label)

        # Set fixed width for left panel
        left_panel.setFixedWidth(300)

        # Create VTK widget for 3D visualization with drag-and-drop support
        self.vtk_widget = DragDropVTKWidget(self)

        # Create a scroll area for joint controls
        joint_scroll_area = QScrollArea()
        joint_scroll_area.setWidgetResizable(True)
        joint_scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        joint_scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        
        # Create a widget to hold all joint controls
        self.joint_container = QWidget()
        self.joint_layout = QVBoxLayout(self.joint_container)
        
        # Add a label explaining the sliders
        self.joint_label = QLabel(self.translation_manager.tr("joint_control"))
        self.joint_layout.addWidget(self.joint_label)

        # We'll add sliders dynamically when a URDF is loaded

        # Set the container as the scroll area's widget
        joint_scroll_area.setWidget(self.joint_container)

        # Create a group box to contain the scroll area
        self.joint_group = QGroupBox(self.translation_manager.tr("joint_group"))
        joint_group_layout = QVBoxLayout(self.joint_group)

        # Add reset and random buttons
        buttons_layout = QHBoxLayout()
        self.btn_reset = QPushButton("重置")
        self.btn_reset.clicked.connect(self.reset_joints)
        self.btn_random = QPushButton("随机")
        self.btn_random.clicked.connect(self.randomize_joints)
        buttons_layout.addWidget(self.btn_reset)
        buttons_layout.addWidget(self.btn_random)
        # Units toggle next to Reset/Random
        self.units_combo = QComboBox()
        self.units_combo.addItems(["rad", "deg"])
        self.units_combo.setCurrentText("rad")
        self.units_combo.currentTextChanged.connect(self.on_units_changed)
        buttons_layout.addWidget(self.units_combo)
        joint_group_layout.addLayout(buttons_layout)
        
        joint_group_layout.addWidget(joint_scroll_area)
        
        # Add joint group to right panel
        right_layout.addWidget(self.joint_group, 1)  # Give it a stretch factor of 1
        
        # Set fixed width for right panel
        right_panel.setFixedWidth(300)

        # Add panels to main layout
        main_layout.addWidget(left_panel)
        main_layout.addWidget(self.vtk_widget, 1)
        main_layout.addWidget(right_panel)

        # Set central widget
        self.setCentralWidget(central_widget)

        # Set up VTK rendering
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.9, 0.9, 0.9)  # Dark gray background
        self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)

        # Set up interactor
        self.interactor = self.vtk_widget.GetRenderWindow().GetInteractor()

        # Configure interactor style for rotation with left mouse button
        style = vtk.vtkInteractorStyleTrackballCamera()
        self.interactor.SetInteractorStyle(style)

        # Initialize interactor
        self.interactor.Initialize()

        # Add world axes for reference
        # self.add_world_axes()

        # Start the interactor
        self.interactor.Start()

    def add_world_axes(self):
        """Add world coordinate axes to the scene"""
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(0.1, 0.1, 0.1)  # Set the length of the axes
        axes.SetShaftType(0)
        axes.SetAxisLabels(0)  # Not Show labels
        axes.SetCylinderRadius(0.02)

        # Add the axes to the renderer
        self.renderer.AddActor(axes)

    def load_urdf_file(self, filename):
        """Load a URDF file and visualize the robot"""
        if filename and os.path.exists(filename):
            # Clear previous models
            self.clear_models()

            # Parse the URDF file
            try:
                parser = URDFParser(filename)
                
                # Get robot info for visualization
                (link_names,
                link_mesh_files,
                link_mesh_transformations,
                link_frames,
                link_colors,
                joint_names,
                joint_frames,
                joint_types,
                joint_axes,
                joint_parent_links,
                joint_child_links,
                joint_limits,
                collision_mesh_files,
                collision_mesh_transformations,
                collision_link_names,
                collision_geometries,
                ) = parser.get_robot_info()

                # Store revolute joints for slider controls
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

                # Create joint sliders
                self.create_joint_sliders()

                # Get chain information
                self.chains, trees = parser.get_chain_info()

                # Create models for each link
                for i in range(len(link_names)):
                    self.add_urdf_model(
                        link_names[i],
                        link_mesh_files[i],
                        link_mesh_transformations[i],
                        link_frames[i],
                        link_colors[i],
                    )

                # Create models for each collision link
                for i, geom in enumerate(collision_geometries):
                    geom_type = geom['type']
                    if geom_type == 'mesh':
                        # Use mesh file
                        self.add_urdf_model(
                            f"collision_{i}",
                            collision_mesh_files[i],
                            collision_mesh_transformations[i],
                            None,
                            None,
                            model_type='collision'
                        )
                    elif geom_type == 'box':
                        # Create box geometry directly
                        self.add_collision_box(
                            f"collision_{i}",
                            geom['size'],
                            collision_mesh_transformations[i]
                        )
                    elif geom_type == 'sphere':
                        # Create sphere geometry directly
                        self.add_collision_sphere(
                            f"collision_{i}",
                            geom['radius'],
                            collision_mesh_transformations[i]
                        )
                    elif geom_type == 'cylinder':
                        # Create cylinder geometry directly
                        self.add_collision_cylinder(
                            f"collision_{i}",
                            geom['radius'],
                            geom['length'],
                            collision_mesh_transformations[i]
                        )
                    
                
                # use link mesh files to be decomposed
                self.collision_mesh_files = [f for f in link_mesh_files if f is not None]

                
                # Populate the chain tree
                self.populate_chain_tree()

                # Reset camera to show all actors
                self.renderer.ResetCamera()
                self.vtk_widget.GetRenderWindow().Render()
                
                # Store the current URDF file path only after successful loading
                self.current_urdf_file = filename
                
                # Update the current file label
                self.update_current_file_label()

            except Exception as e:
                msg = self.translation_manager.tr("load_urdf_failed")
                QMessageBox.critical(
                    self, self.translation_manager.tr("error"), msg.replace("{}", str(e))
                )

    def open_urdf_file(self):
        """Open a URDF file dialog and load the selected file"""
        filename, _ = QFileDialog.getOpenFileName(
            self, self.translation_manager.tr("dialog_open_urdf"), "", self.translation_manager.tr("dialog_urdf_filter")
        )

        if filename:
            self.load_urdf_file(filename)

    def add_urdf_model(self, name, mesh_file, mesh_transform, frame, color, model_type='visual'):

        """Add a URDF model to the scene"""
        try:
            # Create a new URDF model with axis text (using link name as the text)
            model = URDFModel(name, mesh_file, mesh_transform, frame, color, axis_text=name)

             # Add the model to our list
            if model_type == 'visual':
                self.models.append(model)
            elif model_type == 'collision':
                self.models_collision.append(model)

            # Add the actor to the renderer
            self.renderer.AddActor(model.actor)

            # Add the axes actor to the renderer
            if model.axes_actor is not None:
                self.renderer.AddActor(model.axes_actor)
            
            # Add the text actor to the renderer if it exists
            if model.text_actor is not None:
                self.renderer.AddActor(model.text_actor)
            
            # Set initial visibility based on checkbox
            if hasattr(self, "cb_link_frames"):
                if model.axes_actor is not None:
                    model.axes_actor.SetVisibility(self.cb_link_frames.isChecked())
                if model.text_actor is not None:
                    model.text_actor.SetVisibility(self.cb_link_frames.isChecked())

        except Exception as e:
            msg = self.translation_manager.tr("load_model_failed_pattern")
            QMessageBox.warning(
                self, self.translation_manager.tr("warning"),
                msg.replace(" {}", f" {name}").replace("{}", str(e))
            )

    # === 新增：创建基本几何形状碰撞体 ===
    # 当 URDF 中的碰撞体使用基本几何形状（box/sphere/cylinder）而非 mesh 文件时，
    # 直接使用 VTK 的几何体源创建相应的碰撞体可视化
    def add_collision_box(self, name, size, transform):
        """Add a box collision geometry to the scene"""
        try:
            # Create box source
            cube = vtk.vtkCubeSource()
            cube.SetXLength(size[0])
            cube.SetYLength(size[1])
            cube.SetZLength(size[2])
            cube.Update()

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(cube.GetOutputPort())

            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(0.3, 0.6, 1.0)  # Light blue
            actor.GetProperty().SetOpacity(0.4)  # Semi-transparent

            # Apply transform
            vtk_transform = vtk.vtkTransform()
            vtk_transform.SetMatrix(transform.flatten())
            actor.SetUserTransform(vtk_transform)

            self.renderer.AddActor(actor)
            self.models_collision.append(type('obj', (object,), {'actor': actor, 'name': name, 'apply_transform': lambda obj, t: _update_transform_for_actor(actor, t), 'axes_actor': None, 'text_actor': None})())

        except Exception as e:
            print(f"Error creating collision box {name}: {e}")

    def add_collision_sphere(self, name, radius, transform):
        """Add a sphere collision geometry to the scene"""
        try:
            # Create sphere source
            sphere = vtk.vtkSphereSource()
            sphere.SetRadius(radius)
            sphere.SetPhiResolution(16)
            sphere.SetThetaResolution(16)
            sphere.Update()

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(sphere.GetOutputPort())

            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(0.3, 0.6, 1.0)  # Light blue
            actor.GetProperty().SetOpacity(0.4)  # Semi-transparent

            # Apply transform
            vtk_transform = vtk.vtkTransform()
            vtk_transform.SetMatrix(transform.flatten())
            actor.SetUserTransform(vtk_transform)

            self.renderer.AddActor(actor)
            self.models_collision.append(type('obj', (object,), {'actor': actor, 'name': name, 'apply_transform': lambda obj, t: _update_transform_for_actor(actor, t), 'axes_actor': None, 'text_actor': None})())

        except Exception as e:
            print(f"Error creating collision sphere {name}: {e}")

    def add_collision_cylinder(self, name, radius, length, transform):
        """Add a cylinder collision geometry to the scene"""
        try:
            # Create cylinder source
            cylinder = vtk.vtkCylinderSource()
            cylinder.SetRadius(radius)
            cylinder.SetHeight(length)
            cylinder.SetResolution(16)
            cylinder.Update()

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(cylinder.GetOutputPort())

            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(0.3, 0.6, 1.0)  # Light blue
            actor.GetProperty().SetOpacity(0.4)  # Semi-transparent

            # Apply transform
            vtk_transform = vtk.vtkTransform()
            vtk_transform.SetMatrix(transform.flatten())
            actor.SetUserTransform(vtk_transform)

            self.renderer.AddActor(actor)
            self.models_collision.append(type('obj', (object,), {'actor': actor, 'name': name, 'apply_transform': lambda obj, t: _update_transform_for_actor(actor, t), 'axes_actor': None, 'text_actor': None})())

        except Exception as e:
            print(f"Error creating collision cylinder {name}: {e}")

    def _update_actor_transform(self, actor, transform_matrix):
        """Helper method to update actor transform"""
        vtk_transform = vtk.vtkTransform()
        vtk_transform.SetMatrix(transform_matrix.flatten())
        actor.SetUserTransform(vtk_transform)

    def clear_models(self):
        """Clear all models from the scene"""
        # Remove all actors from the renderer
        for model in self.models:
            self.renderer.RemoveActor(model.actor)
            self.renderer.RemoveActor(model.axes_actor)
            if hasattr(model, 'text_actor') and model.text_actor is not None:
                self.renderer.RemoveActor(model.text_actor)

        for model in self.models_collision:
            self.renderer.RemoveActor(model.actor)
            self.renderer.RemoveActor(model.axes_actor)
            if hasattr(model, 'text_actor') and model.text_actor is not None:
                self.renderer.RemoveActor(model.text_actor)

        # Clear the models list
        self.models = []
        self.models_collision = []


        # Clear the combo box and link list
        self.chain_combo.clear()
        self.link_list.clear()

        # Clear joint sliders
        self.clear_joint_sliders()

        # Clear MDH frames
        for actor in self.mdh_frames_actors:
            self.renderer.RemoveActor(actor)
        self.mdh_frames_actors = []

        # Clear MDH text actors
        for text_actor in self.mdh_text_actors:
            self.renderer.RemoveActor(text_actor)
        self.mdh_text_actors = []

        # Clear COM actors
        for actor in self.com_actors:
            self.renderer.RemoveActor(actor)
        self.com_actors = []

        # Clear inertia actors
        for actor in self.inertia_actors:
            self.renderer.RemoveActor(actor)
        self.inertia_actors = []

        # Clear joint axis actors
        for actor in self.joint_axis_actors:
            self.renderer.RemoveActor(actor)
        self.joint_axis_actors = []
        self.joint_axis_info = []

        # Clear inertial data
        self.inertial_data = []
        self.com_actor_info = []
        self.inertia_actor_info = []

        # Reset selected chain and current URDF file
        self.selected_chain = None
        self.current_urdf_file = None

        # Update current file label
        self.update_current_file_label()

        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()

    def populate_chain_tree(self):
        """Populate the chain combo box and link list"""
        self.chain_combo.clear()
        self.link_list.clear()

        # Add each chain to the combo box
        for i, chain in enumerate(self.chains):
            chain_label = self.translation_manager.tr("chain_pattern")
            self.chain_combo.addItem(chain_label.format(i+1, chain['name']), i)
        
        # Select the first chain by default if available
        if self.chains:
            self.selected_chain_index = 0
            self.selected_chain = self.chains[0]
            self.chain_combo.setCurrentIndex(0)
            self.update_link_list()

    def on_chain_selected(self, index):
        """Handle chain selection from combo box"""
        if index >= 0 and index < len(self.chains):
            # Unhighlight all models first
            for model in self.models:
                model.unhighlight()
            
            # Set the selected chain
            self.selected_chain_index = index
            self.selected_chain = self.chains[index]
            
            # Update the link list
            self.update_link_list()
            
            # Update MDH frames if they are currently visible
            if self.cb_mdh_frames.isChecked() and self.current_urdf_file:
                self.create_mdh_frames(self.selected_chain)
            
            # Update the rendering
            self.vtk_widget.GetRenderWindow().Render()
    
    def update_link_list(self):
        """Update the link list based on the selected chain"""
        self.link_list.clear()
        
        if self.selected_chain:
            # Add links to the list
            for link_name in self.selected_chain['link_names']:
                item = QListWidgetItem(link_name)
                item.setData(Qt.UserRole, link_name)
                self.link_list.addItem(item)
    
    def on_link_selection_changed(self):
        """Handle link selection from list"""
        # Unhighlight all models first
        for model in self.models:
            model.unhighlight()
        
        # Get selected link
        selected_items = self.link_list.selectedItems()
        if selected_items:
            link_name = selected_items[0].data(Qt.UserRole)
            
            # Highlight the selected link
            for model in self.models:
                if model.name == link_name:
                    model.highlight()
                    print(model.link_frame)
        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()

    # === 新增：视觉显示切换 ===
    def toggle_visual(self, state):
        """Toggle visibility of visual models"""
        visible = state == Qt.Checked

        for model in self.models:
            model.actor.SetVisibility(visible)

        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()

    # === 新增：碰撞体显示切换 ===
    def toggle_collision(self, state):
        visible = state == Qt.Checked

        for model in self.models_collision:
            model.actor.SetVisibility(visible)
            if model.axes_actor is not None:
                model.axes_actor.SetVisibility(visible)
            if hasattr(model, 'text_actor') and model.text_actor is not None:
                model.text_actor.SetVisibility(visible)

        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()

    def toggle_link_frames(self, state):
        """Toggle visibility of link frames and their text labels"""
        visible = state == Qt.Checked
        
        for model in self.models:
            model.axes_actor.SetVisibility(visible)
            if hasattr(model, 'text_actor') and model.text_actor is not None:
                model.text_actor.SetVisibility(visible)
        
        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()

    def toggle_mdh_frames(self, state):
        """Toggle visibility of MDH frames"""
        visible = state == Qt.Checked

        # If we want to show MDH frames
        if visible:
            if not self.current_urdf_file:
                QMessageBox.warning(
                    self, self.translation_manager.tr("warning"),
                    self.translation_manager.tr("load_urdf_mdh")
                )
                self.cb_mdh_frames.setChecked(False)
                return

            if self.selected_chain:
                # Always recreate MDH frames to ensure they're up to date
                self.create_mdh_frames(self.selected_chain)
            else:
                QMessageBox.warning(
                    self, self.translation_manager.tr("warning"),
                    self.translation_manager.tr("select_chain_warning")
                )
                self.cb_mdh_frames.setChecked(False)
                return
        else:
            # Hide MDH frames and text
            for actor in self.mdh_frames_actors:
                actor.SetVisibility(False)
            for text_actor in self.mdh_text_actors:
                text_actor.SetVisibility(False)
        
        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()

    def create_mdh_frames(self, chain):
        """Create MDH frame actors for the selected chain"""
        # Clear existing MDH frames
        for actor in self.mdh_frames_actors:
            self.renderer.RemoveActor(actor)
        self.mdh_frames_actors = []
        
        # Clear existing MDH text actors
        for text_actor in self.mdh_text_actors:
            self.renderer.RemoveActor(text_actor)
        self.mdh_text_actors = []
        
        # Create a fresh parser instance to ensure we get the latest data
        parser = URDFParser(self.current_urdf_file)
        
        # Get chain information to ensure we have the latest chain data
        chains, _ = parser.get_chain_info()
        
        # Find the matching chain in the updated chains list
        current_chain = None
        for c in chains:
            if c['name'] == chain['name']:
                current_chain = c
                break
        
        # If we couldn't find a matching chain, use the provided chain
        if current_chain is None:
            current_chain = chain
        
        # Get MDH frames using the current chain
        mdh_frames = parser.get_mdh_frames(current_chain)
        # update mdh_frames using joint position

        mdh_frames = parser.update_mdh_frames(mdh_frames, self.joint_values)
        
        # Create axes actors for each MDH frame
        for i, frame in enumerate(mdh_frames):
            axes = vtk.vtkAxesActor()
            axes.SetTotalLength(0.05, 0.05, 0.05)  # Set the length of the axes
            axes.SetShaftType(0)
            axes.SetAxisLabels(0)
            axes.SetCylinderRadius(0.01)
            
            # Create transform
            vtk_transform = vtk.vtkTransform()
            vtk_transform.SetMatrix(frame.flatten())
            axes.SetUserTransform(vtk_transform)
            
            # Add to renderer
            self.renderer.AddActor(axes)
            self.mdh_frames_actors.append(axes)
            
            # Set initial visibility based on checkbox
            axes.SetVisibility(self.cb_mdh_frames.isChecked())
            
            # Create text label for MDH frame
            text_actor = vtk.vtkCaptionActor2D()
            text_actor.SetCaption(f"MDH{i}")
            text_actor.GetTextActor().SetTextScaleModeToNone()
            text_actor.GetCaptionTextProperty().SetFontSize(14)
            text_actor.GetCaptionTextProperty().SetColor(0, 0, 1)  # Blue text for MDH frames
            text_actor.GetCaptionTextProperty().SetBold(False)
            
            # Position the text at the end of the z-axis
            z_endpoint = [0, 0, 0.05, 1]  # Same length as axes
            transformed_point = vtk_transform.TransformPoint(z_endpoint[0], z_endpoint[1], z_endpoint[2])
            text_actor.SetAttachmentPoint(transformed_point[0], transformed_point[1], transformed_point[2])
            
            # Configure the caption
            text_actor.BorderOff()
            text_actor.LeaderOff()
            text_actor.ThreeDimensionalLeaderOff()
            text_actor.SetPadding(2)
            
            # Add to renderer
            self.renderer.AddActor(text_actor)
            self.mdh_text_actors.append(text_actor)
            
            # Set initial visibility based on checkbox
            text_actor.SetVisibility(self.cb_mdh_frames.isChecked())
        
        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()

    # === 新增：关节轴显示切换 ===
    def toggle_joint_axes(self, state):
        """Toggle visibility of joint axes"""
        visible = state == Qt.Checked

        if visible:
            if not self.current_urdf_file:
                QMessageBox.warning(
                    self, self.translation_manager.tr("warning"),
                    self.translation_manager.tr("load_urdf_joint_axes")
                )
                self.cb_joint_axes.setChecked(False)
                return
            # Create joint axes if not already created
            if not self.joint_axis_actors:
                self.create_joint_axes()
            for actor in self.joint_axis_actors:
                actor.SetVisibility(True)
        else:
            for actor in self.joint_axis_actors:
                actor.SetVisibility(False)

        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()

    # === 新增：质心显示切换 ===
    def toggle_com(self, state):
        """Toggle visibility of center of mass markers"""
        visible = state == Qt.Checked

        if visible:
            if not self.current_urdf_file:
                QMessageBox.warning(
                    self, self.translation_manager.tr("warning"),
                    self.translation_manager.tr("load_urdf_com")
                )
                self.cb_com.setChecked(False)
                return
            # Create COM markers if not already created
            if not self.com_actors:
                self.create_com_markers()
            for actor in self.com_actors:
                actor.SetVisibility(True)
        else:
            for actor in self.com_actors:
                actor.SetVisibility(False)

        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()

    # === 新增：惯量显示切换 ===
    def toggle_inertia(self, state):
        """Toggle visibility of inertia boxes"""
        visible = state == Qt.Checked

        if visible:
            if not self.current_urdf_file:
                QMessageBox.warning(
                    self, self.translation_manager.tr("warning"),
                    self.translation_manager.tr("load_urdf_inertia")
                )
                self.cb_inertia.setChecked(False)
                return
            # Create inertia boxes if not already created
            if not self.inertia_actors:
                self.create_inertia_boxes()
            for actor in self.inertia_actors:
                actor.SetVisibility(True)
        else:
            for actor in self.inertia_actors:
                actor.SetVisibility(False)

        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()

    def show_mdh_parameters(self):
        """Show MDH parameters in a dialog"""
        if not self.selected_chain:
            QMessageBox.warning(
                self, self.translation_manager.tr("warning"),
                self.translation_manager.tr("select_chain_mdh_warning")
            )
            return

        if not self.current_urdf_file:
            QMessageBox.warning(
                self, self.translation_manager.tr("warning"),
                self.translation_manager.tr("load_urdf_show_mdh")
            )
            return

        # Create a fresh parser instance to ensure we get the latest MDH parameters
        parser = URDFParser(self.current_urdf_file)

        # Get chain information to ensure we have the latest chain data
        chains, _ = parser.get_chain_info()

        # Find the matching chain in the updated chains list
        current_chain = None
        for chain in chains:
            if chain['name'] == self.selected_chain['name']:
                current_chain = chain
                break

        # If we couldn't find a matching chain, use the selected chain
        if current_chain is None:
            current_chain = self.selected_chain

        # Get MDH parameters using the current chain
        _, _, _, mdh_parameters = parser.get_mdh_parameters(current_chain)

        # Create and show the new MDH dialog
        dialog = MDHDialog(self, current_chain['name'], mdh_parameters)
        dialog.exec_()

    def open_set_joints_dialog(self):
        """Open a dialog to input joint angles and apply them to the robot and sliders."""
        if not self.revolute_joints:
            QMessageBox.warning(self, self.translation_manager.tr("warning"),
                               self.translation_manager.tr("no_revolute_joints"))
            return

        dialog = QDialog(self)
        dialog.setWindowTitle(self.translation_manager.tr("set_joints_title"))
        vbox = QVBoxLayout(dialog)

        vbox.addWidget(QLabel(self.translation_manager.tr("input_joints_prompt")))
        angles_edit = QLineEdit()
        # Pre-fill with current values in currently selected unit for convenience
        try:
            current_vals = []
            for val in (self.joint_values if self.joint_values else [0.0] * len(self.revolute_joints)):
                if self.display_in_degrees:
                    current_vals.append(f"{val * 180.0 / math.pi:.2f}")
                else:
                    current_vals.append(f"{val:.2f}")
            angles_edit.setText(", ".join(current_vals))
        except Exception:
            pass
        vbox.addWidget(angles_edit)

        units_row = QHBoxLayout()
        units_row.addWidget(QLabel(self.translation_manager.tr("units_label")))
        units_combo = QComboBox()
        units_combo.addItems(["rad", "deg"])
        units_combo.setCurrentText("deg" if self.display_in_degrees else "rad")
        units_row.addWidget(units_combo)
        units_row.addStretch(1)
        vbox.addLayout(units_row)

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
            # Support comma or whitespace separated values
            tokens = raw.replace(",", " ").split()
            try:
                vals = [float(t) for t in tokens]
            except ValueError:
                QMessageBox.critical(dialog, self.translation_manager.tr("error"),
                                    self.translation_manager.tr("input_numbers_error"))
                return
            n = len(self.revolute_joints)
            if len(vals) != n:
                msg = self.translation_manager.tr("input_count_warning")
                QMessageBox.warning(dialog, self.translation_manager.tr("warning"),
                                   msg.format(n, len(vals)))
                return
            # Convert to radians if needed
            if units_combo.currentText() == "deg":
                vals = [v * math.pi / 180.0 for v in vals]
            
            # Apply to internal values and sliders (clamped to slider range)
            # Avoid excessive re-render by blocking signals and updating once
            for i, rad in enumerate(vals):
                self.joint_values[i] = rad
                target = int(round(rad * 100.0))
                slider = self.joint_sliders[i]
                clamped = max(slider.minimum(), min(slider.maximum(), target))
                was_blocked = slider.blockSignals(True)
                slider.setValue(clamped)
                slider.blockSignals(was_blocked)
            
            # Refresh labels and model once
            self.update_all_joint_value_labels()
            self.update_model_with_joint_angles()
            dialog.accept()
        
        btn_apply.clicked.connect(parse_and_apply)
        btn_cancel.clicked.connect(dialog.reject)
        dialog.exec_()

    def apply_transparency(self):
        """Apply transparency to virtual loaded models"""
        # Get transparency value from slider (convert from 0-100 to 0.0-1.0)
        transparency = self.transparency_slider.value() / 100.0

        # Apply the transparency to virtual models
        for model in self.models:
            model.set_transparency(transparency)
        
        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()

    def create_joint_sliders(self):
        """Create sliders for controlling joint angles"""
        # Clear existing sliders
        self.clear_joint_sliders()

        # Initialize joint values array with zeros
        self.joint_values = [0.0] * len(self.revolute_joints)
        self.joint_value_labels = []

        # Create a slider for each revolute joint
        for i, joint in enumerate(self.revolute_joints):
            # 获取关节限制（从 URDF 中读取）
            lower = joint.get('lower', -math.pi)
            upper = joint.get('upper', math.pi)

            # Create a group for this joint
            joint_box = QGroupBox(joint['name'])
            joint_box_layout = QVBoxLayout(joint_box)

            # 添加范围标签
            range_text = self.translation_manager.tr("range_label")
            range_label = QLabel(range_text.format(self.format_angle(lower), self.format_angle(upper)))
            range_label.setStyleSheet("color: gray; font-size: 10px;")
            joint_box_layout.addWidget(range_label)

            # Create a horizontal layout for slider and value label
            slider_layout = QHBoxLayout()

            # 显示最小值标签
            min_label = QLabel(f"{lower:.2f}")
            min_label.setStyleSheet("color: gray; font-size: 9px;")
            min_label.setMaximumWidth(35)
            slider_layout.addWidget(min_label)

            # Create a label to show the current value (需要在 slider 之前创建，因为 lambda 会引用它)
            value_label = QLabel(self.format_angle(0.0))
            value_label.setMinimumWidth(60)
            value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

            # Create a slider
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(lower * 100))  # 转换为百分之一弧度
            slider.setMaximum(int(upper * 100))   # 转换为百分之一弧度
            slider.setValue(0)       # Default to 0
            slider.setTickPosition(QSlider.TicksBelow)
            # 计算合适的刻度间隔
            range_val = upper - lower
            if range_val >= 2 * math.pi:
                tick_interval = int(math.pi / 2 * 100)  # π/2
            elif range_val >= math.pi:
                tick_interval = int(math.pi / 4 * 100)   # π/4
            else:
                tick_interval = int(range_val / 4 * 100)  # 范围的1/4
            slider.setTickInterval(max(1, tick_interval))

            # Connect the slider to update function
            slider.valueChanged.connect(lambda val, idx=i, label=value_label: self.update_joint_angle(val, idx, label))

            slider_layout.addWidget(slider, 1)  # Give slider a stretch factor of 1

            # 显示最大值标签
            max_label = QLabel(f"{upper:.2f}")
            max_label.setStyleSheet("color: gray; font-size: 9px;")
            max_label.setMaximumWidth(35)
            max_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            slider_layout.addWidget(max_label)

            slider_layout.addWidget(value_label)

            # Add the horizontal layout to the joint box
            joint_box_layout.addLayout(slider_layout)

            # Add the joint box to the joint layout
            self.joint_layout.addWidget(joint_box)

            # Store the slider for later access
            self.joint_sliders.append(slider)
            self.joint_value_labels.append(value_label)
    
    def clear_joint_sliders(self):
        """Clear all joint sliders"""
        # Clear the joint values
        self.joint_values = []

        # Clear the sliders list
        self.joint_sliders = []
        self.joint_value_labels = []

        # Remove all widgets from the joint layout except the label
        if hasattr(self, 'joint_layout'):
            while self.joint_layout.count() > 1:  # Keep the first item (joint_label)
                item = self.joint_layout.takeAt(1)  # Start from index 1
                widget = item.widget()
                if widget:
                    widget.deleteLater()
    
    def update_joint_angle(self, value, index, label):
        """Update joint angle when slider is moved"""
        # Convert slider value to radians (from hundredths)
        angle = value / 100.0
        
        # Update the label
        label.setText(self.format_angle(angle))
        
        # Store the value
        self.joint_values[index] = angle
        
        # Update the model
        self.update_model_with_joint_angles()

    def on_units_changed(self, text):
        """Handle units toggle between radians and degrees for display."""
        self.display_in_degrees = (text == "deg")
        self.update_all_joint_value_labels()

    def change_language(self, index):
        """Handle language combo box change"""
        lang_code = self.language_combo.itemData(index)
        self.translation_manager.set_language(lang_code, self)

    def format_angle(self, angle_radians):
        """Format angle for display according to current unit setting."""
        if self.display_in_degrees:
            return f"{angle_radians * 180.0 / math.pi:.2f}°"
        return f"{angle_radians:.2f} rad"

    def update_all_joint_value_labels(self):
        """Refresh all joint labels to current unit."""
        for i, label in enumerate(self.joint_value_labels):
            if i < len(self.joint_values):
                label.setText(self.format_angle(self.joint_values[i]))
    
    def reset_joints(self):
        """Reset all joints to zero position"""
        if not self.joint_sliders or not self.joint_values:
            return
            
        # Set all sliders to zero
        for i, slider in enumerate(self.joint_sliders):
            slider.setValue(0)
        
        # Update the model
        self.update_model_with_joint_angles()
    
    def randomize_joints(self):
        """Set all joints to random values"""
        if not self.joint_sliders or not self.joint_values:
            return
            
        # Set all sliders to random values between min and max
        for i, slider in enumerate(self.joint_sliders):
            random_value = np.random.randint(slider.minimum(), slider.maximum())
            slider.setValue(random_value)
        
        # Update the model
        self.update_model_with_joint_angles()
    
    def update_model_with_joint_angles(self):
        """Update the model visualization with current joint angles"""
        if not self.current_urdf_file:
            return
        
        # Create a fresh parser instance
        parser = URDFParser(self.current_urdf_file)
        
        # Get updated robot info with joint angles
        (link_names,
        link_mesh_files,
        link_mesh_transformations,
        link_frames,
        link_colors,
        joint_names,
        joint_frames,
        joint_types,
        joint_axes,
        joint_parent_links,
        joint_child_links,
        _,
        collision_mesh_files,
        collision_mesh_transformations,
        collision_link_names,
        collision_geometries,
        ) = parser.get_robot_info(qs=self.joint_values)
        
        # Update existing models with new transformations
        for i, model in enumerate(self.models):
            if i < len(link_names) and model.name == link_names[i]:
                # Update mesh transformation
                model.apply_transform(link_mesh_transformations[i])
                
                # Update axes and text actors
                if model.axes_actor:
                    vtk_transform = vtk.vtkTransform()
                    vtk_transform.SetMatrix(link_frames[i].flatten())
                    model.axes_actor.SetUserTransform(vtk_transform)
                
                if model.text_actor:
                    # Update text position based on new frame
                    z_endpoint = [0, 0, 0.05, 1]  # Same axis_length as in create_axes_actor
                    vtk_transform = vtk.vtkTransform()
                    vtk_transform.SetMatrix(link_frames[i].flatten())
                    transformed_point = vtk_transform.TransformPoint(z_endpoint[0], z_endpoint[1], z_endpoint[2])
                    model.text_actor.SetAttachmentPoint(transformed_point[0], transformed_point[1], transformed_point[2])

        # === 新增：更新碰撞体 ===
        # 建立从连杆名称到碰撞体变换的映射，然后更新每个碰撞体的位置和方向
        # Build a map from link name to collision transform
        collision_transform_map = {}
        for i, link_name in enumerate(collision_link_names):
            collision_transform_map[link_name] = collision_mesh_transformations[i]

        for i, model in enumerate(self.models_collision):
            # Extract link index from model name (format: "collision_{i}")
            if model.name.startswith("collision_"):
                collision_idx = int(model.name.split("_")[1])
                if collision_idx < len(collision_link_names):
                    link_name = collision_link_names[collision_idx]
                    if link_name in collision_transform_map:
                        # Update mesh transformation
                        model.apply_transform(collision_transform_map[link_name])

                        # Update axes and text actors
                        if model.axes_actor:
                            vtk_transform = vtk.vtkTransform()
                            vtk_transform.SetMatrix(collision_transform_map[link_name].flatten())
                            model.axes_actor.SetUserTransform(vtk_transform)

                        if model.text_actor:
                            # Update text position based on new frame
                            z_endpoint = [0, 0, 0.05, 1]
                            vtk_transform = vtk.vtkTransform()
                            vtk_transform.SetMatrix(collision_transform_map[link_name].flatten())
                            transformed_point = vtk_transform.TransformPoint(z_endpoint[0], z_endpoint[1], z_endpoint[2])
                            model.text_actor.SetAttachmentPoint(transformed_point[0], transformed_point[1], transformed_point[2])

        # === 新增：更新关节轴 ===
        # 根据新的关节位置和旋转，更新关节轴圆柱体的位置和方向
        # 关节轴的局部坐标系中的方向需要通过关节的旋转矩阵变换到世界坐标系
        if hasattr(self, 'joint_axis_info') and self.joint_axis_info:
            # Build a map from joint name to joint frame
            joint_name_to_frame = {name: frame for name, frame in zip(joint_names, joint_frames)}
            joint_name_to_axis = {name: axis for name, axis in zip(joint_names, joint_axes)}

            for info in self.joint_axis_info:
                actor = info['actor']
                joint_name = info['joint_name']
                axis = info['axis']

                if joint_name in joint_name_to_frame:
                    joint_frame = joint_name_to_frame[joint_name]

                    # Recreate transform for joint axis
                    vtk_transform = vtk.vtkTransform()

                    # Extract position from joint_frame
                    joint_position = joint_frame[:3, 3]

                    # Transform axis from local joint frame to world frame
                    R_joint = joint_frame[:3, :3]
                    joint_axis_local = np.array(axis)
                    joint_axis_local = joint_axis_local / (np.linalg.norm(joint_axis_local) + 1e-10)
                    joint_axis_world = R_joint @ joint_axis_local

                    # Position at joint origin
                    vtk_transform.Translate(joint_position[0], joint_position[1], joint_position[2])

                    # Rotate cylinder (default Y axis) to align with joint axis in world frame
                    y_axis = np.array([0, 1, 0])

                    if not np.allclose(y_axis, joint_axis_world, atol=0.01):
                        cross = np.cross(y_axis, joint_axis_world)
                        dot = np.dot(y_axis, joint_axis_world)
                        angle = math.acos(np.clip(dot, -1, 1))
                        if np.linalg.norm(cross) > 1e-6:
                            axis_rot = cross / np.linalg.norm(cross)
                            vtk_transform.RotateWXYZ(math.degrees(angle), axis_rot[0], axis_rot[1], axis_rot[2])

                    actor.SetUserTransform(vtk_transform)

        # === 新增：更新质心(COM)标记 ===
        # 根据连杆的新位置，更新质心小球体的世界坐标位置
        # T_world = link_frame @ T_com_rel (连杆变换 @ 质心相对变换)
        if hasattr(self, 'com_actor_info') and self.com_actor_info:
            # Build a map from link name to link frame
            link_name_to_frame = {name: frame for name, frame in zip(link_names, link_frames)}

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

        # === 新增：更新惯量盒 ===
        # 根据连杆的新位置，更新惯量盒的世界坐标位置
        # 惯量盒的中心位置根据 URDF 中的 inertial.origin 确定
        if hasattr(self, 'inertia_actor_info') and self.inertia_actor_info:
            # Build a map from link name to link frame
            link_name_to_frame = {name: frame for name, frame in zip(link_names, link_frames)}

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

        # Update MDH frames if they are visible
        if hasattr(self, 'cb_mdh_frames') and self.cb_mdh_frames.isChecked() and self.selected_chain:
            self.create_mdh_frames(self.selected_chain)
        
        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()
    
    def edit_urdf_file(self, replace_collision=False):
        """Open the current URDF file in the XML editor"""
        if not self.current_urdf_file:
            QMessageBox.warning(
                self, self.translation_manager.tr("warning"),
                self.translation_manager.tr("load_urdf_edit")
            )
            return
        
        # Create and show the XML editor window with update callback
        self.editor = XMLEditor(self.current_urdf_file, self.update_model_from_xml)
        if replace_collision:
            self.editor.replace_collision()
        self.editor.show()
    
    def update_model_from_xml(self, xml_content):
        """Update the model using XML content from the editor"""
        try:
            # Create a temporary file to store the XML content
            
            if '_temp.urdf' not in self.current_urdf_file:
                temp_path = self.current_urdf_file.lower().replace('.urdf', '_temp.urdf')
            else:
                temp_path = self.current_urdf_file.lower()
            
            with open(temp_path, 'w', encoding='utf-8')as temp_file:
                temp_file.write(xml_content)
            
            # Clear previous models
            self.clear_models()
            
            # Parse the URDF from the temporary file
            parser = URDFParser(temp_path)
            
            # Get robot info for visualization
            (link_names,
            link_mesh_files,
            link_mesh_transformations,
            link_frames,
            link_colors,
            joint_names,
            joint_frames,
            joint_types,
            joint_axes,
            joint_parent_links,
            joint_child_links,
            joint_limits,
            collision_mesh_files,
            collision_mesh_transformations,
            collision_link_names,
            collision_geometries,) = parser.get_robot_info()

            # Store revolute joints for slider controls
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
            
            # Create joint sliders
            self.create_joint_sliders()
            
            # Get chain information
            self.chains, trees = parser.get_chain_info()
            
            # Create models for each link
            for i in range(len(link_names)):
                self.add_urdf_model(
                    link_names[i],
                    link_mesh_files[i],
                    link_mesh_transformations[i],
                    link_frames[i],
                    link_colors[i],
                )
                
            # Create models for each collision link
            for i in range(len(collision_mesh_files)):
                self.add_urdf_model(
                    f"",
                    collision_mesh_files[i],
                    collision_mesh_transformations[i],
                    None,
                    None,
                    model_type='collision'
                )
                
            self.cb_collision.setChecked(True)
            self.transparency_slider.setValue(100)
            
            # Populate the chain tree
            self.populate_chain_tree()

            # Reset camera to show all actors
            # self.renderer.ResetCamera()
            self.vtk_widget.GetRenderWindow().Render()
            
            # Store the temporary file path as the current URDF file
            # This allows further editing and updates
            self.current_urdf_file = temp_path
            
            # Update current file label
            self.update_current_file_label()
            
        except Exception as e:
            msg = self.translation_manager.tr("update_model_failed")
            QMessageBox.critical(
                self, self.translation_manager.tr("error"),
                msg.replace("{}", str(e))
            )

    def update_current_file_label(self):
        """Update the current file label with the current URDF file path"""
        if self.current_urdf_file:
            # Extract just the filename from the path for cleaner display
            filename = os.path.basename(self.current_urdf_file)
            self.current_file_label.setText(self.translation_manager.tr("current_file") + f" {filename}")
        else:
            self.current_file_label.setText(self.translation_manager.tr("current_file") + " " + self.translation_manager.tr("current_file_none"))

    def decompose_collision_meshes(self):
        """Handle decomposition of collision meshes"""
        if not self.collision_mesh_files:
            QMessageBox.warning(
                self, self.translation_manager.tr("warning"),
                self.translation_manager.tr("no_collision_meshes")
            )
            return
        
        # Create and show the decomposition dialog
        dialog = DecompDialog(self, self.collision_mesh_files)
        
        decomposed_mesh_files = dialog.exec_()

        if decomposed_mesh_files is not None:
            self.edit_urdf_file(replace_collision=True)

    # === 新增：创建关节轴可视化 ===
    # 为每个 revolute 或 continuous 类型的关节创建一个圆柱体表示其旋转轴
    # 圆柱体的方向与关节的旋转轴方向一致
    def create_joint_axes(self):
        """Create joint axis visualization for all revolute joints"""
        if not self.current_urdf_file:
            return

        parser = URDFParser(self.current_urdf_file)
        result = parser.get_robot_info(qs=self.joint_values)
        # result has 14 values: link_names, link_mesh_files, link_mesh_transformations, link_frames, link_colors,
        #                         joint_names, joint_frames, joint_types, joint_axes, joint_parent_links, joint_child_links,
        #                         joint_limits, collision_mesh_files, collision_mesh_transformations
        joint_names = result[5]
        joint_frames = result[6]
        joint_types = result[7]
        joint_axes = result[8]

        print(f"[DEBUG] Total joints: {len(joint_names)}")
        print(f"[DEBUG] Joint names: {joint_names}")
        print(f"[DEBUG] Joint types: {joint_types}")

        revolute_count = 0
        for i, (joint_name, joint_frame, joint_type, axis) in enumerate(zip(joint_names, joint_frames, joint_types, joint_axes)):
            # Convert joint_name to string in case it's a numpy array
            joint_name_str = str(joint_name) if hasattr(joint_name, '__iter__') and not isinstance(joint_name, str) else joint_name

            # Only show axes for revolute and continuous joints
            if joint_type not in ['revolute', 'continuous']:
                print(f"[DEBUG] Skipping joint {joint_name_str}: type={joint_type}")
                continue

            revolute_count += 1
            print(f"[DEBUG] Creating joint axis for: {joint_name_str}, axis={axis}")

            # Create a cylinder for the joint axis - thicker and more visible
            axis_length = 0.3  # Longer axis
            cylinder = vtk.vtkCylinderSource()
            cylinder.SetRadius(0.01)  # Thicker tube
            cylinder.SetHeight(axis_length)
            cylinder.SetResolution(16)
            cylinder.Update()

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(cylinder.GetOutputPort())

            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(1, 0, 1)  # Magenta for joint axes
            actor.GetProperty().SetOpacity(1.0)

            # Create transform to position and orient the axis
            vtk_transform = vtk.vtkTransform()

            # Extract position from joint_frame
            joint_position = joint_frame[:3, 3]

            # Transform axis from local joint frame to world frame
            # axis is defined in joint's local coordinate system
            R_joint = joint_frame[:3, :3]  # Rotation matrix of joint frame
            joint_axis_local = np.array(axis)
            joint_axis_local = joint_axis_local / (np.linalg.norm(joint_axis_local) + 1e-10)
            joint_axis_world = R_joint @ joint_axis_local  # Transform to world coordinates

            print(f"[DEBUG] Joint axis local: {joint_axis_local}, world: {joint_axis_world}")

            # Position at joint origin
            vtk_transform.Translate(joint_position[0], joint_position[1], joint_position[2])

            # Rotate cylinder (default Y axis) to align with joint axis in world frame
            y_axis = np.array([0, 1, 0])

            if not np.allclose(y_axis, joint_axis_world, atol=0.01):
                # Calculate rotation from Y to joint axis (in world coordinates)
                cross = np.cross(y_axis, joint_axis_world)
                dot = np.dot(y_axis, joint_axis_world)
                angle = math.acos(np.clip(dot, -1, 1))
                print(f"[DEBUG] Rotation angle: {math.degrees(angle):.2f} degrees")
                if np.linalg.norm(cross) > 1e-6:
                    axis_rot = cross / np.linalg.norm(cross)
                    vtk_transform.RotateWXYZ(math.degrees(angle), axis_rot[0], axis_rot[1], axis_rot[2])

            actor.SetUserTransform(vtk_transform)
            actor.SetVisibility(self.cb_joint_axes.isChecked())

            self.renderer.AddActor(actor)
            self.joint_axis_actors.append(actor)
            # Store joint axis info for updates
            self.joint_axis_info.append({
                'actor': actor,
                'joint_name': joint_name_str,
                'axis': list(axis)  # Store axis direction (local to joint frame)
            })
            print(f"[DEBUG] Added actor to renderer. Total actors: {len(self.joint_axis_actors)}")

        print(f"[DEBUG] Created {revolute_count} joint axis actors")
        self.vtk_widget.GetRenderWindow().Render()

    # === 新增：创建质心(COM)可视化 ===
    # 为每个有质量的连杆创建一个小球体表示其质心位置
    # 小球体的位置根据 URDF 中的 inertial.origin 确定
    def create_com_markers(self):
        """Create center of mass markers for all links"""
        if not self.current_urdf_file:
            return

        parser = URDFParser(self.current_urdf_file)
        link_names, _, _, link_frames, _, _, _, _, _, _, _, _, _, _, _, _ = parser.get_robot_info(qs=self.joint_values)

        for link_name, link_frame in zip(link_names, link_frames):
            # Get inertial data from parser
            link_info = parser.links.get(link_name)
            if link_info is None or link_info.get('inertial') is None:
                continue

            inertial = link_info['inertial']
            if inertial is None or inertial.get('mass') is None:
                continue

            # Get COM position relative to link frame
            com_origin = inertial.get('origin', {})
            com_xyz = np.array(com_origin.get('xyz', [0, 0, 0]))
            com_rpy = np.array(com_origin.get('rpy', [0, 0, 0]))

            # Create COM marker (small sphere)
            sphere = vtk.vtkSphereSource()
            sphere.SetRadius(0.02)  # 2cm radius
            sphere.SetPhiResolution(16)
            sphere.SetThetaResolution(16)
            sphere.Update()

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(sphere.GetOutputPort())

            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            # Blender-style: black and white checkerboard pattern
            # For simplicity, use a distinctive color
            actor.GetProperty().SetColor(0.5, 0.5, 0.5)  # Gray

            # Calculate transform for COM position
            # COM is in link frame, need to transform to world frame
            T_com = self.compute_transformation(com_rpy, com_xyz)
            T_world = link_frame @ T_com

            vtk_transform = vtk.vtkTransform()
            vtk_transform.SetMatrix(T_world.flatten())
            actor.SetUserTransform(vtk_transform)
            actor.SetVisibility(self.cb_com.isChecked())

            self.renderer.AddActor(actor)
            self.com_actors.append(actor)

            # Store COM actor info for updates (T_com is relative to link frame)
            self.com_actor_info.append({
                'actor': actor,
                'link_name': link_name,
                'T_com_rel': T_com  # COM transform relative to link frame
            })

            # Store inertial data for later use
            self.inertial_data.append({
                'link_name': link_name,
                'inertial': inertial,
                'frame': T_world
            })

    # === 惯性张量特征值分解（Jacobi 迭代法） ===
    # 借鉴 robot_viewer 的 MathUtils.computeEigenDecomposition3x3
    @staticmethod
    def compute_eigen_decomposition_3x3(matrix_3x3):
        """对 3x3 对称矩阵进行特征值分解（Jacobi 迭代法）
        Args:
            matrix_3x3: 3x3 numpy 对称矩阵
        Returns:
            eigenvalues: [λ1, λ2, λ3]
            eigenvectors: 3x3 矩阵，列向量为特征向量
        """
        a00, a01, a02 = matrix_3x3[0, 0], matrix_3x3[0, 1], matrix_3x3[0, 2]
        a11, a12 = matrix_3x3[1, 1], matrix_3x3[1, 2]
        a22 = matrix_3x3[2, 2]

        # 特征向量矩阵初始化为单位矩阵
        v = np.eye(3)

        max_iterations = 50
        for _ in range(max_iterations):
            # 找最大的非对角元素
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

            # 收敛判断
            if max_val < 1e-10:
                break

            # 计算旋转角度
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

            # 更新矩阵元素和特征向量
            if p == 0 and q == 1:
                t00, t01, t02 = a00, a01, a02
                t11, t12 = a11, a12
                a00 = c*c*t00 - 2*c*s*t01 + s*s*t11
                a11 = s*s*t00 + 2*c*s*t01 + c*c*t11
                a01 = 0.0
                a02 = c*t02 - s*t12
                a12 = s*t02 + c*t12
                # 更新特征向量（列 0 和列 1）
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
                # 更新特征向量（列 0 和列 2）
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
                # 更新特征向量（列 1 和列 2）
                for row in range(3):
                    tv1 = v[row, 1]
                    tv2 = v[row, 2]
                    v[row, 1] = c * tv1 - s * tv2
                    v[row, 2] = s * tv1 + c * tv2

        return [a00, a11, a22], v

    # === 计算惯量盒参数（借鉴 robot_viewer 的 MathUtils.computeInertiaBox） ===
    @staticmethod
    def compute_inertia_box(inertia, mass):
        """根据惯性张量计算等效长方体的尺寸和主轴旋转
        Args:
            inertia: dict，包含 ixx, ixy, ixz, iyy, iyz, izz
            mass: 质量
        Returns:
            dict {box_x, box_y, box_z, R_principal} 或 None（参数不合理时）
        """
        Ixx = inertia.get('ixx', 0)
        Iyy = inertia.get('iyy', 0)
        Izz = inertia.get('izz', 0)
        Ixy = inertia.get('ixy', 0)
        Ixz = inertia.get('ixz', 0)
        Iyz = inertia.get('iyz', 0)

        # 合理性检查1：质量太小
        min_mass_threshold = 0.01  # 10g
        if mass < min_mass_threshold:
            avg_inertia = (abs(Ixx) + abs(Iyy) + abs(Izz)) / 3.0
            if avg_inertia > 0:
                inertia_radius = math.sqrt(avg_inertia / mass)
                if inertia_radius > 0.05:
                    return None

        # 合理性检查2：所有惯量值接近零
        inertia_threshold = 1e-9
        if abs(Ixx) < inertia_threshold and abs(Iyy) < inertia_threshold and abs(Izz) < inertia_threshold:
            return None

        # 检查是否有非对角分量
        has_off_diagonal = abs(Ixy) > 1e-10 or abs(Ixz) > 1e-10 or abs(Iyz) > 1e-10

        R_principal = np.eye(3)  # 默认无旋转

        if has_off_diagonal:
            # 有非对角分量，需要特征值分解
            inertia_matrix = np.array([
                [Ixx, Ixy, Ixz],
                [Ixy, Iyy, Iyz],
                [Ixz, Iyz, Izz]
            ])
            principal_inertias, eigenvectors = URDFViewer.compute_eigen_decomposition_3x3(inertia_matrix)
            R_principal = eigenvectors  # 特征向量矩阵即为主轴旋转
        else:
            # 无非对角分量，直接使用对角值
            principal_inertias = [Ixx, Iyy, Izz]

        # 反推长方体尺寸
        factor = 6.0 / mass
        width_sq = factor * (principal_inertias[1] + principal_inertias[2] - principal_inertias[0])
        height_sq = factor * (principal_inertias[0] + principal_inertias[2] - principal_inertias[1])
        depth_sq = factor * (principal_inertias[0] + principal_inertias[1] - principal_inertias[2])

        box_x = math.sqrt(abs(width_sq))
        box_y = math.sqrt(abs(height_sq))
        box_z = math.sqrt(abs(depth_sq))

        # 合理性检查3：惯量半径与盒子尺寸比例
        avg_inertia = (abs(principal_inertias[0]) + abs(principal_inertias[1]) + abs(principal_inertias[2])) / 3.0
        inertia_radius = math.sqrt(avg_inertia / mass) if mass > 0 else 0
        avg_box_size = (box_x + box_y + box_z) / 3.0
        if avg_box_size > 0 and inertia_radius > avg_box_size:
            return None

        # 合理性检查4：等效密度
        volume = box_x * box_y * box_z
        if volume > 0:
            equivalent_density = mass / volume
            if equivalent_density < 0.0001:  # 比气凝胶轻100倍
                return None

        # 最小尺寸限制
        min_size = 0.01
        box_x = max(box_x, min_size)
        box_y = max(box_y, min_size)
        box_z = max(box_z, min_size)

        return {
            'box_x': box_x,
            'box_y': box_y,
            'box_z': box_z,
            'R_principal': R_principal  # 主轴旋转矩阵 (3x3)
        }

    # === 创建惯量盒可视化 ===
    # 为每个有惯量信息的连杆创建一个长方体表示其等效惯量
    # 支持非对角惯性张量的主轴分解（借鉴 robot_viewer 方案）
    def create_inertia_boxes(self):
        """Create inertia box visualization for all links"""
        if not self.current_urdf_file:
            return

        parser = URDFParser(self.current_urdf_file)
        link_names, _, _, link_frames, _, _, _, _, _, _, _, _, _, _, _, _ = parser.get_robot_info(qs=self.joint_values)

        for link_name, link_frame in zip(link_names, link_frames):
            # Get inertial data from parser
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

            # 计算惯量盒参数（包括主轴分解）
            box_data = self.compute_inertia_box(inertia, mass)
            if box_data is None:
                print(f"[INERTIA] Link: {link_name} - 惯量参数不合理，跳过显示")
                continue

            box_x = box_data['box_x']
            box_y = box_data['box_y']
            box_z = box_data['box_z']
            R_principal = box_data['R_principal']

            # Debug print
            print(f"[INERTIA] Link: {link_name}")
            print(f"  Mass: {mass:.4f} kg")
            print(f"  Inertia: Ixx={inertia.get('ixx',0):.6f}, Iyy={inertia.get('iyy',0):.6f}, Izz={inertia.get('izz',0):.6f}")
            print(f"  Off-diag: Ixy={inertia.get('ixy',0):.6f}, Ixz={inertia.get('ixz',0):.6f}, Iyz={inertia.get('iyz',0):.6f}")
            print(f"  Computed box: w={box_x:.4f}m, h={box_y:.4f}m, d={box_z:.4f}m")
            has_rotation = not np.allclose(R_principal, np.eye(3))
            if has_rotation:
                print(f"  Principal axis rotation applied")

            # Create box
            cube = vtk.vtkCubeSource()
            cube.SetXLength(box_x)
            cube.SetYLength(box_y)
            cube.SetZLength(box_z)
            cube.Update()

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(cube.GetOutputPort())

            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(0.3, 0.6, 1.0)  # Light blue
            actor.GetProperty().SetOpacity(0.4)  # Semi-transparent

            # Get COM position for inertia box placement
            com_origin = inertial.get('origin', {})
            com_xyz = np.array(com_origin.get('xyz', [0, 0, 0]))
            com_rpy = np.array(com_origin.get('rpy', [0, 0, 0]))

            # 计算变换：link_frame @ T_com @ R_principal
            # T_com: 质心相对于连杆坐标系的位姿
            # R_principal: 主惯性轴旋转（将长方体对齐到主惯性轴方向）
            T_com = self.compute_transformation(com_rpy, com_xyz)

            # 构建主轴旋转的 4x4 齐次矩阵
            T_principal = np.eye(4)
            T_principal[:3, :3] = R_principal

            T_world = link_frame @ T_com @ T_principal

            vtk_transform = vtk.vtkTransform()
            vtk_transform.SetMatrix(T_world.flatten())
            actor.SetUserTransform(vtk_transform)
            actor.SetVisibility(self.cb_inertia.isChecked())

            self.renderer.AddActor(actor)
            self.inertia_actors.append(actor)

            # Store inertia actor info for updates
            # T_com_rel 现在包含主轴旋转
            T_com_with_principal = T_com @ T_principal
            self.inertia_actor_info.append({
                'actor': actor,
                'link_name': link_name,
                'T_com_rel': T_com_with_principal  # 包含主轴旋转的相对变换
            })

    def compute_transformation(self, rpy, xyz):
        """Compute 4x4 transformation matrix from RPY and XYZ"""
        roll, pitch, yaw = rpy

        # Rotation matrices
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

        # Combined rotation
        R = Rz @ Ry @ Rx

        # Transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = xyz

        return T

    def closeEvent(self, event):
        """Handle window close event"""
        # Close any open XML editor windows
        if hasattr(self, 'editor') and self.editor is not None:
            self.editor.close()
        
        # Properly clean up the VTK widget
        self.vtk_widget.GetRenderWindow().Finalize()
        self.vtk_widget.close()
        
        # Exit the application
        QApplication.quit()
        
        event.accept()


def main():
    """Main function to run the application"""
    app = QApplication(sys.argv)
    viewer = URDFViewer()
    viewer.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
