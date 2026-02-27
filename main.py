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
    QSplitter,
    QToolBar,
    QStatusBar,
    QAction,
    QFrame,
    QMenu,
    QMenuBar,
    QTextBrowser,
    QToolButton,
)
from xml_editor import XMLEditor
from mdh_dialog import MDHDialog
from decomp_dialog import DecompDialog
from PyQt5.QtCore import Qt, QUrl, QSize, QEvent, QSettings
from PyQt5.QtGui import QDragEnterEvent, QDropEvent, QKeySequence, QIcon, QDesktopServices
from vtk.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import vtk

from urdf_parser import URDFParser
from urdf_vtk_model import URDFModel
from simplify_mesh import create_detailed_approximation
from translations import TranslationManager, tr, get_translation_manager
from geometry_factory import GeometryFactory
from topology_dialog import TopologyDialog
from inertia_visualizer import InertiaVisualizer
from drag_interaction_style import DragJointInteractorStyle
from theme import ThemeManager, themed_icon
from widgets import CollapsibleSection

try:
    from mjcf_parser import MJCFParser
    HAS_MJCF = True
except ImportError:
    HAS_MJCF = False


class _SimpleCollisionModel:
    """Lightweight wrapper for primitive collision geometry actors.

    Provides the same interface as URDFModel for use in models_collision list.
    """

    def __init__(self, actor):
        self.actor = actor
        self.axes_actor = None
        self.text_actor = None
        self.name = ""
        self.transparency = 1.0

    def set_transparency(self, transparency):
        self.transparency = transparency
        self.actor.GetProperty().SetOpacity(transparency)

    def apply_transform(self, transform_matrix):
        vtk_transform = vtk.vtkTransform()
        vtk_transform.SetMatrix(transform_matrix.flatten())
        self.actor.SetUserTransform(vtk_transform)


class DragDropVTKWidget(QVTKRenderWindowInteractor):
    """Custom VTK widget that supports drag-and-drop of URDF/MJCF files"""

    SUPPORTED_EXTENSIONS = ('.urdf', '.xml')

    def __init__(self, parent=None):
        super().__init__()
        self.parent_viewer = parent
        self.setAcceptDrops(True)

    def dragEnterEvent(self, event: QDragEnterEvent):
        """Handle drag enter event"""
        if event.mimeData().hasUrls():
            for url in event.mimeData().urls():
                if url.isLocalFile():
                    file_path = url.toLocalFile()
                    if file_path.lower().endswith(self.SUPPORTED_EXTENSIONS):
                        event.acceptProposedAction()
                        return
        event.ignore()

    def dropEvent(self, event: QDropEvent):
        """Handle drop event"""
        if event.mimeData().hasUrls():
            for url in event.mimeData().urls():
                if url.isLocalFile():
                    file_path = url.toLocalFile()
                    if file_path.lower().endswith(self.SUPPORTED_EXTENSIONS):
                        if self.parent_viewer:
                            self.parent_viewer.load_urdf_file(file_path)
                        event.acceptProposedAction()
                        return
        event.ignore()


class URDFViewer(QMainWindow):
    """Main application window for URDF viewer"""

    def __init__(self):
        super().__init__()
        self.translation_manager = get_translation_manager()
        self.models = []  # List to store loaded URDF models
        self.models_collision = []
        self.chains = []  # List to store kinematic chains
        self.mdh_frames_actors = []  # List to store MDH frame actors
        self.mdh_text_actors = []  # List to store MDH frame text actors
        self.joint_axis_actors = []  # List to store joint axis actors
        self.joint_axis_info = []  # List to store joint axis info
        self.selected_chain = None  # Currently selected chain
        self.selected_chain_index = 0  # Index of the currently selected chain
        self.current_urdf_file = None  # Path to the currently loaded URDF file
        self.current_parser = None     # Cached parser for efficient joint updates
        self.collision_mesh_files = None

        self.joint_sliders = []  # List to store joint angle sliders
        self.joint_values = []  # List to store joint angle values
        self.revolute_joints = []  # List to store revolute joints
        self.joint_value_labels = []  # List to store joint value labels
        self.display_in_degrees = False  # False: rad, True: deg
        self.inertia_visualizer = None  # Will be created after renderer init
        self.actor_to_link = {}  # actor -> link_name mapping for drag interaction
        self.settings = QSettings("URDFly", "URDFly")
        self.max_recent_files = 10
        self.init_ui()
        

    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle(tr("window_title"))
        self.setWindowIcon(QIcon(os.path.join(os.path.dirname(__file__), "icons", "urdfly-logo.svg")))

        # Set window size
        window_width = 1200
        window_height = 800

        # Get screen size and calculate center position
        screen = QDesktopWidget().availableGeometry()
        x = (screen.width() - window_width) // 2
        y = (screen.height() - window_height) // 2

        # Set window geometry to be centered on screen
        self.setGeometry(x, y, window_width, window_height)

        # ====== QMenuBar ======
        self._create_menubar()

        # ====== QToolBar ======
        self._create_toolbar()

        # ====== QStatusBar ======
        self._create_statusbar()

        # ====== Central Widget: QSplitter 三面板 ======
        self.splitter = QSplitter(Qt.Horizontal)

        # --- Left Panel ---
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(6, 6, 6, 6)
        left_layout.setSpacing(6)

        # CollapsibleSection: 机器人结构
        self.section_structure = CollapsibleSection(tr("section_robot_structure"))
        self.select_chain_label = QLabel(tr("select_chain"))
        self.section_structure.add_widget(self.select_chain_label)
        self.chain_combo = QComboBox()
        self.chain_combo.currentIndexChanged.connect(self.on_chain_selected)
        self.section_structure.add_widget(self.chain_combo)
        self.links_label = QLabel(tr("links"))
        self.section_structure.add_widget(self.links_label)
        self.link_list = QListWidget()
        self.link_list.setSelectionMode(QListWidget.SingleSelection)
        self.link_list.itemSelectionChanged.connect(self.on_link_selection_changed)
        self.section_structure.add_widget(self.link_list)
        left_layout.addWidget(self.section_structure)

        # CollapsibleSection: 透明度
        self.section_transparency = CollapsibleSection(tr("section_transparency"))
        self.transparency_label = QLabel(tr("transparency"))
        self.section_transparency.add_widget(self.transparency_label)
        self.transparency_slider = QSlider(Qt.Horizontal)
        self.transparency_slider.setMinimum(0)
        self.transparency_slider.setMaximum(100)
        self.transparency_slider.setValue(100)
        self.transparency_slider.setTickPosition(QSlider.TicksBelow)
        self.transparency_slider.setTickInterval(10)
        self.transparency_slider.valueChanged.connect(self.apply_transparency)
        self.section_transparency.add_widget(self.transparency_slider)
        left_layout.addWidget(self.section_transparency)

        # CollapsibleSection: 显示设置
        self.section_display = CollapsibleSection(tr("section_display"))

        self.cb_visual = QCheckBox(tr("show_visual"))
        self.cb_visual.setChecked(True)
        self.cb_visual.stateChanged.connect(self.toggle_visual)

        self.cb_link_frames = QCheckBox(tr("show_link_frames"))
        self.cb_link_frames.setChecked(True)
        self.cb_link_frames.stateChanged.connect(self.toggle_link_frames)

        self.cb_mdh_frames = QCheckBox(tr("show_mdh_frames"))
        self.cb_mdh_frames.setChecked(False)
        self.cb_mdh_frames.stateChanged.connect(self.toggle_mdh_frames)

        self.cb_collision = QCheckBox(tr("show_collision"))
        self.cb_collision.setChecked(True)
        self.cb_collision.stateChanged.connect(self.toggle_collision)

        self.cb_joint_axes = QCheckBox(tr("show_joint_axes"))
        self.cb_joint_axes.setChecked(False)
        self.cb_joint_axes.stateChanged.connect(self.toggle_joint_axes)

        self.cb_com = QCheckBox(tr("show_com"))
        self.cb_com.setChecked(False)
        self.cb_com.stateChanged.connect(self.toggle_com)

        self.cb_inertia = QCheckBox(tr("show_inertia"))
        self.cb_inertia.setChecked(False)
        self.cb_inertia.stateChanged.connect(self.toggle_inertia)

        for cb in (self.cb_visual, self.cb_link_frames, self.cb_mdh_frames,
                   self.cb_collision, self.cb_joint_axes, self.cb_com, self.cb_inertia):
            self.section_display.add_widget(cb)
        left_layout.addWidget(self.section_display)

        left_layout.addStretch()

        # Language selection
        lang_layout = QHBoxLayout()
        self.lang_label = QLabel(tr("language"))
        lang_layout.addWidget(self.lang_label)
        self.language_combo = QComboBox()
        self.language_combo.addItem(tr("lang_zh"), "zh_CN")
        self.language_combo.addItem(tr("lang_en"), "en")
        self.language_combo.setCurrentIndex(0)
        self.language_combo.currentIndexChanged.connect(self.change_language)
        lang_layout.addWidget(self.language_combo)
        left_layout.addLayout(lang_layout)

        # Current file label
        self.current_file_label = QLabel(tr("current_file") + " " + tr("current_file_none"))
        self.current_file_label.setWordWrap(True)
        self.current_file_label.setAlignment(Qt.AlignLeft | Qt.AlignBottom)
        left_layout.addWidget(self.current_file_label)

        left_panel.setMinimumWidth(200)

        # --- VTK Widget (center) ---
        self.vtk_widget = DragDropVTKWidget(self)

        # --- Right Panel: 关节控制 ---
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(6, 6, 6, 6)
        right_layout.setSpacing(6)

        # Joint control header
        self.joint_group = QGroupBox(tr("joints_control"))
        joint_group_layout = QVBoxLayout(self.joint_group)

        buttons_layout = QHBoxLayout()
        self.btn_reset = QPushButton(tr("reset"))
        self.btn_reset.clicked.connect(self.reset_joints)
        self.btn_random = QPushButton(tr("random"))
        self.btn_random.clicked.connect(self.randomize_joints)
        buttons_layout.addWidget(self.btn_reset)
        buttons_layout.addWidget(self.btn_random)
        self.units_combo = QComboBox()
        self.units_combo.addItems(["rad", "deg"])
        self.units_combo.setCurrentText("rad")
        self.units_combo.currentTextChanged.connect(self.on_units_changed)
        buttons_layout.addWidget(self.units_combo)
        joint_group_layout.addLayout(buttons_layout)

        # Joint scroll area
        joint_scroll_area = QScrollArea()
        joint_scroll_area.setWidgetResizable(True)
        joint_scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        joint_scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)

        self.joint_container = QWidget()
        self.joint_layout = QVBoxLayout(self.joint_container)
        self.joint_label = QLabel(tr("adjust_joint_angles"))
        self.joint_layout.addWidget(self.joint_label)
        joint_scroll_area.setWidget(self.joint_container)

        joint_group_layout.addWidget(joint_scroll_area)
        right_layout.addWidget(self.joint_group, 1)

        right_panel.setMinimumWidth(180)

        # --- Assemble QSplitter ---
        self.splitter.addWidget(left_panel)
        self.splitter.addWidget(self.vtk_widget)
        self.splitter.addWidget(right_panel)
        self.splitter.setSizes([260, 680, 260])
        self.splitter.setStretchFactor(0, 0)
        self.splitter.setStretchFactor(1, 1)
        self.splitter.setStretchFactor(2, 0)

        self.setCentralWidget(self.splitter)

        # Store panel refs for toggle
        self._left_panel = left_panel
        self._right_panel = right_panel

        # ====== VTK Rendering Setup ======
        self.renderer = vtk.vtkRenderer()
        _bottom, _top = ThemeManager().get_vtk_background()
        self.renderer.SetBackground(*_bottom)
        self.renderer.SetBackground2(*_top)
        self.renderer.SetGradientBackground(True)
        self.vtk_widget.GetRenderWindow().AddRenderer(self.renderer)

        # Initialize inertia visualizer
        self.inertia_visualizer = InertiaVisualizer(self.renderer)

        # Set up interactor
        self.interactor = self.vtk_widget.GetRenderWindow().GetInteractor()

        # Configure interactor style with drag joint support
        self.interaction_style = DragJointInteractorStyle()
        self.interaction_style.set_callbacks(
            get_link_name=self._get_link_name_by_actor,
            get_joint_for_link=self._get_joint_for_child_link,
            on_joint_drag=self._on_joint_drag,
            on_drag_start=self._on_drag_start,
            on_drag_end=self._on_drag_end,
        )
        self.interactor.SetInteractorStyle(self.interaction_style)

        # Initialize interactor
        self.interactor.Initialize()

        # Start the interactor
        self.interactor.Start()

        # ====== View Overlay (floating buttons on VTK viewport) ======
        self._create_view_overlay()

    # ------------------------------------------------------------------
    # Menu / Toolbar / Statusbar builders
    # ------------------------------------------------------------------

    def _icon(self, name):
        """Load an SVG icon with theme-appropriate stroke color."""
        return themed_icon(name)

    def _create_menubar(self):
        """Create the menu bar with File, View, Tools, Help menus."""
        menubar = self.menuBar()

        # File menu
        self.menu_file = menubar.addMenu(tr("menu_file"))
        self.act_open = QAction(self._icon("folder-open.svg"), tr("open_urdf"), self)
        self.act_open.setShortcut(QKeySequence("Ctrl+O"))
        self.act_open.triggered.connect(self.open_urdf_file)
        self.menu_file.addAction(self.act_open)

        self.act_edit = QAction(self._icon("file-edit.svg"), tr("edit_urdf"), self)
        self.act_edit.setShortcut(QKeySequence("Ctrl+E"))
        self.act_edit.triggered.connect(self.edit_urdf_file)
        self.menu_file.addAction(self.act_edit)

        self.menu_recent = self.menu_file.addMenu(tr("recent_files"))
        self._update_recent_files_menu()

        self.menu_file.addSeparator()

        self.act_quit = QAction(tr("quit"), self)
        self.act_quit.setShortcut(QKeySequence("Ctrl+Q"))
        self.act_quit.triggered.connect(self.close)
        self.menu_file.addAction(self.act_quit)

        # View menu
        self.menu_view = menubar.addMenu(tr("menu_view"))
        self.act_toggle_left = QAction(tr("toggle_left_panel"), self)
        self.act_toggle_left.triggered.connect(self._toggle_left_panel)
        self.menu_view.addAction(self.act_toggle_left)

        self.act_toggle_right = QAction(tr("toggle_right_panel"), self)
        self.act_toggle_right.triggered.connect(self._toggle_right_panel)
        self.menu_view.addAction(self.act_toggle_right)

        self.menu_view.addSeparator()

        self.act_dark_theme = QAction(tr("theme_dark"), self)
        self.act_dark_theme.triggered.connect(lambda: self._switch_theme("dark"))
        self.menu_view.addAction(self.act_dark_theme)

        self.act_light_theme = QAction(tr("theme_light"), self)
        self.act_light_theme.triggered.connect(lambda: self._switch_theme("light"))
        self.menu_view.addAction(self.act_light_theme)

        # Camera views submenu
        self.menu_view.addSeparator()
        self.menu_camera_views = self.menu_view.addMenu(tr("camera_views"))

        self.act_view_front = QAction(self._icon("view-front.svg"), tr("view_front"), self)
        self.act_view_front.setShortcut(QKeySequence("1"))
        self.act_view_front.triggered.connect(lambda: self._set_camera_view("front"))
        self.menu_camera_views.addAction(self.act_view_front)

        self.act_view_back = QAction(self._icon("view-back.svg"), tr("view_back"), self)
        self.act_view_back.setShortcut(QKeySequence("Ctrl+1"))
        self.act_view_back.triggered.connect(lambda: self._set_camera_view("back"))
        self.menu_camera_views.addAction(self.act_view_back)

        self.act_view_left = QAction(self._icon("view-left.svg"), tr("view_left"), self)
        self.act_view_left.setShortcut(QKeySequence("3"))
        self.act_view_left.triggered.connect(lambda: self._set_camera_view("left"))
        self.menu_camera_views.addAction(self.act_view_left)

        self.act_view_right = QAction(self._icon("view-right.svg"), tr("view_right"), self)
        self.act_view_right.setShortcut(QKeySequence("Ctrl+3"))
        self.act_view_right.triggered.connect(lambda: self._set_camera_view("right"))
        self.menu_camera_views.addAction(self.act_view_right)

        self.act_view_top = QAction(self._icon("view-top.svg"), tr("view_top"), self)
        self.act_view_top.setShortcut(QKeySequence("7"))
        self.act_view_top.triggered.connect(lambda: self._set_camera_view("top"))
        self.menu_camera_views.addAction(self.act_view_top)

        self.act_view_bottom = QAction(self._icon("view-bottom.svg"), tr("view_bottom"), self)
        self.act_view_bottom.setShortcut(QKeySequence("Ctrl+7"))
        self.act_view_bottom.triggered.connect(lambda: self._set_camera_view("bottom"))
        self.menu_camera_views.addAction(self.act_view_bottom)

        self.act_view_isometric = QAction(self._icon("view-isometric.svg"), tr("view_isometric"), self)
        self.act_view_isometric.setShortcut(QKeySequence("0"))
        self.act_view_isometric.triggered.connect(lambda: self._set_camera_view("isometric"))
        self.menu_camera_views.addAction(self.act_view_isometric)

        # Tools menu
        self.menu_tools = menubar.addMenu(tr("menu_tools"))
        self.act_mdh = QAction(self._icon("table.svg"), tr("show_mdh"), self)
        self.act_mdh.setShortcut(QKeySequence("Ctrl+M"))
        self.act_mdh.triggered.connect(self.show_mdh_parameters)
        self.menu_tools.addAction(self.act_mdh)

        self.act_topology = QAction(self._icon("git-branch.svg"), tr("show_topology"), self)
        self.act_topology.setShortcut(QKeySequence("Ctrl+T"))
        self.act_topology.triggered.connect(self.show_topology_graph)
        self.menu_tools.addAction(self.act_topology)

        self.act_decomp = QAction(self._icon("box.svg"), tr("decompose_collision"), self)
        self.act_decomp.triggered.connect(self.decompose_collision_meshes)
        self.menu_tools.addAction(self.act_decomp)

        self.act_set_joints = QAction(self._icon("sliders.svg"), tr("set_joints"), self)
        self.act_set_joints.triggered.connect(self.open_set_joints_dialog)
        self.menu_tools.addAction(self.act_set_joints)

        # Help menu
        self.menu_help = menubar.addMenu(tr("menu_help"))

        self.act_quick_start = QAction(tr("quick_start_guide"), self)
        self.act_quick_start.triggered.connect(self._show_quick_start)
        self.menu_help.addAction(self.act_quick_start)

        self.act_shortcuts = QAction(tr("keyboard_shortcuts"), self)
        self.act_shortcuts.triggered.connect(self._show_shortcuts)
        self.menu_help.addAction(self.act_shortcuts)

        self.menu_help.addSeparator()

        self.act_tutorial_mdh = QAction(tr("tutorial_mdh"), self)
        self.act_tutorial_mdh.triggered.connect(lambda: self._open_tutorial("MDH_Parameters_Tutorial.md"))
        self.menu_help.addAction(self.act_tutorial_mdh)

        self.act_tutorial_ik = QAction(tr("tutorial_ik"), self)
        self.act_tutorial_ik.triggered.connect(lambda: self._open_tutorial("Analytical_IK_Tutorial.md"))
        self.menu_help.addAction(self.act_tutorial_ik)

        self.menu_help.addSeparator()

        self.act_about = QAction(tr("about"), self)
        self.act_about.triggered.connect(self._show_about)
        self.menu_help.addAction(self.act_about)

    def _create_toolbar(self):
        """Create the main toolbar."""
        self.toolbar = QToolBar()
        self.toolbar.setMovable(False)
        self.toolbar.setIconSize(QSize(18, 18))
        self.toolbar.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.addToolBar(self.toolbar)

        self.tb_open_btn = QToolButton(self.toolbar)
        self.tb_open_btn.setDefaultAction(self.act_open)
        self.tb_open_btn.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.tb_open_btn.setPopupMode(QToolButton.MenuButtonPopup)
        self.tb_open_btn.setMenu(self.menu_recent)
        self.toolbar.addWidget(self.tb_open_btn)

        self.toolbar.addAction(self.act_edit)
        self.toolbar.addAction(self.act_mdh)
        self.toolbar.addAction(self.act_decomp)
        self.toolbar.addAction(self.act_topology)
        self.toolbar.addSeparator()

        # Reset / Random as toolbar buttons
        self.tb_act_reset = QAction(self._icon("rotate-ccw.svg"), tr("reset"), self)
        self.tb_act_reset.setShortcut(QKeySequence("Ctrl+R"))
        self.tb_act_reset.triggered.connect(self.reset_joints)
        self.toolbar.addAction(self.tb_act_reset)

        self.tb_act_random = QAction(self._icon("shuffle.svg"), tr("random"), self)
        self.tb_act_random.triggered.connect(self.randomize_joints)
        self.toolbar.addAction(self.tb_act_random)

    def _create_statusbar(self):
        """Create the status bar."""
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_label = QLabel(tr("ready"))
        self.status_bar.addPermanentWidget(self.status_label)

    def _create_view_overlay(self):
        """Create a floating view-button panel over the VTK viewport."""
        tm = ThemeManager()
        is_dark = tm.current_theme == "dark"

        self._view_overlay = QFrame(self.vtk_widget)
        self._view_overlay.setObjectName("viewOverlay")
        self._update_view_overlay_style()

        layout = QVBoxLayout(self._view_overlay)
        layout.setContentsMargins(3, 3, 3, 3)
        layout.setSpacing(2)

        view_defs = [
            (self.act_view_front, "view_front"),
            (self.act_view_back, "view_back"),
            (self.act_view_left, "view_left"),
            (self.act_view_right, "view_right"),
            (self.act_view_top, "view_top"),
            (self.act_view_bottom, "view_bottom"),
            (self.act_view_isometric, "view_isometric"),
        ]

        self._view_overlay_buttons = []
        for action, tip_key in view_defs:
            btn = QToolButton(self._view_overlay)
            btn.setDefaultAction(action)
            btn.setToolButtonStyle(Qt.ToolButtonIconOnly)
            btn.setFixedSize(28, 28)
            btn.setIconSize(QSize(18, 18))
            btn.setToolTip(tr(tip_key))
            btn.setAutoRaise(True)
            layout.addWidget(btn)
            self._view_overlay_buttons.append((btn, tip_key))

        self._view_overlay.adjustSize()
        self._view_overlay.show()

        self.vtk_widget.installEventFilter(self)
        self._update_view_overlay_position()

    def _update_view_overlay_style(self):
        """Apply theme-aware style to the view overlay panel."""
        tm = ThemeManager()
        is_dark = tm.current_theme == "dark"
        if is_dark:
            bg = "rgba(50, 50, 50, 180)"
            border = "rgba(80, 80, 80, 200)"
        else:
            bg = "rgba(255, 255, 255, 180)"
            border = "rgba(200, 200, 200, 220)"
        self._view_overlay.setStyleSheet(
            f"#viewOverlay {{ background: {bg}; border: 1px solid {border}; border-radius: 6px; }}"
        )

    def _update_view_overlay_position(self):
        """Reposition the view overlay to the top-right of the VTK widget."""
        if not hasattr(self, '_view_overlay'):
            return
        margin = 8
        vw = self.vtk_widget.width()
        ow = self._view_overlay.width()
        self._view_overlay.move(vw - ow - margin, margin)

    def eventFilter(self, obj, event):
        """Reposition the view overlay when the VTK widget is resized."""
        if obj is self.vtk_widget and event.type() == QEvent.Resize:
            self._update_view_overlay_position()
        return super().eventFilter(obj, event)

    # ------------------------------------------------------------------
    # Toggle / Theme helpers
    # ------------------------------------------------------------------

    def _toggle_left_panel(self):
        self._left_panel.setVisible(not self._left_panel.isVisible())

    def _toggle_right_panel(self):
        self._right_panel.setVisible(not self._right_panel.isVisible())

    def _switch_theme(self, name):
        tm = ThemeManager()
        tm.set_theme(name, QApplication.instance())
        _bottom, _top = tm.get_vtk_background()
        self.renderer.SetBackground(*_bottom)
        self.renderer.SetBackground2(*_top)
        self.vtk_widget.GetRenderWindow().Render()
        self._refresh_icons()
        if hasattr(self, '_view_overlay'):
            self._update_view_overlay_style()

    def _refresh_icons(self):
        """Refresh all action icons after theme change."""
        icon_map = {
            self.act_open: "folder-open.svg",
            self.act_edit: "file-edit.svg",
            self.act_mdh: "table.svg",
            self.act_decomp: "box.svg",
            self.act_topology: "git-branch.svg",
            self.act_set_joints: "sliders.svg",
            self.tb_act_reset: "rotate-ccw.svg",
            self.tb_act_random: "shuffle.svg",
            self.act_view_front: "view-front.svg",
            self.act_view_back: "view-back.svg",
            self.act_view_left: "view-left.svg",
            self.act_view_right: "view-right.svg",
            self.act_view_top: "view-top.svg",
            self.act_view_bottom: "view-bottom.svg",
            self.act_view_isometric: "view-isometric.svg",
        }
        for action, icon_name in icon_map.items():
            action.setIcon(self._icon(icon_name))

    def _set_camera_view(self, view_name):
        """Set camera to a predefined view direction (Z-up coordinate system).

        Args:
            view_name: one of 'front', 'back', 'left', 'right', 'top', 'bottom', 'isometric'
        """
        views = {
            'front':     {'direction': ( 1,  0,  0), 'view_up': (0, 0, 1)},
            'back':      {'direction': (-1,  0,  0), 'view_up': (0, 0, 1)},
            'left':      {'direction': ( 0,  1,  0), 'view_up': (0, 0, 1)},
            'right':     {'direction': ( 0, -1,  0), 'view_up': (0, 0, 1)},
            'top':       {'direction': ( 0,  0,  1), 'view_up': (0, 1, 0)},
            'bottom':    {'direction': ( 0,  0, -1), 'view_up': (0, -1, 0)},
            'isometric': {'direction': ( 1, -1,  1), 'view_up': (0, 0, 1)},
        }
        v = views.get(view_name)
        if v is None:
            return

        camera = self.renderer.GetActiveCamera()
        self.renderer.ResetCamera()

        focal = list(camera.GetFocalPoint())
        dist = camera.GetDistance()

        dx, dy, dz = v['direction']
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        dx, dy, dz = dx / length, dy / length, dz / length

        camera.SetPosition(
            focal[0] + dx * dist,
            focal[1] + dy * dist,
            focal[2] + dz * dist,
        )
        camera.SetFocalPoint(*focal)
        camera.SetViewUp(*v['view_up'])
        self.renderer.ResetCameraClippingRange()
        self.vtk_widget.GetRenderWindow().Render()

    def _show_about(self):
        QMessageBox.about(self, tr("about"), tr("about_text"))

    def _show_quick_start(self):
        """Show the Quick Start Guide dialog."""
        dlg = QDialog(self)
        dlg.setWindowTitle(tr("quick_start_title"))
        dlg.resize(700, 500)
        layout = QVBoxLayout(dlg)
        browser = QTextBrowser()
        browser.setOpenExternalLinks(True)
        browser.setHtml(tr("quick_start_html"))
        layout.addWidget(browser)
        btn_close = QPushButton(tr("btn_ok"))
        btn_close.clicked.connect(dlg.accept)
        layout.addWidget(btn_close)
        dlg.exec_()

    def _show_shortcuts(self):
        """Show the Keyboard Shortcuts dialog."""
        dlg = QDialog(self)
        dlg.setWindowTitle(tr("shortcuts_title"))
        dlg.resize(500, 400)
        layout = QVBoxLayout(dlg)

        shortcuts = [
            ("Ctrl+O", tr("open_urdf")),
            ("Ctrl+E", tr("edit_urdf")),
            ("Ctrl+M", tr("show_mdh")),
            ("Ctrl+T", tr("show_topology")),
            ("Ctrl+R", tr("reset")),
            ("Ctrl+Q", tr("quit")),
            ("Ctrl+F", tr("search")),
            ("F3", tr("next")),
            ("Shift+F3", tr("previous")),
            ("1", tr("view_front")),
            ("Ctrl+1", tr("view_back")),
            ("3", tr("view_left")),
            ("Ctrl+3", tr("view_right")),
            ("7", tr("view_top")),
            ("Ctrl+7", tr("view_bottom")),
            ("0", tr("view_isometric")),
        ]

        table = QTableWidget(len(shortcuts), 2)
        table.setHorizontalHeaderLabels([tr("shortcut_key"), tr("shortcut_action")])
        table.horizontalHeader().setStretchLastSection(True)
        table.verticalHeader().setVisible(False)
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        table.setSelectionBehavior(QTableWidget.SelectRows)
        for row, (key, desc) in enumerate(shortcuts):
            table.setItem(row, 0, QTableWidgetItem(key))
            table.setItem(row, 1, QTableWidgetItem(desc))
        layout.addWidget(table)

        btn_close = QPushButton(tr("btn_ok"))
        btn_close.clicked.connect(dlg.accept)
        layout.addWidget(btn_close)
        dlg.exec_()

    def _open_tutorial(self, filename):
        """Open a tutorial markdown file with the system default application."""
        path = os.path.join(os.path.dirname(__file__), "docs", filename)
        QDesktopServices.openUrl(QUrl.fromLocalFile(path))

    def add_world_axes(self):
        """Add world coordinate axes to the scene"""
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(0.1, 0.1, 0.1)  # Set the length of the axes
        axes.SetShaftType(0)
        axes.SetAxisLabels(0)  # Not Show labels
        axes.SetCylinderRadius(0.02)

        # Add the axes to the renderer
        self.renderer.AddActor(axes)

    def _create_parser(self, filename):
        """根据文件扩展名创建对应的解析器

        Args:
            filename: 文件路径

        Returns:
            parser 实例 (URDFParser 或 MJCFParser)

        Raises:
            ValueError: 不支持的文件格式
            ImportError: MJCF 依赖未安装
        """
        ext = os.path.splitext(filename)[1].lower()
        if ext == '.urdf':
            return URDFParser(filename)
        elif ext == '.xml':
            # 验证是否为有效的 MJCF 文件
            try:
                from xml.etree import ElementTree as ET
                tree = ET.parse(filename)
                root = tree.getroot()
                if root.tag != 'mujoco':
                    raise ValueError(tr("invalid_mjcf_file"))
            except ET.ParseError as e:
                raise ValueError(tr("invalid_mjcf_file"))

            if not HAS_MJCF:
                raise ImportError("MJCF 支持需要安装 mujoco: pip install mujoco")
            return MJCFParser(filename)
        else:
            raise ValueError(tr("unsupported_format", ext))

    def load_urdf_file(self, filename):
        """Load a URDF/MJCF file and visualize the robot"""
        if filename and os.path.exists(filename):
            # Clear previous models
            self.clear_models()

            # Parse the file
            try:
                parser = self._create_parser(filename)
                
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
                collision_mesh_files,
                collision_mesh_transformations,
                joint_limits,
                collision_link_names,
                collision_geometries,
                ) = parser.get_robot_info()

                # Store revolute joints for slider controls
                self.revolute_joints = []
                for i, joint_type in enumerate(joint_types):
                    if joint_type == 'revolute':
                        limit = joint_limits[i]
                        self.revolute_joints.append({
                            'name': joint_names[i],
                            'index': i,
                            'parent': joint_parent_links[i],
                            'child': joint_child_links[i],
                            'axis': joint_axes[i],
                            'lower': limit['lower'],
                            'upper': limit['upper'],
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
                
                # Create models for each collision
                for i in range(len(collision_geometries)):
                    coll_link = collision_link_names[i] if i < len(collision_link_names) else None
                    geom = collision_geometries[i]
                    if geom['type'] == 'mesh':
                        self.add_urdf_model(
                            f"collision_{i}",
                            collision_mesh_files[i],
                            collision_mesh_transformations[i],
                            None,
                            None,
                            model_type='collision',
                            link_name=coll_link,
                        )
                    else:
                        self._add_collision_primitive_model(
                            geom, collision_mesh_transformations[i], coll_link,
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
                self._add_recent_file(filename)
                # Cache the parser instance for efficient joint updates
                self.current_parser = parser

                # Apply visibility settings from checkboxes to new models
                self._apply_visibility_settings()

                # Update the current file label
                self.update_current_file_label()

            except Exception as e:
                QMessageBox.critical(
                    self, tr("error"), tr("load_urdf_failed", str(e))
                )

    def open_urdf_file(self):
        """Open a URDF file dialog and load the selected file"""
        filename, _ = QFileDialog.getOpenFileName(
            self, tr("dialog_open_robot"), "", tr("dialog_robot_filter")
        )
        
        if filename:
            self.load_urdf_file(filename)

    def _add_recent_file(self, filepath):
        """Add a file path to the recent files list in QSettings."""
        files = self.settings.value("recent_files", [], type=list)
        filepath = os.path.abspath(filepath)
        if filepath in files:
            files.remove(filepath)
        files.insert(0, filepath)
        files = files[:self.max_recent_files]
        self.settings.setValue("recent_files", files)
        self._update_recent_files_menu()

    def _update_recent_files_menu(self):
        """Rebuild the recent files submenu from QSettings."""
        self.menu_recent.clear()
        files = self.settings.value("recent_files", [], type=list)
        if not files:
            act = self.menu_recent.addAction(tr("current_file_none"))
            act.setEnabled(False)
            return
        for filepath in files:
            act = self.menu_recent.addAction(filepath)
            act.triggered.connect(lambda checked, f=filepath: self._open_recent_file(f))

    def _open_recent_file(self, filepath):
        """Open a model file from the recent files list."""
        if os.path.exists(filepath):
            self.load_urdf_file(filepath)
        else:
            QMessageBox.warning(self, tr("warning"), tr("failed_to_load_file", filepath))

    def add_urdf_model(self, name, mesh_file, mesh_transform, frame, color, model_type='visual', link_name=None):

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

            # Build actor → link_name mapping for drag interaction
            if model_type == 'visual':
                self.actor_to_link[id(model.actor)] = name
            elif model_type == 'collision' and link_name:
                self.actor_to_link[id(model.actor)] = link_name

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
            QMessageBox.warning(
                self, tr("warning"), tr("load_model_failed", name, str(e))
            )

    def _add_collision_primitive_model(self, geom, transform_matrix, link_name=None):
        """Add a collision primitive (box/sphere/cylinder) to the scene.

        Args:
            geom: dict with 'type' and geometry parameters
            transform_matrix: 4x4 transformation matrix
            link_name: optional link name for drag interaction mapping
        """
        geom_type = geom['type']
        if geom_type == 'box':
            actor = GeometryFactory.create_collision_box(geom['size'], transform_matrix)
        elif geom_type == 'sphere':
            actor = GeometryFactory.create_collision_sphere(geom['radius'], transform_matrix)
        elif geom_type == 'cylinder':
            actor = GeometryFactory.create_collision_cylinder(
                geom['radius'], geom['length'], transform_matrix)
        else:
            return

        model = _SimpleCollisionModel(actor)
        self.models_collision.append(model)
        self.renderer.AddActor(actor)

        if link_name:
            self.actor_to_link[id(actor)] = link_name

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
        self.actor_to_link = {}
        self.current_parser = None


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

        # Clear joint axes
        for actor in self.joint_axis_actors:
            self.renderer.RemoveActor(actor)
        self.joint_axis_actors = []
        self.joint_axis_info = []

        # Clear inertia visualizations
        if self.inertia_visualizer:
            self.inertia_visualizer.clear()

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
            self.chain_combo.addItem(tr("chain_pattern").format(i+1, chain['name']), i)
        
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
        
    def toggle_visual(self, state):
        """Toggle visibility of visual models"""
        visible = state == Qt.Checked

        for model in self.models:
            model.actor.SetVisibility(visible)

        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()

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
                    self, tr("warning"), tr("please_load_urdf_mdh")
                )
                self.cb_mdh_frames.setChecked(False)
                return

            if self.selected_chain:
                # Always recreate MDH frames to ensure they're up to date
                self.create_mdh_frames(self.selected_chain)
            else:
                QMessageBox.warning(
                    self, tr("warning"), tr("please_select_chain_first")
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

    def toggle_joint_axes(self, state):
        """Toggle visibility of joint axes"""
        visible = state == Qt.Checked

        # If we want to show joint axes
        if visible:
            if not self.current_urdf_file:
                QMessageBox.warning(
                    self, tr("warning"), tr("please_load_urdf_mdh")
                )
                self.cb_joint_axes.setChecked(False)
                return

            # Create joint axes
            self.create_joint_axes()
        else:
            # Hide joint axes
            for actor in self.joint_axis_actors:
                actor.SetVisibility(False)

        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()

    def create_joint_axes(self):
        """Create joint axis actors for the current URDF"""
        # Clear existing joint axes
        for actor in self.joint_axis_actors:
            self.renderer.RemoveActor(actor)
        self.joint_axis_actors = []
        self.joint_axis_info = []

        # Get joint data with current joint angles
        if self.current_parser and HAS_MJCF and isinstance(self.current_parser, MJCFParser):
            result = self.current_parser.update_transforms(self.joint_values)
            joint_names = result['joint_names']
            joint_frames = result['joint_frames']
            joint_axes = result['joint_axes']
            joint_types = [j['type'] for j in self.current_parser.joints]
        else:
            parser = self._create_parser(self.current_urdf_file)
            (_, _, _, _, _,
             joint_names, joint_frames, joint_types, joint_axes,
             _, _, _, _, _, _, _) = parser.get_robot_info(qs=self.joint_values)

        # Create axis arrows for each revolute and continuous joint
        for joint_name, joint_frame, joint_type, axis in zip(
            joint_names, joint_frames, joint_types, joint_axes
        ):
            # Only create axes for revolute and continuous joints
            if joint_type not in ['revolute', 'continuous']:
                continue

            # Create joint axis arrow
            actor = GeometryFactory.create_joint_axis_arrow(
                joint_frame, axis
            )
            actor.SetVisibility(self.cb_joint_axes.isChecked())

            self.renderer.AddActor(actor)
            self.joint_axis_actors.append(actor)
            self.joint_axis_info.append({
                'joint_name': joint_name,
                'axis': list(axis)
            })

        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()

    def toggle_com(self, state):
        """Toggle visibility of center of mass markers"""
        visible = state == Qt.Checked

        if visible:
            if not self.current_urdf_file:
                QMessageBox.warning(
                    self, tr("warning"), tr("please_load_urdf_com")
                )
                self.cb_com.setChecked(False)
                return

            # Create CoM markers
            parser = self._create_parser(self.current_urdf_file)
            (link_names, _, _, link_frames, _, _, _, _, _, _, _, _, _, _, _, _) = parser.get_robot_info(qs=self.joint_values)
            self.inertia_visualizer.create_com_markers(parser, link_names, link_frames)

            # Register CoM actors for drag interaction
            for info in self.inertia_visualizer.com_actor_info:
                self.actor_to_link[id(info['actor'])] = info['link_name']
        else:
            # Unregister CoM actors
            for info in self.inertia_visualizer.com_actor_info:
                self.actor_to_link.pop(id(info['actor']), None)

        self.inertia_visualizer.set_com_visibility(visible)
        self.vtk_widget.GetRenderWindow().Render()

    def toggle_inertia(self, state):
        """Toggle visibility of inertia boxes"""
        visible = state == Qt.Checked

        if visible:
            if not self.current_urdf_file:
                QMessageBox.warning(
                    self, tr("warning"), tr("please_load_urdf_inertia")
                )
                self.cb_inertia.setChecked(False)
                return

            # Create inertia boxes
            parser = self._create_parser(self.current_urdf_file)
            (link_names, _, _, link_frames, _, _, _, _, _, _, _, _, _, _, _, _) = parser.get_robot_info(qs=self.joint_values)
            self.inertia_visualizer.create_inertia_boxes(parser, link_names, link_frames)

            # Register inertia actors for drag interaction
            for info in self.inertia_visualizer.inertia_actor_info:
                self.actor_to_link[id(info['actor'])] = info['link_name']
        else:
            # Unregister inertia actors
            for info in self.inertia_visualizer.inertia_actor_info:
                self.actor_to_link.pop(id(info['actor']), None)

        self.inertia_visualizer.set_inertia_visibility(visible)
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
        parser = self._create_parser(self.current_urdf_file)
        
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

    def show_mdh_parameters(self):
        """Show MDH parameters in a dialog"""
        if not self.selected_chain:
            QMessageBox.warning(
                self, tr("warning"), tr("please_select_chain_mdh")
            )
            return

        if not self.current_urdf_file:
            QMessageBox.warning(
                self, tr("warning"), tr("please_load_urdf_show_mdh")
            )
            return

        # MDH 仅支持 URDF 文件
        if not self.current_urdf_file.lower().endswith('.urdf'):
            QMessageBox.warning(
                self, tr("warning"), tr("mdh_urdf_only")
            )
            return
        
        # Create a fresh parser instance to ensure we get the latest MDH parameters
        parser = self._create_parser(self.current_urdf_file)
        
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
            QMessageBox.warning(self, tr("warning"), tr("no_revolute_joints"))
            return

        dialog = QDialog(self)
        dialog.setWindowTitle(tr("set_joints_title"))
        vbox = QVBoxLayout(dialog)

        vbox.addWidget(QLabel(tr("enter_joint_angles")))
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
        units_row.addWidget(QLabel(tr("units")))
        units_combo = QComboBox()
        units_combo.addItems(["rad", "deg"])
        units_combo.setCurrentText("deg" if self.display_in_degrees else "rad")
        units_row.addWidget(units_combo)
        units_row.addStretch(1)
        vbox.addLayout(units_row)

        buttons_row = QHBoxLayout()
        btn_apply = QPushButton(tr("apply"))
        btn_cancel = QPushButton(tr("btn_cancel"))
        buttons_row.addStretch(1)
        buttons_row.addWidget(btn_cancel)
        buttons_row.addWidget(btn_apply)
        vbox.addLayout(buttons_row)

        def parse_and_apply():
            raw = angles_edit.text().strip()
            if not raw:
                QMessageBox.warning(dialog, tr("warning"), tr("please_input_angles"))
                return
            # Support comma or whitespace separated values
            tokens = raw.replace(",", " ").split()
            try:
                vals = [float(t) for t in tokens]
            except ValueError:
                QMessageBox.critical(dialog, tr("error"), tr("invalid_number"))
                return
            n = len(self.revolute_joints)
            if len(vals) != n:
                QMessageBox.warning(dialog, tr("warning"), tr("expected_values", n, len(vals)))
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

    def _get_link_name_by_actor(self, actor):
        """根据 VTK actor 获取对应的 link 名称"""
        return self.actor_to_link.get(id(actor))

    def _get_joint_for_child_link(self, link_name):
        """根据 link 名称找到以该 link 为 child 的关节

        Returns:
            (joint_index, joint_info) 或 (None, None)
        """
        for i, joint in enumerate(self.revolute_joints):
            if joint['child'] == link_name:
                return i, joint
        return None, None

    def _on_joint_drag(self, joint_index, delta_angle):
        """拖拽产生的关节角度变化回调"""
        if joint_index < 0 or joint_index >= len(self.joint_values):
            return

        # 获取关节限位
        joint_info = self.revolute_joints[joint_index]
        lower = joint_info['lower']
        upper = joint_info['upper']

        # 更新角度
        new_angle = self.joint_values[joint_index] + delta_angle

        # 限制在关节限位范围内
        new_angle = max(lower, min(upper, new_angle))

        # 更新值
        self.joint_values[joint_index] = new_angle

        # 同步 slider（阻止信号避免重复更新）
        if joint_index < len(self.joint_sliders):
            slider = self.joint_sliders[joint_index]
            slider_val = int(round(new_angle * 100.0))
            was_blocked = slider.blockSignals(True)
            slider.setValue(slider_val)
            slider.blockSignals(was_blocked)

        # 更新标签
        if joint_index < len(self.joint_value_labels):
            self.joint_value_labels[joint_index].setText(self.format_angle(new_angle))

        # 更新模型
        self.update_model_with_joint_angles()

    def _on_drag_start(self, link_name):
        """拖拽开始回调 - 高亮显示被拖拽的 link"""
        for model in self.models:
            if model.name == link_name:
                model.highlight()
                break
        self.vtk_widget.GetRenderWindow().Render()

    def _on_drag_end(self, link_name):
        """拖拽结束回调 - 取消高亮显示"""
        for model in self.models:
            model.unhighlight()
        self.vtk_widget.GetRenderWindow().Render()

    def show_topology_graph(self):
        """Show the robot topology graph dialog"""
        if not self.current_urdf_file:
            QMessageBox.warning(
                self, tr("warning"), tr("load_urdf_topology")
            )
            return

        parser = self._create_parser(self.current_urdf_file)
        dialog = TopologyDialog(self, parser, self.translation_manager)
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
        """Create compact sliders for controlling joint angles"""
        # Clear existing sliders
        self.clear_joint_sliders()

        # Initialize joint values array with zeros
        self.joint_values = [0.0] * len(self.revolute_joints)
        self.joint_value_labels = []

        # Create a slider for each revolute joint
        for i, joint in enumerate(self.revolute_joints):
            # Get joint limits
            lower = joint.get('lower', -math.pi)
            upper = joint.get('upper', math.pi)

            # Container widget for this joint (compact layout)
            joint_widget = QWidget()
            joint_widget.setObjectName("compactJoint")
            joint_vbox = QVBoxLayout(joint_widget)
            joint_vbox.setContentsMargins(4, 4, 4, 2)
            joint_vbox.setSpacing(2)

            # Row 1: joint name (bold) + current value (right-aligned)
            header_row = QHBoxLayout()
            name_label = QLabel(joint['name'])
            name_label.setStyleSheet("font-weight: bold;")
            header_row.addWidget(name_label)
            header_row.addStretch()

            value_label = QLabel(self.format_angle(0.0))
            value_label.setMinimumWidth(60)
            value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            header_row.addWidget(value_label)
            joint_vbox.addLayout(header_row)

            # Row 2: slider
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(round(lower * 100)))
            slider.setMaximum(int(round(upper * 100)))
            slider.setValue(0)

            range_val = upper - lower
            if range_val > math.pi:
                tick_interval = int(round(math.pi / 2 * 100))
            elif range_val > math.pi / 2:
                tick_interval = int(round(math.pi / 4 * 100))
            else:
                tick_interval = max(1, int(round(range_val / 4 * 100)))
            slider.setTickInterval(tick_interval)

            slider.valueChanged.connect(lambda val, idx=i, label=value_label: self.update_joint_angle(val, idx, label))
            joint_vbox.addWidget(slider)

            # Row 3: range info (small, secondary color)
            range_text = tr("range_label", None, f"{lower:.2f}", f"{upper:.2f}")
            range_label = QLabel(range_text)
            range_label.setStyleSheet("font-size: 10px; color: #A0A0AA;")
            joint_vbox.addWidget(range_label)

            # Separator line
            sep = QFrame()
            sep.setFrameShape(QFrame.HLine)
            sep.setFrameShadow(QFrame.Sunken)

            self.joint_layout.addWidget(joint_widget)
            self.joint_layout.addWidget(sep)

            # Store references
            self.joint_sliders.append(slider)
            self.joint_value_labels.append(value_label)
    
    def clear_joint_sliders(self):
        """Clear all joint sliders"""
        # Clear the joint values
        self.joint_values = []

        # Clear the sliders list
        self.joint_sliders = []
        self.joint_value_labels = []

        # Remove all widgets from the joint layout (except joint_label)
        if hasattr(self, 'joint_layout'):
            while self.joint_layout.count() > 1:  # Keep the first item (joint_label)
                item = self.joint_layout.takeAt(1)  # Start from index 1 to skip joint_label
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

        # Use cached parser's lightweight update_transforms() for MJCF
        if self.current_parser and HAS_MJCF and isinstance(self.current_parser, MJCFParser):
            result = self.current_parser.update_transforms(self.joint_values)
            link_names = result['link_names']
            link_mesh_transformations = result['link_mesh_transformations']
            link_frames = result['link_frames']
            collision_mesh_transformations = result['collision_mesh_transformations']
        else:
            # For URDF, recreate parser (lightweight) and get full info
            parser = self._create_parser(self.current_urdf_file)
            (link_names,
            _link_mesh_files,
            link_mesh_transformations,
            link_frames,
            _link_colors,
            _joint_names,
            _joint_frames,
            _joint_types,
            _joint_axes,
            _joint_parent_links,
            _joint_child_links,
            collision_mesh_files,
            collision_mesh_transformations,
            _joint_limits,
            _collision_link_names,
            _collision_geometries,
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
                    
        # Update existing models with new transformations
        for i, model in enumerate(self.models_collision):
            # Update mesh transformation
            model.apply_transform(collision_mesh_transformations[i])
            
            # Update axes and text actors
            if model.axes_actor:
                vtk_transform = vtk.vtkTransform()
                vtk_transform.SetMatrix(collision_mesh_transformations[i].flatten())
                model.axes_actor.SetUserTransform(vtk_transform)
            
            if model.text_actor:
                # Update text position based on new frame
                z_endpoint = [0, 0, 0.05, 1]  # Same axis_length as in create_axes_actor
                vtk_transform = vtk.vtkTransform()
                vtk_transform.SetMatrix(collision_mesh_transformations[i].flatten())
                transformed_point = vtk_transform.TransformPoint(z_endpoint[0], z_endpoint[1], z_endpoint[2])
                model.text_actor.SetAttachmentPoint(transformed_point[0], transformed_point[1], transformed_point[2])
        
        # Update MDH frames if they are visible
        if hasattr(self, 'cb_mdh_frames') and self.cb_mdh_frames.isChecked() and self.selected_chain:
            self.create_mdh_frames(self.selected_chain)

        # Update joint axes if they are visible
        if hasattr(self, 'cb_joint_axes') and self.cb_joint_axes.isChecked():
            self.create_joint_axes()

        # Update inertia visualizations if visible
        if self.inertia_visualizer and (self.cb_com.isChecked() or self.cb_inertia.isChecked()):
            self.inertia_visualizer.update_transforms(link_names, link_frames)

        # Update the rendering
        self.vtk_widget.GetRenderWindow().Render()
    
    def edit_urdf_file(self, replace_collision=False):
        """Open the current URDF file in the XML editor"""
        if not self.current_urdf_file:
            QMessageBox.warning(
                self, tr("warning"), tr("please_load_urdf_edit")
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
            parser = self._create_parser(temp_path)
            
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
            collision_mesh_files,
            collision_mesh_transformations,
            joint_limits,
            collision_link_names,
            collision_geometries,) = parser.get_robot_info()

            # Store revolute joints for slider controls
            self.revolute_joints = []
            for i, joint_type in enumerate(joint_types):
                if joint_type == 'revolute':
                    limit = joint_limits[i]
                    self.revolute_joints.append({
                        'name': joint_names[i],
                        'index': i,
                        'parent': joint_parent_links[i],
                        'child': joint_child_links[i],
                        'axis': joint_axes[i],
                        'lower': limit['lower'],
                        'upper': limit['upper'],
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
                
            # Create models for each collision
            for i in range(len(collision_geometries)):
                coll_link = collision_link_names[i] if i < len(collision_link_names) else None
                geom = collision_geometries[i]
                if geom['type'] == 'mesh':
                    self.add_urdf_model(
                        f"",
                        collision_mesh_files[i],
                        collision_mesh_transformations[i],
                        None,
                        None,
                        model_type='collision',
                        link_name=coll_link,
                    )
                else:
                    self._add_collision_primitive_model(
                        geom, collision_mesh_transformations[i], coll_link,
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

            # Apply visibility settings from checkboxes to new models
            self._apply_visibility_settings()

            # Update current file label
            self.update_current_file_label()
            
        except Exception as e:
            QMessageBox.critical(
                self, tr("error"), tr("update_model_failed", str(e))
            )

    def update_current_file_label(self):
        """Update the current file label with the current URDF file path"""
        if self.current_urdf_file:
            filename = os.path.basename(self.current_urdf_file)
            self.current_file_label.setText(tr("current_file") + " " + filename)
            # Update status bar
            n_joints = len(self.revolute_joints)
            n_links = len(self.models)
            self.status_label.setText(
                tr("status_file_info", None, filename, n_joints, n_links)
            )
            self.status_bar.showMessage(tr("model_loaded", None, filename), 3000)
        else:
            self.current_file_label.setText(tr("current_file") + " " + tr("current_file_none"))
            self.status_label.setText(tr("ready"))

    def _apply_visibility_settings(self):
        """Apply current checkbox visibility settings to all loaded models.

        Called after loading a new model or updating from XML to ensure
        the displayed state matches the checkbox state.
        """
        # Visual models
        visual_visible = self.cb_visual.isChecked()
        for model in self.models:
            model.actor.SetVisibility(visual_visible)

        # Collision models
        collision_visible = self.cb_collision.isChecked()
        for model in self.models_collision:
            model.actor.SetVisibility(collision_visible)
            if model.axes_actor is not None:
                model.axes_actor.SetVisibility(collision_visible)
            if hasattr(model, 'text_actor') and model.text_actor is not None:
                model.text_actor.SetVisibility(collision_visible)

        # Link frames (axes + text on visual models)
        frames_visible = self.cb_link_frames.isChecked()
        for model in self.models:
            if model.axes_actor is not None:
                model.axes_actor.SetVisibility(frames_visible)
            if hasattr(model, 'text_actor') and model.text_actor is not None:
                model.text_actor.SetVisibility(frames_visible)

        # Joint axes - recreate if checked
        if self.cb_joint_axes.isChecked() and self.current_urdf_file:
            self.create_joint_axes()

        # CoM markers - recreate if checked
        if self.cb_com.isChecked() and self.current_urdf_file:
            parser = self._create_parser(self.current_urdf_file)
            (link_names, _, _, link_frames, _, _, _, _, _, _, _, _, _, _, _, _) = parser.get_robot_info(qs=self.joint_values)
            self.inertia_visualizer.create_com_markers(parser, link_names, link_frames)

        # Inertia boxes - recreate if checked
        if self.cb_inertia.isChecked() and self.current_urdf_file:
            parser = self._create_parser(self.current_urdf_file)
            (link_names, _, _, link_frames, _, _, _, _, _, _, _, _, _, _, _, _) = parser.get_robot_info(qs=self.joint_values)
            self.inertia_visualizer.create_inertia_boxes(parser, link_names, link_frames)

        # Apply current transparency setting to newly loaded models
        self.apply_transparency()

        self.vtk_widget.GetRenderWindow().Render()
            
    def decompose_collision_meshes(self):
        """Handle decomposition of collision meshes"""
        if not self.collision_mesh_files:
            QMessageBox.warning(
                self, tr("warning"), tr("no_collision_meshes")
            )
            return
        
        # Create and show the decomposition dialog
        dialog = DecompDialog(self, self.collision_mesh_files)
        
        decomposed_mesh_files = dialog.exec_()

        if decomposed_mesh_files is not None:
            self.edit_urdf_file(replace_collision=True)

    def change_language(self, index):
        """Handle language change from the language combo box"""
        lang_code = self.language_combo.itemData(index)
        if lang_code:
            self.translation_manager.set_language(lang_code, self)

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
    app_icon_path = os.path.join(os.path.dirname(__file__), "icons", "urdfly-logo.svg")
    app.setWindowIcon(QIcon(app_icon_path))
    ThemeManager().apply(app)
    viewer = URDFViewer()
    viewer.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
