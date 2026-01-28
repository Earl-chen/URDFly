#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
拓扑图对话框模块

将机器人的关节拓扑结构可视化显示，类似 urdf_to_graphviz 工具。
支持缩放、平移、点击交互和 PNG/SVG 导出。
"""

from collections import deque

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QGraphicsView, QGraphicsScene,
    QGraphicsRectItem, QGraphicsEllipseItem, QGraphicsLineItem,
    QGraphicsTextItem, QGraphicsItem, QPushButton, QToolBar,
    QFileDialog, QMessageBox
)
from PyQt5.QtCore import Qt, QRectF, QPointF
from PyQt5.QtGui import (
    QPen, QBrush, QColor, QFont, QPainter, QFontMetrics
)
from PyQt5.QtSvg import QSvgGenerator


# 节点颜色配置
LINK_FILL_COLOR = QColor("#E6F3FF")  # 浅蓝色
LINK_BORDER_COLOR = QColor("#336699")  # 深蓝边框
LINK_SELECTED_COLOR = QColor("#FFD700")  # 金色高亮

# 关节类型颜色映射
JOINT_COLORS = {
    "revolute": QColor("#4682B4"),   # 钢蓝色
    "continuous": QColor("#4682B4"), # 同revolute
    "prismatic": QColor("#3CB371"),  # 海绿色
    "fixed": QColor("#A9A9A9"),      # 灰色
    "floating": QColor("#FF8C00"),   # 橙色
    "ball": QColor("#BA55D3"),       # 紫色
    "free": QColor("#FF8C00"),       # 同floating
    "slide": QColor("#3CB371"),      # 同prismatic
    "hinge": QColor("#4682B4"),      # 同revolute (MuJoCo)
}
JOINT_SELECTED_COLOR = QColor("#FFD700")  # 金色高亮

# 连接线颜色
LINE_COLOR = QColor("#888888")

# 布局参数
LEVEL_SPACING = 100  # 层级间距（Y方向）
NODE_SPACING = 150   # 同层节点间距（X方向）
LINK_HEIGHT = 28
LINK_PADDING = 16    # 文字左右内边距
JOINT_RADIUS = 10    # 关节圆半径（小圆点）


class LinkNode(QGraphicsRectItem):
    """连杆节点 - 圆角矩形，宽度自适应文本"""

    def __init__(self, name, x, y):
        # 计算文本宽度来确定节点宽度
        font = QFont("Arial", 9)
        fm = QFontMetrics(font)
        text_width = fm.horizontalAdvance(name)
        width = text_width + LINK_PADDING * 2
        height = LINK_HEIGHT

        super().__init__(0, 0, width, height)
        self.name = name
        self.node_type = "link"
        self._width = width
        self._height = height

        # 设置位置（中心对齐）
        self.setPos(x - width / 2, y - height / 2)

        # 设置样式
        self.setPen(QPen(LINK_BORDER_COLOR, 2))
        self.setBrush(QBrush(LINK_FILL_COLOR))

        # 添加文本标签
        self.text_item = QGraphicsTextItem(name, self)
        self.text_item.setFont(font)
        self.text_item.setDefaultTextColor(QColor("#333333"))

        # 计算文本居中位置
        text_rect = self.text_item.boundingRect()
        text_x = (width - text_rect.width()) / 2
        text_y = (height - text_rect.height()) / 2
        self.text_item.setPos(text_x, text_y)

        # 允许选择
        self.setFlag(QGraphicsRectItem.ItemIsSelectable, True)
        self.setAcceptHoverEvents(True)

        # 保存原始颜色
        self._original_brush = QBrush(LINK_FILL_COLOR)
        self._selected = False

    def get_center(self):
        """获取节点中心坐标"""
        pos = self.pos()
        return QPointF(pos.x() + self._width / 2, pos.y() + self._height / 2)

    def get_top_center(self):
        """获取顶部中心点"""
        pos = self.pos()
        return QPointF(pos.x() + self._width / 2, pos.y())

    def get_bottom_center(self):
        """获取底部中心点"""
        pos = self.pos()
        return QPointF(pos.x() + self._width / 2, pos.y() + self._height)

    def set_selected(self, selected):
        """设置选中状态"""
        self._selected = selected
        if selected:
            self.setBrush(QBrush(LINK_SELECTED_COLOR))
        else:
            self.setBrush(self._original_brush)

    def hoverEnterEvent(self, event):
        """鼠标进入高亮"""
        if not self._selected:
            self.setBrush(QBrush(LINK_SELECTED_COLOR.lighter(150)))
        super().hoverEnterEvent(event)

    def hoverLeaveEvent(self, event):
        """鼠标离开恢复"""
        if not self._selected:
            self.setBrush(self._original_brush)
        super().hoverLeaveEvent(event)


class JointNode(QGraphicsItem):
    """关节节点 - 小圆点 + 右侧文字标签"""

    def __init__(self, name, joint_type, x, y, radius=JOINT_RADIUS):
        super().__init__()
        self.name = name
        self.node_type = "joint"
        self.joint_type = joint_type
        self.radius = radius

        # 获取关节类型对应的颜色
        self.color = JOINT_COLORS.get(joint_type.lower(), QColor("#808080"))

        # 文字标签
        self.font = QFont("Arial", 8)
        fm = QFontMetrics(self.font)
        self.text_width = fm.horizontalAdvance(name)
        self.text_height = fm.height()

        # 计算包围盒：圆 + 间隙 + 文字
        self.gap = 6  # 圆和文字之间的间隙
        self.total_width = radius * 2 + self.gap + self.text_width
        self.total_height = max(radius * 2, self.text_height)

        # 设置位置（以圆心为锚点）
        self.setPos(x, y)

        # 允许选择和悬停
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setAcceptHoverEvents(True)

        # 状态
        self._selected = False
        self._hovered = False

    def boundingRect(self):
        """返回边界矩形"""
        # 包含圆和文字的区域
        return QRectF(
            -self.radius - 2,
            -self.total_height / 2 - 2,
            self.total_width + 4,
            self.total_height + 4
        )

    def paint(self, painter, option, widget):
        """绘制节点"""
        painter.setRenderHint(QPainter.Antialiasing)

        # 确定颜色
        if self._selected:
            fill_color = JOINT_SELECTED_COLOR
        elif self._hovered:
            fill_color = JOINT_SELECTED_COLOR.lighter(150)
        else:
            fill_color = self.color

        # 绘制圆
        painter.setPen(QPen(fill_color.darker(120), 2))
        painter.setBrush(QBrush(fill_color))
        painter.drawEllipse(
            QPointF(0, 0),
            self.radius, self.radius
        )

        # 绘制文字（圆的右侧）
        painter.setFont(self.font)
        painter.setPen(QPen(QColor("#555555")))
        text_x = self.radius + self.gap
        text_y = self.text_height / 4  # 垂直居中微调
        painter.drawText(QPointF(text_x, text_y), self.name)

    def get_center(self):
        """获取圆心坐标"""
        return self.pos()

    def get_top_center(self):
        """获取顶部中心点"""
        pos = self.pos()
        return QPointF(pos.x(), pos.y() - self.radius)

    def get_bottom_center(self):
        """获取底部中心点"""
        pos = self.pos()
        return QPointF(pos.x(), pos.y() + self.radius)

    def set_selected(self, selected):
        """设置选中状态"""
        self._selected = selected
        self.update()

    def hoverEnterEvent(self, event):
        """鼠标进入"""
        self._hovered = True
        self.update()
        super().hoverEnterEvent(event)

    def hoverLeaveEvent(self, event):
        """鼠标离开"""
        self._hovered = False
        self.update()
        super().hoverLeaveEvent(event)


class TopologyGraphView(QGraphicsView):
    """支持缩放和平移的交互视图"""

    def __init__(self, scene, parent=None):
        super().__init__(scene, parent)

        # 设置渲染质量
        self.setRenderHint(QPainter.Antialiasing)
        self.setRenderHint(QPainter.TextAntialiasing)
        self.setRenderHint(QPainter.SmoothPixmapTransform)

        # 设置拖拽模式
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)

        # 设置视口更新模式
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)

        # 缩放因子
        self._zoom_factor = 1.15
        self._min_zoom = 0.1
        self._max_zoom = 10.0

    def wheelEvent(self, event):
        """滚轮缩放"""
        if event.angleDelta().y() > 0:
            factor = self._zoom_factor
        else:
            factor = 1 / self._zoom_factor

        # 检查缩放限制
        current_scale = self.transform().m11()
        new_scale = current_scale * factor

        if self._min_zoom <= new_scale <= self._max_zoom:
            self.scale(factor, factor)

    def fit_in_view(self):
        """适应窗口"""
        scene_rect = self.scene().itemsBoundingRect()
        if not scene_rect.isEmpty():
            margin = 50
            scene_rect.adjust(-margin, -margin, margin, margin)
            self.fitInView(scene_rect, Qt.KeepAspectRatio)


class TopologyDialog(QDialog):
    """拓扑图对话框"""

    def __init__(self, parent, parser, translation_manager):
        super().__init__(parent)
        self.parser = parser
        self.tr_mgr = translation_manager

        # 存储节点引用
        self.node_items = {}  # name -> GraphicsItem
        self.line_items = []

        self.init_ui()
        self.build_graph()

    def init_ui(self):
        """初始化 UI"""
        self.setWindowTitle(self.tr_mgr.tr("topology_title"))
        self.resize(900, 700)

        layout = QVBoxLayout(self)

        # 工具栏
        toolbar = QToolBar()

        self.btn_fit = QPushButton(self.tr_mgr.tr("fit_view"))
        self.btn_fit.clicked.connect(self.fit_view)
        toolbar.addWidget(self.btn_fit)

        toolbar.addSeparator()

        self.btn_export_png = QPushButton(self.tr_mgr.tr("export_png"))
        self.btn_export_png.clicked.connect(self.export_png)
        toolbar.addWidget(self.btn_export_png)

        self.btn_export_svg = QPushButton(self.tr_mgr.tr("export_svg"))
        self.btn_export_svg.clicked.connect(self.export_svg)
        toolbar.addWidget(self.btn_export_svg)

        toolbar.addSeparator()

        self.btn_close = QPushButton(self.tr_mgr.tr("btn_close"))
        self.btn_close.clicked.connect(self.close)
        toolbar.addWidget(self.btn_close)

        layout.addWidget(toolbar)

        # 图形场景和视图
        self.scene = QGraphicsScene()
        self.view = TopologyGraphView(self.scene, self)
        layout.addWidget(self.view)

        # 设置背景色
        self.view.setBackgroundBrush(QBrush(QColor("#F8F8F8")))

    def build_graph(self):
        """从 parser 构建拓扑图"""
        if not hasattr(self.parser, 'joints') or not self.parser.joints:
            return

        # 构建父子关系图
        parent_map = {}  # child -> (parent, joint_name, joint_type)
        children_map = {}  # parent -> [(child, joint_name, joint_type), ...]

        for joint in self.parser.joints:
            parent = joint['parent']
            child = joint['child']
            joint_name = joint['name']
            joint_type = joint['type']

            parent_map[child] = (parent, joint_name, joint_type)

            if parent not in children_map:
                children_map[parent] = []
            children_map[parent].append((child, joint_name, joint_type))

        # 找到根节点
        all_links = set()
        child_links = set()

        for joint in self.parser.joints:
            all_links.add(joint['parent'])
            all_links.add(joint['child'])
            child_links.add(joint['child'])

        root_links = all_links - child_links

        if not root_links:
            return

        # BFS 计算层级
        level_nodes = {}  # level -> [node_name, ...]
        node_levels = {}  # node_name -> level

        queue = deque()
        for root in root_links:
            queue.append((root, 0))

        while queue:
            node, level = queue.popleft()

            if node in node_levels:
                continue

            node_levels[node] = level

            if level not in level_nodes:
                level_nodes[level] = []
            level_nodes[level].append(node)

            if node in children_map:
                for child, _, _ in children_map[node]:
                    if child not in node_levels:
                        queue.append((child, level + 1))

        # 计算每层宽度，用于动态调整 NODE_SPACING
        max_level = max(level_nodes.keys()) if level_nodes else 0

        # 计算每个节点的位置
        positions = {}

        for level in range(max_level + 1):
            nodes = level_nodes.get(level, [])
            n = len(nodes)
            if n == 0:
                continue

            # 动态间距：根据节点数量调整
            spacing = max(NODE_SPACING, 120)
            total_width = (n - 1) * spacing

            for i, node in enumerate(nodes):
                x = i * spacing - total_width / 2
                y = level * LEVEL_SPACING
                positions[node] = (x, y)

        # 创建 Link 节点
        for name, (x, y) in positions.items():
            link_item = LinkNode(name, x, y)
            self.scene.addItem(link_item)
            self.node_items[name] = link_item

        # 创建 Joint 节点和连接线
        for child, (parent, joint_name, joint_type) in parent_map.items():
            if parent not in self.node_items or child not in self.node_items:
                continue

            parent_item = self.node_items[parent]
            child_item = self.node_items[child]

            # Joint 位于连线中点
            parent_bottom = parent_item.get_bottom_center()
            child_top = child_item.get_top_center()

            joint_x = (parent_bottom.x() + child_top.x()) / 2
            joint_y = (parent_bottom.y() + child_top.y()) / 2

            joint_item = JointNode(joint_name, joint_type, joint_x, joint_y)
            self.scene.addItem(joint_item)
            self.node_items[joint_name] = joint_item

            # 连接线
            pen = QPen(LINE_COLOR, 1.5)

            # 父节点 -> 关节
            line1 = QGraphicsLineItem(
                parent_bottom.x(), parent_bottom.y(),
                joint_item.get_top_center().x(), joint_item.get_top_center().y()
            )
            line1.setPen(pen)
            line1.setZValue(-1)
            self.scene.addItem(line1)
            self.line_items.append(line1)

            # 关节 -> 子节点
            line2 = QGraphicsLineItem(
                joint_item.get_bottom_center().x(), joint_item.get_bottom_center().y(),
                child_top.x(), child_top.y()
            )
            line2.setPen(pen)
            line2.setZValue(-1)
            self.scene.addItem(line2)
            self.line_items.append(line2)

        # 适应视图
        self.view.fit_in_view()

    def fit_view(self):
        """适应窗口"""
        self.view.fit_in_view()

    def export_png(self):
        """导出为 PNG"""
        filename, _ = QFileDialog.getSaveFileName(
            self, self.tr_mgr.tr("export_png"),
            "topology.png", "PNG Files (*.png)"
        )
        if not filename:
            return

        scene_rect = self.scene.itemsBoundingRect()
        margin = 30
        scene_rect.adjust(-margin, -margin, margin, margin)

        from PyQt5.QtGui import QImage

        width = int(scene_rect.width())
        height = int(scene_rect.height())

        if width <= 0 or height <= 0:
            QMessageBox.warning(
                self, self.tr_mgr.tr("warning"),
                self.tr_mgr.tr("export_empty_scene")
            )
            return

        image = QImage(width, height, QImage.Format_ARGB32)
        image.fill(Qt.white)

        painter = QPainter(image)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.TextAntialiasing)

        self.scene.render(painter, QRectF(0, 0, width, height), scene_rect)
        painter.end()

        if image.save(filename):
            QMessageBox.information(
                self, self.tr_mgr.tr("info"),
                self.tr_mgr.tr("export_success").format(filename)
            )
        else:
            QMessageBox.warning(
                self, self.tr_mgr.tr("warning"),
                self.tr_mgr.tr("export_failed")
            )

    def export_svg(self):
        """导出为 SVG"""
        filename, _ = QFileDialog.getSaveFileName(
            self, self.tr_mgr.tr("export_svg"),
            "topology.svg", "SVG Files (*.svg)"
        )
        if not filename:
            return

        scene_rect = self.scene.itemsBoundingRect()
        margin = 30
        scene_rect.adjust(-margin, -margin, margin, margin)

        width = int(scene_rect.width())
        height = int(scene_rect.height())

        if width <= 0 or height <= 0:
            QMessageBox.warning(
                self, self.tr_mgr.tr("warning"),
                self.tr_mgr.tr("export_empty_scene")
            )
            return

        generator = QSvgGenerator()
        generator.setFileName(filename)
        generator.setSize(scene_rect.size().toSize())
        generator.setViewBox(QRectF(0, 0, width, height))
        generator.setTitle("Robot Topology")
        generator.setDescription("Generated by URDFly")

        painter = QPainter(generator)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.TextAntialiasing)

        self.scene.render(painter, QRectF(0, 0, width, height), scene_rect)
        painter.end()

        QMessageBox.information(
            self, self.tr_mgr.tr("info"),
            self.tr_mgr.tr("export_success").format(filename)
        )
