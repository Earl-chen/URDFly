#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
拓扑图对话框模块

将机器人的关节拓扑结构可视化显示，类似 urdf_to_graphviz 工具。
支持缩放、平移、点击交互和 PNG/SVG 导出。
"""

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
from theme import ThemeManager


def _topo_color(token):
    """从 ThemeManager 获取拓扑图颜色"""
    tm = ThemeManager()
    return tm.get_color(token)


# 节点颜色配置 - 使用主题色
LINK_FILL_COLOR = QColor("#2A4A6E")       # ACCENT_MUTED
LINK_BORDER_COLOR = QColor("#4A9EFF")     # ACCENT
LINK_SELECTED_COLOR = QColor("#FFD700")   # 金色高亮

# 关节类型颜色映射
JOINT_COLORS = {
    "revolute": QColor("#4A9EFF"),   # 强调色
    "continuous": QColor("#4A9EFF"),
    "prismatic": QColor("#4CAF50"),  # 成功色
    "fixed": QColor("#606068"),      # 禁用文字色
    "floating": QColor("#FFB74D"),   # 警告色
    "ball": QColor("#BA55D3"),       # 紫色
    "free": QColor("#FFB74D"),
    "slide": QColor("#4CAF50"),
    "hinge": QColor("#4A9EFF"),
}
JOINT_SELECTED_COLOR = QColor("#FFD700")

# 连接线颜色
LINE_COLOR = QColor("#505058")  # BORDER_STRONG

# 布局参数
LEVEL_SPACING = 120  # 层级间距（Y方向）
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
        self.text_item.setDefaultTextColor(QColor("#E0E0E6"))

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
    """关节节点 - 小圆点 + 下方文字标签"""

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

        # 计算包围盒：圆在上，文字在下
        self.gap = 4  # 圆底部和文字之间的间隙
        self.total_width = max(radius * 2, self.text_width)
        self.total_height = radius * 2 + self.gap + self.text_height

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
        return QRectF(
            -self.total_width / 2 - 2,
            -self.radius - 2,
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

        # 绘制文字（圆的下方，水平居中）
        painter.setFont(self.font)
        painter.setPen(QPen(QColor("#A0A0AA")))
        text_x = -self.text_width / 2
        text_y = self.radius + self.gap + self.text_height * 0.8
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

        # 设置背景色 (深色主题)
        tm = ThemeManager()
        bg = tm.get_color("BG_BASE")
        self.view.setBackgroundBrush(QBrush(QColor(bg)))

    def build_graph(self):
        """从 parser 构建拓扑图（子树宽度优先布局）"""
        if not hasattr(self.parser, 'joints') or not self.parser.joints:
            return

        # 构建父子关系图
        children_map = {}  # parent -> [(child, joint_name, joint_type), ...]

        for joint in self.parser.joints:
            parent = joint['parent']
            child = joint['child']
            joint_name = joint['name']
            joint_type = joint['type']

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

        root_links = sorted(all_links - child_links)

        if not root_links:
            return

        # 递归计算每个子树所需宽度
        subtree_widths = {}

        def compute_subtree_width(link):
            if link not in children_map or not children_map[link]:
                subtree_widths[link] = NODE_SPACING
                return NODE_SPACING
            total = 0
            for child, _, _ in children_map[link]:
                total += compute_subtree_width(child)
            subtree_widths[link] = max(total, NODE_SPACING)
            return subtree_widths[link]

        total_root_width = 0
        for root in root_links:
            total_root_width += compute_subtree_width(root)

        # 自顶向下分配 x, y 坐标
        link_positions = {}  # link_name -> (x, y)

        def assign_positions(link, x_center, level):
            link_positions[link] = (x_center, level * LEVEL_SPACING)

            if link not in children_map:
                return

            children = children_map[link]
            total_width = subtree_widths[link]
            x_start = x_center - total_width / 2

            for child, _, _ in children:
                child_width = subtree_widths[child]
                child_x = x_start + child_width / 2
                assign_positions(child, child_x, level + 1)
                x_start += child_width

        # 为所有根节点分配位置
        x_offset = -total_root_width / 2
        for root in root_links:
            w = subtree_widths[root]
            assign_positions(root, x_offset + w / 2, 0)
            x_offset += w

        # 创建 Link 节点
        for name, (x, y) in link_positions.items():
            link_item = LinkNode(name, x, y)
            self.scene.addItem(link_item)
            self.node_items[name] = link_item

        # 创建 Joint 节点和 L 形连接线
        pen = QPen(LINE_COLOR, 1.5)

        for parent_link, children in children_map.items():
            if parent_link not in self.node_items:
                continue

            parent_item = self.node_items[parent_link]
            parent_bottom = parent_item.get_bottom_center()

            for child_link, joint_name, joint_type in children:
                if child_link not in self.node_items:
                    continue

                child_item = self.node_items[child_link]
                child_top = child_item.get_top_center()

                # Joint 位置：x 对齐 child，y 在 parent 和 child 中间
                joint_x = child_top.x()
                joint_y = (parent_bottom.y() + child_top.y()) / 2

                joint_item = JointNode(joint_name, joint_type, joint_x, joint_y)
                self.scene.addItem(joint_item)
                self.node_items[joint_name] = joint_item

                joint_top = joint_item.get_top_center()
                joint_bottom = joint_item.get_bottom_center()

                # L 形连线：parent → 垂直下 → 水平拐弯 → joint → 垂直下 → child
                # 第 1 段：parent 底部垂直向下到 joint 的 y 水平线
                line_v1 = QGraphicsLineItem(
                    parent_bottom.x(), parent_bottom.y(),
                    parent_bottom.x(), joint_top.y()
                )
                line_v1.setPen(pen)
                line_v1.setZValue(-1)
                self.scene.addItem(line_v1)
                self.line_items.append(line_v1)

                # 第 2 段：水平连到 joint（仅当 parent_x != child_x 时）
                if abs(parent_bottom.x() - joint_top.x()) > 1:
                    line_h = QGraphicsLineItem(
                        parent_bottom.x(), joint_top.y(),
                        joint_top.x(), joint_top.y()
                    )
                    line_h.setPen(pen)
                    line_h.setZValue(-1)
                    self.scene.addItem(line_h)
                    self.line_items.append(line_h)

                # 第 3 段：joint 底部到 child 顶部
                line_v2 = QGraphicsLineItem(
                    joint_bottom.x(), joint_bottom.y(),
                    child_top.x(), child_top.y()
                )
                line_v2.setPen(pen)
                line_v2.setZValue(-1)
                self.scene.addItem(line_v2)
                self.line_items.append(line_v2)

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
