# -*- coding: utf-8 -*-
"""
可复用自定义控件

CollapsibleSection - 可折叠面板（Blender 风格）
ColorSwatchButton  - 颜色色块按钮（带 alpha 通道拾色器）
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QPushButton, QSizePolicy, QColorDialog
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor


class CollapsibleSection(QWidget):
    """可折叠分组面板：点击标题栏展开/收起内容区域。"""

    def __init__(self, title="", parent=None, expanded=True):
        super().__init__(parent)
        self._expanded = expanded

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # 标题按钮
        self.toggle_btn = QPushButton(self._arrow() + "  " + title)
        self.toggle_btn.setObjectName("collapseToggle")
        self.toggle_btn.setCheckable(False)
        self.toggle_btn.clicked.connect(self._on_toggle)
        self.toggle_btn.setCursor(Qt.PointingHandCursor)
        layout.addWidget(self.toggle_btn)

        # 内容区域
        self.content = QWidget()
        self.content_layout = QVBoxLayout(self.content)
        self.content_layout.setContentsMargins(4, 4, 4, 4)
        self.content_layout.setSpacing(4)
        self.content.setVisible(self._expanded)
        layout.addWidget(self.content)

        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Maximum)

    def _arrow(self):
        return "\u25BC" if self._expanded else "\u25B6"

    def _on_toggle(self):
        self._expanded = not self._expanded
        self.content.setVisible(self._expanded)
        # 更新箭头 — 保留当前文字（去掉旧箭头前缀）
        text = self.toggle_btn.text()
        # 去除旧箭头（前两个字符 + 空格）
        label = text[2:].lstrip() if len(text) > 2 else text
        self.toggle_btn.setText(self._arrow() + "  " + label)

    def set_title(self, title):
        """更新标题文字（保留箭头）。"""
        self.toggle_btn.setText(self._arrow() + "  " + title)

    def add_widget(self, widget):
        """向内容区域添加控件。"""
        self.content_layout.addWidget(widget)

    def add_layout(self, layout):
        """向内容区域添加布局。"""
        self.content_layout.addLayout(layout)


class ColorSwatchButton(QPushButton):
    """颜色色块按钮：显示当前颜色，点击弹出带 alpha 的拾色器。

    信号:
        colorChanged(float, float, float, float) — r, g, b, a (0-1)
    """

    colorChanged = pyqtSignal(float, float, float, float)

    def __init__(self, r=1.0, g=1.0, b=1.0, a=1.0, parent=None):
        super().__init__(parent)
        self._r, self._g, self._b, self._a = r, g, b, a
        self._hovered = False
        self.setFixedSize(16, 16)
        self.setCursor(Qt.PointingHandCursor)
        self.clicked.connect(self._pick_color)
        self._update_style()

    # --- public helpers ---
    def set_color(self, r, g, b, a=None):
        """编程式设置颜色（不触发信号）。"""
        self._r, self._g, self._b = r, g, b
        if a is not None:
            self._a = a
        self._update_style()

    def get_color(self):
        """返回 (r, g, b, a) float 元组。"""
        return (self._r, self._g, self._b, self._a)

    # --- event overrides ---
    def enterEvent(self, event):
        self._hovered = True
        self._update_style()
        super().enterEvent(event)

    def leaveEvent(self, event):
        self._hovered = False
        self._update_style()
        super().leaveEvent(event)

    # --- internals ---
    def _update_style(self):
        r8 = int(self._r * 255)
        g8 = int(self._g * 255)
        b8 = int(self._b * 255)
        a8 = int(self._a * 255)
        border_alpha = "0.45" if self._hovered else "0.15"
        self.setStyleSheet(
            "QPushButton {"
            f"  min-width:0; max-width:16px;"
            f"  min-height:0; max-height:16px;"
            f"  padding:0; margin:0;"
            f"  border:1.5px solid rgba(255,255,255,{border_alpha});"
            f"  border-radius:8px;"
            f"  background-color: rgba({r8},{g8},{b8},{a8});"
            "}"
        )

    def _pick_color(self):
        initial = QColor.fromRgbF(self._r, self._g, self._b, self._a)
        color = QColorDialog.getColor(
            initial, self, "",
            QColorDialog.ShowAlphaChannel
        )
        if color.isValid():
            self._r = color.redF()
            self._g = color.greenF()
            self._b = color.blueF()
            self._a = color.alphaF()
            self._update_style()
            self.colorChanged.emit(self._r, self._g, self._b, self._a)
