# -*- coding: utf-8 -*-
"""
可复用自定义控件

CollapsibleSection - 可折叠面板（Blender 风格）
"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QPushButton, QSizePolicy
)
from PyQt5.QtCore import Qt


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
