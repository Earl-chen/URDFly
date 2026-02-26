# -*- coding: utf-8 -*-
"""
ThemeManager - 深色/浅色主题管理

提供 QSS 样式表生成、VTK 背景色和主题切换功能。
"""


DARK_PALETTE = {
    # 背景层级
    "BG_BASE": "#1B1B1F",
    "BG_SURFACE": "#252529",
    "BG_ELEVATED": "#2D2D33",
    "BG_HOVER": "#35353D",
    "BG_PRESSED": "#3D3D47",
    # 文字
    "TEXT_PRIMARY": "#E0E0E6",
    "TEXT_SECONDARY": "#A0A0AA",
    "TEXT_DISABLED": "#606068",
    # 强调
    "ACCENT": "#4A9EFF",
    "ACCENT_HOVER": "#6BB0FF",
    "ACCENT_PRESSED": "#3588E0",
    "ACCENT_MUTED": "#2A4A6E",
    # 状态色
    "STATUS_SUCCESS": "#4CAF50",
    "STATUS_WARNING": "#FFB74D",
    "STATUS_ERROR": "#EF5350",
    # 边框
    "BORDER_SUBTLE": "#3A3A42",
    "BORDER_STRONG": "#505058",
    # VTK 渐变背景 (RGB 0-1)
    "VTK_BG_TOP": (0.18, 0.20, 0.25),
    "VTK_BG_BOTTOM": (0.12, 0.13, 0.16),
}

LIGHT_PALETTE = {
    "BG_BASE": "#F5F5F7",
    "BG_SURFACE": "#FFFFFF",
    "BG_ELEVATED": "#E8E8EC",
    "BG_HOVER": "#DDDDE3",
    "BG_PRESSED": "#D0D0D8",
    "TEXT_PRIMARY": "#1A1A1F",
    "TEXT_SECONDARY": "#6B6B75",
    "TEXT_DISABLED": "#A0A0AA",
    "ACCENT": "#2979FF",
    "ACCENT_HOVER": "#448AFF",
    "ACCENT_PRESSED": "#1565C0",
    "ACCENT_MUTED": "#BBDEFB",
    "STATUS_SUCCESS": "#43A047",
    "STATUS_WARNING": "#FB8C00",
    "STATUS_ERROR": "#E53935",
    "BORDER_SUBTLE": "#D8D8E0",
    "BORDER_STRONG": "#B0B0BA",
    "VTK_BG_TOP": (0.92, 0.93, 0.96),
    "VTK_BG_BOTTOM": (0.82, 0.84, 0.88),
}


def _generate_qss(p):
    """从色板字典 *p* 生成完整 QSS 字符串。"""
    return f"""
/* ===== 全局 ===== */
QWidget {{
    background-color: {p['BG_BASE']};
    color: {p['TEXT_PRIMARY']};
    font-size: 13px;
}}

QMainWindow {{
    background-color: {p['BG_BASE']};
}}

/* ===== QMenuBar ===== */
QMenuBar {{
    background-color: {p['BG_SURFACE']};
    color: {p['TEXT_PRIMARY']};
    border-bottom: 1px solid {p['BORDER_SUBTLE']};
    padding: 2px 0;
}}
QMenuBar::item {{
    padding: 4px 10px;
    border-radius: 4px;
}}
QMenuBar::item:selected {{
    background-color: {p['BG_HOVER']};
}}
QMenuBar::item:pressed {{
    background-color: {p['BG_PRESSED']};
}}

/* ===== QMenu ===== */
QMenu {{
    background-color: {p['BG_SURFACE']};
    color: {p['TEXT_PRIMARY']};
    border: 1px solid {p['BORDER_SUBTLE']};
    border-radius: 6px;
    padding: 4px 0;
}}
QMenu::item {{
    padding: 6px 28px 6px 20px;
}}
QMenu::item:selected {{
    background-color: {p['ACCENT_MUTED']};
    color: {p['TEXT_PRIMARY']};
}}
QMenu::item:disabled {{
    color: {p['TEXT_DISABLED']};
}}
QMenu::separator {{
    height: 1px;
    background-color: {p['BORDER_SUBTLE']};
    margin: 4px 8px;
}}

/* ===== QToolBar ===== */
QToolBar {{
    background-color: {p['BG_SURFACE']};
    border-bottom: 1px solid {p['BORDER_SUBTLE']};
    spacing: 4px;
    padding: 3px 6px;
}}
QToolBar::separator {{
    width: 1px;
    background-color: {p['BORDER_SUBTLE']};
    margin: 4px 6px;
}}
QToolBar QToolButton {{
    background-color: transparent;
    color: {p['TEXT_PRIMARY']};
    border: none;
    border-radius: 4px;
    padding: 4px 10px;
    font-size: 13px;
}}
QToolBar QToolButton:hover {{
    background-color: {p['BG_HOVER']};
}}
QToolBar QToolButton:pressed {{
    background-color: {p['BG_PRESSED']};
}}

/* ===== QStatusBar ===== */
QStatusBar {{
    background-color: {p['BG_SURFACE']};
    color: {p['TEXT_SECONDARY']};
    border-top: 1px solid {p['BORDER_SUBTLE']};
    font-size: 12px;
    padding: 2px 8px;
}}
QStatusBar QLabel {{
    color: {p['TEXT_SECONDARY']};
    padding: 0 6px;
    background-color: transparent;
}}

/* ===== QPushButton ===== */
QPushButton {{
    background-color: {p['BG_ELEVATED']};
    color: {p['TEXT_PRIMARY']};
    border: 1px solid {p['BORDER_SUBTLE']};
    border-radius: 5px;
    padding: 5px 14px;
    min-height: 22px;
}}
QPushButton:hover {{
    background-color: {p['BG_HOVER']};
    border-color: {p['BORDER_STRONG']};
}}
QPushButton:pressed {{
    background-color: {p['BG_PRESSED']};
}}
QPushButton:disabled {{
    color: {p['TEXT_DISABLED']};
    background-color: {p['BG_BASE']};
    border-color: {p['BORDER_SUBTLE']};
}}

/* ===== QComboBox ===== */
QComboBox {{
    background-color: {p['BG_ELEVATED']};
    color: {p['TEXT_PRIMARY']};
    border: 1px solid {p['BORDER_SUBTLE']};
    border-radius: 5px;
    padding: 4px 8px;
    min-height: 22px;
}}
QComboBox:hover {{
    border-color: {p['BORDER_STRONG']};
}}
QComboBox::drop-down {{
    border: none;
    width: 20px;
}}
QComboBox::down-arrow {{
    image: none;
    border-left: 4px solid transparent;
    border-right: 4px solid transparent;
    border-top: 5px solid {p['TEXT_SECONDARY']};
    margin-right: 6px;
}}
QComboBox QAbstractItemView {{
    background-color: {p['BG_SURFACE']};
    color: {p['TEXT_PRIMARY']};
    border: 1px solid {p['BORDER_SUBTLE']};
    selection-background-color: {p['ACCENT_MUTED']};
    selection-color: {p['TEXT_PRIMARY']};
    outline: none;
}}

/* ===== QListWidget ===== */
QListWidget {{
    background-color: {p['BG_SURFACE']};
    color: {p['TEXT_PRIMARY']};
    border: 1px solid {p['BORDER_SUBTLE']};
    border-radius: 5px;
    outline: none;
}}
QListWidget::item {{
    padding: 4px 8px;
    border-radius: 3px;
}}
QListWidget::item:hover {{
    background-color: {p['BG_HOVER']};
}}
QListWidget::item:selected {{
    background-color: {p['ACCENT_MUTED']};
    color: {p['TEXT_PRIMARY']};
}}

/* ===== QCheckBox ===== */
QCheckBox {{
    color: {p['TEXT_PRIMARY']};
    spacing: 6px;
    padding: 2px 0;
    background-color: transparent;
}}
QCheckBox::indicator {{
    width: 16px;
    height: 16px;
    border: 1px solid {p['BORDER_STRONG']};
    border-radius: 3px;
    background-color: {p['BG_ELEVATED']};
}}
QCheckBox::indicator:hover {{
    border-color: {p['ACCENT']};
}}
QCheckBox::indicator:checked {{
    background-color: {p['ACCENT']};
    border-color: {p['ACCENT']};
}}

/* ===== QSlider ===== */
QSlider::groove:horizontal {{
    border: none;
    height: 4px;
    background-color: {p['BORDER_STRONG']};
    border-radius: 2px;
}}
QSlider::handle:horizontal {{
    background-color: {p['ACCENT']};
    border: none;
    width: 14px;
    height: 14px;
    margin: -5px 0;
    border-radius: 7px;
}}
QSlider::handle:horizontal:hover {{
    background-color: {p['ACCENT_HOVER']};
}}
QSlider::handle:horizontal:pressed {{
    background-color: {p['ACCENT_PRESSED']};
}}
QSlider::sub-page:horizontal {{
    background-color: {p['ACCENT']};
    border-radius: 2px;
}}

/* ===== QGroupBox ===== */
QGroupBox {{
    background-color: {p['BG_SURFACE']};
    border: 1px solid {p['BORDER_SUBTLE']};
    border-radius: 6px;
    margin-top: 8px;
    padding: 12px 8px 8px 8px;
    font-weight: bold;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 2px 8px;
    color: {p['TEXT_SECONDARY']};
}}

/* ===== QScrollArea ===== */
QScrollArea {{
    border: none;
    background-color: transparent;
}}
QScrollArea > QWidget > QWidget {{
    background-color: transparent;
}}

/* ===== QScrollBar 垂直 ===== */
QScrollBar:vertical {{
    background-color: transparent;
    width: 10px;
    margin: 0;
}}
QScrollBar::handle:vertical {{
    background-color: {p['BORDER_STRONG']};
    min-height: 30px;
    border-radius: 5px;
    margin: 2px;
}}
QScrollBar::handle:vertical:hover {{
    background-color: {p['TEXT_DISABLED']};
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0;
}}
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical {{
    background: none;
}}

/* ===== QScrollBar 水平 ===== */
QScrollBar:horizontal {{
    background-color: transparent;
    height: 10px;
    margin: 0;
}}
QScrollBar::handle:horizontal {{
    background-color: {p['BORDER_STRONG']};
    min-width: 30px;
    border-radius: 5px;
    margin: 2px;
}}
QScrollBar::handle:horizontal:hover {{
    background-color: {p['TEXT_DISABLED']};
}}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
    width: 0;
}}
QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal {{
    background: none;
}}

/* ===== QTableWidget ===== */
QTableWidget {{
    background-color: {p['BG_SURFACE']};
    color: {p['TEXT_PRIMARY']};
    border: 1px solid {p['BORDER_SUBTLE']};
    border-radius: 5px;
    gridline-color: {p['BORDER_SUBTLE']};
    outline: none;
}}
QTableWidget::item {{
    padding: 4px 6px;
}}
QTableWidget::item:selected {{
    background-color: {p['ACCENT_MUTED']};
    color: {p['TEXT_PRIMARY']};
}}
QHeaderView::section {{
    background-color: {p['BG_ELEVATED']};
    color: {p['TEXT_SECONDARY']};
    border: none;
    border-bottom: 1px solid {p['BORDER_SUBTLE']};
    border-right: 1px solid {p['BORDER_SUBTLE']};
    padding: 5px 8px;
    font-weight: bold;
}}

/* ===== QTextEdit / QLineEdit ===== */
QTextEdit {{
    background-color: {p['BG_SURFACE']};
    color: {p['TEXT_PRIMARY']};
    border: 1px solid {p['BORDER_SUBTLE']};
    border-radius: 5px;
    selection-background-color: {p['ACCENT_MUTED']};
    selection-color: {p['TEXT_PRIMARY']};
}}
QLineEdit {{
    background-color: {p['BG_ELEVATED']};
    color: {p['TEXT_PRIMARY']};
    border: 1px solid {p['BORDER_SUBTLE']};
    border-radius: 5px;
    padding: 4px 8px;
    min-height: 22px;
    selection-background-color: {p['ACCENT_MUTED']};
    selection-color: {p['TEXT_PRIMARY']};
}}
QLineEdit:focus {{
    border-color: {p['ACCENT']};
}}

/* ===== QSpinBox ===== */
QSpinBox {{
    background-color: {p['BG_ELEVATED']};
    color: {p['TEXT_PRIMARY']};
    border: 1px solid {p['BORDER_SUBTLE']};
    border-radius: 5px;
    padding: 3px 6px;
    min-height: 22px;
}}
QSpinBox::up-button, QSpinBox::down-button {{
    background-color: {p['BG_HOVER']};
    border: none;
    width: 16px;
}}
QSpinBox::up-button:hover, QSpinBox::down-button:hover {{
    background-color: {p['BG_PRESSED']};
}}

/* ===== QToolTip ===== */
QToolTip {{
    background-color: {p['BG_ELEVATED']};
    color: {p['TEXT_PRIMARY']};
    border: 1px solid {p['BORDER_STRONG']};
    border-radius: 4px;
    padding: 4px 8px;
}}

/* ===== QSplitter ===== */
QSplitter::handle {{
    background-color: {p['BORDER_SUBTLE']};
}}
QSplitter::handle:horizontal {{
    width: 3px;
}}
QSplitter::handle:vertical {{
    height: 3px;
}}
QSplitter::handle:hover {{
    background-color: {p['ACCENT']};
}}

/* ===== QDialog ===== */
QDialog {{
    background-color: {p['BG_BASE']};
    color: {p['TEXT_PRIMARY']};
}}

/* ===== QLabel ===== */
QLabel {{
    background-color: transparent;
    color: {p['TEXT_PRIMARY']};
}}

/* ===== QMessageBox ===== */
QMessageBox {{
    background-color: {p['BG_BASE']};
}}
QMessageBox QLabel {{
    color: {p['TEXT_PRIMARY']};
}}

/* ===== QFrame HLine / VLine ===== */
QFrame[frameShape="4"] {{
    color: {p['BORDER_SUBTLE']};
    max-height: 1px;
}}
QFrame[frameShape="5"] {{
    color: {p['BORDER_SUBTLE']};
    max-width: 1px;
}}

/* ===== CollapsibleSection ===== */
CollapsibleSection > QPushButton#collapseToggle {{
    background-color: {p['BG_ELEVATED']};
    color: {p['TEXT_PRIMARY']};
    border: 1px solid {p['BORDER_SUBTLE']};
    border-radius: 4px;
    padding: 6px 10px;
    text-align: left;
    font-weight: bold;
    font-size: 12px;
}}
CollapsibleSection > QPushButton#collapseToggle:hover {{
    background-color: {p['BG_HOVER']};
}}
"""


class ThemeManager:
    """主题管理器：深色/浅色主题切换，QSS 生成与 VTK 背景色。"""

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._current = "dark"
            cls._instance._palettes = {
                "dark": DARK_PALETTE,
                "light": LIGHT_PALETTE,
            }
        return cls._instance

    # --- 公开接口 ---

    @property
    def palette(self):
        return self._palettes[self._current]

    @property
    def current_theme(self):
        return self._current

    def get_vtk_background(self):
        """返回 (bottom_rgb, top_rgb)，每个 RGB 为 (r, g, b) 范围 0-1。"""
        p = self.palette
        return p["VTK_BG_BOTTOM"], p["VTK_BG_TOP"]

    def generate_qss(self):
        return _generate_qss(self.palette)

    def apply(self, app):
        """将当前主题应用到 QApplication。"""
        app.setStyleSheet(self.generate_qss())

    def set_theme(self, name, app=None):
        """切换主题。 *name* 为 'dark' 或 'light'。"""
        if name not in self._palettes:
            return
        self._current = name
        if app is not None:
            self.apply(app)

    def get_color(self, token):
        """按 token 名获取当前色板中的颜色值。"""
        return self.palette.get(token)
