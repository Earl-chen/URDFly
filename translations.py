# -*- coding: utf-8 -*-
"""
翻译模块 - 支持中英文切换
使用 Qt 的国际化机制实现语言切换功能
"""

from PyQt5.QtCore import QCoreApplication, QTranslator
import os

# 所有翻译字符串
# 格式: "翻译键": {"zh_CN": "中文", "en": "English"}
TRANSLATIONS = {
    # === 窗口标题 ===
    "window_title": {"zh_CN": "URDFly", "en": "URDFly"},

    # === 左侧面板 ===
    "robot_structure": {"zh_CN": "机器人结构：", "en": "Robot Structure:"},
    "select_chain": {"zh_CN": "选择运动链：", "en": "Select Chain:"},
    "links": {"zh_CN": "连杆：", "en": "Links:"},
    "open_urdf": {"zh_CN": "打开模型", "en": "Open Model"},
    "edit_urdf": {"zh_CN": "编辑模型", "en": "Edit Model"},
    "edit_urdf_tooltip": {"zh_CN": "在 XML 编辑器中打开当前模型文件 (URDF/MJCF)", "en": "Open the current model file (URDF/MJCF) in an XML editor"},
    "show_mdh": {"zh_CN": "显示 MDH 参数", "en": "Show MDH Parameters"},
    "decompose_collision": {"zh_CN": "凸分解碰撞体", "en": "Decompose As Collision"},
    "set_joints": {"zh_CN": "设置关节", "en": "Set Joints"},

    # === 透明度控制 ===
    "transparency": {"zh_CN": "透明度：", "en": "Transparency:"},

    # === 显示设置 ===
    "visibility_settings": {"zh_CN": "显示设置", "en": "Visibility"},
    "show_visual": {"zh_CN": "显示视觉", "en": "Show Visual"},
    "show_link_frames": {"zh_CN": "显示连杆坐标系", "en": "Show Link Frames"},
    "show_mdh_frames": {"zh_CN": "显示 MDH 坐标系", "en": "Show MDH Frames"},
    "show_collision": {"zh_CN": "显示碰撞体", "en": "Show Collision"},
    "show_joint_axes": {"zh_CN": "显示关节轴", "en": "Show Joint Axes"},
    "show_com": {"zh_CN": "显示质心 (COM)", "en": "Show CoM"},
    "show_inertia": {"zh_CN": "显示惯量", "en": "Show Inertia"},

    # === 关节控制 ===
    "joints_control": {"zh_CN": "关节控制", "en": "Joints Control"},
    "joint_group": {"zh_CN": "关节控制", "en": "Joint Control"},
    "joint_control": {"zh_CN": "调整关节角度：", "en": "Joint Angles:"},
    "adjust_joint_angles": {"zh_CN": "调整关节角度：", "en": "Adjust joint angles:"},
    "angle_unit": {"zh_CN": "角度单位：", "en": "Angle Unit:"},
    "units_radian": {"zh_CN": "弧度", "en": "Radian"},
    "units_degree": {"zh_CN": "度", "en": "Degree"},
    "range_label": {"zh_CN": "范围: {} ~ {}", "en": "Range: {} ~ {}"},
    "reset": {"zh_CN": "重置", "en": "Reset"},
    "btn_reset": {"zh_CN": "重置", "en": "Reset"},
    "random": {"zh_CN": "随机", "en": "Random"},
    "btn_random": {"zh_CN": "随机", "en": "Random"},
    "btn_save": {"zh_CN": "保存", "en": "Save"},

    # === 当前文件 ===
    "current_file": {"zh_CN": "当前文件：", "en": "Current File:"},
    "current_file_none": {"zh_CN": "无", "en": "None"},
    "no_file_loaded": {"zh_CN": "没有加载文件", "en": "No file loaded"},

    # === 按钮 ===
    "btn_cancel": {"zh_CN": "取消", "en": "Cancel"},
    "btn_ok": {"zh_CN": "确定", "en": "OK"},

    # === MDH 对话框 ===
    "mdh_title": {"zh_CN": "MDH 参数", "en": "MDH Parameters"},
    "mdh_parameters": {"zh_CN": "MDH 参数", "en": "MDH Parameters"},
    "joint": {"zh_CN": "关节", "en": "Joint"},
    "forward_kinematics": {"zh_CN": "正运动学", "en": "Forward Kinematics"},
    "jacobian": {"zh_CN": "雅可比", "en": "Jacobian"},
    "dynamic_base_regressor": {"zh_CN": "动力学基回归器", "en": "Dynamic Base Regressor"},
    "save_mdh": {"zh_CN": "保存 MDH", "en": "Save MDH"},
    "header_file_or_python_usage": {"zh_CN": "头文件或 Python 使用", "en": "Header File or Python Usage"},
    "cpp_source_file": {"zh_CN": "C++ 源文件", "en": ".cpp Source File"},
    "copy": {"zh_CN": "复制", "en": "Copy"},

    # === 凸分解对话框 ===
    "decomp_title": {"zh_CN": "凸分解碰撞体", "en": "Decompose Collision Meshes"},
    "decomp_info": {"zh_CN": "为每个连杆的碰撞网格创建简化的近似版本", "en": "Create simplified approximations for each link's collision mesh"},
    "select_method": {"zh_CN": "选择方法：", "en": "Method:"},
    "method_vhc": {"zh_CN": "VHACD算法", "en": "VHACD"},
    "method_bounding": {"zh_CN": "包围盒", "en": "Bounding Box"},
    "decomp_progress": {"zh_CN": "正在分解...", "en": "Decomposing..."},
    "mesh_file": {"zh_CN": "网格文件", "en": "Mesh File"},
    "max_convex_hulls": {"zh_CN": "最大凸包数", "en": "maxConvexHulls"},
    "decomposition_complete": {"zh_CN": "分解完成", "en": "Decomposition Complete"},
    "decomposition_success": {"zh_CN": "成功分解 {} 个网格文件。", "en": "Successfully decomposed {} mesh files."},
    "decomposition_error": {"zh_CN": "分解错误", "en": "Decomposition Error"},
    "decomposition_error_msg": {"zh_CN": "分解过程中出错：{}", "en": "Error during decomposition: {}"},

    # === XML 编辑器 ===
    "urdf_editor": {"zh_CN": "URDF 编辑器", "en": "URDF Editor"},
    "save_as": {"zh_CN": "另存为", "en": "Save As"},
    "update": {"zh_CN": "更新", "en": "Update"},
    "update_tooltip": {"zh_CN": "更新查看器中的模型而不保存文件", "en": "Update the model in the viewer without saving the file"},
    "search": {"zh_CN": "搜索：", "en": "Search:"},
    "search_placeholder": {"zh_CN": "输入搜索文本...", "en": "Enter search text..."},
    "previous": {"zh_CN": "上一个", "en": "Previous"},
    "previous_tooltip": {"zh_CN": "查找上一个出现位置 (Shift+F3)", "en": "Find previous occurrence (Shift+F3)"},
    "next": {"zh_CN": "下一个", "en": "Next"},
    "next_tooltip": {"zh_CN": "查找下一个出现位置 (F3)", "en": "Find next occurrence (F3)"},
    "precision": {"zh_CN": "精度：", "en": "Precision:"},
    "pi_tooltip": {"zh_CN": "在光标位置插入 π 值", "en": "Insert π value at cursor position"},
    "pi_half_tooltip": {"zh_CN": "在光标位置插入 π/2 值", "en": "Insert π/2 value at cursor position"},
    "pi_quarter_tooltip": {"zh_CN": "在光标位置插入 π/4 值", "en": "Insert π/4 value at cursor position"},
    "precision_tooltip": {"zh_CN": "π 值的小数位数", "en": "Number of decimal places for π values"},
    "header_placeholder": {"zh_CN": "头文件或 Python 使用内容将显示在这里...", "en": "Header file or Python usage content will be displayed here..."},
    "cpp_placeholder": {"zh_CN": "源文件内容将显示在这里...", "en": "Source file content will be displayed here..."},

    # === 警告/错误/信息 ===
    "warning": {"zh_CN": "警告", "en": "Warning"},
    "error": {"zh_CN": "错误", "en": "Error"},
    "info": {"zh_CN": "提示", "en": "Info"},
    "success": {"zh_CN": "成功", "en": "Success"},
    "search": {"zh_CN": "搜索", "en": "Search"},
    "no_more_occurrences": {"zh_CN": "没有找到更多匹配项。", "en": "No more occurrences found."},
    "update_callback_not_set": {"zh_CN": "更新回调未设置。", "en": "Update callback not set."},

    # === 文件操作 ===
    "load_urdf_failed": {"zh_CN": "加载 URDF 文件失败: {}", "en": "Failed to load URDF file: {}"},
    "load_model_failed": {"zh_CN": "加载模型 {} 失败: {}", "en": "Failed to load model {}: {}"},
    "file_saved_successfully": {"zh_CN": "文件保存成功。", "en": "File saved successfully."},
    "failed_to_load_file": {"zh_CN": "加载文件失败：{}", "en": "Failed to load file: {}"},
    "failed_to_save_file": {"zh_CN": "保存文件失败：{}", "en": "Failed to save file: {}"},
    "failed_to_save_mdh": {"zh_CN": "保存 MDH 参数失败：\n{}", "en": "Failed to save MDH parameters:\n{}"},
    "mdh_saved": {"zh_CN": "MDH 参数已保存到:\n{}", "en": "MDH parameters saved to:\n{}"},
    "header_copied": {"zh_CN": "头代码已复制到剪贴板！", "en": "Header code copied to clipboard!"},
    "source_copied": {"zh_CN": "源代码已复制到剪贴板！", "en": "Source code copied to clipboard!"},
    "copied": {"zh_CN": "已复制", "en": "Copied"},

    # === 对话框标题 ===
    "dialog_open_urdf": {"zh_CN": "打开模型文件", "en": "Open Model File"},
    "dialog_urdf_filter": {"zh_CN": "机器人模型文件 (*.urdf *.xml)", "en": "Robot Model Files (*.urdf *.xml)"},
    "dialog_open_robot": {"zh_CN": "打开机器人文件", "en": "Open Robot File"},
    "dialog_robot_filter": {"zh_CN": "机器人文件 (*.urdf *.xml);;URDF 文件 (*.urdf);;MJCF 文件 (*.xml)", "en": "Robot Files (*.urdf *.xml);;URDF Files (*.urdf);;MJCF Files (*.xml)"},
    "dialog_save_file_as": {"zh_CN": "文件另存为", "en": "Save File As"},
    "dialog_save_mdh": {"zh_CN": "保存 MDH 参数", "en": "Save MDH Parameters"},
    "dialog_text_files": {"zh_CN": "文本文件 (*.txt)", "en": "Text Files (*.txt)"},

    # === MJCF 相关 ===
    "load_mjcf_failed": {"zh_CN": "加载 MJCF 文件失败: {}", "en": "Failed to load MJCF file: {}"},
    "unsupported_format": {"zh_CN": "不支持的文件格式: {}", "en": "Unsupported file format: {}"},
    "invalid_mjcf_file": {"zh_CN": "无效的 MuJoCo MJCF 文件（XML 文件必须包含 <mujoco> 根元素）", "en": "Invalid MuJoCo MJCF file (XML file must contain <mujoco> root element)"},
    "mdh_urdf_only": {"zh_CN": "MDH 参数功能仅支持 URDF 文件", "en": "MDH parameters are only supported for URDF files"},

    # === 语言切换 ===
    "language": {"zh_CN": "语言：", "en": "Language:"},
    "lang_zh": {"zh_CN": "简体中文", "en": "Simplified Chinese"},
    "lang_en": {"zh_CN": "English", "en": "English"},

    # === 提示信息 ===
    "please_load_urdf_first": {"zh_CN": "请先加载 URDF 文件。", "en": "Please load a URDF file first."},
    "please_load_urdf_edit": {"zh_CN": "请先加载 URDF 文件。[编辑]", "en": "Please load a URDF file first. [Edit]"},
    "please_load_urdf_mdh": {"zh_CN": "请先加载 URDF 文件。[MDH]", "en": "Please load a URDF file first. [MDH]"},
    "please_load_urdf_show_mdh": {"zh_CN": "请先加载 URDF 文件。[显示 MDH]", "en": "Please load a URDF file first. [Show MDH]"},
    "please_select_chain_first": {"zh_CN": "请先选择一个运动链。", "en": "Please select a chain first."},
    "please_select_chain_mdh": {"zh_CN": "请先选择一个运动链以查看 MDH 参数。", "en": "Please select a kinematic chain to view MDH parameters."},
    "no_collision_meshes": {"zh_CN": "没有可用的碰撞网格。请先加载 URDF 文件。", "en": "No collision meshes available. Please load a URDF file first."},
    "no_revolute_joints": {"zh_CN": "没有可用的旋转关节。请先加载 URDF 文件。", "en": "No revolute joints available. Please load a URDF file first."},
    "please_load_urdf_com": {"zh_CN": "请先加载 URDF 文件。[质心]", "en": "Please load a URDF file first [CoM]"},
    "please_load_urdf_inertia": {"zh_CN": "请先加载 URDF 文件。[惯量]", "en": "Please load a URDF file first [Inertia]"},

    # === 运动链 ===
    "chain_pattern": {"zh_CN": "运动链 {}: {}", "en": "Chain {}: {}"},

    # === 设置关节对话框 ===
    "set_joints_title": {"zh_CN": "设置关节角度", "en": "Set Joint Angles"},
    "set_joints_info": {"zh_CN": "手动输入所有关节的角度值", "en": "Manually enter angle values for all joints"},
    "joint_name_header": {"zh_CN": "关节", "en": "Joint"},
    "angle_header": {"zh_CN": "角度（{}）", "en": "Angle ({})"},
    "enter_joint_angles": {"zh_CN": "输入关节角度（用逗号或空格分隔）：", "en": "Enter joint angles (comma or space separated):"},
    "units": {"zh_CN": "单位：", "en": "Units:"},
    "apply": {"zh_CN": "应用", "en": "Apply"},
    "please_input_angles": {"zh_CN": "请输入角度值。", "en": "Please input angles."},
    "invalid_number": {"zh_CN": "输入列表中包含无效数字。", "en": "Invalid number in input list."},
    "expected_values": {"zh_CN": "需要 {} 个值，收到 {} 个。", "en": "Expected {} values, got {}."},

    # === 保存 MDH 文件内容 ===
    "mdh_file_header": {"zh_CN": "MDH 参数 - {}", "en": "MDH Parameters for {}"},
    "mdh_column_header": {"zh_CN": "{:<10} {:<12} {:<12} {:<12} {:<12}", "en": "{:<10} {:<12} {:<12} {:<12} {:<12}"},
    "mdh_total_joints": {"zh_CN": "总关节数：{}", "en": "Total joints: {}"},

    # === 表头 ===
    "theta_rad": {"zh_CN": "θ (rad)", "en": "θ (rad)"},
    "alpha_rad": {"zh_CN": "α (rad)", "en": "α (rad)"},

    # === 拓扑图 ===
    "show_topology": {"zh_CN": "显示拓扑图", "en": "Show Topology"},
    "topology_title": {"zh_CN": "机器人拓扑图", "en": "Robot Topology"},
    "fit_view": {"zh_CN": "适应窗口", "en": "Fit View"},
    "export_png": {"zh_CN": "导出 PNG", "en": "Export PNG"},
    "export_svg": {"zh_CN": "导出 SVG", "en": "Export SVG"},
    "btn_close": {"zh_CN": "关闭", "en": "Close"},
    "export_success": {"zh_CN": "已导出到: {}", "en": "Exported to: {}"},
    "export_failed": {"zh_CN": "导出失败。", "en": "Export failed."},
    "export_empty_scene": {"zh_CN": "场景为空，无法导出。", "en": "Scene is empty, cannot export."},
    "load_urdf_topology": {"zh_CN": "请先加载 URDF 文件。[拓扑图]", "en": "Please load a URDF file first [Topology]"},

    # === 菜单栏 ===
    "menu_file": {"zh_CN": "文件(&F)", "en": "&File"},
    "menu_view": {"zh_CN": "视图(&V)", "en": "&View"},
    "menu_tools": {"zh_CN": "工具(&T)", "en": "&Tools"},
    "menu_help": {"zh_CN": "帮助(&H)", "en": "&Help"},
    "quit": {"zh_CN": "退出", "en": "Quit"},
    "recent_files": {"zh_CN": "最近打开", "en": "Recent Files"},
    "toggle_left_panel": {"zh_CN": "切换左侧面板", "en": "Toggle Left Panel"},
    "toggle_right_panel": {"zh_CN": "切换右侧面板", "en": "Toggle Right Panel"},
    "theme_dark": {"zh_CN": "深色主题", "en": "Dark Theme"},
    "theme_light": {"zh_CN": "浅色主题", "en": "Light Theme"},
    "about": {"zh_CN": "关于", "en": "About"},
    "about_text": {"zh_CN": "URDFly - URDF/MJCF 机器人模型可视化工具", "en": "URDFly - URDF/MJCF Robot Model Visualization Tool"},

    # === 状态栏 ===
    "ready": {"zh_CN": "就绪", "en": "Ready"},
    "status_file_info": {"zh_CN": "{} | {} 个关节 | {} 个连杆", "en": "{} | {} joints | {} links"},
    "model_loaded": {"zh_CN": "模型已加载: {}", "en": "Model loaded: {}"},

    # === 可折叠面板 ===
    "section_robot_structure": {"zh_CN": "机器人结构", "en": "Robot Structure"},
    "section_transparency": {"zh_CN": "透明度", "en": "Transparency"},
    "section_display": {"zh_CN": "显示设置", "en": "Display Settings"},

    # === 帮助菜单扩展 ===
    "quick_start_guide": {"zh_CN": "快速入门指南", "en": "Quick Start Guide"},
    "keyboard_shortcuts": {"zh_CN": "快捷键一览", "en": "Keyboard Shortcuts"},
    "tutorial_mdh": {"zh_CN": "MDH 参数教程", "en": "MDH Parameters Tutorial"},
    "tutorial_ik": {"zh_CN": "解析逆运动学教程", "en": "Analytical IK Tutorial"},
    "quick_start_title": {"zh_CN": "快速入门指南", "en": "Quick Start Guide"},
    "shortcuts_title": {"zh_CN": "快捷键一览", "en": "Keyboard Shortcuts"},
    "shortcut_key": {"zh_CN": "快捷键", "en": "Shortcut"},
    "shortcut_action": {"zh_CN": "功能", "en": "Action"},

    # === 相机视图 ===
    "camera_views": {"zh_CN": "相机视图", "en": "Camera Views"},
    "view_front": {"zh_CN": "前视图", "en": "Front View"},
    "view_back": {"zh_CN": "后视图", "en": "Back View"},
    "view_left": {"zh_CN": "左视图", "en": "Left View"},
    "view_right": {"zh_CN": "右视图", "en": "Right View"},
    "view_top": {"zh_CN": "顶视图", "en": "Top View"},
    "view_bottom": {"zh_CN": "底视图", "en": "Bottom View"},
    "view_isometric": {"zh_CN": "等轴测视图", "en": "Isometric View"},
    "quick_start_html": {
        "zh_CN": """<h2>URDFly 快速入门</h2>
<h3>1. 打开模型</h3>
<ul>
<li>点击工具栏 <b>打开模型</b> 按钮，或使用 <code>Ctrl+O</code></li>
<li>支持直接拖放 URDF / MJCF 文件到窗口</li>
</ul>
<h3>2. 关节控制</h3>
<ul>
<li>在右侧面板拖动滑块调整关节角度</li>
<li>在 3D 视图中直接拖拽连杆来交互式调节关节</li>
<li>使用工具栏 <b>重置</b> / <b>随机</b> 按钮快速设置角度</li>
</ul>
<h3>3. 显示设置</h3>
<ul>
<li>在右侧面板切换坐标系、碰撞体、质心、惯量等显示</li>
<li>使用透明度滑块查看内部结构</li>
</ul>
<h3>4. MDH 参数 &amp; 代码生成</h3>
<ul>
<li>选择运动链，点击 <b>工具 → 显示 MDH 参数</b>（<code>Ctrl+M</code>）</li>
<li>查看正运动学、雅可比矩阵，生成 C++/Python 代码</li>
</ul>
<h3>5. 拓扑图 &amp; 凸分解</h3>
<ul>
<li><b>工具 → 显示拓扑图</b>（<code>Ctrl+T</code>）查看连杆-关节树结构</li>
<li><b>工具 → 凸分解碰撞体</b> 为碰撞网格生成简化近似</li>
</ul>
<h3>6. 视图 &amp; 主题</h3>
<ul>
<li>3D 视口右上角有 <b>浮动视图面板</b>，提供 7 个相机预设视图（前 / 后 / 左 / 右 / 顶 / 底 / 等轴测）</li>
<li>也可通过数字快捷键快速切换视图：<code>1</code> 前视图、<code>3</code> 左视图、<code>7</code> 顶视图、<code>0</code> 等轴测；按住 <code>Ctrl</code> 切换为后/右/底视图</li>
<li><b>视图</b> 菜单切换深色/浅色主题</li>
<li>可隐藏/显示左右面板获得更大 3D 视口</li>
</ul>
<h3>7. 快捷键</h3>
<ul>
<li><code>Ctrl+O</code> 打开模型　<code>Ctrl+E</code> 编辑 XML　<code>Ctrl+M</code> MDH 参数　<code>Ctrl+T</code> 拓扑图</li>
<li><code>Ctrl+R</code> 重置关节　<code>Ctrl+Q</code> 退出　<code>Ctrl+F</code> XML 编辑器搜索</li>
<li><code>1</code>/<code>Ctrl+1</code> 前/后　<code>3</code>/<code>Ctrl+3</code> 左/右　<code>7</code>/<code>Ctrl+7</code> 顶/底　<code>0</code> 等轴测</li>
<li>完整列表请查看 <b>帮助 → 快捷键一览</b></li>
</ul>""",
        "en": """<h2>URDFly Quick Start</h2>
<h3>1. Open a Model</h3>
<ul>
<li>Click the <b>Open Model</b> toolbar button, or press <code>Ctrl+O</code></li>
<li>You can also drag &amp; drop URDF / MJCF files onto the window</li>
</ul>
<h3>2. Joint Control</h3>
<ul>
<li>Drag the sliders in the right panel to adjust joint angles</li>
<li>Drag links directly in the 3D view for interactive joint control</li>
<li>Use toolbar <b>Reset</b> / <b>Random</b> buttons to quickly set angles</li>
</ul>
<h3>3. Visibility Settings</h3>
<ul>
<li>Toggle coordinate frames, collision bodies, CoM, inertia in the right panel</li>
<li>Use the transparency slider to see internal structures</li>
</ul>
<h3>4. MDH Parameters &amp; Code Generation</h3>
<ul>
<li>Select a kinematic chain, then <b>Tools → Show MDH Parameters</b> (<code>Ctrl+M</code>)</li>
<li>View forward kinematics, Jacobian matrix, and generate C++/Python code</li>
</ul>
<h3>5. Topology &amp; Convex Decomposition</h3>
<ul>
<li><b>Tools → Show Topology</b> (<code>Ctrl+T</code>) to view the link-joint tree</li>
<li><b>Tools → Decompose Collision</b> to create simplified collision meshes</li>
</ul>
<h3>6. View &amp; Themes</h3>
<ul>
<li>A <b>floating view panel</b> in the top-right corner of the 3D viewport offers 7 camera presets (Front / Back / Left / Right / Top / Bottom / Isometric)</li>
<li>Quick-switch views with number keys: <code>1</code> Front, <code>3</code> Left, <code>7</code> Top, <code>0</code> Isometric; hold <code>Ctrl</code> for Back / Right / Bottom</li>
<li>Switch between dark/light themes from the <b>View</b> menu</li>
<li>Hide/show left/right panels for a larger 3D viewport</li>
</ul>
<h3>7. Keyboard Shortcuts</h3>
<ul>
<li><code>Ctrl+O</code> Open　<code>Ctrl+E</code> Edit XML　<code>Ctrl+M</code> MDH　<code>Ctrl+T</code> Topology</li>
<li><code>Ctrl+R</code> Reset Joints　<code>Ctrl+Q</code> Quit　<code>Ctrl+F</code> Search in XML Editor</li>
<li><code>1</code>/<code>Ctrl+1</code> Front/Back　<code>3</code>/<code>Ctrl+3</code> Left/Right　<code>7</code>/<code>Ctrl+7</code> Top/Bottom　<code>0</code> Isometric</li>
<li>See the full list at <b>Help → Keyboard Shortcuts</b></li>
</ul>"""
    },
}


class TranslationManager:
    """翻译管理器 - 处理语言切换和翻译获取"""

    def __init__(self):
        self.current_language = "zh_CN"  # 默认中文
        self.translator = None
        self.available_languages = ["zh_CN", "en"]

    def tr(self, key, default=None, *args):
        """
        获取指定键的翻译
        :param key: 翻译键
        :param default: 默认值（如果键不存在，返回键本身）
        :param args: 格式化参数
        """
        if key not in TRANSLATIONS:
            result = default if default is not None else key
        else:
            result = TRANSLATIONS[key].get(self.current_language, default if default is not None else key)

        # 支持格式化参数，如 "Hello {}" -> "Hello World"
        if args:
            try:
                return result.format(*args)
            except (IndexError, KeyError, ValueError):
                return result
        return result

    def set_language(self, lang_code, main_window=None):
        """
        切换语言
        :param lang_code: 语言代码 ('zh_CN' 或 'en')
        :param main_window: 主窗口对象（用于重新翻译UI）
        """
        if lang_code not in self.available_languages:
            print(f"Warning: Unsupported language code: {lang_code}")
            return False

        old_lang = self.current_language
        self.current_language = lang_code

        if main_window:
            # 重新翻译整个界面
            self.retranslate_ui(main_window)

        print(f"Language changed from {old_lang} to {lang_code}")
        return True

    def retranslate_ui(self, main_window):
        """重新翻译主窗口的所有UI元素"""
        # 更新窗口标题
        main_window.setWindowTitle(self.tr("window_title"))

        # 更新机器人结构标签
        if hasattr(main_window, 'robot_structure_label'):
            main_window.robot_structure_label.setText(self.tr("robot_structure"))

        # 更新选择运动链标签
        if hasattr(main_window, 'select_chain_label'):
            main_window.select_chain_label.setText(self.tr("select_chain"))

        # 更新连杆标签
        if hasattr(main_window, 'links_label'):
            main_window.links_label.setText(self.tr("links"))

        # 更新当前文件标签
        if hasattr(main_window, 'current_file_label'):
            if main_window.current_urdf_file:
                import os
                filename = os.path.basename(main_window.current_urdf_file)
                main_window.current_file_label.setText(self.tr("current_file") + " " + filename)
            else:
                main_window.current_file_label.setText(self.tr("current_file") + " " + self.tr("current_file_none"))

        # 更新透明度标签
        if hasattr(main_window, 'transparency_label'):
            main_window.transparency_label.setText(self.tr("transparency"))

        # 更新显示设置组 (QGroupBox 或 CollapsibleSection)
        if hasattr(main_window, 'visibility_group'):
            main_window.visibility_group.setTitle(self.tr("visibility_settings"))

        if hasattr(main_window, 'cb_link_frames'):
            main_window.cb_link_frames.setText(self.tr("show_link_frames"))
        if hasattr(main_window, 'cb_visual'):
            main_window.cb_visual.setText(self.tr("show_visual"))
        if hasattr(main_window, 'cb_mdh_frames'):
            main_window.cb_mdh_frames.setText(self.tr("show_mdh_frames"))
        if hasattr(main_window, 'cb_collision'):
            main_window.cb_collision.setText(self.tr("show_collision"))
        if hasattr(main_window, 'cb_com'):
            main_window.cb_com.setText(self.tr("show_com"))
        if hasattr(main_window, 'cb_inertia'):
            main_window.cb_inertia.setText(self.tr("show_inertia"))

        # 更新关节控制标签
        if hasattr(main_window, 'joint_label'):
            main_window.joint_label.setText(self.tr("adjust_joint_angles"))

        # 更新关节控制组
        if hasattr(main_window, 'joint_group'):
            main_window.joint_group.setTitle(self.tr("joints_control"))

        # 更新重置和随机按钮
        if hasattr(main_window, 'btn_reset'):
            main_window.btn_reset.setText(self.tr("reset"))
        if hasattr(main_window, 'btn_random'):
            main_window.btn_random.setText(self.tr("random"))

        # 更新语言标签
        if hasattr(main_window, 'lang_label'):
            main_window.lang_label.setText(self.tr("language"))

        # 更新语言选择下拉框
        if hasattr(main_window, 'language_combo'):
            main_window.language_combo.setItemText(0, self.tr("lang_zh"))
            main_window.language_combo.setItemText(1, self.tr("lang_en"))

        # === 菜单栏 ===
        if hasattr(main_window, 'menu_file'):
            main_window.menu_file.setTitle(self.tr("menu_file"))
        if hasattr(main_window, 'menu_view'):
            main_window.menu_view.setTitle(self.tr("menu_view"))
        if hasattr(main_window, 'menu_tools'):
            main_window.menu_tools.setTitle(self.tr("menu_tools"))
        if hasattr(main_window, 'menu_help'):
            main_window.menu_help.setTitle(self.tr("menu_help"))

        # 菜单动作
        if hasattr(main_window, 'act_open'):
            main_window.act_open.setText(self.tr("open_urdf"))
        if hasattr(main_window, 'act_edit'):
            main_window.act_edit.setText(self.tr("edit_urdf"))
        if hasattr(main_window, 'act_quit'):
            main_window.act_quit.setText(self.tr("quit"))
        if hasattr(main_window, 'menu_recent'):
            main_window.menu_recent.setTitle(self.tr("recent_files"))
            main_window._update_recent_files_menu()
        if hasattr(main_window, 'act_toggle_left'):
            main_window.act_toggle_left.setText(self.tr("toggle_left_panel"))
        if hasattr(main_window, 'act_toggle_right'):
            main_window.act_toggle_right.setText(self.tr("toggle_right_panel"))
        if hasattr(main_window, 'act_dark_theme'):
            main_window.act_dark_theme.setText(self.tr("theme_dark"))
        if hasattr(main_window, 'act_light_theme'):
            main_window.act_light_theme.setText(self.tr("theme_light"))
        if hasattr(main_window, 'act_mdh'):
            main_window.act_mdh.setText(self.tr("show_mdh"))
        if hasattr(main_window, 'act_topology'):
            main_window.act_topology.setText(self.tr("show_topology"))
        if hasattr(main_window, 'act_decomp'):
            main_window.act_decomp.setText(self.tr("decompose_collision"))
        if hasattr(main_window, 'act_set_joints'):
            main_window.act_set_joints.setText(self.tr("set_joints"))
        if hasattr(main_window, 'act_about'):
            main_window.act_about.setText(self.tr("about"))
        if hasattr(main_window, 'act_quick_start'):
            main_window.act_quick_start.setText(self.tr("quick_start_guide"))
        if hasattr(main_window, 'act_shortcuts'):
            main_window.act_shortcuts.setText(self.tr("keyboard_shortcuts"))
        if hasattr(main_window, 'act_tutorial_mdh'):
            main_window.act_tutorial_mdh.setText(self.tr("tutorial_mdh"))
        if hasattr(main_window, 'act_tutorial_ik'):
            main_window.act_tutorial_ik.setText(self.tr("tutorial_ik"))

        # 相机视图菜单
        if hasattr(main_window, 'menu_camera_views'):
            main_window.menu_camera_views.setTitle(self.tr("camera_views"))
        if hasattr(main_window, 'act_view_front'):
            main_window.act_view_front.setText(self.tr("view_front"))
        if hasattr(main_window, 'act_view_back'):
            main_window.act_view_back.setText(self.tr("view_back"))
        if hasattr(main_window, 'act_view_left'):
            main_window.act_view_left.setText(self.tr("view_left"))
        if hasattr(main_window, 'act_view_right'):
            main_window.act_view_right.setText(self.tr("view_right"))
        if hasattr(main_window, 'act_view_top'):
            main_window.act_view_top.setText(self.tr("view_top"))
        if hasattr(main_window, 'act_view_bottom'):
            main_window.act_view_bottom.setText(self.tr("view_bottom"))
        if hasattr(main_window, 'act_view_isometric'):
            main_window.act_view_isometric.setText(self.tr("view_isometric"))

        # === 工具栏动作 (同菜单共用，已更新) ===
        if hasattr(main_window, 'tb_act_reset'):
            main_window.tb_act_reset.setText(self.tr("reset"))
        if hasattr(main_window, 'tb_act_random'):
            main_window.tb_act_random.setText(self.tr("random"))

        # === 可折叠面板标题 ===
        if hasattr(main_window, 'section_structure'):
            main_window.section_structure.set_title(self.tr("section_robot_structure"))
        if hasattr(main_window, 'section_transparency'):
            main_window.section_transparency.set_title(self.tr("section_transparency"))
        if hasattr(main_window, 'section_display'):
            main_window.section_display.set_title(self.tr("section_display"))

        # === 浮动视图面板 tooltip ===
        if hasattr(main_window, '_view_overlay_buttons'):
            for btn, tip_key in main_window._view_overlay_buttons:
                btn.setToolTip(self.tr(tip_key))

    @staticmethod
    def get_available_languages():
        """获取可用的语言列表"""
        return [
            {"code": "zh_CN", "name": "简体中文", "native_name": "简体中文"},
            {"code": "en", "name": "English", "native_name": "English"},
        ]


# 全局翻译实例
_translation_manager = TranslationManager()


def tr(key, default=None, *args):
    """
    全局翻译函数的快捷方式
    :param key: 翻译键
    :param default: 默认值
    :param args: 格式化参数
    :return: 翻译后的字符串
    """
    return _translation_manager.tr(key, default, *args)


def get_translation_manager():
    """获取全局翻译管理器实例"""
    return _translation_manager
