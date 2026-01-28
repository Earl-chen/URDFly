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
    "show_joint_axes": {"zh_CN": "显示关节轴", "en": "Show Joint Axes"},
    "show_collision": {"zh_CN": "显示碰撞体", "en": "Show Collision"},
    "show_com": {"zh_CN": "显示质心 (COM)", "en": "Show CoM"},
    "show_inertia": {"zh_CN": "显示惯量", "en": "Show Inertia"},

    # === 角度单位 ===
    "angle_unit": {"zh_CN": "角度单位：", "en": "Angle Unit:"},
    "units_radian": {"zh_CN": "弧度", "en": "Radian"},
    "units_degree": {"zh_CN": "度", "en": "Degree"},

    # === 关节控制 ===
    "joint_control": {"zh_CN": "调整关节角度：", "en": "Joint Angles:"},
    "range": {"zh_CN": "范围", "en": "Range"},

    # === 当前文件 ===
    "current_file": {"zh_CN": "当前文件：", "en": "Current File:"},
    "current_file_none": {"zh_CN": "无", "en": "None"},

    # === 按钮 ===
    "btn_update": {"zh_CN": "更新", "en": "Update"},
    "btn_close": {"zh_CN": "关闭", "en": "Close"},
    "btn_save": {"zh_CN": "保存", "en": "Save"},
    "btn_cancel": {"zh_CN": "取消", "en": "Cancel"},

    # === MDH 对话框 ===
    "mdh_title": {"zh_CN": "MDH 参数", "en": "MDH Parameters"},

    # === 凸分解对话框 ===
    "decomp_title": {"zh_CN": "凸分解碰撞体", "en": "Decompose Collision Meshes"},
    "decomp_info": {"zh_CN": "为每个连杆的碰撞网格创建简化的近似版本", "en": "Create simplified approximations for each link's collision mesh"},
    "select_method": {"zh_CN": "选择方法：", "en": "Method:"},
    "method_vhc": {"zh_CN": "VHACD算法", "en": "VHACD"},
    "method_bounding": {"zh_CN": "包围盒", "en": "Bounding Box"},
    "decomp_progress": {"zh_CN": "正在分解...", "en": "Decomposing..."},

    # === 设置关节对话框 ===
    "set_joints_title": {"zh_CN": "设置关节角度", "en": "Set Joint Angles"},
    "set_joints_info": {"zh_CN": "手动输入所有关节的角度值", "en": "Manually enter angle values for all joints"},
    "joint_name_header": {"zh_CN": "关节", "en": "Joint"},
    "angle_header": {"zh_CN": "角度（{}）", "en": "Angle ({})"},

    # === 警告信息 ===
    "warning": {"zh_CN": "警告", "en": "Warning"},
    "error": {"zh_CN": "错误", "en": "Error"},
    "info": {"zh_CN": "提示", "en": "Info"},
    "load_urdf_first": {"zh_CN": "请先加载 URDF 文件。", "en": "Please load a URDF file first."},
    "load_urdf_edit": {"zh_CN": "请先加载 URDF 文件。[编辑]", "en": "Please load a URDF file first [Edit]"},
    "load_urdf_mdh": {"zh_CN": "请先加载 URDF 文件。[MDH]", "en": "Please load a URDF file first [MDH]"},
    "load_urdf_decomp": {"zh_CN": "请先加载 URDF 文件。[凸分解]", "en": "Please load a URDF file first [Decompose]"},
    "load_urdf_set_joints": {"zh_CN": "请先加载 URDF 文件。[设置关节]", "en": "Please load a URDF file first [Set Joints]"},
    "load_model_failed": {"zh_CN": "加载模型 {name} 失败: {str(e)}", "en": "Failed to load model {name}: {str(e)}"},
    "no_urdf_loaded": {"zh_CN": "没有加载的 URDF 文件！", "en": "No URDF file loaded!"},

    # === 语言切换 ===
    "language": {"zh_CN": "语言：", "en": "Language:"},
    "lang_zh": {"zh_CN": "简体中文", "en": "Simplified Chinese"},
    "lang_en": {"zh_CN": "English", "en": "English"},

    # === 关节控制区 ===
    "joint_group": {"zh_CN": "关节控制", "en": "Joint Control"},
    "btn_reset": {"zh_CN": "重置", "en": "Reset"},
    "btn_random": {"zh_CN": "随机", "en": "Random"},

    # === 对话框 ===
    "dialog_open_urdf": {"zh_CN": "打开 URDF 文件", "en": "Open URDF File"},
    "dialog_urdf_filter": {"zh_CN": "URDF 文件 (*.urdf)", "en": "URDF Files (*.urdf)"},
    "dialog_open_robot": {"zh_CN": "打开机器人文件", "en": "Open Robot File"},
    "dialog_robot_filter": {"zh_CN": "机器人文件 (*.urdf *.xml);;URDF 文件 (*.urdf);;MJCF 文件 (*.xml)", "en": "Robot Files (*.urdf *.xml);;URDF Files (*.urdf);;MJCF Files (*.xml)"},
    "load_urdf_failed": {"zh_CN": "加载 URDF 文件失败: {}", "en": "Failed to load URDF file: {}"},
    "load_mjcf_failed": {"zh_CN": "加载 MJCF 文件失败: {}", "en": "Failed to load MJCF file: {}"},
    "invalid_mjcf_file": {"zh_CN": "无效的 MuJoCo MJCF 文件（XML 文件必须包含 <mujoco> 根元素）", "en": "Invalid MuJoCo MJCF file (XML file must contain <mujoco> root element)"},
    "mdh_urdf_only": {"zh_CN": "MDH 参数功能仅支持 URDF 文件", "en": "MDH parameters are only supported for URDF files"},
    "load_model_failed_pattern": {"zh_CN": "加载模型 {} 失败: {}", "en": "Failed to load model {}: {}"},
    "update_model_failed": {"zh_CN": "从 XML 更新模型失败: {}", "en": "Failed to update model from XML: {}"},

    # === 关节输入对话框 ===
    "input_joints_prompt": {"zh_CN": "输入关节角度（用逗号或空格分隔）：", "en": "Enter joint angles (separated by comma or space):"},
    "units_label": {"zh_CN": "单位：", "en": "Unit:"},
    "btn_apply": {"zh_CN": "应用", "en": "Apply"},
    "input_angles_warning": {"zh_CN": "请输入角度值。", "en": "Please enter angle values."},
    "input_numbers_error": {"zh_CN": "输入列表中包含无效数字。", "en": "Input list contains invalid numbers."},
    "input_count_warning": {"zh_CN": "需要 {} 个值，收到 {} 个。", "en": "Expected {} values, got {}."},
    "no_revolute_joints": {"zh_CN": "没有可用的旋转关节。请先加载 URDF 文件。", "en": "No revolute joints available. Please load a URDF file first."},

    # === 运动链相关 ===
    "chain_pattern": {"zh_CN": "运动链 {}: {}", "en": "Chain {}: {}"},
    "select_chain_warning": {"zh_CN": "请先选择一个运动链。", "en": "Please select a kinematic chain first."},

    # === 范围标签 ===
    "range_label": {"zh_CN": "范围: {} ~ {}", "en": "Range: {} ~ {}"},

    # === 其他警告信息 ===
    "load_urdf_joint_axes": {"zh_CN": "请先加载 URDF 文件。[关节轴]", "en": "Please load a URDF file first [Joint Axes]"},
    "load_urdf_com": {"zh_CN": "请先加载 URDF 文件。[质心]", "en": "Please load a URDF file first [CoM]"},
    "load_urdf_inertia": {"zh_CN": "请先加载 URDF 文件。[惯量]", "en": "Please load a URDF file first [Inertia]"},
    "select_chain_mdh_warning": {"zh_CN": "请先选择一个运动链以查看 MDH 参数。", "en": "Please select a kinematic chain to view MDH parameters."},
    "load_urdf_show_mdh": {"zh_CN": "请先加载 URDF 文件。[显示 MDH]", "en": "Please load a URDF file first [Show MDH]"},
    "no_collision_meshes": {"zh_CN": "没有可用的碰撞网格。请先加载 URDF 文件。", "en": "No collision meshes available. Please load a URDF file first."},

    # === 拓扑图 ===
    "show_topology": {"zh_CN": "显示拓扑图", "en": "Show Topology"},
    "topology_title": {"zh_CN": "机器人拓扑图", "en": "Robot Topology"},
    "fit_view": {"zh_CN": "适应窗口", "en": "Fit View"},
    "export_png": {"zh_CN": "导出 PNG", "en": "Export PNG"},
    "export_svg": {"zh_CN": "导出 SVG", "en": "Export SVG"},
    "export_success": {"zh_CN": "已导出到: {}", "en": "Exported to: {}"},
    "export_failed": {"zh_CN": "导出失败。", "en": "Export failed."},
    "export_empty_scene": {"zh_CN": "场景为空，无法导出。", "en": "Scene is empty, cannot export."},
    "load_urdf_topology": {"zh_CN": "请先加载 URDF 文件。[拓扑图]", "en": "Please load a URDF file first [Topology]"},
}


class TranslationManager:
    """翻译管理器 - 处理语言切换和翻译获取"""

    def __init__(self):
        self.current_language = "zh_CN"  # 默认中文
        self.translator = None
        self.available_languages = ["zh_CN", "en"]

    def tr(self, key, default=None):
        """
        获取指定键的翻译
        :param key: 翻译键
        :param default: 默认值（如果键不存在，返回键本身）
        """
        if key not in TRANSLATIONS:
            return default if default is not None else key

        return TRANSLATIONS[key].get(self.current_language, default if default is not None else key)

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

        # 更新左侧面板标签
        main_window.current_file_label.setText(self.tr("current_file") + " " + self.tr("current_file_none"))

        # 更新按钮
        main_window.btn_open.setText(self.tr("open_urdf"))
        main_window.btn_edit.setText(self.tr("edit_urdf"))
        main_window.btn_edit.setToolTip(self.tr("edit_urdf_tooltip"))
        main_window.btn_mdh.setText(self.tr("show_mdh"))
        main_window.btn_decomp.setText(self.tr("decompose_collision"))
        main_window.btn_set_joints.setText(self.tr("set_joints"))

        # 更新透明度标签
        main_window.transparency_label.setText(self.tr("transparency"))

        # 更新显示设置组
        main_window.visibility_group.setTitle(self.tr("visibility_settings"))

        main_window.cb_visual.setText(self.tr("show_visual"))
        main_window.cb_link_frames.setText(self.tr("show_link_frames"))
        main_window.cb_mdh_frames.setText(self.tr("show_mdh_frames"))
        main_window.cb_joint_axes.setText(self.tr("show_joint_axes"))
        main_window.cb_collision.setText(self.tr("show_collision"))
        main_window.cb_com.setText(self.tr("show_com"))
        main_window.cb_inertia.setText(self.tr("show_inertia"))

        # 更新关节控制标签
        main_window.joint_label.setText(self.tr("joint_control"))

        # 更新关节控制组
        if hasattr(main_window, 'joint_group'):
            main_window.joint_group.setTitle(self.tr("joint_group"))

        # 更新重置和随机按钮
        if hasattr(main_window, 'btn_reset'):
            main_window.btn_reset.setText(self.tr("btn_reset"))
        if hasattr(main_window, 'btn_random'):
            main_window.btn_random.setText(self.tr("btn_random"))

        # 更新语言标签
        if hasattr(main_window, 'lang_label'):
            main_window.lang_label.setText(self.tr("language"))

        # 更新语言选择下拉框
        if hasattr(main_window, 'language_combo'):
            main_window.language_combo.setItemText(0, self.tr("lang_zh"))
            main_window.language_combo.setItemText(1, self.tr("lang_en"))

    @staticmethod
    def get_available_languages():
        """获取可用的语言列表"""
        return [
            {"code": "zh_CN", "name": "简体中文", "native_name": "简体中文"},
            {"code": "en", "name": "English", "native_name": "English"},
        ]


# 全局翻译实例
_translation_manager = TranslationManager()


def tr(key, default=None):
    """
    全局翻译函数的快捷方式
    :param key: 翻译键
    :param default: 默认值
    :return: 翻译后的字符串
    """
    return _translation_manager.tr(key, default)
