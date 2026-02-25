# URDFly

![GX7 七轴机械臂](assets/gx7.png)
![Poppy 人形机器人](assets/poppy.png)

## 概述

URDFly 是一款基于 Python 的 URDF/MJCF 机器人模型编辑与可视化工具，采用 **PyQt5 + VTK** 构建三面板 GUI（结构树 / 3D 视图 / 关节控制），支持拖放加载模型文件。核心功能包括：

- 坐标系可视化（连杆坐标系、MDH 坐标系、关节轴）
- URDF XML 实时编辑与同步渲染
- MDH 参数自动转换与表格显示
- 正运动学 / 雅可比 / 动力学基回归器 C++ 代码生成
- 质心与惯量可视化、凸分解碰撞体、拓扑图导出

## 功能特性

### 模型加载与格式支持

- 支持 URDF (`.urdf`) 和 MuJoCo MJCF (`.xml`，需安装 `mujoco`) 两种格式
- 拖放加载：直接将文件拖入窗口即可打开
- 自动解析 `package://`、`file://` 等 URI 协议
- 支持 STL 和 OBJ 网格文件
- 内置示例模型：GX7 七轴机械臂、Poppy 人形机器人

### 3D 可视化

- VTK 实时 3D 渲染
- 透明度滑块控制
- 点击连杆列表高亮显示对应连杆
- 视觉模型 / 碰撞模型独立开关
- 虚拟连杆以红色球体标识

![碰撞模型可视化](assets/gx7_collision.png)

![Schunk 机器人模型](assets/schunk.png)

### 关节控制

- 每个关节独立滑块控制
- 弧度 / 度单位切换
- 重置归零、随机姿态、批量输入关节角度
- **拖拽交互**：在 3D 视图中直接拖拽连杆控制关节角度

### 坐标系与轴可视化

- **连杆坐标系**：显示每个连杆的原始坐标系
- **MDH 坐标系**：显示 Modified DH 参数对应的坐标系
- **关节轴**：显示每个关节的旋转/平移轴方向
- 三种可视化各自独立开关，可自由组合

### 质心与惯量可视化

- **质心标记 (CoM)**：双层球体标识每个连杆的质心位置
- **惯量可视化**：通过惯量矩阵主轴特征值分解，以惯量盒形式展示

### MDH 参数转换

URDF → Modified DH 参数自动转换，以表格形式显示。支持多运动链机器人，可选择不同运动链分别查看。参数可保存为文本文件。

![MDH 参数表格](assets/gx7-mdh.png)

### 运动学与动力学代码生成

基于 SymPy 符号计算，从 MDH 参数生成：

- **正运动学** C++ 代码（头文件 + 源文件）
- **雅可比矩阵** C++ 代码
- **动力学基回归器** C 代码

代码高亮显示，支持一键复制、保存为文件。同时支持通过 `yaik` 生成解析逆运动学，详见 [解析逆运动学教程](docs/Analytical_IK_Tutorial.md)。

![正运动学代码生成](assets/gx7-mdh-fk.png)

### XML 编辑器

- 语法高亮的 XML 编辑器
- F3 / Shift+F3 查找上一个/下一个
- `π`、`π/2`、`π/4` 快捷插入，精度可调（1-15 位小数）
- 点击"更新"按钮实时同步 3D 视图（无需保存文件）

![XML 编辑器](assets/urdf-editor.png)

### 凸分解碰撞体

- 使用 V-HACD 算法将网格分解为凸包
- 每个连杆独立配置最大凸包数（1-256）
- 分解后自动更新 URDF 文件中的碰撞体定义

![凸分解碰撞体](assets/convex_decomp.png)

### 拓扑图可视化

- 以有向图展示机器人连杆-关节树形结构
- 连杆显示为圆角矩形，关节显示为彩色圆点（按关节类型颜色编码）
- 支持缩放、平移浏览
- 导出为 PNG 或 SVG 格式

### 中英文国际化

- 运行时切换中文 / English，无需重启
- 覆盖 197+ 翻译键，包括所有菜单、按钮、提示信息

## 安装

### 环境要求

- Python 3.8+

### 安装依赖

```bash
pip install numpy sympy pyqt5 vtk anytree transformations trimesh
```

可选依赖（用于 MJCF 格式支持）：

```bash
pip install mujoco
```

## 使用方法

```bash
python main.py
```

启动后可通过以下方式加载模型：

1. 点击"打开模型"按钮选择 `.urdf` 或 `.xml` 文件
2. 直接将文件拖放到窗口中
3. 使用内置示例模型（`descriptions/` 目录下）

加载模型后，可在左侧面板选择运动链、切换可视化选项，在右侧面板通过滑块或拖拽控制关节角度。

## 项目结构

```
URDFly/
├── main.py                     # 主窗口 GUI（PyQt5 三面板布局 + VTK 3D 视图）
├── urdf_parser.py              # URDF 解析（网格 URI 解析、正运动学、运动链构建）
├── mjcf_parser.py              # MuJoCo MJCF 文件解析
├── urdf_vtk_model.py           # VTK 模型封装（STL/OBJ 加载、颜色/透明度管理）
├── geometry_factory.py         # VTK 几何体工厂（盒体、球体、圆柱、胶囊体、坐标轴、文本标签）
├── xml_editor.py               # XML 编辑器（语法高亮、查找替换、π 值插入）
├── mdh_dialog.py               # MDH 参数对话框（表格显示、代码生成、代码高亮）
├── topology_dialog.py          # 拓扑有向图对话框（缩放/平移、PNG/SVG 导出）
├── decomp_dialog.py            # 凸分解配置对话框
├── inertia_visualizer.py       # 质心标记与惯量盒可视化
├── drag_interaction_style.py   # 拖拽交互控制关节角度
├── translations.py             # 中英文国际化（197+ 翻译键）
├── simplify_mesh.py            # 网格凸分解（trimesh + V-HACD）
├── codegen/                    # 代码生成模块
│   ├── forward_kinematics.py   # 符号正运动学（SymPy → C++）
│   ├── jacobian.py             # 雅可比矩阵符号计算
│   ├── dynamic_base_regressor.py  # 动力学基回归器代码生成
│   ├── usage/                  # 使用示例
│   └── sympybotics/            # 第三方符号机器人学库（RNE、Khalil/Park 公式）
├── third_parties/              # 第三方工具
│   ├── urdf2dh.py              # URDF → DH 参数转换（Apache 2.0）
│   ├── kinematics_helpers.py   # 旋转矩阵、齐次变换工具函数
│   ├── geometry_helpers.py     # 几何工具函数
│   └── urdf_helpers.py         # URDF XML 解析辅助
├── descriptions/               # 示例模型
│   ├── gx7/                    # GX7 七轴机械臂（URDF + STL）
│   └── poppy/                  # Poppy 人形机器人（URDF + STL）
├── docs/                       # 文档
│   ├── MDH_Parameters_Tutorial.md
│   └── Analytical_IK_Tutorial.md
└── assets/                     # 截图资源
```

## 文档

- [MDH 参数教程](docs/MDH_Parameters_Tutorial.md)
- [解析逆运动学教程](docs/Analytical_IK_Tutorial.md)

## 已知限制

- URDF 中 mesh 标签的 `scale` 属性暂不支持
- 仅支持 STL 和 OBJ 网格格式
- 仅支持开链机器人（不支持闭链/并联机构）
- MDH 坐标系不随关节滑块实时更新

## 贡献

欢迎提交 Pull Request 和 Issue。

## 许可证

本项目采用 MIT 许可证，详见 LICENSE 文件。

## 致谢

- [sympybotics](https://github.com/cdsousa/SymPyBotics/tree/master/sympybotics) — 符号机器人学计算库
- [urdf_to_dh](https://github.com/mcevoyandy/urdf_to_dh) — URDF 到 DH 参数转换工具

