# URDFly

[English](README.md) | 中文

![GX7 Robot Arm](assets/gx7.png)
![Poppy Robot](assets/poppy.png)

## 项目简介

URDFly 是一个基于 Python 的机器人模型工具箱，专注于 URDF（统一机器人描述格式）和 MJCF（MuJoCo 格式）文件的解析、可视化和分析。该项目为机械臂的建模、仿真和控制提供了一套完整的工具链，核心功能包括：

- 多坐标系可视化（连杆坐标系、MDH 坐标系）
- URDF 实时编辑与预览
- MDH 参数自动提取与转换
- 正运动学和动力学代码生成
- 碰撞体凸分解

## 功能特性

### 文件支持

| 格式 | 扩展名 | 支持程度 |
|------|--------|----------|
| URDF | `.urdf` | 完全支持 |
| MJCF | `.xml` | 部分支持 (MuJoCo 格式) |
| STL 网格 | `.stl` | 完全支持 |
| OBJ 网格 | `.obj` | 完全支持 |

### 可视化功能

| 功能 | 说明 |
|------|------|
| 视觉模型显示 | 显示机器人的外观网格 |
| 连杆坐标系 | 每个连杆的 XYZ 坐标轴（RGB 对应 XYZ）|
| MDH 坐标系 | Modified DH 参数对应的坐标系 |
| 关节轴显示 | 旋转关节的旋转轴方向（黄色箭头）|
| 碰撞体显示 | 支持 Box / Sphere / Cylinder / Mesh |
| 质心显示 | 每个连杆的质心位置（CoM）|
| 惯量可视化 | 惯量椭球的近似长方体（支持主轴分解）|
| 透明度控制 | 0-100% 可调节 |
| 连杆高亮 | 点击列表中的连杆名称高亮显示 |

### 关节控制

- 滑块实时调节每个关节角度
- 文本框批量输入关节角度
- 一键重置所有关节到零位
- 随机生成关节角度
- 弧度/度数单位切换

### 编辑与工具

- **XML 编辑器**：实时编辑 URDF 并预览效果
- **MDH 参数**：自动提取和显示 MDH 参数表
- **凸分解**：使用 VHACD 算法分解碰撞网格
- **代码生成**：生成正运动学和动力学回归器代码

### 界面特性

- 中英文双语界面切换
- 拖放文件加载
- 运动链选择（支持多链机器人）

## 亮点功能

### URDF 可视化

URDFly 提供强大的 3D 可视化功能：

- 支持 `stl` 和 `obj` 网格文件格式
- 可调节透明度和多种坐标系同时显示
- 点击连杆列表即可高亮对应连杆
- 支持原始连杆坐标系和 M-DH 坐标系对比显示
- 虚拟连杆以红色球体标识
- 碰撞体可视化

![碰撞体可视化](assets/gx7_collision.png)

### MDH 参数转换

项目内置 URDF 到 Modified DH (MDH) 参数的自动转换工具，支持多运动链机器人：

![MDH 参数表](assets/gx7-mdh.png)

MDH 参数可用于：
- 计算机器人正运动学
- 生成动力学回归器（用于动力学参数辨识）

![正运动学代码生成](assets/gx7-mdh-fk.png)

MDH 参数还可以配合 `yaik` 工具生成解析形式的逆运动学解，详见 [解析逆运动学教程](docs/Analytical_IK_Tutorial.md)。

### URDF 编辑器

内置的 URDF 编辑器让您可以实时修改和预览机器人模型：

1. 点击 `Edit URDF` 按钮打开编辑器
2. 修改 URDF 内容
3. 点击 `Update` 按钮即时预览修改效果

编辑器特性：
- 语法高亮
- 搜索功能
- 自动补全 `pi` 的不同精度表示

![URDF 编辑器](assets/urdf-editor.png)

### 凸分解

URDFly 提供碰撞网格的凸分解功能，这对于物理仿真和碰撞检测非常有用：

1. 点击 `Decompose As Collision` 按钮
2. 设置 `maxConvexHulls` 参数（最大凸包数量）
3. 点击 `OK` 执行分解（需要一些时间）
4. 分解完成后 URDF 编辑器会弹出，点击 `Update` 预览结果

![凸分解结果](assets/convex_decomp.png)

### MuJoCo MJCF 支持

URDFly 新增了对 MuJoCo MJCF (.xml) 格式的支持：

- 解析 MuJoCo 模型文件
- 支持基本几何体（Box、Sphere、Cylinder、Capsule）的可视化
- 支持网格文件加载
- 与 URDF 共用同一套可视化和关节控制界面

**注意**：MJCF 支持目前为实验性功能，部分高级特性可能尚未完全支持。

## 安装说明

### 环境要求

- Python 3.8+
- 支持 OpenGL 的显示环境

### 依赖安装

```bash
pip install numpy sympy pyqt5 vtk anytree transformations trimesh mujoco
```

或使用 requirements.txt：

```bash
pip install -r requirements.txt
```

**注意**：VTK 和 PyQt5 可能需要额外的系统依赖，请根据您的操作系统参考相关文档。

## 使用方法

### 启动程序

```bash
python main.py
```

### 加载模型

- **方式一**：直接将 `.urdf` 或 `.xml` 文件拖放到窗口
- **方式二**：使用菜单 `File > Open` 选择文件
- **方式三**：命令行指定文件路径

```bash
python main.py path/to/your/robot.urdf
```

### 基本操作

1. **视角控制**：鼠标左键旋转，中键平移，滚轮缩放
2. **关节调节**：使用右侧滑块调节关节角度
3. **坐标系显示**：勾选对应选项显示/隐藏坐标系
4. **透明度**：拖动透明度滑块调节网格透明度
5. **语言切换**：菜单栏切换中英文界面

## 项目结构

```
URDFly/
├── main.py                 # 主程序入口
├── urdf_parser.py          # URDF 文件解析器
├── mjcf_parser.py          # MJCF 文件解析器 (MuJoCo)
├── urdf_vtk_model.py       # VTK 可视化模型
├── mdh_dialog.py           # MDH 参数转换对话框
├── xml_editor.py           # XML 编辑器组件
├── translations.py         # 多语言翻译
├── codegen/                # 代码生成模块
│   ├── forward_kinematics.py      # 正运动学代码生成
│   ├── dynamic_base_regressor.py  # 动力学回归器生成
│   └── sympybotics/               # 符号计算库
├── descriptions/           # 示例机器人模型
│   ├── gx7/               # GX7 七轴机械臂
│   └── poppy/             # Poppy 人形机器人
├── docs/                   # 文档目录
├── assets/                 # 截图和资源文件
└── third_parties/          # 第三方工具
```

## 文档

更多详细信息请参考 `docs/` 目录下的文档：

- [MDH 参数教程](docs/MDH_Parameters_Tutorial.md) - 详解 MDH 参数的提取和使用
- [解析逆运动学教程](docs/Analytical_IK_Tutorial.md) - 使用 yaik 生成解析逆运动学

## 已知限制

- URDF 中 mesh 标签的 `scale` 属性暂不支持
- 仅支持 `.stl` 和 `.obj` 网格格式
- 仅支持开链机器人（不支持闭链）
- MDH 坐标系不会随关节滑块变化而更新
- MJCF 支持为实验性功能，部分特性可能不完整

## 贡献指南

欢迎贡献代码！请随时提交 Pull Request。

贡献方式：
1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m '添加某个特性'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 提交 Pull Request

## 许可证

本项目基于 MIT 许可证开源。

## 致谢

本项目参考和使用了以下开源项目：

- [sympybotics](https://github.com/cdsousa/SymPyBotics/tree/master/sympybotics) - 符号机器人学计算库
- [urdf_to_dh](https://github.com/mcevoyandy/urdf_to_dh) - URDF 到 DH 参数转换
- [VHACD](https://github.com/kmammou/v-hacd) - 凸分解算法
- [VTK](https://vtk.org/) - 3D 可视化工具包
- [MuJoCo](https://mujoco.org/) - 物理仿真引擎

## 联系方式

如果您对深度合作或二次开发感兴趣，欢迎联系我。

Email: <yunlongdong@outlook.com>

---

如果这个项目对您有帮助，欢迎 Star 支持！
