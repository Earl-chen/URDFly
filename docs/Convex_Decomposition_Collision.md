# 凸分解碰撞体 — 功能流程与实现分析

## 概述

凸分解碰撞体功能将机器人 URDF 中原始的高面数碰撞 mesh（通常是视觉 mesh 的副本）替换为经 V-HACD 算法分解后的低复杂度凸包近似 mesh，以提升物理仿真中碰撞检测的性能。

---

## 调用链总览

```
用户点击 菜单 Tools → "凸分解碰撞体"
    │
    ▼
main.py  act_decomp.triggered → decompose_collision_meshes()
    │  ① 检查 self.collision_mesh_files 是否存在
    │  ② 创建 DecompDialog 对话框（传入碰撞网格路径列表）
    │  ③ dialog.exec_() 阻塞等待用户操作
    │
    ▼
decomp_dialog.py  用户点击 OK → accept() → perform_decomposition()
    │  逐个 mesh 调用 create_detailed_approximation()
    │
    ▼
simplify_mesh.py  create_detailed_approximation()
    │  trimesh.load() → mesh.convex_decomposition(V-HACD) → 合并 → 导出 *_approx.STL
    │
    ▼  返回 decomposed_mesh_files
main.py  edit_urdf_file(replace_collision=True)
    │  打开 XML 编辑器 → 自动执行 replace_collision()
    │
    ▼
xml_editor.py  replace_collision()
    │  正则替换 URDF 中 <collision> 内的 mesh filename：
    │  Link6.STL → Link6_approx.STL
    │
    ▼  用户在编辑器中点"更新"→ 重新加载模型
```

---

## 逐阶段详解

### 阶段 1：数据准备 — collision_mesh_files 的来源

**位置**：`main.py` — `load_urdf_file()` 方法

```python
self.collision_mesh_files = [f for f in link_mesh_files if f is not None]
```

在加载 URDF 时，`parser.get_robot_info()` 返回 16 元组，其中 `collision_mesh_files`（索引 11）是所有碰撞体 mesh 文件的绝对路径列表。这里过滤掉 `None`（对应没有 mesh 只有基本几何体如 box/sphere/cylinder 的碰撞体），存入 `self.collision_mesh_files`。

这个列表就是后续凸分解的 **输入源**。

---

### 阶段 2：用户交互 — DecompDialog

**位置**：`main.py` — `decompose_collision_meshes()` + `decomp_dialog.py`

```python
def decompose_collision_meshes(self):
    if not self.collision_mesh_files:    # 守卫：没有碰撞 mesh 直接弹警告
        return
    dialog = DecompDialog(self, self.collision_mesh_files)
    decomposed_mesh_files = dialog.exec_()  # 阻塞
    if decomposed_mesh_files is not None:
        self.edit_urdf_file(replace_collision=True)
```

`DecompDialog` 的职责：

1. **表格展示**（`decomp_dialog.py` `init_ui()`）：每个碰撞 mesh 一行，左列文件名（只读），右列是 `QSpinBox`（可调 `maxConvexHulls`，范围 1–256，默认 32）
2. **参数存储**：`self.max_convex_hulls` 字典，key 是 mesh 完整路径，value 是用户设定的最大凸包数
3. **确认触发**（`accept()`）：点 OK → 如果还没分解就调 `perform_decomposition()`

---

### 阶段 3：核心算法 — V-HACD 凸分解

**位置**：`decomp_dialog.py` — `perform_decomposition()` → `simplify_mesh.py` — `create_detailed_approximation()`

```python
def perform_decomposition(self):
    for mesh_file, max_hulls in decomp_params.items():
        decomposed_files = create_detailed_approximation(
            [mesh_file], maxConvexHulls=max_hulls
        )
```

`create_detailed_approximation()` 对 **每个 mesh** 做：

| 步骤 | 操作 |
|------|------|
| 1 | `trimesh.load(mesh_file)` — 加载原始 STL/OBJ |
| 2 | `mesh.convex_decomposition(maxConvexHulls=N)` — 调用 V-HACD 算法，将非凸网格分解为最多 N 个凸包片段 |
| 3 | 如果返回多片段：遍历所有 piece，拼接 vertices + faces（修正 face 索引偏移），合并为单一 `Trimesh` |
| 4 | 异常回退：V-HACD 失败则降级为 `mesh.convex_hull`（单凸包近似） |
| 5 | 输出文件命名 `{原名}_approx.{原扩展名}`，导出到原目录同级 |

#### V-HACD 算法简介

V-HACD（Volumetric Hierarchical Approximate Convex Decomposition）是 trimesh 内置的凸分解后端，核心思想：

- 将 3D 网格体素化
- 递归地沿最佳切割平面分割
- 每个子块计算凸包
- `maxConvexHulls` 控制分解精度：越大越精确但碰撞检测越慢

---

### 阶段 4：URDF 文本替换

**位置**：`main.py` — `edit_urdf_file(replace_collision=True)` → `xml_editor.py` — `replace_collision()`

分解完成后：

1. 打开 XML 编辑器，加载当前 URDF 文件
2. 自动调用 `replace_collision()`

正则替换逻辑：

```python
pattern = r'(<collision>.*?<mesh\s+filename=")([^"]+)(".*?</collision>)'
```

对 URDF 中每个 `<collision>...</collision>` 块内的 `<mesh filename="...">` 执行：

- `../meshes/Link6.STL` → `../meshes/Link6_approx.STL`
- 已有 `_approx` 后缀的跳过（幂等）

替换结果写入编辑器文本区，用户可以 **审查修改后的 XML**，然后点"更新"按钮 → 触发 `update_model_from_xml()` → 重新解析 + 渲染。

---

## 数据流图

```
原始碰撞 mesh 文件                  URDF XML 文件
  Link1.STL                     <collision>
  Link2.STL                       <mesh filename="Link1.STL"/>
  Link3.STL                     </collision>
      │                               │
      ▼  V-HACD 凸分解                │
  Link1_approx.STL               (磁盘上已存在)
  Link2_approx.STL                    │
  Link3_approx.STL                    ▼  正则替换
      │                         <collision>
      └─── 写入磁盘 ──→            <mesh filename="Link1_approx.STL"/>
                                </collision>
                                      │
                                      ▼  用户点"更新"
                                重新解析 → VTK 渲染新碰撞体
```

---

## 涉及文件

| 文件 | 职责 |
|------|------|
| `main.py` | 入口 `decompose_collision_meshes()`，收集 collision_mesh_files，调用对话框，触发编辑器替换 |
| `decomp_dialog.py` | 用户交互对话框，参数配置（每个 mesh 独立 maxConvexHulls），触发分解 |
| `simplify_mesh.py` | 核心算法，trimesh + V-HACD 凸分解，输出 `*_approx` 文件 |
| `xml_editor.py` | `replace_collision()` 正则替换 URDF 中碰撞 mesh 文件名 |

---

## 设计特点

1. **非破坏性**：原始 mesh 不被修改，新文件加 `_approx` 后缀并存
2. **逐 mesh 独立参数**：每个碰撞体可以设不同的 `maxConvexHulls`，精细控制精度/性能平衡
3. **优雅降级**：V-HACD 失败自动回退为单凸包
4. **人工审查环节**：替换后不直接生效，而是在 XML 编辑器中展示，给用户检查和修改的机会
5. **幂等替换**：已有 `_approx` 的文件名不会重复添加后缀
