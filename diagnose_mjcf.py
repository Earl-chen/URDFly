#!/usr/bin/env python3
"""诊断 MJCF 视觉/碰撞体变换问题"""

import sys
import os
import numpy as np
import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mjcf_parser import MJCFParser

def diagnose_geom_transforms(mjcf_file):
    """诊断几何体变换问题"""
    print(f"\n{'='*60}")
    print(f"诊断文件: {mjcf_file}")
    print('='*60)

    parser = MJCFParser(mjcf_file)
    model = parser.model
    data = parser.data

    # 获取 robot_info
    result = parser.get_robot_info()
    (link_names, link_mesh_files, link_mesh_transformations, link_frames,
     link_colors, joint_names, joint_frames, joint_types, joint_axes,
     joint_parent_links, joint_child_links, joint_limits,
     collision_mesh_files, collision_mesh_transformations,
     collision_link_names, collision_geometries) = result

    print(f"\n返回的视觉几何体数量: {len(link_names)}")
    print(f"返回的碰撞几何体数量: {len(collision_geometries)}")
    print(f"MuJoCo 模型中的 geom 数量: {model.ngeom}")

    print("\n[1] 验证视觉几何体变换 (逐个对比):")
    print("-" * 60)

    # 新的验证逻辑：由于现在是每个 geom 一个条目，
    # 需要跳过 PLANE 和 HFIELD 类型的 geom 来对应
    geom_idx = 0
    entry_idx = 0
    all_correct = True

    while geom_idx < model.ngeom and entry_idx < len(link_names):
        geom_type = model.geom_type[geom_idx]

        # 跳过 PLANE 和 HFIELD 类型
        if geom_type in (mujoco.mjtGeom.mjGEOM_PLANE, mujoco.mjtGeom.mjGEOM_HFIELD):
            geom_idx += 1
            continue

        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_idx)
        if geom_name is None:
            geom_name = f"geom_{geom_idx}"

        # MuJoCo 计算的正确位置
        geom_world_pos_mj = data.geom_xpos[geom_idx]
        geom_world_mat_mj = data.geom_xmat[geom_idx].reshape(3, 3)

        # 我们的计算结果
        T_computed = link_mesh_transformations[entry_idx]
        computed_pos = T_computed[:3, 3]
        computed_mat = T_computed[:3, :3]

        pos_error = np.linalg.norm(geom_world_pos_mj - computed_pos)
        rot_error = np.linalg.norm(geom_world_mat_mj - computed_mat)

        if pos_error > 0.001 or rot_error > 0.001:
            print(f"\n⚠ Geom {geom_idx} ({geom_name}):")
            print(f"    MuJoCo 位置: {geom_world_pos_mj}")
            print(f"    计算位置:    {computed_pos}")
            print(f"    位置误差: {pos_error:.6f}, 旋转误差: {rot_error:.6f}")
            all_correct = False

        geom_idx += 1
        entry_idx += 1

    if all_correct:
        print("\n✓ 所有视觉几何体变换正确!")
    else:
        print(f"\n共检测到误差")

    print("\n[2] 验证碰撞几何体变换:")
    print("-" * 60)

    # 同样的验证逻辑
    geom_idx = 0
    coll_idx = 0
    all_coll_correct = True

    while geom_idx < model.ngeom and coll_idx < len(collision_geometries):
        geom_type = model.geom_type[geom_idx]

        # 跳过 PLANE 和 HFIELD 类型
        if geom_type in (mujoco.mjtGeom.mjGEOM_PLANE, mujoco.mjtGeom.mjGEOM_HFIELD):
            geom_idx += 1
            continue

        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_idx)
        if geom_name is None:
            geom_name = f"geom_{geom_idx}"

        geom_world_pos_mj = data.geom_xpos[geom_idx]
        geom_world_mat_mj = data.geom_xmat[geom_idx].reshape(3, 3)

        T_computed = collision_mesh_transformations[coll_idx]
        computed_pos = T_computed[:3, 3]
        computed_mat = T_computed[:3, :3]

        pos_error = np.linalg.norm(geom_world_pos_mj - computed_pos)
        rot_error = np.linalg.norm(geom_world_mat_mj - computed_mat)

        if pos_error > 0.001 or rot_error > 0.001:
            print(f"\n⚠ Collision {coll_idx} ({geom_name}):")
            print(f"    MuJoCo 位置: {geom_world_pos_mj}")
            print(f"    计算位置:    {computed_pos}")
            print(f"    位置误差: {pos_error:.6f}, 旋转误差: {rot_error:.6f}")
            all_coll_correct = False

        geom_idx += 1
        coll_idx += 1

    if all_coll_correct:
        print("\n✓ 所有碰撞几何体变换正确!")

    print(f"\n{'='*60}")
    print("诊断完成")
    print('='*60)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        test_file = sys.argv[1]
    else:
        test_file = "test_models/joint_offset_test.xml"

    if not os.path.exists(test_file):
        print(f"错误: 文件不存在: {test_file}")
        sys.exit(1)

    diagnose_geom_transforms(test_file)
