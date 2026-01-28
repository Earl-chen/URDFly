#!/usr/bin/env python3
"""详细对比 MJCF 中 geom 的变换"""

import sys
import os
import numpy as np
import mujoco

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mjcf_parser import MJCFParser

def debug_transforms(mjcf_file):
    """详细输出变换信息"""
    print(f"文件: {mjcf_file}")
    print("="*80)

    parser = MJCFParser(mjcf_file)
    model = parser.model
    data = parser.data

    # 获取 robot_info
    result = parser.get_robot_info()
    (link_names, link_mesh_files, link_mesh_transformations, link_frames,
     link_colors, *_) = result

    print(f"\n返回条目数量: {len(link_names)}")
    print(f"MuJoCo geom 数量: {model.ngeom}")

    # 遍历每个 geom
    print("\n" + "="*80)
    print("MuJoCo 原始数据 vs 返回数据:")
    print("="*80)

    entry_idx = 0
    for g in range(model.ngeom):
        geom_type = model.geom_type[g]

        # 跳过 PLANE 和 HFIELD
        if geom_type in (mujoco.mjtGeom.mjGEOM_PLANE, mujoco.mjtGeom.mjGEOM_HFIELD):
            continue

        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
        body_id = model.geom_bodyid[g]
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)

        # MuJoCo 数据
        mj_geom_xpos = data.geom_xpos[g]
        mj_geom_xmat = data.geom_xmat[g].reshape(3, 3)
        mj_body_xpos = data.xpos[body_id]
        mj_body_xmat = data.xmat[body_id].reshape(3, 3)

        # 本地偏移
        local_pos = model.geom_pos[g]
        local_quat = model.geom_quat[g]

        # 我们返回的数据
        if entry_idx < len(link_names):
            our_name = link_names[entry_idx]
            our_transform = link_mesh_transformations[entry_idx]
            our_pos = our_transform[:3, 3]
            our_frame = link_frames[entry_idx]
            our_frame_pos = our_frame[:3, 3]

        # 只显示 mesh 类型的 geom
        if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
            mesh_id = model.geom_dataid[g]
            mesh_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id) if mesh_id >= 0 else "N/A"

            print(f"\n--- Geom {g}: {geom_name} (body: {body_name}, mesh: {mesh_name}) ---")
            print(f"  Body 世界位置:    {mj_body_xpos}")
            print(f"  Geom 局部位置:    {local_pos}")
            print(f"  Geom 局部四元数:  {local_quat}")
            print(f"  Geom 世界位置:    {mj_geom_xpos}")
            print(f"  我们返回的位置:   {our_pos}")
            print(f"  link_frame 位置:  {our_frame_pos}")

            pos_diff = np.linalg.norm(mj_geom_xpos - our_pos)
            if pos_diff > 0.001:
                print(f"  ⚠ 位置差异: {pos_diff:.6f}")

        entry_idx += 1


if __name__ == "__main__":
    if len(sys.argv) > 1:
        test_file = sys.argv[1]
    else:
        test_file = "/home/cm/work/Robot/Humaniod/1_HighTorque/0_RL/HighTorque_Pi_Isaaclab/LeggedLab/legged_lab/robot/pai/mjcf/pi_12dof_release_v1.xml"

    debug_transforms(test_file)
