#!/usr/bin/env python3
"""测试 MJCF 解析器的关节位置计算"""

import sys
import os
import numpy as np
import mujoco

# 添加项目路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mjcf_parser import MJCFParser

def test_mjcf_parser(mjcf_file):
    """测试 MJCF 解析器"""
    print(f"\n{'='*60}")
    print(f"测试文件: {mjcf_file}")
    print('='*60)

    # 1. 加载模型
    print("\n[1] 加载模型...")
    try:
        parser = MJCFParser(mjcf_file)
        print(f"    ✓ 模型加载成功")
        print(f"    - 连杆数量: {len(parser.links)}")
        print(f"    - 关节数量: {len(parser.joints)}")
    except Exception as e:
        print(f"    ✗ 模型加载失败: {e}")
        return False

    # 2. 检查关节信息
    print("\n[2] 关节信息:")
    for i, joint in enumerate(parser.joints):
        print(f"\n    关节 {i}: {joint['name']}")
        print(f"      类型: {joint['type']}")
        print(f"      父连杆: {joint['parent']}")
        print(f"      子连杆: {joint['child']}")
        print(f"      轴向: {joint['axis']}")
        print(f"      位置 (body frame): {joint['origin']['xyz']}")
        print(f"      姿态 (rpy): {joint['origin']['rpy']}")
        print(f"      限制: [{joint['limit']['lower']:.3f}, {joint['limit']['upper']:.3f}]")

    # 3. 验证关节位置计算
    print("\n[3] 验证关节位置计算 (使用 MuJoCo 数据对比):")
    model = parser.model
    data = parser.data

    for i in range(model.njnt):
        jnt_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if jnt_name is None:
            jnt_name = f"joint_{i}"

        # MuJoCo 中的关节位置 (局部坐标系偏移)
        jnt_pos = model.jnt_pos[i]
        body_id = model.jnt_bodyid[i]
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)

        # Body 的世界位置
        body_world_pos = data.xpos[body_id]
        body_world_mat = data.xmat[body_id].reshape(3, 3)

        # 关节在世界坐标系中的真实位置 (考虑 jnt_pos 偏移)
        # 关节位置 = Body世界位置 + Body旋转 * jnt_pos
        true_joint_world_pos = body_world_pos + body_world_mat @ jnt_pos

        print(f"\n    关节: {jnt_name}")
        print(f"      所属 Body: {body_name}")
        print(f"      jnt_pos (局部偏移): {jnt_pos}")
        print(f"      Body 世界位置: {body_world_pos}")
        print(f"      关节真实世界位置: {true_joint_world_pos}")

        # 检查偏移是否为 0
        if np.linalg.norm(jnt_pos) > 1e-6:
            print(f"      ⚠ 警告: 关节位置与 Body 原点不重合! 偏移量: {np.linalg.norm(jnt_pos):.6f}")
        else:
            print(f"      ✓ 关节位置与 Body 原点重合")

    # 4. 获取 robot_info 并验证
    print("\n[4] 调用 get_robot_info():")
    (link_names, link_mesh_files, link_mesh_transformations, link_frames,
     link_colors, joint_names, joint_frames, joint_types, joint_axes,
     joint_parent_links, joint_child_links, joint_limits,
     collision_mesh_files, collision_mesh_transformations,
     collision_link_names, collision_geometries) = parser.get_robot_info()

    print(f"    返回的连杆数量: {len(link_names)}")
    print(f"    返回的关节数量: {len(joint_names)}")

    # 比较关节位置
    print("\n[5] 关节坐标系位置对比 (get_robot_info 返回值 vs 真实值):")
    for i, jname in enumerate(joint_names):
        frame = joint_frames[i]
        pos_from_api = frame[:3, 3]

        # 找到对应的 MuJoCo 关节索引
        mj_jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if mj_jnt_id >= 0:
            jnt_pos = model.jnt_pos[mj_jnt_id]
            body_id = model.jnt_bodyid[mj_jnt_id]
            body_world_pos = data.xpos[body_id]
            body_world_mat = data.xmat[body_id].reshape(3, 3)
            true_pos = body_world_pos + body_world_mat @ jnt_pos

            error = np.linalg.norm(pos_from_api - true_pos)
            print(f"\n    关节: {jname}")
            print(f"      API 返回位置: {pos_from_api}")
            print(f"      真实位置:     {true_pos}")
            print(f"      误差: {error:.6f}")
            if error > 1e-6:
                print(f"      ⚠ 位置误差超过阈值!")

    # 6. 检查视觉几何体
    print("\n[6] 视觉几何体信息:")
    visual_geoms = parser.get_visual_geometries()
    for i, (name, geom) in enumerate(zip(link_names, visual_geoms)):
        if geom is not None:
            print(f"    {name}: {geom['type']}, size={geom['size']}")
        elif link_mesh_files[i] is not None:
            print(f"    {name}: mesh ({link_mesh_files[i]})")
        else:
            print(f"    {name}: 无视觉几何体")

    print(f"\n{'='*60}")
    print("测试完成")
    print('='*60)
    return True

if __name__ == "__main__":
    # 使用 MuJoCo 自带的测试模型
    default_test = "/home/cm/.local/lib/python3.10/site-packages/mujoco/testdata/model.xml"

    if len(sys.argv) > 1:
        test_file = sys.argv[1]
    else:
        test_file = default_test

    if not os.path.exists(test_file):
        print(f"错误: 文件不存在: {test_file}")
        sys.exit(1)

    test_mjcf_parser(test_file)
