import numpy as np
import math
import os
import xml.etree.ElementTree as ET

import mujoco
from anytree import Node


class MJCFParser:
    """Class to parse MuJoCo MJCF (.xml) files and extract link and joint information.

    Provides the same get_robot_info() interface as URDFParser, enabling reuse of the
    existing VTK rendering logic.
    """

    def __init__(self, mjcf_file):
        self.mjcf_file = mjcf_file
        self.mesh_dir = os.path.dirname(mjcf_file)
        self.links = {}
        self.joints = []

        # Load model using mujoco
        self.model = mujoco.MjModel.from_xml_path(mjcf_file)
        self.data = mujoco.MjData(self.model)

        # Run forward kinematics at default pose
        mujoco.mj_forward(self.model, self.data)

        # Parse the meshdir from XML compiler options
        self._meshdir = self._parse_meshdir()

        # Parse model structure
        self._parse_bodies()
        self._parse_joints()

        # 缓存几何体索引分类，用于 update_transforms() 快速更新
        self._visual_geom_indices = []  # (geom_idx, body_id, is_mesh)
        self._collision_geom_indices = []  # (geom_idx, body_id, geom_type)
        self._cache_geom_indices()

    def _parse_meshdir(self):
        """Parse the meshdir compiler option from the MJCF XML file."""
        try:
            tree = ET.parse(self.mjcf_file)
            root = tree.getroot()
            compiler = root.find('compiler')
            if compiler is not None:
                meshdir = compiler.get('meshdir', '')
                if meshdir:
                    return os.path.join(self.mesh_dir, meshdir)
        except Exception:
            pass
        return self.mesh_dir

    def _parse_bodies(self):
        """Parse all bodies (links) from the MuJoCo model."""
        for i in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            if name is None:
                name = f"body_{i}"

            # Get visual geom info for this body
            visual_info = self._get_body_visual_geom(i)
            collision_info = self._get_body_collision_geom(i)
            inertial_info = self._get_body_inertial(i)

            if visual_info is not None:
                self.links[name] = {
                    'mesh': visual_info.get('mesh_file'),
                    'origin': visual_info.get('origin', {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}),
                    'color': visual_info.get('color', [0.5, 0.5, 0.5, 1.0]),
                    'geom_type': visual_info.get('geom_type'),
                    'geom_size': visual_info.get('geom_size'),
                }
            else:
                self.links[name] = {
                    'mesh': None,
                }

            # Add collision info
            if collision_info is not None:
                self.links[name]['collision_type'] = collision_info.get('collision_type')
                self.links[name]['collision_mesh'] = collision_info.get('collision_mesh')
                self.links[name]['collision_origin'] = collision_info.get('collision_origin',
                                                                          {"xyz": [0, 0, 0], "rpy": [0, 0, 0]})
                if collision_info.get('collision_type') == 'box':
                    self.links[name]['collision_box_size'] = collision_info.get('collision_box_size', [0, 0, 0])
                elif collision_info.get('collision_type') == 'sphere':
                    self.links[name]['collision_sphere_radius'] = collision_info.get('collision_sphere_radius', 0)
                elif collision_info.get('collision_type') == 'cylinder':
                    self.links[name]['collision_cylinder_radius'] = collision_info.get('collision_cylinder_radius', 0)
                    self.links[name]['collision_cylinder_length'] = collision_info.get('collision_cylinder_length', 0)
            else:
                self.links[name]['collision_type'] = None
                self.links[name]['collision_mesh'] = None

            # Add inertial info
            self.links[name]['inertial'] = inertial_info

    def _get_body_visual_geom(self, body_id):
        """Get visual geometry information for a body."""
        # Find geoms belonging to this body
        for g in range(self.model.ngeom):
            if self.model.geom_bodyid[g] != body_id:
                continue

            # Check contype/conaffinity - visual geoms typically have contype=0, conaffinity=0
            # But in practice, we just take the first geom that has a visual representation
            geom_group = self.model.geom_group[g]
            # Group 0 is typically visual, group 3 is collision in many models
            # But we should be flexible - try to find mesh geoms first
            geom_type = self.model.geom_type[g]
            rgba = self.model.geom_rgba[g].tolist()

            if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
                mesh_id = self.model.geom_dataid[g]
                if mesh_id >= 0:
                    mesh_file = self._get_mesh_file(mesh_id)
                    geom_pos = self.model.geom_pos[g].tolist()
                    geom_quat = self.model.geom_quat[g].tolist()  # (w, x, y, z)
                    rpy = self._quat_to_rpy(geom_quat)
                    return {
                        'mesh_file': mesh_file,
                        'origin': {"xyz": geom_pos, "rpy": rpy},
                        'color': rgba if any(c > 0 for c in rgba[:3]) else [0.5, 0.5, 0.5, 1.0],
                        'geom_type': 'mesh',
                    }
            elif geom_type in (mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_SPHERE,
                               mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE,
                               mujoco.mjtGeom.mjGEOM_ELLIPSOID):
                # For primitive geoms, we don't have a mesh file
                # but we record the info for potential rendering
                geom_pos = self.model.geom_pos[g].tolist()
                geom_quat = self.model.geom_quat[g].tolist()
                rpy = self._quat_to_rpy(geom_quat)
                size = self.model.geom_size[g].tolist()
                type_name = self._geom_type_name(geom_type)
                return {
                    'mesh_file': None,
                    'origin': {"xyz": geom_pos, "rpy": rpy},
                    'color': rgba if any(c > 0 for c in rgba[:3]) else [0.5, 0.5, 0.5, 1.0],
                    'geom_type': type_name,
                    'geom_size': size,
                }
        return None

    def _get_body_collision_geom(self, body_id):
        """Get collision geometry information for a body."""
        for g in range(self.model.ngeom):
            if self.model.geom_bodyid[g] != body_id:
                continue

            geom_type = self.model.geom_type[g]
            geom_pos = self.model.geom_pos[g].tolist()
            geom_quat = self.model.geom_quat[g].tolist()
            rpy = self._quat_to_rpy(geom_quat)
            size = self.model.geom_size[g].tolist()

            collision_origin = {"xyz": geom_pos, "rpy": rpy}

            if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
                mesh_id = self.model.geom_dataid[g]
                if mesh_id >= 0:
                    mesh_file = self._get_mesh_file(mesh_id)
                    return {
                        'collision_type': 'mesh',
                        'collision_mesh': mesh_file,
                        'collision_origin': collision_origin,
                    }
            elif geom_type == mujoco.mjtGeom.mjGEOM_BOX:
                return {
                    'collision_type': 'box',
                    'collision_mesh': None,
                    'collision_origin': collision_origin,
                    'collision_box_size': [s * 2 for s in size[:3]],  # MuJoCo uses half-sizes
                }
            elif geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
                return {
                    'collision_type': 'sphere',
                    'collision_mesh': None,
                    'collision_origin': collision_origin,
                    'collision_sphere_radius': size[0],
                }
            elif geom_type == mujoco.mjtGeom.mjGEOM_CYLINDER:
                return {
                    'collision_type': 'cylinder',
                    'collision_mesh': None,
                    'collision_origin': collision_origin,
                    'collision_cylinder_radius': size[0],
                    'collision_cylinder_length': size[1] * 2,  # MuJoCo uses half-length
                }
            elif geom_type == mujoco.mjtGeom.mjGEOM_CAPSULE:
                # Approximate capsule as cylinder
                return {
                    'collision_type': 'cylinder',
                    'collision_mesh': None,
                    'collision_origin': collision_origin,
                    'collision_cylinder_radius': size[0],
                    'collision_cylinder_length': size[1] * 2,
                }
        return None

    def _get_body_inertial(self, body_id):
        """Get inertial properties for a body."""
        mass = self.model.body_mass[body_id]
        if mass <= 0:
            return None

        # Center of mass position relative to body frame
        ipos = self.model.body_ipos[body_id].tolist()
        iquat = self.model.body_iquat[body_id].tolist()
        rpy = self._quat_to_rpy(iquat)

        # Inertia (diagonal in principal frame)
        inertia_diag = self.model.body_inertia[body_id]

        return {
            'origin': {"xyz": ipos, "rpy": rpy},
            'mass': float(mass),
            'inertia': {
                "ixx": float(inertia_diag[0]),
                "ixy": 0.0,
                "ixz": 0.0,
                "iyy": float(inertia_diag[1]),
                "iyz": 0.0,
                "izz": float(inertia_diag[2]),
            }
        }

    def _get_mesh_file(self, mesh_id):
        """Get the file path for a mesh asset."""
        mesh_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_MESH, mesh_id)
        if mesh_name is None:
            return None

        # Try to find the mesh file in the meshdir
        # MuJoCo supports STL and OBJ formats
        for ext in ['.stl', '.STL', '.obj', '.OBJ', '.msh']:
            mesh_path = os.path.join(self._meshdir, mesh_name + ext)
            if os.path.exists(mesh_path):
                return mesh_path

        # Also try the mesh_name directly as a file path
        mesh_path = os.path.join(self._meshdir, mesh_name)
        if os.path.exists(mesh_path):
            return mesh_path

        # Try parsing the XML to find the actual file attribute
        try:
            tree = ET.parse(self.mjcf_file)
            root = tree.getroot()
            for mesh_elem in root.iter('mesh'):
                if mesh_elem.get('name') == mesh_name:
                    mesh_file = mesh_elem.get('file')
                    if mesh_file:
                        full_path = os.path.join(self._meshdir, mesh_file)
                        if os.path.exists(full_path):
                            return full_path
        except Exception:
            pass

        return None

    def _geom_type_name(self, geom_type):
        """Convert MuJoCo geom type enum to string name."""
        type_map = {
            mujoco.mjtGeom.mjGEOM_PLANE: 'plane',
            mujoco.mjtGeom.mjGEOM_HFIELD: 'hfield',
            mujoco.mjtGeom.mjGEOM_SPHERE: 'sphere',
            mujoco.mjtGeom.mjGEOM_CAPSULE: 'capsule',
            mujoco.mjtGeom.mjGEOM_ELLIPSOID: 'ellipsoid',
            mujoco.mjtGeom.mjGEOM_CYLINDER: 'cylinder',
            mujoco.mjtGeom.mjGEOM_BOX: 'box',
            mujoco.mjtGeom.mjGEOM_MESH: 'mesh',
        }
        return type_map.get(geom_type, 'unknown')

    def _map_joint_type(self, mj_joint_type):
        """Map MuJoCo joint type to URDF joint type string."""
        if mj_joint_type == mujoco.mjtJoint.mjJNT_HINGE:
            return 'revolute'
        elif mj_joint_type == mujoco.mjtJoint.mjJNT_SLIDE:
            return 'prismatic'
        elif mj_joint_type == mujoco.mjtJoint.mjJNT_BALL:
            return 'ball'
        elif mj_joint_type == mujoco.mjtJoint.mjJNT_FREE:
            return 'floating'
        return 'fixed'

    def _parse_joints(self):
        """Parse all joints from the MuJoCo model."""
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name is None:
                name = f"joint_{i}"

            joint_type = self._map_joint_type(self.model.jnt_type[i])

            # Get parent body (the body the joint belongs to)
            body_id = self.model.jnt_bodyid[i]
            child_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            if child_name is None:
                child_name = f"body_{body_id}"

            # Parent body
            parent_body_id = self.model.body_parentid[body_id]
            parent_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, parent_body_id)
            if parent_name is None:
                parent_name = f"body_{parent_body_id}"

            # Joint axis (in body frame)
            axis = self.model.jnt_axis[i].tolist()

            # Joint position relative to body
            jnt_pos = self.model.jnt_pos[i].tolist()

            # Joint limits
            limited = self.model.jnt_limited[i]
            if limited:
                lower = float(self.model.jnt_range[i][0])
                upper = float(self.model.jnt_range[i][1])
            else:
                lower = -math.pi
                upper = math.pi

            # Joint origin: use body position relative to parent
            body_pos = self.model.body_pos[body_id].tolist()
            body_quat = self.model.body_quat[body_id].tolist()  # (w, x, y, z)
            rpy = self._quat_to_rpy(body_quat)

            self.joints.append({
                "name": name,
                "type": joint_type,
                "parent": parent_name,
                "child": child_name,
                "origin": {"xyz": body_pos, "rpy": rpy},
                "axis": axis,
                "limit": {
                    "lower": lower,
                    "upper": upper,
                    "effort": 0.0,
                    "velocity": 0.0,
                },
            })

    @staticmethod
    def _quat_to_rpy(quat_wxyz):
        """Convert quaternion (w, x, y, z) to roll-pitch-yaw (rpy)."""
        w, x, y, z = quat_wxyz

        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return [roll, pitch, yaw]

    @staticmethod
    def _quat_to_rotmat(quat_wxyz):
        """Convert quaternion (w, x, y, z) to 3x3 rotation matrix."""
        w, x, y, z = quat_wxyz
        R = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
            [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
        ])
        return R

    def compute_transformation(self, rpy, xyz):
        """Compute 4x4 transformation matrix from RPY and XYZ (same as URDFParser)."""
        roll, pitch, yaw = rpy

        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)],
        ])

        Ry = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)],
        ])

        Rz = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1],
        ])

        R = Rz @ Ry @ Rx

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = xyz

        return T

    def get_robot_info(self, qs=None):
        """Return robot information in the same format as URDFParser.get_robot_info().

        Uses MuJoCo's forward kinematics to compute all body/geom positions.
        Now properly handles multiple geoms per body.
        """
        # Set joint positions if provided
        if qs is not None:
            # Map revolute joint values to qpos
            rev_idx = 0
            for i in range(self.model.njnt):
                if self.model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE:
                    if rev_idx < len(qs):
                        qpos_addr = self.model.jnt_qposadr[i]
                        self.data.qpos[qpos_addr] = qs[rev_idx]
                        rev_idx += 1
        else:
            # Reset to default pose
            mujoco.mj_resetData(self.model, self.data)

        # Run forward kinematics
        mujoco.mj_forward(self.model, self.data)

        link_names = []
        link_mesh_files = []
        link_mesh_transformations = []
        link_frames = []
        link_colors = []

        collision_mesh_files = []
        collision_mesh_transformations = []
        collision_link_names = []
        collision_geometries = []

        joint_names = []
        joint_frames = []
        joint_types = []
        joint_axes = []
        joint_parent_links = []
        joint_child_links = []
        joint_limits = []

        # Store visual geometry info for primitive shapes
        self._visual_geometries = []

        # Process each geom (not body) for visual geometry
        # This properly handles multiple geoms per body
        for g in range(self.model.ngeom):
            # 跳过碰撞几何体 - 只处理视觉几何体 (contype=0 表示不参与碰撞)
            contype = self.model.geom_contype[g]
            if contype != 0:
                continue  # 这是碰撞几何体，跳过

            body_id = self.model.geom_bodyid[g]
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            if body_name is None:
                body_name = f"body_{body_id}"

            # Get geom world transformation directly from MuJoCo data
            geom_pos = self.data.geom_xpos[g]
            geom_mat = self.data.geom_xmat[g].reshape(3, 3)

            T_geom = np.eye(4)
            T_geom[:3, :3] = geom_mat
            T_geom[:3, 3] = geom_pos

            # Get body world transformation for link_frames
            body_pos = self.data.xpos[body_id]
            body_mat = self.data.xmat[body_id].reshape(3, 3)

            T_body = np.eye(4)
            T_body[:3, :3] = body_mat
            T_body[:3, 3] = body_pos

            # Get geom type and properties
            geom_type = self.model.geom_type[g]
            rgba = self.model.geom_rgba[g].tolist()
            color = rgba if any(c > 0 for c in rgba[:3]) else [0.5, 0.5, 0.5, 1.0]
            size = self.model.geom_size[g].tolist()

            if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
                # Mesh geometry - use BODY transform, not geom transform!
                # MuJoCo's geom_xpos/xmat represents the mesh's geometric center,
                # but the mesh file vertices are relative to the body origin.
                mesh_id = self.model.geom_dataid[g]
                if mesh_id >= 0:
                    mesh_file = self._get_mesh_file(mesh_id)
                else:
                    mesh_file = None
                self._visual_geometries.append(None)
                # Use T_body for mesh transformation
                link_mesh_transformations.append(T_body)
            elif geom_type in (mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_SPHERE,
                               mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE,
                               mujoco.mjtGeom.mjGEOM_ELLIPSOID):
                # Primitive geometry - use geom transform (center is at geom_xpos)
                mesh_file = None
                type_name = self._geom_type_name(geom_type)
                self._visual_geometries.append({
                    'type': type_name,
                    'size': size,
                })
                # Use T_geom for primitive geometry transformation
                link_mesh_transformations.append(T_geom)
            elif geom_type == mujoco.mjtGeom.mjGEOM_PLANE:
                # Skip ground plane for visual
                continue
            elif geom_type == mujoco.mjtGeom.mjGEOM_HFIELD:
                # Skip heightfield for visual
                continue
            else:
                # Unknown or unsupported geometry type
                mesh_file = None
                color = None
                self._visual_geometries.append(None)
                link_mesh_transformations.append(T_body)

            link_names.append(body_name)
            link_mesh_files.append(mesh_file)
            link_frames.append(T_body)
            link_colors.append(color)

        # Process collision geometries (same approach - iterate over geoms)
        for g in range(self.model.ngeom):
            # 只处理碰撞几何体 - 跳过视觉几何体 (contype=0 表示不参与碰撞)
            contype = self.model.geom_contype[g]
            if contype == 0:
                continue  # 这是视觉几何体，跳过

            body_id = self.model.geom_bodyid[g]
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            if body_name is None:
                body_name = f"body_{body_id}"

            geom_type = self.model.geom_type[g]

            # Skip ground plane for collision
            if geom_type == mujoco.mjtGeom.mjGEOM_PLANE:
                continue

            # Get geom world transformation
            geom_pos = self.data.geom_xpos[g]
            geom_mat = self.data.geom_xmat[g].reshape(3, 3)

            T_geom = np.eye(4)
            T_geom[:3, :3] = geom_mat
            T_geom[:3, 3] = geom_pos

            # Get body world transformation
            body_pos = self.data.xpos[body_id]
            body_mat = self.data.xmat[body_id].reshape(3, 3)

            T_body = np.eye(4)
            T_body[:3, :3] = body_mat
            T_body[:3, 3] = body_pos

            size = self.model.geom_size[g].tolist()

            if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
                # Mesh collision - use body transform
                mesh_id = self.model.geom_dataid[g]
                if mesh_id >= 0:
                    mesh_file = self._get_mesh_file(mesh_id)
                    collision_mesh_files.append(mesh_file)
                    collision_mesh_transformations.append(T_body)  # Use T_body for mesh
                    collision_link_names.append(body_name)
                    collision_geometries.append({'type': 'mesh', 'file': mesh_file})
            elif geom_type == mujoco.mjtGeom.mjGEOM_BOX:
                # Primitive - use geom transform
                collision_mesh_files.append(None)
                collision_mesh_transformations.append(T_geom)
                collision_link_names.append(body_name)
                collision_geometries.append({'type': 'box', 'size': [s * 2 for s in size[:3]]})
            elif geom_type in (mujoco.mjtGeom.mjGEOM_SPHERE, mujoco.mjtGeom.mjGEOM_ELLIPSOID):
                # Ellipsoid approximated as sphere using max radius
                collision_mesh_files.append(None)
                collision_mesh_transformations.append(T_geom)
                collision_link_names.append(body_name)
                collision_geometries.append({'type': 'sphere', 'radius': max(size[:3])})
            elif geom_type in (mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE):
                collision_mesh_files.append(None)
                collision_mesh_transformations.append(T_geom)
                collision_link_names.append(body_name)
                collision_geometries.append({'type': 'cylinder', 'radius': size[0], 'length': size[1] * 2})
            elif geom_type == mujoco.mjtGeom.mjGEOM_HFIELD:
                # Skip heightfield for now - not supported as collision
                continue
            else:
                # Unknown type - skip
                continue

        # Process each joint
        for i, joint in enumerate(self.joints):
            parent_name = joint["parent"]
            child_name = joint["child"]

            # Get child body id and its world frame
            child_body_id = None
            for b in range(self.model.nbody):
                bname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b)
                if bname is None:
                    bname = f"body_{b}"
                if bname == child_name:
                    child_body_id = b
                    break

            if child_body_id is not None:
                # Get body world position and orientation
                body_pos = self.data.xpos[child_body_id]
                body_mat = self.data.xmat[child_body_id].reshape(3, 3)

                # Get joint position offset in body frame (jnt_pos)
                # Find the MuJoCo joint index by name
                mj_jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint["name"])
                if mj_jnt_id >= 0:
                    jnt_pos = self.model.jnt_pos[mj_jnt_id]
                else:
                    jnt_pos = np.zeros(3)

                # Joint world position = Body world position + Body rotation * jnt_pos
                joint_world_pos = body_pos + body_mat @ jnt_pos

                joint_frame = np.eye(4)
                joint_frame[:3, :3] = body_mat
                joint_frame[:3, 3] = joint_world_pos
            else:
                joint_frame = np.eye(4)

            joint_names.append(joint["name"])
            joint_frames.append(joint_frame)
            joint_types.append(joint["type"])
            joint_axes.append(joint["axis"])
            joint_parent_links.append(parent_name)
            joint_child_links.append(child_name)
            joint_limits.append(joint["limit"])

        return (
            link_names,
            link_mesh_files,
            link_mesh_transformations,
            link_frames,
            link_colors,
            joint_names,
            joint_frames,
            joint_types,
            joint_axes,
            joint_parent_links,
            joint_child_links,
            collision_mesh_files,
            collision_mesh_transformations,
            joint_limits,
            collision_link_names,
            collision_geometries,
        )

    def get_visual_geometries(self):
        """Return the visual geometry info for primitive shapes.

        Should be called after get_robot_info() to get geometry info for
        bodies that use primitive shapes instead of mesh files.

        Returns a list parallel to link_names, where each element is either:
        - None (for mesh-based visuals or empty bodies)
        - A dict with 'type' and 'size' keys for primitive shapes
        """
        return getattr(self, '_visual_geometries', [])

    def _cache_geom_indices(self):
        """缓存几何体索引分类，用于 update_transforms() 快速更新"""
        self._visual_geom_indices = []
        self._collision_geom_indices = []

        # 缓存视觉几何体索引
        for g in range(self.model.ngeom):
            contype = self.model.geom_contype[g]
            if contype != 0:
                continue  # 碰撞几何体，跳过

            geom_type = self.model.geom_type[g]
            if geom_type in (mujoco.mjtGeom.mjGEOM_PLANE, mujoco.mjtGeom.mjGEOM_HFIELD):
                continue  # 跳过地面和高度场

            body_id = self.model.geom_bodyid[g]
            is_mesh = (geom_type == mujoco.mjtGeom.mjGEOM_MESH)
            self._visual_geom_indices.append((g, body_id, is_mesh))

        # 缓存碰撞几何体索引
        for g in range(self.model.ngeom):
            contype = self.model.geom_contype[g]
            if contype == 0:
                continue  # 视觉几何体，跳过

            geom_type = self.model.geom_type[g]
            if geom_type == mujoco.mjtGeom.mjGEOM_PLANE:
                continue  # 跳过地面

            body_id = self.model.geom_bodyid[g]
            self._collision_geom_indices.append((g, body_id, geom_type))

    def update_transforms(self, qs):
        """轻量级变换更新 - 仅用于关节滑块交互

        使用 mj_kinematics() 替代 mj_forward()，跳过动力学计算，
        直接返回几何体变换矩阵，大幅提升性能。

        Args:
            qs: 关节角度值列表

        Returns:
            dict: 包含以下键的字典
                - link_names: 连杆名称列表
                - link_mesh_transformations: 视觉几何体变换矩阵列表
                - link_frames: 连杆坐标系变换矩阵列表
                - collision_mesh_transformations: 碰撞体变换矩阵列表
                - joint_names: 关节名称列表
                - joint_frames: 关节坐标系变换矩阵列表
                - joint_axes: 关节轴列表
        """
        # 设置关节位置
        if qs is not None:
            rev_idx = 0
            for i in range(self.model.njnt):
                if self.model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE:
                    if rev_idx < len(qs):
                        qpos_addr = self.model.jnt_qposadr[i]
                        self.data.qpos[qpos_addr] = qs[rev_idx]
                        rev_idx += 1

        # 只执行运动学计算 (不含动力学) - 核心优化点
        mujoco.mj_kinematics(self.model, self.data)

        # 快速提取视觉几何体变换
        link_names = []
        link_mesh_transformations = []
        link_frames = []

        for g, body_id, is_mesh in self._visual_geom_indices:
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            if body_name is None:
                body_name = f"body_{body_id}"

            # 获取 body 世界变换
            body_pos = self.data.xpos[body_id]
            body_mat = self.data.xmat[body_id].reshape(3, 3)

            T_body = np.eye(4)
            T_body[:3, :3] = body_mat
            T_body[:3, 3] = body_pos

            if is_mesh:
                # Mesh 几何体使用 body 变换
                link_mesh_transformations.append(T_body)
            else:
                # 基本几何体使用 geom 变换
                geom_pos = self.data.geom_xpos[g]
                geom_mat = self.data.geom_xmat[g].reshape(3, 3)

                T_geom = np.eye(4)
                T_geom[:3, :3] = geom_mat
                T_geom[:3, 3] = geom_pos
                link_mesh_transformations.append(T_geom)

            link_names.append(body_name)
            link_frames.append(T_body)

        # 快速提取碰撞几何体变换
        collision_mesh_transformations = []

        for g, body_id, geom_type in self._collision_geom_indices:
            if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
                # Mesh 碰撞体使用 body 变换
                body_pos = self.data.xpos[body_id]
                body_mat = self.data.xmat[body_id].reshape(3, 3)

                T_body = np.eye(4)
                T_body[:3, :3] = body_mat
                T_body[:3, 3] = body_pos
                collision_mesh_transformations.append(T_body)
            else:
                # 基本几何体使用 geom 变换
                geom_pos = self.data.geom_xpos[g]
                geom_mat = self.data.geom_xmat[g].reshape(3, 3)

                T_geom = np.eye(4)
                T_geom[:3, :3] = geom_mat
                T_geom[:3, 3] = geom_pos
                collision_mesh_transformations.append(T_geom)

        # 快速提取关节变换
        joint_names = []
        joint_frames = []
        joint_axes = []

        for joint in self.joints:
            child_name = joint["child"]

            # 找到子 body ID
            child_body_id = None
            for b in range(self.model.nbody):
                bname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b)
                if bname is None:
                    bname = f"body_{b}"
                if bname == child_name:
                    child_body_id = b
                    break

            if child_body_id is not None:
                body_pos = self.data.xpos[child_body_id]
                body_mat = self.data.xmat[child_body_id].reshape(3, 3)

                # 获取关节偏移
                mj_jnt_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint["name"])
                if mj_jnt_id >= 0:
                    jnt_pos = self.model.jnt_pos[mj_jnt_id]
                else:
                    jnt_pos = np.zeros(3)

                joint_world_pos = body_pos + body_mat @ jnt_pos

                joint_frame = np.eye(4)
                joint_frame[:3, :3] = body_mat
                joint_frame[:3, 3] = joint_world_pos
            else:
                joint_frame = np.eye(4)

            joint_names.append(joint["name"])
            joint_frames.append(joint_frame)
            joint_axes.append(joint["axis"])

        return {
            'link_names': link_names,
            'link_mesh_transformations': link_mesh_transformations,
            'link_frames': link_frames,
            'collision_mesh_transformations': collision_mesh_transformations,
            'joint_names': joint_names,
            'joint_frames': joint_frames,
            'joint_axes': joint_axes,
        }

    def build_multiple_trees(self):
        """Build multiple kinematic trees from the MuJoCo body hierarchy.

        Uses MuJoCo's body_parentid to construct anytree Node objects,
        matching the URDFParser interface.
        """
        self.nodes = {}

        # Create all body (link) nodes
        for i in range(self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
            if name is None:
                name = f"body_{i}"
            self.nodes[name] = Node(name, node_type="link", body_id=i)

        # Create joint nodes and establish parent-child relationships
        for i, joint in enumerate(self.joints):
            parent_link = joint["parent"]
            child_link = joint["child"]
            joint_name = joint["name"]

            # Create joint node with parent being the parent link
            if parent_link in self.nodes:
                joint_node = Node(
                    joint_name,
                    parent=self.nodes[parent_link],
                    node_type="joint",
                    joint_type=joint["type"],
                    joint_axis=joint["axis"]
                )
                self.nodes[joint_name] = joint_node

                # Set the child link's parent to be this joint
                if child_link in self.nodes:
                    self.nodes[child_link].parent = joint_node

        # Find all root nodes (links with no parent)
        roots = []
        for node in self.nodes.values():
            if node.is_root and node.node_type == "link":
                roots.append(node)

        return roots

    def identify_chains(self):
        """Identify all kinematic chains in the MJCF model.

        A chain is defined as a path from a root node to a leaf node.
        Returns chains in the same format as URDFParser.
        """
        trees = self.build_multiple_trees()
        chains = []

        # For each tree, find all paths from root to leaves
        for root in trees:
            # Find all leaf nodes in this tree
            leaves = [node for node in root.descendants if not node.children and node.node_type == "link"]

            # For each leaf, create a chain from root to leaf
            for leaf in leaves:
                chain = []
                current = leaf

                # Traverse up from leaf to root
                while current:
                    chain.append(current)
                    current = current.parent

                # Reverse to get root-to-leaf order
                chain.reverse()

                # Create a chain object with relevant information
                chain_info = {
                    "name": f"{root.name}_to_{leaf.name}",
                    "nodes": chain,
                    "links": [node for node in chain if node.node_type == "link"],
                    "joints": [node for node in chain if node.node_type == "joint"],
                    "root": root,
                    "leaf": leaf
                }

                chains.append(chain_info)

        return chains, trees

    def get_chain_info(self):
        """Get detailed information about all kinematic chains.

        Returns a list of dictionaries matching URDFParser format.
        """
        chains, trees = self.identify_chains()
        chain_info_list = []

        # Get transformations for all bodies
        mujoco.mj_forward(self.model, self.data)

        for chain in chains:
            # Extract link and joint names in order
            link_names = [node.name for node in chain["nodes"] if node.node_type == "link"]
            joint_names = [node.name for node in chain["nodes"] if node.node_type == "joint"]

            # Get joint types and axes
            joint_types = []
            joint_axes = []

            for joint_name in joint_names:
                joint_node = self.nodes[joint_name]
                joint_types.append(getattr(joint_node, "joint_type", "unknown"))
                joint_axes.append(getattr(joint_node, "joint_axis", [0, 0, 1]))

            # Get link transformations from MuJoCo
            link_transforms = []
            for link_name in link_names:
                link_node = self.nodes.get(link_name)
                if link_node and hasattr(link_node, 'body_id'):
                    body_id = link_node.body_id
                    body_pos = self.data.xpos[body_id]
                    body_mat = self.data.xmat[body_id].reshape(3, 3)

                    T = np.eye(4)
                    T[:3, :3] = body_mat
                    T[:3, 3] = body_pos
                    link_transforms.append(T)
                else:
                    link_transforms.append(np.eye(4))

            # Create detailed chain info
            detailed_info = {
                "name": chain["name"],
                "root": chain["root"].name,
                "leaf": chain["leaf"].name,
                "link_names": link_names,
                "joint_names": joint_names,
                "joint_types": joint_types,
                "joint_axes": joint_axes,
                "link_transforms": link_transforms,
                "num_links": len(link_names),
                "num_joints": len(joint_names)
            }

            chain_info_list.append(detailed_info)

        return chain_info_list, trees

    def get_joint_axes(self, chain):
        """Get the axis of each joint in the chain.

        Returns joint positions, vectors, x-axes, and types matching URDFParser format.
        """
        link_frames = chain["link_transforms"]
        joint_axes = [[0, 0, 1]] + chain["joint_axes"]  # First joint axis is fixed base, default z-up
        joint_types = ['base'] + chain["joint_types"]
        link_names = chain["link_names"]

        joint_positions = []
        joint_vectors = []
        joint_xs = []

        for name, T, axis, joint_type in zip(link_names, link_frames, joint_axes, joint_types):
            # Extract position and orientation
            pos = T[:3, 3]
            rot = T[:3, :3] @ np.array(axis)

            if np.allclose(np.abs(np.array(axis)), np.array([0, 0, 1])):  # z axis rotation
                joint_x = T[:3, :3] @ np.array([1, 0, 0])
            elif np.allclose(np.abs(np.array(axis)), np.array([0, 1, 0])):  # y axis rotation
                joint_x = T[:3, :3] @ np.array([1, 0, 0])
            elif np.allclose(np.abs(np.array(axis)), np.array([1, 0, 0])):  # x axis rotation
                joint_x = T[:3, :3] @ np.array([0, 1, 0])
            else:
                # For arbitrary axis, compute perpendicular
                joint_x = T[:3, :3] @ np.array([1, 0, 0])

            joint_pos = pos
            joint_vector = rot

            joint_positions.append(joint_pos)
            joint_vectors.append(joint_vector)
            joint_xs.append(joint_x)

        return joint_positions, joint_vectors, joint_xs, joint_types

    def calculate_mdh_origin_position(self, joint_pos, joint_vector, joint_pos_next, joint_vector_next):
        """Calculate the MDH coordinate origin position between two adjacent joints.

        Same algorithm as URDFParser.
        """
        # Ensure inputs are numpy arrays
        zi = np.asarray(joint_vector)
        zi_next = np.asarray(joint_vector_next)
        pi = np.asarray(joint_pos)
        pi_next = np.asarray(joint_pos_next)

        common_perpendicular = None

        # Case 1: zi and zi+1 coincident (same or opposite direction)
        if np.allclose(np.cross(zi, zi_next), np.zeros(3)):
            diff_pos = pi_next - pi
            if np.allclose(np.cross(diff_pos, zi), np.zeros(3)):
                # On the same line
                return pi, 'coincident', None
            # Parallel but not coincident - fall through to case 3
        else:
            # Check if intersecting
            cross_z = np.cross(zi, zi_next)
            diff_pos = pi_next - pi
            distance = np.abs(np.dot(diff_pos, cross_z)) / np.linalg.norm(cross_z)

            if np.isclose(distance, 0):
                # Case 2: Intersecting
                A = np.column_stack((zi, -zi_next))
                b = pi_next - pi
                t, s = np.linalg.lstsq(A, b, rcond=None)[0]
                oi = pi + t * zi
                common_perpendicular = (oi, oi)
                return oi, 'intersect', common_perpendicular

        # Case 3: Parallel or Case 4: Skew (not intersecting, not parallel)
        n = np.cross(zi, zi_next)

        if np.allclose(n, np.zeros(3)):
            # Case 3: Parallel
            oi = pi
            point1 = pi
            point2 = pi_next - np.dot(pi_next - pi, zi) * zi
            common_perpendicular = (point1, point2)
            return oi, 'parallel', common_perpendicular
        else:
            # Case 4: Skew lines
            n = n / np.linalg.norm(n)
            A = np.column_stack((zi, -zi_next, n))
            b = pi_next - pi
            t, s, _ = np.linalg.lstsq(A, b, rcond=None)[0]
            oi = pi + t * zi
            point1 = pi + t * zi
            point2 = pi_next + s * zi_next
            common_perpendicular = (point1, point2)
            return oi, 'skew', common_perpendicular

    def get_mdh_parameters(self, chain):
        """Get the MDH parameters of the chain.

        Same algorithm as URDFParser.
        """
        joint_positions, joint_vectors, joint_xs, joint_types = self.get_joint_axes(chain)

        # Skip fixed joints
        joint_positions = [joint_positions[i] for i, t in enumerate(joint_types) if t == 'revolute' or t == 'base']
        joint_vectors = [joint_vectors[i] for i, t in enumerate(joint_types) if t == 'revolute' or t == 'base']
        joint_xs = [joint_xs[i] for i, t in enumerate(joint_types) if t == 'revolute' or t == 'base']

        num_joints = len(joint_positions) - 1

        mdh_origins = []
        mdh_zs = []
        mdh_xs = []
        mdh_cases = []

        for i in range(num_joints):
            joint_pos = joint_positions[i]
            joint_vector = joint_vectors[i]
            joint_x = joint_xs[i]

            joint_pos_next = joint_positions[i + 1]
            joint_vector_next = joint_vectors[i + 1]

            oi, case, common_perpendicular = self.calculate_mdh_origin_position(
                joint_pos, joint_vector, joint_pos_next, joint_vector_next
            )
            mdh_cases.append(case)

            mdh_origins.append(oi)
            mdh_zs.append(joint_vector)

            if case == 'coincident':
                xi = joint_x
            elif case == 'skew' or case == 'intersect':
                xi = np.cross(joint_vector, joint_vector_next)
                xi = xi / np.linalg.norm(xi)
            elif case == 'parallel':
                xi = common_perpendicular[1] - common_perpendicular[0]
                xi = xi / np.linalg.norm(xi)
            else:
                xi = joint_x

            mdh_xs.append(xi)

        # Last joint
        mdh_origins.append(joint_positions[-1])
        mdh_zs.append(joint_vectors[-1])
        mdh_xs.append(joint_xs[-1])

        mdh_parameters = []

        for i in range(num_joints):
            o_prev = mdh_origins[i]
            z_prev = mdh_zs[i]
            x_prev = mdh_xs[i]

            oi = mdh_origins[i + 1]
            zi = mdh_zs[i + 1]
            xi = mdh_xs[i + 1]

            # theta: rotation around zi from xi-1 to xi
            p_prev = x_prev - np.dot(x_prev, zi) * zi
            pi = xi - np.dot(xi, zi) * zi

            pi_norm = pi / np.linalg.norm(pi)
            p_prev_norm = p_prev / np.linalg.norm(p_prev)

            cos_theta = np.dot(p_prev_norm, pi_norm)
            sin_theta = np.dot(np.cross(p_prev_norm, pi_norm), zi)
            theta = np.arctan2(sin_theta, cos_theta)

            # d: distance along zi from xi-1 to xi
            d = (oi - o_prev).dot(zi)

            # a: distance along xi-1 from zi-1 to zi
            a = (oi - o_prev).dot(x_prev)

            # alpha: rotation around xi-1 from zi-1 to zi
            p_prev = z_prev - np.dot(z_prev, x_prev) * x_prev
            pi = zi - np.dot(zi, x_prev) * x_prev

            pi_norm = pi / np.linalg.norm(pi)
            p_prev_norm = p_prev / np.linalg.norm(p_prev)

            cos_alpha = np.dot(p_prev_norm, pi_norm)
            sin_alpha = np.dot(np.cross(p_prev_norm, pi_norm), x_prev)
            alpha = np.arctan2(sin_alpha, cos_alpha)

            mdh_parameters.append([theta, d, a, alpha])

        return mdh_origins, mdh_zs, mdh_xs, mdh_parameters

    def get_mdh_frames(self, chain):
        """Generate MDH coordinate frames from MDH parameters."""
        mdh_origins, mdh_zs, mdh_xs, mdh_parameters = self.get_mdh_parameters(chain)

        mdh_frames = []

        for origin, z, x in zip(mdh_origins, mdh_zs, mdh_xs):
            y = np.cross(z, x)

            transform = np.eye(4)
            transform[:3, 3] = origin
            transform[:3, 0] = x
            transform[:3, 1] = y
            transform[:3, 2] = z

            mdh_frames.append(transform)

        return mdh_frames

    def update_mdh_frames(self, mdh_frames, joint_values):
        """Update MDH frames based on joint values."""
        T_i_iplus1 = []
        for i, frame in enumerate(mdh_frames):
            if i > 0:
                joint = joint_values[i - 1]
                rot_z = np.eye(4)
                rot_z[0, 0] = math.cos(joint)
                rot_z[1, 0] = math.sin(joint)
                rot_z[1, 1] = math.cos(joint)
                rot_z[0, 1] = -1 * math.sin(joint)
                transform = np.linalg.inv(mdh_frames[i - 1]) @ frame @ rot_z
                T_i_iplus1.append(transform)

        for i, frame in enumerate(mdh_frames):
            if i > 0:
                mdh_frames[i] = mdh_frames[i - 1] @ T_i_iplus1[i - 1]

        return mdh_frames

    @staticmethod
    def is_valid_mjcf(filepath):
        """Check if a file is a valid MuJoCo MJCF file (contains <mujoco> root element)."""
        try:
            tree = ET.parse(filepath)
            root = tree.getroot()
            return root.tag == 'mujoco'
        except Exception:
            return False
