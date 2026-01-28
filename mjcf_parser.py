import numpy as np
import math
import os
import xml.etree.ElementTree as ET

import mujoco


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
            joint_limits,
            collision_mesh_files,
            collision_mesh_transformations,
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

    @staticmethod
    def is_valid_mjcf(filepath):
        """Check if a file is a valid MuJoCo MJCF file (contains <mujoco> root element)."""
        try:
            tree = ET.parse(filepath)
            root = tree.getroot()
            return root.tag == 'mujoco'
        except Exception:
            return False
