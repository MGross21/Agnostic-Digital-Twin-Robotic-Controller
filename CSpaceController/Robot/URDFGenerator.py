import json
import numpy as np
from scipy.spatial.transform import Rotation as R
from datetime import datetime
import os
from dataclasses import dataclass

import numpy as np
import json
from typing import List, Union, Tuple
from Robot import DH, RobotMath

# class Parameter:
#     def __init__(self, value):
#         self._value = self.to_tuple(value)

#     def __len__(self):
#         return len(self._value)
    
#     def isRange(self):
#         return len(self._value) > 1
    
#     @staticmethod
#     def to_tuple(num):
#         return num if isinstance(num, tuple) else (num,)

#     def __iter__(self):
#         return iter(self._value)

#     def __getitem__(self, index):
#         return self._value[index]
    
#     @property
#     def value(self):
#         return self._value[0]
#
#     def __repr__(self):
#         return f"Parameter(value={self._value})"


# @dataclass
# class DH:
#     def __init__(self, a=0, alpha=0, d=0, theta=0):
#         self.a = Parameter(a)
#         self.alpha = Parameter(alpha)
#         self.d = Parameter(d)
#         self.theta = Parameter(theta)

#         parameters = [self.theta, self.d, self.a, self.alpha]
#         assert sum(len(param) > 1 for param in parameters) <= 1, "Only one parameter may have a ranged tuple"

#         if len(self.theta) > 1:
#             self.type = 'revolute'
#         elif len(self.d) > 1:
#             self.type = 'prismatic'
#         else:
#             self.type = 'fixed'

#     def __getattr__(self, item):
#         param = self.__dict__.get(item)
#         if isinstance(param, Parameter):
#             return param.value
#         raise AttributeError(f"'DH' object has no attribute '{item}'")

#     def DOF(self, chain:'DH.create_chain') -> int:
#         return sum(1 for dh in chain if dh.type == 'revolute') + sum(1 for dh in chain if dh.type == 'prismatic')

#     @staticmethod
#     def create_chain(*args: 'DH') -> List['DH']:
#         return list(args)
    
#     @staticmethod
#     def frames(chain: List['DH']) -> List[np.ndarray]:
#         frames = [chain[0]]
#         for dh in chain[1:]:
#             frames.append(frames[-1] @ dh)
#         return frames

    # @staticmethod
    # def reachable_space(chain: List['DH']) -> List[np.ndarray]:
    #     transforms = []
    #     cumulative_transform = np.eye(4)
    #     for dh in chain:
    #         cumulative_transform = cumulative_transform @ RobotMath.H(dh.a[0], dh.alpha[0], dh.d[0], dh.theta[0])
    #         transforms.append(cumulative_transform[:3, 3])  # Extract position vector
    #     return transforms
    


@dataclass
class URDFGenerator:
    DH_params: List[DH]
    name: str = "Robot"


    def _generate_urdf(self):
        urdf = "<!-- Generated on {} -->\n".format(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
        urdf += self._generate_header()
        urdf += self._generate_materials()
        urdf += self._generate_links_and_joints()
        urdf += "</robot>\n"
        return urdf

    def _generate_header(self):
        return f"<robot name='{self.name}'>\n"

    def _generate_materials(self):
        return """
        <material name='blue'>
            <color rgba='0 0 0.8 1'/>
        </material>
        <material name='red'>
            <color rgba='0.8 0 0 1'/>
        </material>
        """

    def _generate_links_and_joints(self):
        urdf = ""
        transforms = self._joint_transforms()
        frames = self._joint_frames(transforms)

        for i in range(len(self.joints)):
            urdf += self._generate_link(i, transforms[i], frames[i])
            if i > 0:
                urdf += self._generate_fixed_joint(i)
            urdf += self._generate_connecting_link(i, transforms[i+1] if i+1 < len(transforms) else None)
            urdf += self._generate_moving_joint(i, frames[i])

        return urdf

    def _joint_transforms(self):
        return [np.eye(4)] + [RobotMath.H(dh.a.value, dh.alpha.value, dh.d.value, dh.theta.value) for dh in self.DH_params]

    def _joint_frames(self, transforms):
        frames = [transforms[0]]
        for trans in transforms[1:]:
            frames.append(frames[-1] @ trans)
        return frames

    def _dh_trans(self, joint):
        d, theta, a, alpha = joint['d'], joint['theta'], joint['a'], joint['alpha']
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def _generate_link(self, i, transform, frame):
        rpy = R.from_matrix(frame[0:3, 0:3]).as_euler('XYZ')
        return f"""
    <link name='a{i}'>
        <visual>
            <origin rpy='{rpy[0]} {rpy[1]} {rpy[2]}' xyz='{transform[0,3]} {transform[1,3]} {transform[2,3]}'/>
            <geometry>
                <cylinder length='0.1' radius='0.05'/>
            </geometry>
            <material name='blue'/>
        </visual>
        <inertial>
            <mass value='{self.links[i]["mass"]}'/>
            <inertia ixx='1e-3' ixy='0' ixz='0' iyy='1e-3' iyz='0' izz='1e-3'/>
            <origin xyz='{self.links[i]["CoM"][0]} {self.links[i]["CoM"][1]} {self.links[i]["CoM"][2]}'/>
        </inertial>
    </link>
    """

    def _generate_fixed_joint(self, i):
        return f"""
    <joint name='fix_a{i}_to_l{i-1}' type='fixed'>
        <parent link='l{i-1}'/>
        <child link='a{i}'/>
        <origin rpy='0 0 0' xyz='0 0 0'/>
    </joint>
    """

    def _generate_connecting_link(self, i, next_transform):
        if next_transform is None:
            return ""
        
        origins_vector = next_transform[0:3, 3]
        origins_vector_norm = np.linalg.norm(origins_vector)
        cylinder_origin = origins_vector / 2

        rpy = [0, 0, 0]
        if origins_vector_norm != 0.0:
            origins_vector_unit = origins_vector / origins_vector_norm
            axis = np.cross(origins_vector, np.array([0, 0, -1]))
            axis_norm = np.linalg.norm(axis)
            if axis_norm != 0.0:
                axis = axis / axis_norm
            angle = np.arccos(origins_vector_unit @ np.array([0, 0, 1]))
            rpy = R.from_rotvec(angle * axis).as_euler('XYZ')

        return f"""
    <link name='l{i}'>
        <visual>
            <origin rpy='{rpy[0]} {rpy[1]} {rpy[2]}' xyz='{cylinder_origin[0]} {cylinder_origin[1]} {cylinder_origin[2]}'/>
            <geometry>
                <cylinder length='{origins_vector_norm}' radius='0.05'/>
            </geometry>
            <material name='red'/>
        </visual>
    </link>
    """

    def _generate_moving_joint(self, i, frame):
        joint = self.joints[i]
        joint_type = 'continuous' if joint['type'] == 'revolute' else 'prismatic'
        limit = joint.get('limit', {})
        limit_str = f"<limit lower='{limit.get('lower', -3.14)}' upper='{limit.get('upper', 3.14)}'/>" if limit else ""
        
        return f"""
    <joint name='move_l{i}_from_a{i}' type='{joint_type}'>
        <parent link='a{i}'/>
        <child link='l{i}'/>
        <axis xyz='{frame[0,2]} {frame[1,2]} {frame[2,2]}'/>
        <origin rpy='0 0 0' xyz='{joint["a"]} 0 {joint["d"]}'/>
        {limit_str}
    </joint>
    """

    def _save_urdf(self, directory='.'):
        robot_name = self.robot_data['name']
        filename = f"{robot_name}.urdf"
        filepath = os.path.join(directory, filename)
        
        with open(filepath, 'w') as f:
            f.write(self._generate_urdf())
        
        print(f"URDF file '{filename}' has been saved in '{os.path.abspath(directory)}'")

        return filepath

    def from_DH(self, DH_params: List[DH]):
        assert all(isinstance(dh, DH) for dh in DH_params), "All parameters must be of type DH"
        self.DH_params = DH_params
        return self._save_urdf()
    
    def from_json(self, json_file: str, key: str = 'DH_Parameters'):
        assert os.path.exists(json_file), "The file does not exist"
        self.DH_params = DH.from_json(json_file, key)
        return self._save_urdf()
    



# # Usage example
# from Robot import UniversalRobots

# URDFGenerator.from_DH(UniversalRobots.UR5.dh_params)
# Compute reachable space
# reachable_points = DH.reachable_space(chain)
# print("Reachable points:", reachable_points)
