import casadi as ca
import numpy as np
import xml.etree.ElementTree as ET
import os


class FKGenerator:
    def __init__(self, urdf_path):
        self.urdf_path = urdf_path
        self.joints = []
        self.links = []

        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF 文件不存在: {urdf_path}")

        self.parse_urdf()

    def parse_urdf(self):
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()

        for joint in root.findall('joint'):
            joint_name = joint.attrib['name']
            joint_type = joint.attrib['type']

            # 跳过底盘关节
            if joint_name in ['position_base_theta', 'position_base_x', 'position_base_y']:
                continue

            if joint_type != 'revolute' and joint_type != 'continuous':
                continue

            parent = joint.find('parent').attrib['link']
            child = joint.find('child').attrib['link']
            axis = joint.find('axis').attrib['xyz']
            axis = [float(a) for a in axis.strip().split(' ')]

            origin_tag = joint.find('origin')
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]

            if origin_tag is not None:
                xyz_str = origin_tag.attrib.get('xyz', '0 0 0')
                rpy_str = origin_tag.attrib.get('rpy', '0 0 0')

                xyz = [float(val) for val in xyz_str.strip().split()]
                rpy = [float(val) for val in rpy_str.strip().split()]

            joint_info = {
                'name': joint_name,
                'type': joint_type,
                'parent': parent,
                'child': child,
                'axis': axis,
                'xyz': xyz,
                'rpy': rpy
            }
            self.joints.append(joint_info)

        print(f"[FKGenerator] 已解析机械臂关节数量: {len(self.joints)} 个")
        for joint in self.joints:
            print(f"  -> {joint['name']}")

    def rpy_to_rot(self, rpy):
        roll, pitch, yaw = rpy
        Rx = ca.SX([
            [1, 0, 0],
            [0, ca.cos(roll), -ca.sin(roll)],
            [0, ca.sin(roll), ca.cos(roll)]
        ])
        Ry = ca.SX([
            [ca.cos(pitch), 0, ca.sin(pitch)],
            [0, 1, 0],
            [-ca.sin(pitch), 0, ca.cos(pitch)]
        ])
        Rz = ca.SX([
            [ca.cos(yaw), -ca.sin(yaw), 0],
            [ca.sin(yaw), ca.cos(yaw), 0],
            [0, 0, 1]
        ])
        return ca.mtimes([Rz, Ry, Rx])

    def rot_to_quaternion(self, R):
        """
        旋转矩阵转四元数（支持 CasADi 符号）
        """
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        eps = 1e-6

        def case1():
            qw = ca.sqrt(1.0 + trace) / 2.0
            qx = (R[2, 1] - R[1, 2]) / (4.0 * qw)
            qy = (R[0, 2] - R[2, 0]) / (4.0 * qw)
            qz = (R[1, 0] - R[0, 1]) / (4.0 * qw)
            return ca.vertcat(qx, qy, qz, qw)

        def case2():
            qx = ca.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) / 2.0
            qw = (R[2, 1] - R[1, 2]) / (4.0 * qx)
            qy = (R[0, 1] + R[1, 0]) / (4.0 * qx)
            qz = (R[0, 2] + R[2, 0]) / (4.0 * qx)
            return ca.vertcat(qx, qy, qz, qw)

        def case3():
            qy = ca.sqrt(1.0 - R[0, 0] + R[1, 1] - R[2, 2]) / 2.0
            qw = (R[0, 2] - R[2, 0]) / (4.0 * qy)
            qx = (R[0, 1] + R[1, 0]) / (4.0 * qy)
            qz = (R[1, 2] + R[2, 1]) / (4.0 * qy)
            return ca.vertcat(qx, qy, qz, qw)

        def case4():
            qz = ca.sqrt(1.0 - R[0, 0] - R[1, 1] + R[2, 2]) / 2.0
            qw = (R[1, 0] - R[0, 1]) / (4.0 * qz)
            qx = (R[0, 2] + R[2, 0]) / (4.0 * qz)
            qy = (R[1, 2] + R[2, 1]) / (4.0 * qz)
            return ca.vertcat(qx, qy, qz, qw)

        cond1 = trace > eps
        cond2 = ca.logic_and(R[0, 0] > R[1, 1], R[0, 0] > R[2, 2])
        cond3 = R[1, 1] > R[2, 2]

        q = ca.if_else(cond1, case1(),
                       ca.if_else(cond2, case2(),
                                  ca.if_else(cond3, case3(), case4())))
        return q

    def skew_symmetric(self, K):
        return ca.vertcat(
            ca.horzcat(0, -K[2], K[1]),
            ca.horzcat(K[2], 0, -K[0]),
            ca.horzcat(-K[1], K[0], 0)
        )

    def fk_with_quat_with_base(self, q):
        """
        完整正运动学（底盘 + 机械臂），输出 xyz + 四元数
        """
        base_x = q[0]
        base_y = q[1]
        base_theta = q[2]
        q_arm = q[3:]  # 机械臂关节

        # 底盘变换
        T_base = ca.SX.eye(4)
        T_base[0:3, 3] = ca.vertcat(base_x, base_y, 0.0)

        R_base = ca.vertcat(
            ca.horzcat(ca.cos(base_theta), -ca.sin(base_theta), 0.0),
            ca.horzcat(ca.sin(base_theta), ca.cos(base_theta), 0.0),
            ca.horzcat(0.0, 0.0, 1.0)
        )
        T_base[0:3, 0:3] = R_base

        # 机械臂 FK
        T_arm = ca.SX.eye(4)

        for idx, joint in enumerate(self.joints):
            trans = joint['xyz']
            rpy = joint['rpy']
            R = self.rpy_to_rot(rpy)
            p = ca.SX(trans)

            T_offset = ca.SX.eye(4)
            T_offset[0:3, 0:3] = R
            T_offset[0:3, 3] = p

            axis = joint['axis']
            theta = q_arm[idx]
            K = ca.SX(axis).T
            K_hat = self.skew_symmetric(K)

            R_joint = ca.SX.eye(3) + ca.sin(theta) * K_hat + (1 - ca.cos(theta)) * ca.mtimes(K_hat, K_hat)

            T_joint = ca.SX.eye(4)
            T_joint[0:3, 0:3] = R_joint

            T_arm = ca.mtimes(T_arm, ca.mtimes(T_offset, T_joint))

        T_total = ca.mtimes(T_base, T_arm)

        position = T_total[0:3, 3]
        quat = self.rot_to_quaternion(T_total[0:3, 0:3])

        return ca.vertcat(position, quat)

    def fk_func_with_base_quat(self):
        """
        返回 casadi 符号函数（xyz + quaternion）
        """
        total_dof = 3 + len(self.joints)
        q = ca.SX.sym('q', total_dof)
        ee_pose = self.fk_with_quat_with_base(q)
        return ca.Function('fk_with_base_quat', [q], [ee_pose])
