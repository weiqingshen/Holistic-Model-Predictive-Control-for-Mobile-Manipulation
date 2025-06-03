import os
import numpy as np
import fcl
import trimesh
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
from moveit.core.robot_state import RobotState


class FCLDistanceCalculator(Node):
    def __init__(self):
        super().__init__('fcl_distance_calculator')

        self.urdf_path = "/home/fins/myrobot_move/src/robot_arm_description/urdf/robot_arm_description.urdf"
        self.mesh_base_path = "/home/fins/myrobot_move/src/robot_arm_description/"

        self.link_collisions = {}
        self.joint_info = {}

        self.load_robot_links_and_joints()

    def load_robot_links_and_joints(self):
        """解析 URDF 文件，加载 link 和 joint 结构"""
        try:
            tree = ET.parse(self.urdf_path)
        except Exception as e:
            self.get_logger().error(f"无法解析 URDF 文件: {e}")
            return

        root = tree.getroot()

        for link in root.findall("link"):
            link_name = link.get("name")
            visual = link.find("visual")
            if visual is None:
                continue

            geometry = visual.find("geometry")
            if geometry is None:
                continue

            mesh = geometry.find("mesh")
            if mesh is None:
                continue

            mesh_file = mesh.get("filename")
            if not mesh_file:
                continue

            if mesh_file.startswith("package://robot_arm_description/"):
                mesh_file = mesh_file.replace("package://robot_arm_description/", self.mesh_base_path)

            if not os.path.isfile(mesh_file):
                self.get_logger().error(f"网格文件不存在: {mesh_file}")
                continue

            try:
                mesh = trimesh.load_mesh(mesh_file)
                if not isinstance(mesh, trimesh.Trimesh):
                    self.get_logger().error(f"无法加载网格文件: {mesh_file}")
                    continue

                vertices = np.array(mesh.vertices, dtype=np.float32)
                triangles = np.array(mesh.faces, dtype=np.int32)

                bvh_model = fcl.BVHModel()
                bvh_model.beginModel(len(vertices), len(triangles))
                bvh_model.addSubModel(vertices, triangles)
                bvh_model.endModel()

                collision_object = fcl.CollisionObject(bvh_model)

                self.link_collisions[link_name] = {'geometry': collision_object, 'transform': np.eye(4)}
            except Exception as e:
                self.get_logger().error(f"加载 {link_name} 的网格失败: {e}")

        for joint in root.findall("joint"):
            joint_name = joint.get("name")
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            joint_type = joint.get("type")

            origin = joint.find("origin")
            if origin is not None:
                xyz = [float(x) for x in origin.get("xyz", "0 0 0").split()]
                rpy = [float(r) for r in origin.get("rpy", "0 0 0").split()]
            else:
                xyz = [0.0, 0.0, 0.0]
                rpy = [0.0, 0.0, 0.0]

            axis_element = joint.find("axis")
            axis = np.array([1, 0, 0])  # 默认 X 轴
            if axis_element is not None:
                axis = np.array([float(a) for a in axis_element.get("xyz", "1 0 0").split()])

            # ✅ **确保 joint["name"] 被存入**
            self.joint_info[joint_name] = {
                "name": joint_name,  # 这里确保 name 被存入
                "parent": parent,
                "child": child,
                "type": joint_type,
                "axis": axis,
                "xyz": xyz,
                "rpy": rpy
            }

    def compute_forward_kinematics(self, joint_values):
        """使用 MoveIt2 计算 FK，并更新 FCL 碰撞对象"""

        # ✅ 1. 初始化 MoveIt2
        moveit2 = MoveIt2(
            node=self,
            joint_names=list(joint_values.keys()),
            base_link_name="world",
            end_effector_name="link6",
            group_name="arm_move"
        )
        moveit2.set_joint_goal(
            joint_positions=list(joint_values.values()),
            joint_names=list(joint_values.keys()))
        # ✅ 2. 设置关节位置

        # ✅ 3. 计算 FK
        fk_results = moveit2.compute_fk(
            joint_state=list(joint_values.values()),
            fk_link_names=list(self.link_collisions.keys())
        )

        if not fk_results:
            print("❌ MoveIt2 计算 FK 失败！")
            return

        # ✅ 4. 更新 FCL 变换
        for link_name, pose_stamped in zip(self.link_collisions.keys(), fk_results):
            if pose_stamped is None:
                print(f"⚠️ 无法获取 `{link_name}` 的变换，跳过...")
                continue

            translation = np.array([
                pose_stamped.pose.position.x,
                pose_stamped.pose.position.y,
                pose_stamped.pose.position.z
            ])
            quaternion = np.array([
                pose_stamped.pose.orientation.x,
                pose_stamped.pose.orientation.y,
                pose_stamped.pose.orientation.z,
                pose_stamped.pose.orientation.w
            ])

            transform_fcl = fcl.Transform(quaternion, translation)
            self.link_collisions[link_name]['geometry'].setTransform(transform_fcl)

    def compute_distances(self):
        """计算所有 link 之间的最小距离"""
        link_names = list(self.link_collisions.keys())
        results = []
        for i in range(len(link_names)):
            for j in range(i + 1, len(link_names)):
                obj1 = self.link_collisions[link_names[i]]['geometry']
                obj2 = self.link_collisions[link_names[j]]['geometry']
                req = fcl.DistanceRequest()
                res = fcl.DistanceResult()

                req.enable_nearest_points = True

                fcl.distance(obj1, obj2, req, res)
                results.append((link_names[i], link_names[j], res.min_distance))
        return results

    def update_and_compute(self, joint_values):
        """更新变换并计算所有 link 之间的最小距离"""
        print("🔹 正在更新关节角度:")
        for joint, value in joint_values.items():
            print(f"   ↪ {joint}: {value:.4f}")

        self.compute_forward_kinematics(joint_values)
        distances = self.compute_distances()

        for link1, link2, distance in distances:
            print(f"{link1} 和 {link2} 之间的距离: {distance:.6f} 米")

def main():
    rclpy.init()
    node = FCLDistanceCalculator()

    joint_values = {
        "position_base_x": 0.0,
        "position_base_y": 0.0,
        "position_base_theta": 0.0,
        "joint1": 0.0,
        "joint2": -1.2,
        "joint3": 0.0,
        "joint4": 0.0,
        "joint5": 0.0,
        "joint6": 0.0,
        "joint_right": 0.01,

    }

    node.update_and_compute(joint_values)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
