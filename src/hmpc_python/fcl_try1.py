import rclpy
from rclpy.node import Node
import numpy as np
import fcl
import trimesh
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R
import os

class FCLDistanceCalculator(Node):
    def __init__(self):
        super().__init__('fcl_distance_calculator')

        # 设置 URDF 文件的绝对路径
        self.urdf_path = "/home/fins/myrobot_move/src/robot_arm_description/urdf/robot_arm_description.urdf"
        self.mesh_base_path = "/home/fins/myrobot_move/src/"  # Mesh 基础路径

        self.link_collisions = self.load_robot_links()
        self.compute_distances()

    def load_robot_links(self):
        """解析 URDF 并加载网格模型"""
        link_collisions = {}

        # 解析 URDF 文件
        tree = ET.parse(self.urdf_path)
        root = tree.getroot()

        for link in root.findall("link"):
            link_name = link.get("name")

            visual = link.find("visual")
            if visual is None:
                continue  # 跳过没有视觉信息的链接

            geometry = visual.find("geometry")
            if geometry is None:
                continue

            mesh = geometry.find("mesh")
            if mesh is None:
                continue  # 跳过非网格的链接

            mesh_file = mesh.get("filename")
            if not mesh_file:
                continue

            # 处理 package:// 路径
            if mesh_file.startswith("package://"):
                mesh_file = mesh_file.replace("package://", self.mesh_base_path)

            try:
                # 加载网格文件
                mesh = trimesh.load_mesh(mesh_file)
                if not isinstance(mesh, trimesh.Trimesh):
                    self.get_logger().error(f"无法加载网格文件: {mesh_file}")
                    continue

                # 将 trimesh 转换为 FCL 的 BVHModel
                vertices = np.array(mesh.vertices, dtype=np.float32)
                triangles = np.array(mesh.faces, dtype=np.int32)

                bvh_model = fcl.BVHModel()
                bvh_model.beginModel(len(vertices), len(triangles))
                bvh_model.addSubModel(vertices, triangles)
                bvh_model.endModel()

                collision_object = fcl.CollisionObject(bvh_model)

                # 解析位姿变换
                transform = np.eye(4)
                origin = visual.find("origin")
                if origin is not None:
                    xyz = [float(x) for x in origin.get("xyz", "0 0 0").split()]
                    rpy = [float(r) for r in origin.get("rpy", "0 0 0").split()]
                    transform[:3, :3] = R.from_euler('xyz', rpy).as_matrix()
                    transform[:3, 3] = xyz

                link_collisions[link_name] = {
                    'geometry': collision_object,
                    'transform': transform
                }
            except Exception as e:
                self.get_logger().error(f"加载 {link_name} 的网格失败: {e}")

        return link_collisions

    def compute_distances(self):
        """计算所有关节对之间的最小距离"""
        link_names = list(self.link_collisions.keys())
        num_links = len(link_names)

        for i in range(num_links):
            for j in range(i + 1, num_links):
                link1, link2 = link_names[i], link_names[j]
                obj1 = self.link_collisions[link1]['geometry']
                obj2 = self.link_collisions[link2]['geometry']

                req = fcl.DistanceRequest()
                res = fcl.DistanceResult()
                fcl.distance(obj1, obj2, req, res)

                self.get_logger().info(f"{link1} 和 {link2} 之间的距离: {res.min_distance:.4f} 米")

def main(args=None):
    rclpy.init(args=args)
    node = FCLDistanceCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
