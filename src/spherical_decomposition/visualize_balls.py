import pybullet as p
import pybullet_data
import json
import numpy as np
import time

def visualize_link2_spheres(json_file):
    """
    从 JSON 文件中读取 link2 的球体数据，并在世界坐标系下使用 PyBullet 可视化这些球体。
    """
    # 连接到 PyBullet
    physicsClient = p.connect(p.GUI)  # 或 p.DIRECT for headless mode
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)  # 设置重力

    # 创建地面 (可选)
    planeId = p.loadURDF("plane.urdf")

    # 读取 JSON 文件
    with open(json_file, 'r') as f:
        data = json.load(f)

    # 检查 link2 是否存在于 JSON 数据中
    if 'link2' not in data:
        print("Error: 'link2' not found in the JSON file.")
        p.disconnect()
        return

    link2_spheres = data['link2']

    # 创建球体
    for sphere in link2_spheres:
        center = sphere['center']
        color = sphere['color'] if 'color' in sphere else [1, 0, 0, 1]  # 默认红色
        radius = 0.02  # 球体半径

        # 创建视觉形状
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=radius,
            rgbaColor=color
        )

        # 创建碰撞形状
        collision_shape_id = p.createCollisionShape(
            shapeType=p.GEOM_SPHERE,
            radius=radius
        )

        # 创建多体对象
        p.createMultiBody(
            baseMass=0,  # 质量为0，保持固定
            baseCollisionShapeIndex=collision_shape_id,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=center  # 直接使用世界坐标系坐标
        )

    # 运行仿真
    while True:
        p.stepSimulation()
        time.sleep(1./240.)  # 模拟时间步

    p.disconnect()


if __name__ == '__main__':
    json_file = 'link_spheres.json'  # 替换为你的 JSON 文件路径
    visualize_link2_spheres(json_file)
