import pybullet as p
import pybullet_data
import time
import json

def visualize_relative_coords(link_name="link1", relative_json="link_spheres_relative.json", sphere_radius=0.02):
    # 加载 JSON 文件
    with open(relative_json, "r") as f:
        data = json.load(f)

    if link_name not in data:
        print(f"⚠️ link: {link_name} 不存在于 {relative_json} 中")
        return

    positions = data[link_name]

    # 初始化 PyBullet
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

    # 可视化球体
    for pos in positions:
        visual_shape = p.createVisualShape(
            shapeType=p.GEOM_SPHERE,
            radius=sphere_radius,
            rgbaColor=[0, 1, 0, 0.6]  # 绿色透明
        )
        p.createMultiBody(
            baseVisualShapeIndex=visual_shape,
            basePosition=pos  # ⬅️ 相对坐标，直接作为世界坐标展示
        )

    print(f"✅ 成功显示 {link_name} 的 {len(positions)} 个球体（按相对坐标绘制）")

    while True:
        p.stepSimulation()
        time.sleep(1. / 240.)

# 🚀 启动程序
visualize_relative_coords("link4")
