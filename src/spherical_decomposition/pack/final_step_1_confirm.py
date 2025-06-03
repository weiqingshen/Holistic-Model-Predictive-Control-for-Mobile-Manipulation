import pybullet as p
import pybullet_data
import time
import json

# 与之前保持一致的颜色列表
COLORS = [
    [1, 0, 0, 0.3], [0, 1, 0, 0.3], [0, 0, 1, 0.3],
    [1, 1, 0, 0.3], [0, 1, 1, 0.3], [1, 0, 1, 0.3],
    [1, 0.5, 0, 0.3], [0.5, 0, 0.5, 0.3], [0.5, 0.5, 0.5, 0.3],
]

SPHERE_RADIUS = 0.01  # 和之前保持一致

def visualize_spheres_from_json(json_path):
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")  # 加个地面方便观察

    with open(json_path, "r") as f:
        link_spheres = json.load(f)

    color_index = 0
    for link_name, spheres in link_spheres.items():
        color = COLORS[color_index % len(COLORS)]
        color_index += 1

        for pos in spheres:
            vs_id = p.createVisualShape(
                shapeType=p.GEOM_SPHERE,
                radius=SPHERE_RADIUS,
                rgbaColor=color
            )
            p.createMultiBody(baseVisualShapeIndex=vs_id, basePosition=pos, baseMass=0)

        print(f"Link [{link_name}] 渲染了 {len(spheres)} 个球。")

    print("✅ 所有球体已加载，按 Ctrl+C 或关闭窗口退出。")

    while True:
        p.stepSimulation()
        time.sleep(1. / 240.)

    p.disconnect()

# ✅ 执行入口
visualize_spheres_from_json("link_spheres.json")
