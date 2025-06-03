#现在我有一个urdf文件（"/home/fins/myrobot_move/src/robot_arm_description/urdf/robot_arm_description.urdf"）
# 我将输入一个10个角度（弧度制）的数组，要求让文件里的link按照我输入的角度旋转（第一个角度对应baselink，第二个对应link1，第三个对应link2，。。。，第八个对应linkhand，第九个对应linkleft，第十个对应linkrighgt）
import pybullet as p
import pybullet_data
import time
import math

# 连接 PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# 加载 URDF 机器人
robot_id = p.loadURDF("/home/fins/myrobot_move/src/robot_arm_description/urdf/robot_arm_description.urdf", useFixedBase=True)

# 打印所有关节的信息，确认 joint index 和 name 的对应关系
num_joints = p.getNumJoints(robot_id)
print("== 机器人关节信息 ==")
for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    print(f"Joint {i}: name={info[1].decode('utf-8')}")

# 让用户输入10个关节角度（单位为弧度）
input_str = input("请输入10个关节角度（以空格分隔，单位为弧度）：\n")
joint_angles = list(map(float, input_str.strip().split()))
if len(joint_angles) != 10:
    raise ValueError("请输入恰好10个关节角度！")

# 将输入的角度映射到正确的关节编号
joint_name_to_index = {
    'joint1': 3,
    'joint2': 4,
    'joint3': 5,
    'joint4': 6,
    'joint5': 7,
    'joint6': 8,
    'joint_hand': 9,
    'joint_right': 10,
    'joint_left': 11
}

mapping = [
    ('joint1', joint_angles[0]),
    ('joint2', joint_angles[1]),
    ('joint3', joint_angles[2]),
    ('joint4', joint_angles[3]),
    ('joint5', joint_angles[4]),
    ('joint6', joint_angles[5]),
    ('joint_hand', joint_angles[6]),
    ('joint_right', joint_angles[7]),
    ('joint_left', joint_angles[8])
    # joint_angles[9] 保留给 base_link（默认不动）
]

# 设置关节角度
for joint_name, angle in mapping:
    joint_index = joint_name_to_index[joint_name]
    p.resetJointState(robot_id, joint_index, angle)

print("角度设置完成，进入仿真可视化...")

# 保持仿真运行
while True:
    p.stepSimulation()
    time.sleep(1 / 240.0)

# 断开连接（不会执行，除非中断）
p.disconnect()
