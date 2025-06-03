import pybullet as p
import pybullet_data
import numpy as np
import time
import math

def draw_coordinate_system(base_position, base_orientation, scale=0.1):
    """绘制局部坐标系"""
    # 将四元数转换为旋转矩阵
    rotation_matrix = p.getMatrixFromQuaternion(base_orientation)
    rotation_matrix = np.array(rotation_matrix).reshape((3, 3))

    # X轴（红色）
    x_axis = rotation_matrix.dot(np.array([scale, 0, 0]))
    p.addUserDebugLine(base_position,
                        np.array(base_position) + x_axis,
                        lineColorRGB=[1, 0, 0],
                        lifeTime=0)
    # Y轴（绿色）
    y_axis = rotation_matrix.dot(np.array([0, scale, 0]))
    p.addUserDebugLine(base_position,
                        np.array(base_position) + y_axis,
                        lineColorRGB=[0, 1, 0],
                        lifeTime=0)
    # Z轴（蓝色）
    z_axis = rotation_matrix.dot(np.array([0, 0, scale]))
    p.addUserDebugLine(base_position,
                        np.array(base_position) + z_axis,
                        lineColorRGB=[0, 0, 1],
                        lifeTime=0)

def main():
    # 启动 PyBullet
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # 加载数据路径
    p.setGravity(0, 0, -9.81)

    # 加载 URDF 文件
    robot_id = p.loadURDF("/home/fins/myrobot_move/src/robot_arm_description/urdf/robot_arm_description.urdf",
                         basePosition=[0,0,0])

    # 获取链接数量
    num_joints = p.getNumJoints(robot_id)

    # 运行仿真
    while True:
        p.stepSimulation()

        # 遍历每个链接
        for joint_index in range(num_joints):
            # 获取链接的状态
            link_state = p.getLinkState(robot_id, joint_index, computeForwardKinematics=True)
            position = link_state[0]  # 获取位置
            orientation = link_state[1]  # 获取朝向

            # 绘制局部坐标系
            draw_coordinate_system(position, orientation)

        time.sleep(1./240.)

if __name__ == "__main__":
    main()
