输入为关节姿态角矩阵
对于每个link以link为坐标系将其分解出的球的坐标矩阵记录
直接读取记录，按照输入的姿态角计算计算出零件对应的球在大地坐标系下的坐标
对每个link进行球形分解后，计算其球体距离其他link分解出的球体的最短距离，其中忽略
**定义需要跳过计算的关节对**
const std::set<std::pair<std::string, std::string>> disable_collisions = {
    {"base_link", "link1"}, {"link1", "link2"}, {"link1", "link3"},
    {"link2", "link3"}, {"link3", "link4"}, {"link3", "link5"},
    {"link3", "link6"}, {"link4", "link5"}, {"link5", "link6"},
    {"link5", "link_hand"}, {"link5", "link_left"}, {"link5", "link_right"},
    {"link6", "link_hand"}, {"link6", "link_left"}, {"link6", "link_right"},
    {"link_hand", "link_left"}, {"link_hand", "link_right"},{"link_left", "link_right"}
};
输出为解析解


代码看后缀有final的就行



下面给出一种基于解析方法的思路和伪代码示例，说明如何基于关节姿态角矩阵对每个 link 的球形分解进行处理，并计算各个 link 之间球体的最短距离，同时跳过预定义的关节对。以下内容分为以下几个部分：

---

## 1. 问题描述

假设在已有的球形分解中，每个 link 的球中心（例如存储在文件中）默认是以 link 坐标系表示。现在要求：

- **输入：** 各个关节的姿态角矩阵（例如欧拉角或者直接给出变换矩阵）。
- **处理：**
  1. 根据每个 link 的关节姿态，计算该 link 从局部坐标系到大地（世界）坐标系的变换矩阵。
  2. 读取事先分解保存的球体位置信息，进而将每个球体坐标从 link 坐标系变换到世界坐标系。变换公式为
     \[
     \mathbf{s}_{world} = \mathbf{T} \times \mathbf{s}_{local} = \mathbf{R} \, \mathbf{s}_{local} + \mathbf{t}
     \]
     其中 \(\mathbf{R}\) 为旋转矩阵，\(\mathbf{t}\) 为平移向量。
  3. 对于每对属于不同 link 的球体（注意：需要跳过在**disable_collisions**集合内定义的关节对），计算两球之间的距离。由于每个球的半径均为 **MAX_RADIUS**，两个球的最小距离可写为
     \[
     d_{min} = \max\left(0,\;\|\mathbf{s}_1 - \mathbf{s}_2\| - 2 \cdot \text{MAX\_RADIUS}\right)
     \]
     如果计算结果为负，则意味着球体有重叠，取零作为最小距离。
- **输出：** 对于每个 link，给出其所有球与其他 link 球体的最小距离，形成解析解（Closed-form solution）。

---

## 2. 数据转换及计算流程

### 2.1 坐标系变换

每个 link 都会对应一个位姿矩阵 \(T_{link}\)。假设输入为关节的姿态角（例如欧拉角），转换流程如下：

1. **旋转矩阵计算：** 根据欧拉角（或其他角度表示）计算旋转矩阵 \(R_{link}\)。
2. **平移向量：** 若关节给出了平移信息，则作为 \(\mathbf{t}_{link}\)。
3. **组合变换：** 最终的变换矩阵为
   \[
   T_{link} = \begin{bmatrix} R_{link} & \mathbf{t}_{link} \\ 0 & 1 \end{bmatrix}
   \]
4. **变换操作：** 对于球在 link 坐标系中的位置 \(\mathbf{s}_{local}\)（注意通常为 3 维向量），扩展为齐次坐标（末尾附1），计算
   \[
   \mathbf{s}_{world} = T_{link} \cdot \begin{bmatrix}\mathbf{s}_{local} \\ 1 \end{bmatrix}
   \]
   最终结果去掉齐次分量即得到球体在世界坐标系下的坐标。

### 2.2 球体之间距离计算

对于两个转换到世界坐标系下的球体中心 \(\mathbf{s}_1\) 和 \(\mathbf{s}_2\)，计算欧式距离：
\[
d = \|\mathbf{s}_1 - \mathbf{s}_2\|
\]
然后从该距离中减去两个球体的直径（注：由于我们关注的是边界之间的最短距离，所以直接减去两个半径之和），即：
\[
d_{min} = \max\left(0,\, d - \left(\text{MAX\_RADIUS} + \text{MAX\_RADIUS}\right)\right) = \max\left(0,\, d - 2 \cdot \text{MAX\_RADIUS}\right)
\]

### 2.3 跳过预定义的关节对

预定义的跳过计算的关节对保存在一个集合（例如 C++ 的 `std::set<std::pair<std::string, std::string>>`），例如：
```cpp
const std::set<std::pair<std::string, std::string>> disable_collisions = {
    {"base_link", "link1"}, {"link1", "link2"}, {"link1", "link3"},
    {"link2", "link3"}, {"link3", "link4"}, {"link3", "link5"},
    {"link3", "link6"}, {"link4", "link5"}, {"link5", "link6"},
    {"link5", "link_hand"}, {"link5", "link_left"}, {"link5", "link_right"},
    {"link6", "link_hand"}, {"link6", "link_left"}, {"link6", "link_right"},
    {"link_hand", "link_left"}, {"link_hand", "link_right"}, {"link_left", "link_right"}
};
```
在遍历各个 link 之间的组合时，需要判断当前 link 对（例如 linkA 与 linkB）是否在该集合中（注意无序性，可以双向查询），如果在集合中，则跳过其之间的距离计算。

---
def compute_transforms(joint_angles):
    T = {}

    # base_link的变换矩阵
    T["base_link"] = np.eye(4)

    # joint1的变换矩阵
    rpy1 = [0, 0, 0] #    特意跳过joint——angles【0】，因为这个是baselink的角度，而在本程序中默认baselink不动
    xyz1 = [0, 0, 0.1284]
    R1 = euler_to_rotation_matrix(rpy1)
    T1 = np.eye(4)
    T1[:3, :3] = R1
    T1[:3, 3] = xyz1
    T["joint1"] = T["base_link"].dot(T1)


    # joint2的变换矩阵
    rpy3 = [1.5708, 0, 1.5708]
    xyz3 = [0, 0, 0.0927]
    R3 = euler_to_rotation_matrix(rpy3)
    T3 = np.eye(4)
    T3[:3, :3] = R3
    T3[:3, 3] = xyz3
    T["joint2"] = T["joint1"].dot(T3)


    # joint3的变换矩阵
    rpy5 = [ - 3.1416, 0, -1.5708]
    xyz5 = [0, 0.22, 0]
    R5 = euler_to_rotation_matrix(rpy5)
    T5 = np.eye(4)
    T5[:3, :3] = R5
    T5[:3, 3] = xyz5
    T["joint3"] = T["joint2"].dot(T5)


    # joint4的变换矩阵
    rpy7 = [1.5708,0, 0]
    xyz7 = [0, 0, 0]
    R7 = euler_to_rotation_matrix(rpy7)
    T7 = np.eye(4)
    T7[:3, :3] = R7
    T7[:3, 3] = xyz7
    T["joint4"] = T["joint3"].dot(T7)


    # joint5的变换矩阵
    rpy9 = [0, 0, 0]
    xyz9 = [0, 0, 0]
    R9 = euler_to_rotation_matrix(rpy9)
    T9 = np.eye(4)
    T9[:3, :3] = R9
    T9[:3, 3] = xyz9
    T["joint5"] = T["joint4"].dot(T9)


    # joint6的变换矩阵
    rpy11 = [1.5708, 0, 0]]
    xyz11 = [0, 0, 0]
    R11 = euler_to_rotation_matrix(rpy11)
    T11 = np.eye(4)
    T11[:3, :3] = R11
    T11[:3, 3] = xyz11
    T["joint6"] = T["joint5"].dot(T11)


    # joint_hand的变换矩阵
    rpy13 = [0, 0, 0]
    xyz13 = [0, 0, 0]
    R13 = euler_to_rotation_matrix(rpy13)
    T13 = np.eye(4)
    T13[:3, :3] = R13
    T13[:3, 3] = xyz13
    T["joint_hand"] = T["joint6"].dot(T13)


    # joint_right的变换矩阵
    rpy15 = [-1.5708, 0, 0]
    xyz15 = [0, 0, 0.1465]
    R15 = euler_to_rotation_matrix(rpy15)
    T15 = np.eye(4)
    T15[:3, :3] = R15
    T15[:3, 3] = xyz15
    T["joint_right"] = T["joint_hand"].dot(T15)


    # joint_left的变换矩阵
    rpy17 = [-1.5708, 0, 0]
    xyz17 = [0, 0, 0.1465]
    R17 = euler_to_rotation_matrix(rpy17)
    T17 = np.eye(4)
    T17[:3, :3] = R17
    T17[:3, 3] = xyz17
    T["joint_left"] = T["joint_hand"].dot(T17)


    return T



0.5 0.5 0.5  0 -1 0 0 -1 1 0.01
0 0 0 0 0 0 0 0 0 0


下一步：所有坐标系可视化排查错误（现在的问题似乎是旋转正确但是平移有问题）


现在的问题是输入的角度没有被代入关节旋转矩阵里

三步走：
先存所有球的绝对坐标
对绝对坐标进行joint变换矩阵变换，得相对坐标
对相对坐标，按照我们输入的角度再次进行矩阵变换，得最终的绝对坐标


封装函数：输入10个角度，找到最近小球，输出它们的绝对坐标系矩阵（带未知数）