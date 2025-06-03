

# Holistic-Model-Predictive-Control-for-Mobile-Manipulation

read me
=======

# SRC

基本程序

arm_control.cpp/.h 机械臂运动规划主要封装的程序 包括主要的类 抓取 物体 执行

communication.cpp/.h 通讯程序 控制接受信息 阻断 接受状态机和相机信息

trajectory.cpp/h 轨迹保存和执行保存的轨迹 读取config里面的yaml 

trajectory_buffer.cpp 缓存轨迹点 分批次发送给fines_serial  发送下位机执行

执行程序

arm_execute_circle 转盘

arm_execute_processing 放下在拿起

arm_execute_storage 放下

arm_execute_total 整个流程

后面加了trajectory的是 中间有轨迹被保存 会直接执行轨迹而不是规划

dynamic_grasping 实现障碍物一直旋转

# launch

只要用到demo就行了 直接运行demo启动程序 后面在运行src

# config

param.yaml记录物体的大小 位置的提前已知

circle11.yaml 轨迹

processing11.yaml 轨迹

# Rviz

对于移动机械臂
在RViz里面打开的时候注意
Displays 里面的Fixed Frame 要选择world
Views里面的Target Frame 也要选择world

## python调用cpp库如果发现库没有更新

rm -rf build/ install/ log/
colcon build --symlink-install --cmake-force-configure

## mpc_python

v10: 初步框架实现

v11: 主要用来实现机械臂和底盘的优先与否 在这一版本代码里面 没有考虑避障

v12: 来实现动态抓取的第一次尝试 初步实现分段抓取 但是发现思路不对 没办法确保cpp程序执行完 但是可以作为静态抓取的一个思路 长距离抓取的时候适用 或者说需要比较多步骤来进行的时候完全可以用

v13: 动态抓取进一步尝试 写了一半 写的是接受name 发现思路错了

v14: 动态抓取正确的思路 把调配任务交给cpp文件完成

v15:发现动态抓取每次调用python程序耗时1s 其中run_mpc只需要0.07秒 所以尝试写一个不用重复初始化的python程序 成功实现 非常丝滑

v16: 实现任务分层 control units

v17: 实现指定轨迹的mpc跟随算法 成功实现

v18:尝试加一个连续性惩罚

v19: 从v15改进过来 完善动态抓取

v20: 加一个自避障

v21:从v19 改过来 在执行动态抓取的时候加一个碰撞检测 不执行会碰撞的步骤

v22:从v18改进过来 输出速度加速度等图像 用于论文出图

v23:从v21改变而来 加一个对比 底盘固定

v24:从v16改进过来 加入避障

v25:避障改良版本

ps:26和27需要一起改进

v26:避障继续改良, 不考虑加入物体避障

v27：从v26改进过来，加入抓取物体避障

28：本来是为了21抓完 往会放 

29:和21对比 动态follow

30:从29改进过来 加一个圆盘的约束

task1仿真数据表：

ros2 lauch mybot move_group_v3.launch

conda activate

然后运行

ros2 launch mybot task1_hmpc.launch

运行对比的

ros2 launch mybot BiTRRT_base.launch

依次类推

一共对比6中算法

BFMT BFPIECE BiEST BiTRRT SPARStwo PRMstar

数据保存：

运行python 文件里面的plot_subscribe

cd my_pose_project_

source install 

ros2 run pose_publisher pose_publisher_task1  (_random)

改进一下别人的动态跟随算法

把task2的图画一下

把task3加个避障圆柱

task1\2\3的仿真和实际的对比出视频

> > > > > > > 
