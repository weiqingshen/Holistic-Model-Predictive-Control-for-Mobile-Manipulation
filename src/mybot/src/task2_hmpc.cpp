#include "arm_control.h"
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "Communication.h"
#include <queue>

using namespace std;

int main(int argc, char** argv) {
    rclcpp::init(argc,argv);// 初始化ROS2节点系统，使系统可以处理ROS2的参数和通信功能
    rclcpp::NodeOptions node_options;//
    node_options.automatically_declare_parameters_from_overrides(true);

    // 创建节点 命名为move_group_interface
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);
    auto node_communication = std::make_shared<Communication>();
    // 创建一个单线程执行器
    rclcpp::executors::SingleThreadedExecutor executor;// 添加节点到执行器中，以便使其可以处理回调
    executor.add_node(move_group_node);// 在一个新线程中启动执行器的事件循环，以便节点可以异步处理事件。
    executor.add_node(node_communication);

    std::thread([&]() { executor.spin(); }).detach();

    //实例化
    static const std::string PLANNING_GROUP = "arm_move";
    static const std::string GRIPPER_GROUP = "hand";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface gripper_group(move_group_node, GRIPPER_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects1,collision_objects3,collision_objects2;
    move_group.setMaxVelocityScalingFactor(1);  // 增加速度到 1 倍

    // 创建 Object, TargetPose, GraspPose 实例
    Object object1(move_group_node, "cylinder1");
    Object object2(move_group_node, "cylinder2");
    Object object3(move_group_node, "cylinder3");
    TargetPose place_pose1(move_group_node, move_group, "place_pose1");
    TargetPose place_pose2(move_group_node, move_group, "place_pose2");
    TargetPose place_pose3(move_group_node, move_group, "place_pose3");
    GraspPose grasp_pose(move_group_node, "grasp1");

    position target1,target2,target3;
    // 调用类中的阻塞方法等待消息
    node_communication->waitForMessage();

    // 获取接收到的目标位置信息并赋值给 target1
    node_communication->getPosition(target1);

    // object1.setPosition(target1.x1,target1.y1,0.115);
    //
    // // 获取 CollisionObject 并添加到场景中
    // collision_objects1.push_back(object1.getCollisionObject());
    //
    // // 批量添加物体到场景中
    // planning_scene_interface.addCollisionObjects(collision_objects1);

    object1.setPosition(target1.x1,target1.y1,0.115);
    collision_objects1.push_back(object1.getCollisionObject());

    // 设置圆柱体颜色为红色
    std::vector<moveit_msgs::msg::ObjectColor> object_colors;
    moveit_msgs::msg::ObjectColor color_cylinder;
    color_cylinder.id = object1.getName();
    color_cylinder.color.r = 1.0f;
    color_cylinder.color.g = 0.0f;
    color_cylinder.color.b = 0.0f;
    color_cylinder.color.a = 1.0f;  // 不透明
    object_colors.push_back(color_cylinder);

    planning_scene_interface.applyCollisionObjects(collision_objects1, object_colors);


    // 执行 ManipulatorAction
    ManipulatorAction manipulator_action(move_group_node, move_group, planning_scene_interface, gripper_group);
    // manipulator_action.executeAction_trajectory(object1, place_pose1, grasp_pose,"","","","");
    manipulator_action.executeAction_mpc_trajectory(object1, place_pose1, grasp_pose);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::vector<std::string> objects_to_remove1 = {object1.getName()};
    planning_scene_interface.removeCollisionObjects(objects_to_remove1);

    rclcpp::shutdown();
    return 0;
}
