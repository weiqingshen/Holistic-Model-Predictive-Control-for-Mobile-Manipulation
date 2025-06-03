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
    //auto node_communication = std::make_shared<Communication>();
    // 创建一个单线程执行器
    rclcpp::executors::SingleThreadedExecutor executor;// 添加节点到执行器中，以便使其可以处理回调
    executor.add_node(move_group_node);// 在一个新线程中启动执行器的事件循环，以便节点可以异步处理事件。
    // executor.add_node(node_communication);

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
//    move_group.setPlannerId("BFMTkConfigDefault");
    // 执行 ManipulatorAction
    ManipulatorAction manipulator_action(move_group_node, move_group, planning_scene_interface, gripper_group);

    manipulator_action.dynamic_grasping(object1,place_pose1,grasp_pose,false);

    std::vector<std::string> objects_to_remove1 = {object1.getName()};
    planning_scene_interface.removeCollisionObjects(objects_to_remove1);

    rclcpp::shutdown();
    return 0;
}
