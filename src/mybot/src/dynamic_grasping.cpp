#include "arm_control.h"
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "Communication.h"
#include <queue>

using namespace std;

bool gripper_state_received = false; // 标志位，表示是否已抓取物体

void gripper_state_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    // 当接收到gripper_state为3.0时，标记为已经抓取物体
    if (msg->data == 3.0) {
        gripper_state_received = true;
        RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "Gripper has grabbed the object.");
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc,argv);// 初始化ROS2节点系统，使系统可以处理ROS2的参数和通信功能
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    // 创建节点 命名为move_group_interface
    auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);
    auto node_communication = std::make_shared<Communication>();
    // 创建一个单线程执行器
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(node_communication);

    std::thread([&]() { executor.spin(); }).detach();

    // 订阅gripper_state话题
    auto gripper_state_subscription = move_group_node->create_subscription<std_msgs::msg::Float32>(
        "/gripper_state", 10, gripper_state_callback);

    //实例化
    static const std::string PLANNING_GROUP = "arm_move";
    static const std::string GRIPPER_GROUP = "hand";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface gripper_group(move_group_node, GRIPPER_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects1,collision_objects3,collision_objects2;
    move_group.setMaxVelocityScalingFactor(1);

    // 创建 Object, TargetPose, GraspPose 实例
    Object object1(move_group_node, "cylinder1");
    Object object2(move_group_node, "cylinder2");
    Object object3(move_group_node, "cylinder3");
    TargetPose place_pose1(move_group_node, move_group, "place_pose1");
    TargetPose place_pose2(move_group_node, move_group, "place_pose2");
    TargetPose place_pose3(move_group_node, move_group, "place_pose3");
    GraspPose grasp_pose(move_group_node, "grasp1");

    position target1,target2,target3;

    node_communication->waitForMessage();

    while (rclcpp::ok()) {


        node_communication->getPosition(target1);

        // 更新物体的位置
        object1.setPosition(target1.x1, target1.y1, 0.115);

        // 更新场景中的物体
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(object1.getCollisionObject());

        planning_scene_interface.addCollisionObjects(collision_objects);

        // // 打印当前物体位置
        // RCLCPP_INFO(move_group_node->get_logger(), "Updated Object position: x=%.2f, y=%.2f, z=%.2f", target1.x1, target1.y1, 0.115);

        // 每隔一段时间更新一次物体的位置
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        node_communication->subscribeToPositionUpdates();

        if (gripper_state_received) {
            // 如果抓取了物体，停止更新物体位置
            RCLCPP_INFO(move_group_node->get_logger(), "Gripper state is 3.0. Stopping object position updates.");
            break; // 停止更新物体位置
        }

        std::vector<std::string> objects_to_remove1 = {object1.getName()};
        planning_scene_interface.removeCollisionObjects(objects_to_remove1);
    }

    rclcpp::shutdown();
    return 0;
}
