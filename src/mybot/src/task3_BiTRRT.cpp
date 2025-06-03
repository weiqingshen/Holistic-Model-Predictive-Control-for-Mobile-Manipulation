#include "arm_control.h"
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "Communication.h"
#include <queue>

using namespace std;

// 主程序
int main(int argc, char** argv) {
    rclcpp::init(argc,argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    auto move_group_node = rclcpp::Node::make_shared("move_group_interface", node_options);
    auto node_communication = std::make_shared<Communication>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    executor.add_node(node_communication);

    std::thread([&]() { executor.spin(); }).detach();

    static const std::string PLANNING_GROUP = "arm_move";
    static const std::string GRIPPER_GROUP = "hand";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface gripper_group(move_group_node, GRIPPER_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects1,collision_objects3,collision_objects2;

    //move_group.setPlannerId("BFMTkConfigDefault");

    // move_group.setPlannerId("BFPIECEkConfigDefault");
    //可以

    //move_group.setPlannerId("BiESTkConfigDefault");
    //可以

    //move_group.setPlannerId("BiTRRTkConfigDefault");
    //可以

    move_group.setPlannerId("PRMstarkConfigDefault");
    //可以用于动态抓取对比
    //
    //move_group.setPlannerId("SPARStwokConfigDefault");
    //可以 但是要规划蛮久

    // move_group.setPlannerId("BFMTkConfigDefault");
    // move_group.setPlannerId("BiTRRTkConfigDefault");

    move_group.setMaxVelocityScalingFactor(1);

    // 创建 Object, TargetPose, GraspPose 实例
    Object object1(move_group_node, "cylinder1");

    GraspPose grasp_pose(move_group_node, "grasp1");
    ManipulatorAction manipulator_action(move_group_node, move_group, planning_scene_interface, gripper_group);
    position target1;
//    node_communication->waitForMessage();
//    node_communication->getPosition(target1);

//    object1.setPosition(target1.x1,target1.y1,0.035);
//    collision_objects1.push_back(object1.getCollisionObject());
//    planning_scene_interface.addCollisionObjects(collision_objects1);
//
//    manipulator_action.execute_dynamic(object1, grasp_pose);
    // 设置循环次数
    const int max_iterations = 100;
    int iteration_count = 0;

    // 等待并获取新位置
    node_communication->waitForMessage();

    while (rclcpp::ok() && iteration_count < max_iterations) {

        node_communication->getPosition(target1);

        // 设置物体位置并添加到场景
        object1.setPosition(target1.x1, target1.y1, 0.035);
        // 更新场景中的物体
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        collision_objects.push_back(object1.getCollisionObject());

        planning_scene_interface.addCollisionObjects(collision_objects);
        // 执行抓取动作
        manipulator_action.execute_dynamic(object1, grasp_pose);

        node_communication->subscribeToPositionUpdates();
        // 移除物体
        std::vector<std::string> objects_to_remove = {object1.getName()};
        planning_scene_interface.removeCollisionObjects(objects_to_remove);

        iteration_count++;
        RCLCPP_INFO(move_group_node->get_logger(), "Completed iteration %d of %d", iteration_count, max_iterations);
//        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 移除物体
    std::vector<std::string> objects_to_remove1 = {object1.getName()};
    planning_scene_interface.removeCollisionObjects(objects_to_remove1);

//    // 机械臂回到初始位
//    move_group.setNamedTarget("initial");
//    move_group.move();


    rclcpp::shutdown();
    return 0;
}
