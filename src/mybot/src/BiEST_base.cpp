#include "arm_control.h"
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "Communication.h"
#include <queue>

using namespace std;

// 添加开口长方体壳子的函数（带顶部，可选开哪些墙，且带橙色）
void add_open_box_obstacle(
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
    const std::shared_ptr<rclcpp::Node>& node,
    const std::string& obstacle_name_prefix,
    double center_x, double center_y, double center_z,
    double length, double width, double height,
    double thickness,
    bool add_left = true, bool add_right = true,
    bool add_front = true, bool add_back = true,
    bool add_top = true)
{
    std::string frame_id = "world";
    if (node->has_parameter("robot_description_planning")) {
        frame_id = node->get_parameter("robot_description_planning").as_string();
    }

    std::vector<moveit_msgs::msg::CollisionObject> walls;
    std::vector<moveit_msgs::msg::ObjectColor> colors;

    auto setup_wall = [&](const std::string& id, double dim_x, double dim_y, double dim_z,
                          double pos_x, double pos_y, double pos_z)
    {
        moveit_msgs::msg::CollisionObject wall;
        wall.header.frame_id = frame_id;
        wall.id = id;
        wall.primitives.resize(1);
        wall.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        wall.primitives[0].dimensions = {dim_x, dim_y, dim_z};
        wall.operation = wall.ADD;

        geometry_msgs::msg::Pose pose;
        pose.position.x = pos_x;
        pose.position.y = pos_y;
        pose.position.z = pos_z;
        pose.orientation.w = 1.0;
        wall.pose = pose;

        walls.push_back(wall);

        // 设置每个墙壁的颜色（橙色）
        moveit_msgs::msg::ObjectColor color;
        color.id = id;
        color.color.r = 1.0f; // 红色成分
        color.color.g = 0.5f; // 绿色成分
        color.color.b = 0.0f; // 蓝色成分
        color.color.a = 1.0f; // 透明度
        colors.push_back(color);
    };

    if (add_left) {
        setup_wall(obstacle_name_prefix + "_left",
                   thickness, width + 2 * thickness, height + (add_top ? thickness : 0),
                   center_x - (length / 2.0 + thickness / 2.0),
                   center_y,
                   center_z + (add_top ? thickness / 2.0 : 0));
    }
    if (add_right) {
        setup_wall(obstacle_name_prefix + "_right",
                   thickness, width + 2 * thickness, height + (add_top ? thickness : 0),
                   center_x + (length / 2.0 + thickness / 2.0),
                   center_y,
                   center_z + (add_top ? thickness / 2.0 : 0));
    }
    if (add_front) {
        setup_wall(obstacle_name_prefix + "_front",
                   length, thickness, height + (add_top ? thickness : 0),
                   center_x,
                   center_y + (width / 2.0 + thickness / 2.0),
                   center_z + (add_top ? thickness / 2.0 : 0));
    }
    if (add_back) {
        setup_wall(obstacle_name_prefix + "_back",
                   length, thickness, height + (add_top ? thickness : 0),
                   center_x,
                   center_y - (width / 2.0 + thickness / 2.0),
                   center_z + (add_top ? thickness / 2.0 : 0));
    }
    if (add_top) {
        setup_wall(obstacle_name_prefix + "_top",
                   length + 2 * thickness, width + 2 * thickness, thickness,
                   center_x,
                   center_y,
                   center_z + height / 2.0 + thickness / 2.0);
    }

    planning_scene_interface.applyCollisionObjects(walls, colors);
}


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
    //FMTk会规划失败 BFMTk就可以
    //move_group.setPlannerId("BFMTkConfigDefault");

    //move_group.setPlannerId("BFPIECEkConfigDefault");
    //可以

    move_group.setPlannerId("BiESTkConfigDefault");
	//可以

    //move_group.setPlannerId("BiTRRTkConfigDefault");
	//可以

    //move_group.setPlannerId("PRMstarkConfigDefault");
	//可以

//    move_group.setPlannerId("SPARStwokConfigDefault");
	//可以 但是要规划蛮久

    // move_group.setPlannerId("FMTkConfigDefault");
    //move_group.setPlannerId("BFMTkConfigDefault");
    move_group.setMaxVelocityScalingFactor(1);

    // ---- 添加壳子障碍物 ----
    std::string shell_prefix = "my_shell";
    add_open_box_obstacle(
        planning_scene_interface,
        move_group_node,
        "my_shell",
        0.0, 0.7, 0.2,  // 中心位置 (X, Y, Z)
        0.4, 0.4, 0.4,   // 长宽高
        0.02,            // 壁厚
        true,  // left
        true,  // right
        true,  // front
        false, // back (不加后面)
        true   // top (加顶部)
    );

    // 创建 Object, TargetPose, GraspPose 实例
    Object object1(move_group_node, "cylinder1");
    Object object2(move_group_node, "cylinder2");
    Object object3(move_group_node, "cylinder3");
    TargetPose place_pose1(move_group_node, move_group, "place_pose4");
    TargetPose place_pose2(move_group_node, move_group, "place_pose2");
    TargetPose place_pose3(move_group_node, move_group, "place_pose3");
    GraspPose grasp_pose(move_group_node, "grasp1");

    position target1,target2,target3;
    node_communication->waitForMessage();
    node_communication->getPosition(target1);

    object1.setPosition(target1.x1,target1.y1,0.035);
    collision_objects1.push_back(object1.getCollisionObject());
    planning_scene_interface.addCollisionObjects(collision_objects1);

    ManipulatorAction manipulator_action(move_group_node, move_group, planning_scene_interface, gripper_group);
    manipulator_action.execute_contrast(object1, place_pose1, grasp_pose);

    // 移除物体
    std::vector<std::string> objects_to_remove1 = {object1.getName()};
    planning_scene_interface.removeCollisionObjects(objects_to_remove1);

//    // 机械臂回到初始位
//    move_group.setNamedTarget("initial");
//    move_group.move();

    // ---- 移除壳子障碍物 ----
    std::vector<std::string> shell_ids = {
        shell_prefix + "_left",
        shell_prefix + "_right",
        shell_prefix + "_front",
        shell_prefix + "_back",
        shell_prefix+"_top"
    };
    planning_scene_interface.removeCollisionObjects(shell_ids);



    rclcpp::shutdown();
    return 0;
}
