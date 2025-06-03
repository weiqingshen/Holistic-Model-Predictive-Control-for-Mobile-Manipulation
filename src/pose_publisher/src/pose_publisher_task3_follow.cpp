#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <cmath>

// 将角度转换为弧度
double degree_to_radian(double degree) {
    return degree * M_PI / 180.0;
}

// 计算旋转后的位姿
geometry_msgs::msg::Pose rotate_pose(const geometry_msgs::msg::Pose& pose, double angle_deg, const geometry_msgs::msg::Pose& center) {
    double angle_rad = degree_to_radian(angle_deg);
    double dx = pose.position.x - center.position.x;
    double dy = pose.position.y - center.position.y;

    geometry_msgs::msg::Pose new_pose = pose;
    new_pose.position.x = center.position.x + dx * cos(angle_rad) - dy * sin(angle_rad);
    new_pose.position.y = center.position.y + dx * sin(angle_rad) + dy * cos(angle_rad);
    return new_pose;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pose_publisher");
    auto publisher = node->create_publisher<geometry_msgs::msg::PoseArray>("arm_pose", 10);
    rclcpp::Rate rate(100);  // 100Hz = 0.01s

    // 圆心和半径
    geometry_msgs::msg::Pose center;
    center.position.x = 0.0;
    center.position.y = 0.4;
    center.position.z = 0.0;

    double radius = 0.1;

    // 初始角度
    double angle1 = 0;
    double angle2 = 120;
    double angle3 = 240;

    // 初始点
    geometry_msgs::msg::Pose pose1, pose2, pose3;
    pose1.position.x = center.position.x + radius * cos(degree_to_radian(angle1));
    pose1.position.y = center.position.y + radius * sin(degree_to_radian(angle1));
    pose1.position.z = 1.0;

    pose2.position.x = center.position.x + radius * cos(degree_to_radian(angle2));
    pose2.position.y = center.position.y + radius * sin(degree_to_radian(angle2));
    pose2.position.z = 1.0;

    pose3.position.x = center.position.x + radius * cos(degree_to_radian(angle3));
    pose3.position.y = center.position.y + radius * sin(degree_to_radian(angle3));
    pose3.position.z = 1.0;

    pose1.orientation.w = pose2.orientation.w = pose3.orientation.w = 1.0;

    // 旋转参数
    const double total_rotation = 120.0; // 6秒转120度
    const int rotate_steps = 800;
    const double angle_per_step = total_rotation / rotate_steps;
    const int pause_steps = 600;

    bool rotating = true;
    int step = 0;

    while (rclcpp::ok()) {
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = node->now();
        pose_array.header.frame_id = "world";

        if (rotating) {
	    pose1 = rotate_pose(pose1, -angle_per_step, center);
	    pose2 = rotate_pose(pose2, -angle_per_step, center);
	    pose3 = rotate_pose(pose3, -angle_per_step, center);

            step++;
            if (step >= rotate_steps) {
                rotating = false;
                step = 0;
            }
        } else {
            step++;
            if (step >= pause_steps) {
                rotating = true;
                step = 0;
            }
        }

        pose_array.poses = {pose1, pose2, pose3};
        publisher->publish(pose_array);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

