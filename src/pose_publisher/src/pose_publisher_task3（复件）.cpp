#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <cmath>
#include <chrono>

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

    // 原始坐标
    geometry_msgs::msg::Pose pose1_origin, pose2_origin, pose3_origin;
    pose1_origin.position.x = 0.2586; pose1_origin.position.y = 0.17; pose1_origin.position.z = 3;
    pose2_origin.position.x = -0.01856; pose2_origin.position.y = 0.17; pose2_origin.position.z = 1;
    pose3_origin.position.x = 0.12; pose3_origin.position.y = 0.41; pose3_origin.position.z = 2;
    pose1_origin.orientation.w = pose2_origin.orientation.w = pose3_origin.orientation.w = 1.0;

    // 中心点
    geometry_msgs::msg::Pose center;
    center.position.x = (pose1_origin.position.x + pose2_origin.position.x + pose3_origin.position.x) / 3.0;
    center.position.y = (pose1_origin.position.y + pose2_origin.position.y + pose3_origin.position.y) / 3.0;

    // 旋转参数
    const double total_rotation = 120.0;         // 6秒旋转120度
    const int rotate_steps = 600;                // 6秒内600步
    const double angle_per_step = total_rotation / rotate_steps;
    const int pause_steps = 600;                 // 停4秒，400步

    // 旋转和停顿状态
    bool rotating = true;
    int step = 0;
    int total_steps = rotate_steps + pause_steps;

    // 初始坐标用于保存当前坐标
    geometry_msgs::msg::Pose pose1_current = pose1_origin;
    geometry_msgs::msg::Pose pose2_current = pose2_origin;
    geometry_msgs::msg::Pose pose3_current = pose3_origin;

    while (rclcpp::ok()) {
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = node->now();
        pose_array.header.frame_id = "world";

        // 旋转阶段
        if (rotating) {
            pose1_current = rotate_pose(pose1_current, angle_per_step, center);
            pose2_current = rotate_pose(pose2_current, angle_per_step, center);
            pose3_current = rotate_pose(pose3_current, angle_per_step, center);

            // 每步增加一个步骤
            step++;
            if (step >= rotate_steps) {
                rotating = false;  // 旋转完成，开始停顿
                step = 0;          // 重置步骤
            }
        } else {
            // 停顿阶段，继续发布当前坐标
            step++;

            if (step >= pause_steps) {
                rotating = true;   // 停顿完毕，开始旋转
                step = 0;          // 重置步骤
            }
        }

        pose_array.poses = {pose1_current, pose2_current, pose3_current};
        publisher->publish(pose_array);  // 每 0.01 秒都发
        rate.sleep();

    }

    rclcpp::shutdown();
    return 0;
}
