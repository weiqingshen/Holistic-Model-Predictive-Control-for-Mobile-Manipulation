#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

int main(int argc, char *argv[]) {
    // 初始化 ROS 2
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = rclcpp::Node::make_shared("pose_publisher");

    // 创建发布者，发布到 "output_topic"
    auto publisher = node->create_publisher<geometry_msgs::msg::PoseArray>("arm_pose", 10);

    // 定时器，发布 PoseArray 消息 0.012 0.24744 0.01
    auto timer = node->create_wall_timer(
        std::chrono::seconds(1),
        [&]() {
            geometry_msgs::msg::PoseArray pose_array_msg;
            pose_array_msg.header.stamp = node->now();
            pose_array_msg.header.frame_id = "world";

            // 创建三个 Pose，只设置 x 和 y，z 设为 0.0
            geometry_msgs::msg::Pose pose1;
            pose1.position.x = 3.0836233;
            pose1.position.y = 2.599458;
            pose1.position.z = 3;  // z = 0
            pose1.orientation.w = 1.0;  // 默认无旋转
            geometry_msgs::msg::Pose pose2;
            pose2.position.x = 0.0363116;
            pose2.position.y =  0.304015;
            pose2.position.z = 1;
            pose2.orientation.w = 1.0;

            geometry_msgs::msg::Pose pose3;
            pose3.position.x = -0.0861694;
            pose3.position.y = 0.23885499999999998;
            pose3.position.z = 2.0;
            pose3.orientation.w = 1.0;

            pose_array_msg.poses.push_back(pose1);
            pose_array_msg.poses.push_back(pose2);
            pose_array_msg.poses.push_back(pose3);

            // 发布 PoseArray
            publisher->publish(pose_array_msg);
            RCLCPP_INFO(node->get_logger(), "Published PoseArray with 3 poses.");
        }
    );

    // 让节点一直运行
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

