#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <random>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pose_publisher");
    auto publisher = node->create_publisher<geometry_msgs::msg::PoseArray>("arm_pose", 10);

    // 程序启动时只生成一次随机数
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> x_dist(-0.13, 0.13);
    std::uniform_real_distribution<> y_dist(0.55, 0.8);

    double rand_x = x_dist(gen);
    double rand_y = y_dist(gen);

    RCLCPP_INFO(node->get_logger(), "Generated fixed random Pose1: x=%.3f, y=%.3f", rand_x, rand_y);

    auto timer = node->create_wall_timer(
        std::chrono::seconds(1),
        [=]() {
            geometry_msgs::msg::PoseArray pose_array_msg;
            pose_array_msg.header.stamp = node->now();
            pose_array_msg.header.frame_id = "world";

            geometry_msgs::msg::Pose pose1;
            pose1.position.x = rand_x;
            pose1.position.y = rand_y;
            pose1.position.z = 3.0;
            pose1.orientation.w = 1.0;

            geometry_msgs::msg::Pose pose2;
            pose2.position.x = 0.0363116;
            pose2.position.y = 0.304015;
            pose2.position.z = 1.0;
            pose2.orientation.w = 1.0;

            geometry_msgs::msg::Pose pose3;
            pose3.position.x = -0.0861694;
            pose3.position.y = 0.238855;
            pose3.position.z = 2.0;
            pose3.orientation.w = 1.0;

            pose_array_msg.poses.push_back(pose1);
            pose_array_msg.poses.push_back(pose2);
            pose_array_msg.poses.push_back(pose3);

            publisher->publish(pose_array_msg);
            RCLCPP_INFO(node->get_logger(), "Published fixed PoseArray with Pose1: x=%.3f, y=%.3f", rand_x, rand_y);
        }
    );

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

