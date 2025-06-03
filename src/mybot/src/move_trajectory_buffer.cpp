#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>  // 用于底盘状态
#include <queue>
#include <mutex>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <unordered_map>

bool is_gripper_operating = false;
using JointStateMsg = sensor_msgs::msg::JointState;
using JointTrajectoryPointMsg = trajectory_msgs::msg::JointTrajectoryPoint;
std::queue<JointStateMsg> joint_state_queue_;
std::mutex queue_mutex_;

// 记录状态
std::vector<double> recv_last_arm_positions(6, 0.0);
double recv_last_gripper_value = 0.0;
double pub_last_gripper_value = 0.0;
std::vector<double> recv_last_base_positions(3, 0.0);

// 记录状态
std::vector<double> pub_last_arm_positions(6, 0.0);

std::vector<double> pub_last_base_positions(3, 0.0);

bool is_first_recv_message = true;
bool is_first_pub_message = true;

bool arm_same_published = false;
bool gripper_same_published = false;
bool base_same_published = false;

// 工具函数
bool are_positions_equal(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::fabs(a[i] - b[i]) > 1e-6) {
            return false;
        }
    }
    return true;
}

void joint_state_callback(const JointStateMsg::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    std::unordered_map<std::string, double> joint_map;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        joint_map[msg->name[i]] = msg->position[i];
    }

    std::vector<double> current_arm_positions = {
        joint_map["joint1"],
        joint_map["joint2"],
        joint_map["joint3"],
        joint_map["joint4"],
        joint_map["joint5"],
        joint_map["joint6"]
    };
    double current_gripper_value = joint_map["joint_right"];
    std::vector<double> current_base_positions = {
        joint_map["position_base_x"],
        joint_map["position_base_y"],
        joint_map["position_base_theta"]
    };

    if (is_first_recv_message) {
        recv_last_arm_positions = current_arm_positions;
        recv_last_gripper_value = current_gripper_value;
        recv_last_base_positions = current_base_positions;
        is_first_recv_message = false;
        joint_state_queue_.push(*msg);  // 直接入队

        // 新增打印：
        std::ostringstream oss;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            oss << msg->name[i] << ":" << msg->position[i] << " ";
        }
        RCLCPP_INFO(rclcpp::get_logger("JointStateQueue"), "Enqueue (first): %s", oss.str().c_str());
        return;
    }
    // 提前判断
    bool arm_changed = !are_positions_equal(recv_last_arm_positions, current_arm_positions);
    bool gripper_changed = std::fabs(current_gripper_value - recv_last_gripper_value) > 1e-5;
    bool base_changed = !are_positions_equal(recv_last_base_positions, current_base_positions);

    if (arm_changed || gripper_changed || base_changed) {
        joint_state_queue_.push(*msg);

        std::ostringstream oss;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            oss << msg->name[i] << ":" << msg->position[i] << " ";
        }
        RCLCPP_INFO(rclcpp::get_logger("JointStateQueue"), "Enqueue: %s", oss.str().c_str());

        // ✨关键：只有真的入队之后，才更新 last_xxx
        recv_last_arm_positions = current_arm_positions;
        recv_last_gripper_value = current_gripper_value;
        recv_last_base_positions = current_base_positions;
    }
}



void publish_all(
    std::shared_ptr<rclcpp::Publisher<JointTrajectoryPointMsg>> arm_publisher,
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> gripper_publisher,
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Pose2D>> base_publisher) {

    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (is_gripper_operating) {
        return;
    }

    if (!joint_state_queue_.empty()) {
        auto msg = joint_state_queue_.front();
        joint_state_queue_.pop();

        std::unordered_map<std::string, double> joint_map;
        for (size_t i = 0; i < msg.name.size(); ++i) {
            joint_map[msg.name[i]] = msg.position[i];
        }

        std::vector<double> current_arm_positions = {
            joint_map["joint1"], joint_map["joint2"], joint_map["joint3"],
            joint_map["joint4"], joint_map["joint5"], joint_map["joint6"]};

        double current_gripper_value = joint_map["joint_right"];

        std::vector<double> current_base_positions = {
            joint_map["position_base_x"],
            joint_map["position_base_y"],
            joint_map["position_base_theta"]};

        if (is_first_pub_message) {
            JointTrajectoryPointMsg trajectory_msg;
            trajectory_msg.positions = current_arm_positions;
            arm_publisher->publish(trajectory_msg);

            pub_last_gripper_value = current_gripper_value;
            is_first_pub_message = false;

            arm_same_published = true;
            gripper_same_published = true;
            base_same_published = true;

            pub_last_arm_positions = current_arm_positions;
            pub_last_gripper_value = current_gripper_value;
            pub_last_base_positions = current_base_positions;

            RCLCPP_INFO(rclcpp::get_logger("Publisher"), "First publish: only arm, no gripper");
            return;
        }


        // ===== ✅ 发布机械臂 =====
        if (!are_positions_equal(pub_last_arm_positions, current_arm_positions) || !arm_same_published) {
            JointTrajectoryPointMsg trajectory_msg;
            trajectory_msg.positions = current_arm_positions;
            arm_publisher->publish(trajectory_msg);
            std::ostringstream oss;
            oss << "[";
            for (size_t i = 0; i < current_arm_positions.size(); ++i) {
                oss << current_arm_positions[i];
                if (i != current_arm_positions.size() - 1) oss << ", ";
            }
            oss << "]";
            RCLCPP_INFO(rclcpp::get_logger("Publisher"), "Published ARM: %s", oss.str().c_str());


            arm_same_published = are_positions_equal(pub_last_arm_positions, current_arm_positions);
        }

        // ===== ✅ 发布夹爪 =====
        if (std::fabs(current_gripper_value - pub_last_gripper_value) > 1e-8 || !gripper_same_published) {
            is_gripper_operating = true;

            JointTrajectoryPointMsg trajectory_msg;
            trajectory_msg.positions = current_arm_positions;
            arm_publisher->publish(trajectory_msg);

            std::this_thread::sleep_for(std::chrono::duration<double>(0.3));

            std_msgs::msg::Float32 gripper_msg;
            gripper_msg.data = (current_gripper_value < pub_last_gripper_value) ? 1.0 : 0.0;

            RCLCPP_INFO(rclcpp::get_logger("GripperState"), "Gripper is %s (Value: %.3f)",
                        (gripper_msg.data == 1.0 ? "opening" : "closing"), current_gripper_value);

            for (int i = 0; i < 5; ++i) {
                gripper_publisher->publish(gripper_msg);
            }

            std::this_thread::sleep_for(std::chrono::duration<double>(0.5));

            pub_last_gripper_value = current_gripper_value;
            is_gripper_operating = false;

            gripper_same_published = are_positions_equal(std::vector<double>{pub_last_gripper_value}, std::vector<double>{current_gripper_value});
        }


        // ===== ✅ 发布底盘 =====
        bool is_last_message = joint_state_queue_.empty(); // 只在这里增加一行

        geometry_msgs::msg::Pose2D base_msg;
        base_msg.x = current_base_positions[0];
        base_msg.y = current_base_positions[1];
        base_msg.theta = current_base_positions[2];

        std::ostringstream oss;
        oss << "x=" << base_msg.x << ", y=" << base_msg.y << ", theta=" << base_msg.theta;
            // ⚡️如果是最后一条消息，则底盘消息发布两次
        if (is_last_message) {
            base_publisher->publish(base_msg);
            RCLCPP_INFO(rclcpp::get_logger("Publisher"), "Published BASE: %s", oss.str().c_str());
            base_publisher->publish(base_msg);
            RCLCPP_INFO(rclcpp::get_logger("Publisher"), "Published BASE twice (last message)");
            RCLCPP_INFO(rclcpp::get_logger("Publisher"), "Published BASE: %s", oss.str().c_str());
        } else {
            base_publisher->publish(base_msg);
            RCLCPP_INFO(rclcpp::get_logger("Publisher"), "Published BASE: %s", oss.str().c_str());
        }

        // if (!are_positions_equal(pub_last_base_positions, current_base_positions) || !base_same_published) {
        //     geometry_msgs::msg::Pose2D base_msg;
        //     base_msg.x = current_base_positions[0];
        //     base_msg.y = current_base_positions[1];
        //     base_msg.theta = current_base_positions[2];
        //
        //     // ⚡️如果是最后一条消息，则底盘消息发布两次
        //     if (is_last_message) {
        //         base_publisher->publish(base_msg);
        //         base_publisher->publish(base_msg);
        //         RCLCPP_INFO(rclcpp::get_logger("Publisher"), "Published BASE twice (last message)");
        //     } else {
        //         base_publisher->publish(base_msg);
        //     }
        //
        //     base_same_published = are_positions_equal(pub_last_base_positions, current_base_positions);
        // }

        // ===== ✅ 更新记录 =====
        pub_last_arm_positions = current_arm_positions;
        pub_last_gripper_value = current_gripper_value;
        pub_last_base_positions = current_base_positions;
    }
}


// void publish_all(
//     std::shared_ptr<rclcpp::Publisher<JointTrajectoryPointMsg>> arm_publisher,
//     std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> gripper_publisher,
//     std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Pose2D>> base_publisher) {
//
//     std::lock_guard<std::mutex> lock(queue_mutex_);
//     if (is_gripper_operating) {
//         return;
//     }
//
//     if (!joint_state_queue_.empty()) {
//         auto msg = joint_state_queue_.front();
//         joint_state_queue_.pop();
//
//         std::unordered_map<std::string, double> joint_map;
//         for (size_t i = 0; i < msg.name.size(); ++i) {
//             joint_map[msg.name[i]] = msg.position[i];
//         }
//
//         std::vector<double> current_arm_positions = {
//             joint_map["joint1"],
//             joint_map["joint2"],
//             joint_map["joint3"],
//             joint_map["joint4"],
//             joint_map["joint5"],
//             joint_map["joint6"]
//         };
//         double current_gripper_value = joint_map["joint_right"];
//         std::vector<double> current_base_positions = {
//             joint_map["position_base_x"],
//             joint_map["position_base_y"],
//             joint_map["position_base_theta"]
//         };
//         // ===== ✅ 首次发布逻辑 =====
//         if (is_first_pub_message) {
//             JointTrajectoryPointMsg trajectory_msg;
//             trajectory_msg.positions = current_arm_positions;
//
//             arm_publisher->publish(trajectory_msg);
//
//             pub_last_gripper_value = current_gripper_value;
//             is_first_pub_message = false;
//
//             RCLCPP_INFO(rclcpp::get_logger("Publisher"), "First publish: only arm, no gripper");
//             return;
//         }
//
//         if (!are_positions_equal(recv_last_arm_positions, current_arm_positions)) {
//             JointTrajectoryPointMsg trajectory_msg;
//             trajectory_msg.positions = current_arm_positions;
//             arm_publisher->publish(trajectory_msg);
//         }
//
//
//         // 发布夹爪状态
//         if (std::fabs(current_gripper_value - pub_last_gripper_value) > 1e-5) {
//             // 在发指令前标记夹爪操作进行中
//             is_gripper_operating = true;
//
//             // 先发布机械臂状态，确保同步
//             JointTrajectoryPointMsg trajectory_msg;
//             trajectory_msg.positions = current_arm_positions;
//             arm_publisher->publish(trajectory_msg);
//
//             // 模拟夹爪动作（0.3 秒延迟）
//             std::this_thread::sleep_for(std::chrono::duration<double>(0.3));
//
//             // 创建夹爪开关消息
//             std_msgs::msg::Float32 gripper_msg;
//             gripper_msg.data = (current_gripper_value < pub_last_gripper_value) ? 1.0 : 0.0;
//
//             RCLCPP_INFO(rclcpp::get_logger("GripperState"), "Gripper is %s (Value: %.3f)",
//                         (gripper_msg.data == 1.0 ? "opening" : "closing"), current_gripper_value);
//
//             // 连发5次
//             for (int i = 0; i < 5; ++i) {
//                 gripper_publisher->publish(gripper_msg);
//             }
//
//             // 可选再 sleep 0.5 秒
//             std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
//
//             // 更新夹爪状态记录
//             pub_last_gripper_value = current_gripper_value;
//
//             // 恢复状态
//             is_gripper_operating = false;
//         }
//
//
//         // 发布底盘
//         if (!are_positions_equal(recv_last_base_positions, current_base_positions)) {
//             geometry_msgs::msg::Pose2D base_msg;
//             base_msg.x = current_base_positions[0];
//             base_msg.y = current_base_positions[1];
//             base_msg.theta = current_base_positions[2];
//             base_publisher->publish(base_msg);
//         }
//
//         // 更新记录
//         recv_last_arm_positions = current_arm_positions;
//         recv_last_gripper_value = current_gripper_value;
//         recv_last_base_positions = current_base_positions;
//     }
// }

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("trajectory_buffer_node");

    auto joint_state_subscription = node->create_subscription<JointStateMsg>(
        "/joint_states", 500, joint_state_callback);

    auto joint_publisher = node->create_publisher<JointTrajectoryPointMsg>("/buffered_display_trajectory_point", 500);
    auto gripper_publisher = node->create_publisher<std_msgs::msg::Float32>("/buffered_gripper_state", 500);
    auto base_publisher = node->create_publisher<geometry_msgs::msg::Pose2D>("/buffered_base_state", 500);

    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(15),
        [joint_publisher, gripper_publisher, base_publisher]() {
            publish_all(joint_publisher, gripper_publisher, base_publisher);
        });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
