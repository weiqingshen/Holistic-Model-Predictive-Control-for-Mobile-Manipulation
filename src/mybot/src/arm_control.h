#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <string>
#include "std_msgs/msg/float32.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/pose_array.hpp"
#include <moveit_visual_tools/moveit_visual_tools.h>
class TargetPose {
public:
    // 通过提供目标姿态名称从 YAML 文件中加载姿态
    TargetPose(const rclcpp::Node::SharedPtr& node,
               moveit::planning_interface::MoveGroupInterface& move_group,
               const std::string& pose_name);

    // 通过直接提供位置和四元数来设置目标姿态
    TargetPose(const rclcpp::Node::SharedPtr& node,
               moveit::planning_interface::MoveGroupInterface& move_group,
               double x, double y, double z, double qx, double qy, double qz, double qw);

    geometry_msgs::msg::Pose getTargetPose() const;

    // 直接设定目标姿态
    void setTargetPose(double x, double y, double z, double qx, double qy, double qz, double qw);

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface& move_group_;
    geometry_msgs::msg::Pose target_pose_;
    std::string pose_name_;

    // 加载 YAML 文件中的参数
    bool loadPoseFromConfig(const YAML::Node& config);
    bool isloadTargetPose();
    void setTargetPose();

    // 默认 YAML 文件路径
    const std::string getDefaultYamlPath() const {
        // 动态获取路径
        try {
            return ament_index_cpp::get_package_share_directory("mybot") + "/config/param.yaml";
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("TargetPose"), "Failed to find package path: %s", e.what());
            throw;
        }
    }
};


class Object {
public:
    // 构造函数，读取 YAML 文件中的形状定义
    Object(const rclcpp::Node::SharedPtr& node, const std::string& object_name);

    // 设置物体的位置
    void setPosition(float x,float y,float z);

    void setPositionFromYaml(const std::string& pose_name);
    // 获取物体的碰撞对象
    moveit_msgs::msg::CollisionObject getCollisionObject() const;

    // 将物体添加到规划场景中
    void addToPlanningScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);

    //获取id
    std::string getName()const;

private:
    rclcpp::Node::SharedPtr node_;
    shape_msgs::msg::SolidPrimitive primitive_;
    geometry_msgs::msg::Pose pose_;
    std::string object_name_;
    std::string yaml_file_path_;

    // 从 YAML 配置中加载形状
    bool loadShapeFromConfig(const YAML::Node& config);
    std::string getDefaultYamlPath()const;
};

// GraspPose 类
class GraspPose {
public:
    // 构造函数
    GraspPose(const rclcpp::Node::SharedPtr& node, const std::string& grasp_name);

    // 获取抓取偏移姿态
    geometry_msgs::msg::Pose getGraspOffsetPose(const geometry_msgs::msg::Pose& object_pose) const;
    geometry_msgs::msg::Pose getObjectOffsetPose(const geometry_msgs::msg::Pose& object_pose) const;

private:
    rclcpp::Node::SharedPtr node_;
    geometry_msgs::msg::Pose offset_pose_;  // 偏移和方向
    std::string grasp_name_;
    std::string yaml_file_path_;

    // 从 YAML 配置中加载抓取偏移
    bool loadGraspOffsetFromConfig(const YAML::Node& config);
    std::string getDefaultYamlPath()const;
};

struct position_dynamic {
    float x1, y1,x2, y2, x3, y3;
    int order1,order2,order3;
};

// ManipulatorAction 类
class ManipulatorAction {
public:
    ManipulatorAction(const rclcpp::Node::SharedPtr& node,
                      moveit::planning_interface::MoveGroupInterface& move_group,
                      moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                      moveit::planning_interface::MoveGroupInterface& gripper_group);

    void execution_hmpc(Object& object, TargetPose& target_pose,GraspPose& grasp_pose);
    void executeAction_mpc_trajectory(Object& object, TargetPose& target_pose,GraspPose& grasp_pose) ;
    void execute_contrast(Object& object, TargetPose& target_pose,GraspPose& grasp_pose) ;
    void execute_dynamic(Object& object,GraspPose& grasp_pose) ;
    void dynamic_grasping(Object& object, TargetPose& target_pose,GraspPose& grasp_pose,bool isFixed=false);
    bool received_valid_pose;  // 标记是否接收到有效的位置信息
private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface& move_group_;
    moveit::planning_interface::PlanningSceneInterface& planning_scene_interface_;
    moveit::planning_interface::MoveGroupInterface& gripper_group_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_subscriber;  // 订阅者
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr mpc_pose_publisher_;  // 发布器
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_state_publisher_;  // 发布器
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;

    bool isupdate_visual = false;
    bool isupdate=false;

    void closeGripper();
    void openGripper();
    void openGripper_max();
    void attachObject(Object& object);
    void detachObject(Object& object);
    void moveToPose_compare(const geometry_msgs::msg::Pose& pose);
    bool mpc_planning(const geometry_msgs::msg::Pose& pose,bool cylinder);
    void mpc_planning_follow(Object& object,const geometry_msgs::msg::Pose& pose);
    void dynamic_mpc_planning(const geometry_msgs::msg::Pose& pose,bool isFixed=false);
    void execute_trajectory();
    void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void subscribeTopositionUpdate();
    void publish_to_python(const geometry_msgs::msg::Pose &pose);
    void publish_stop_to_python();
    bool collisionCheckAndExecute(moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string& label);
    position_dynamic target_position_;  // 存储接收到的目标位置


};
#endif // ARM_CONTROL_H
