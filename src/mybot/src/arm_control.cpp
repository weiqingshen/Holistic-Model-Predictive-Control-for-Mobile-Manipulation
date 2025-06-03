#include "arm_control.h"
#include <filesystem>
#include <fstream>  // 用于文件操作
#include <sys/stat.h>  // 用于检查文件是否存在

// 构造函数，只需提供目标姿态的名字，从 YAML 文件加载
TargetPose::TargetPose(const rclcpp::Node::SharedPtr& node,
                       moveit::planning_interface::MoveGroupInterface& move_group,
                       const std::string& pose_name)
        : node_(node), move_group_(move_group), pose_name_(pose_name) {
    if (!isloadTargetPose())
        setTargetPose();
}

// 构造函数，直接提供位置信息和四元数来设置目标姿态
TargetPose::TargetPose(const rclcpp::Node::SharedPtr& node,
                       moveit::planning_interface::MoveGroupInterface& move_group,
                       double x, double y, double z, double qx, double qy, double qz, double qw)
        : node_(node), move_group_(move_group) {
    // 使用传入的参数设置目标姿态
    setTargetPose(x, y, z, qx, qy, qz, qw);
}

// 从 YAML 文件加载指定名字的目标姿态
bool TargetPose::isloadTargetPose() {
    try {
        std::string yaml_file_path=getDefaultYamlPath();
        YAML::Node config = YAML::LoadFile(yaml_file_path);
        if (config["poses"] && config["poses"][pose_name_]) {
            return loadPoseFromConfig(config["poses"][pose_name_]);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Pose '%s' not found in YAML file.", pose_name_.c_str());
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load target pose by name: %s", e.what());
        return false;
    }
}

// 从配置中加载目标姿态
bool TargetPose::loadPoseFromConfig(const YAML::Node& config) {
    try {
        target_pose_.position.x = config["position"]["x"].as<double>();
        target_pose_.position.y = config["position"]["y"].as<double>();
        target_pose_.position.z = config["position"]["z"].as<double>();
        target_pose_.orientation.x = config["orientation"]["qx"].as<double>();
        target_pose_.orientation.y = config["orientation"]["qy"].as<double>();
        target_pose_.orientation.z = config["orientation"]["qz"].as<double>();
        target_pose_.orientation.w = config["orientation"]["qw"].as<double>();
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load pose from config: %s", e.what());
        return false;
    }
}

// 通过直接传入位置和四元数设置目标姿态
void TargetPose::setTargetPose(double x, double y, double z, double qx, double qy, double qz, double qw) {
    target_pose_.position.x = x;
    target_pose_.position.y = y;
    target_pose_.position.z = z;
    target_pose_.orientation.x = qx;
    target_pose_.orientation.y = qy;
    target_pose_.orientation.z = qz;
    target_pose_.orientation.w = qw;

    // 设置 MoveGroup 的目标位姿
    move_group_.setPoseTarget(target_pose_);
}

// 设置目标姿态（从加载或直接设定）
void TargetPose::setTargetPose() {
    move_group_.setPoseTarget(target_pose_);
}

// 获取当前目标姿态
geometry_msgs::msg::Pose TargetPose::getTargetPose() const {
    return target_pose_;
}
std::string Object::getDefaultYamlPath()const {
    try {
        // 动态获取路径并拼接
        return ament_index_cpp::get_package_share_directory("mybot") + "/config/param.yaml";
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("Object"), "Failed to find package path: %s", e.what());
        throw;  // 抛出异常，保证调用者可以处理
    }
}
Object::Object(const rclcpp::Node::SharedPtr& node, const std::string& object_name)
    : node_(node), object_name_(object_name) {

    // 直接使用动态路径获取 YAML 文件路径
    yaml_file_path_ = getDefaultYamlPath();  // 使用动态路径获取 YAML 文件

    try {
        // 尝试加载 YAML 文件
        YAML::Node config = YAML::LoadFile(yaml_file_path_);

        // 检查 YAML 文件是否成功加载
        if (!config) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load YAML file: %s", yaml_file_path_.c_str());
            return;
        }

        // 检查 "obstacles" 部分是否存在
        if (!config["obstacles"]) {
            RCLCPP_ERROR(node_->get_logger(), "'obstacles' section not found in YAML file.");
            return;
        }

        // 检查是否存在指定的 object_name
        if (config["obstacles"][object_name_]) {
            RCLCPP_INFO(node_->get_logger(), "Object '%s' found in YAML file.", object_name_.c_str());

            // 调用 loadShapeFromConfig 来加载形状
            if (!loadShapeFromConfig(config["obstacles"][object_name_])) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to load shape configuration for object '%s'.", object_name_.c_str());
            } else {
                RCLCPP_INFO(node_->get_logger(), "Shape configuration for object '%s' loaded successfully.", object_name_.c_str());
            }
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Object '%s' not found in 'obstacles' section of YAML file.", object_name_.c_str());
        }
    } catch (const std::exception& e) {
        // 捕捉并记录异常
        RCLCPP_ERROR(node_->get_logger(), "Exception occurred while loading YAML file: %s", e.what());
    }

    // 初始化姿态（如果加载形状成功，姿态可能已由 config 设置）
    pose_.orientation.w = 1.0; // 默认姿态
}

// 从 YAML 配置中加载形状
bool Object::loadShapeFromConfig(const YAML::Node& config) {
    try {
        std::string type = config["type"].as<std::string>();

        if (type == "box") {
            primitive_.type = primitive_.BOX;
            primitive_.dimensions = {
                    config["dimensions"][0].as<double>(),
                    config["dimensions"][1].as<double>(),
                    config["dimensions"][2].as<double>()
            };
        } else if (type == "cylinder") {
            primitive_.type = primitive_.CYLINDER;
            primitive_.dimensions = {
                    config["dimensions"]["height"].as<double>(), // 高度
                    config["dimensions"]["radius"].as<double>()  // 半径
            };
        } else {
            RCLCPP_WARN(node_->get_logger(), "Unknown shape type '%s' for object '%s'.", type.c_str(), object_name_.c_str());
            return false;
        }
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load shape from config: %s", e.what());
        return false;
    }
}

// 设置物体的位置
void Object::setPosition(float x,float y,float z) {

    pose_.position.x = x;
    pose_.position.y = y;
    pose_.position.z = z;
}

void Object::setPositionFromYaml(const std::string& pose_name) {
    try {
        // 加载 YAML 文件
        YAML::Node config = YAML::LoadFile(yaml_file_path_);
        if (!config) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load YAML file: %s", yaml_file_path_.c_str());
            return;
        }

        // 检查 "poses" 部分是否存在
        if (!config["poses"]) {
            RCLCPP_ERROR(node_->get_logger(), "'poses' section not found in YAML file.");
            return;
        }

        // 检查是否存在指定的 pose_name
        if (!config["poses"][pose_name]) {
            RCLCPP_ERROR(node_->get_logger(), "Pose '%s' not found in 'poses' section of YAML file.", pose_name.c_str());
            return;
        }

        // 提取位置信息
        YAML::Node pose_config = config["poses"][pose_name]["position"];
        if (pose_config) {
            pose_.position.x = pose_config["x"].as<float>();
            pose_.position.y = pose_config["y"].as<float>();
            pose_.position.z = pose_config["z"].as<float>();
            RCLCPP_INFO(node_->get_logger(), "Position for pose '%s' set to [x: %f, y: %f, z: %f].",
                        pose_name.c_str(), pose_.position.x, pose_.position.y, pose_.position.z);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "'position' section not found for pose '%s' in YAML file.", pose_name.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Exception occurred while setting position from YAML file: %s", e.what());
    }
}

std::string GraspPose::getDefaultYamlPath() const{
    try {
        // 动态获取路径并拼接
        return ament_index_cpp::get_package_share_directory("mybot") + "/config/param.yaml";
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("Object"), "Failed to find package path: %s", e.what());
        throw;  // 抛出异常，保证调用者可以处理
    }
}

GraspPose::GraspPose(const rclcpp::Node::SharedPtr& node, const std::string& grasp_name)
        : node_(node), grasp_name_(grasp_name) {
    try {
        // 使用 getDefaultYamlPath() 获取默认的 YAML 文件路径
        std::string yaml_file_path = getDefaultYamlPath();
        YAML::Node config = YAML::LoadFile(yaml_file_path);

        if (config["grasp_poses"] && config["grasp_poses"][grasp_name]) {
            loadGraspOffsetFromConfig(config["grasp_poses"][grasp_name]);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Grasp pose '%s' not found in YAML file.", grasp_name_.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load grasp pose for '%s': %s", grasp_name_.c_str(), e.what());
    }
}

// 从 YAML 配置中加载抓取偏移
bool GraspPose::loadGraspOffsetFromConfig(const YAML::Node& config) {
    try {
        offset_pose_.position.x = config["offset"]["x"].as<double>();
        offset_pose_.position.y = config["offset"]["y"].as<double>();
        offset_pose_.position.z = config["offset"]["z"].as<double>();
        offset_pose_.orientation.x = config["orientation"]["qx"].as<double>();
        offset_pose_.orientation.y = config["orientation"]["qy"].as<double>();
        offset_pose_.orientation.z = config["orientation"]["qz"].as<double>();
        offset_pose_.orientation.w = config["orientation"]["qw"].as<double>();
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load grasp offset from config: %s", e.what());
        return false;
    }
}


// 获取抓取偏移姿态
geometry_msgs::msg::Pose GraspPose::getGraspOffsetPose(const geometry_msgs::msg::Pose& object_pose) const {
    geometry_msgs::msg::Pose grasp_pose = object_pose;

    // 应用偏移
    grasp_pose.position.x += offset_pose_.position.x;
    grasp_pose.position.y += offset_pose_.position.y;
    grasp_pose.position.z += offset_pose_.position.z;

    // 应用抓取方向（忽略实际合并方向）
    grasp_pose.orientation = offset_pose_.orientation;

    return grasp_pose;
}

geometry_msgs::msg::Pose GraspPose::getObjectOffsetPose(const geometry_msgs::msg::Pose &target_pose) const {
    geometry_msgs::msg::Pose adjusted_pose = target_pose;

    // 将偏移添加到目标位置
    adjusted_pose.position.x -= offset_pose_.position.x;
    adjusted_pose.position.y -= offset_pose_.position.y;
    adjusted_pose.position.z -= offset_pose_.position.z;

    // 如果需要，你可以根据应用的逻辑调整方向（四元数）。
    // 这里暂时假设目标姿态的方向完全由偏移姿态来决定
    adjusted_pose.orientation = offset_pose_.orientation;

    return adjusted_pose;
}


// 获取物体的碰撞对象
moveit_msgs::msg::CollisionObject Object::getCollisionObject() const {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = object_name_;
    collision_object.header.frame_id = "world"; // 根据实际需要修改
    collision_object.primitives.push_back(primitive_);
    collision_object.primitive_poses.push_back(pose_);
    collision_object.operation = collision_object.ADD;
    return collision_object;
}

// 将物体添加到规划场景中
void Object::addToPlanningScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
    planning_scene_interface.applyCollisionObject(getCollisionObject());
}
std::string Object::getName() const {
    return object_name_; // 返回物体名称
}

// ManipulatorAction 构造函数
ManipulatorAction::ManipulatorAction(const rclcpp::Node::SharedPtr& node,
                                     moveit::planning_interface::MoveGroupInterface& move_group,
                                     moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                                     moveit::planning_interface::MoveGroupInterface& gripper_group)
        : node_(node), move_group_(move_group), planning_scene_interface_(planning_scene_interface), gripper_group_(gripper_group){  // 只传递 YAML 文件
    // 其他初始化代码...
    gripper_state_publisher_ = node_->create_publisher<std_msgs::msg::Float32>("gripper_state", 10);
    mpc_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::Pose>("/mpc_pose", 10);
    pose_subscriber = node_->create_subscription<geometry_msgs::msg::PoseArray>(
        "arm_pose", 10,
        std::bind(&ManipulatorAction::poseArrayCallback, this, std::placeholders::_1)
    );
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(node_, "world","visualization_marker_array",move_group.getRobotModel());
    // visual_tools_->loadMarkerPub();  // <<< 必须有！！
    visual_tools_->loadMarkerPub();  // 发布Marker
    visual_tools_->deleteAllMarkers();
    visual_tools_->trigger();

    // 初始化 PlanningSceneMonitor
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description");

    planning_scene_monitor_->requestPlanningSceneState();  // <<<<<< 这句非常关键！

    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();
    planning_scene_monitor_->startStateMonitor();



}

void ManipulatorAction::execute_dynamic(Object &object, GraspPose &grasp_pose) {
    geometry_msgs::msg::Pose grasp_offset_pose = grasp_pose.getGraspOffsetPose(object.getCollisionObject().primitive_poses[0]);
    grasp_offset_pose.position.z += 0.05;
    moveToPose_compare(grasp_offset_pose);
}

void ManipulatorAction::execute_contrast(Object &object, TargetPose &target_pose, GraspPose &grasp_pose) {
    geometry_msgs::msg::Pose placepoce=target_pose.getTargetPose();

    geometry_msgs::msg::Pose grasp_offset_pose = grasp_pose.getGraspOffsetPose(object.getCollisionObject().primitive_poses[0]);
    moveToPose_compare(grasp_offset_pose);
    //moveToPose(grasp_offset_pose);
    // 2. 闭合夹爪
    closeGripper();

    // 3. Attach 物体
    attachObject(object);

    moveToPose_compare(placepoce);
    //moveToPose(placepoce);
    // 5. 打开夹爪
    openGripper();

    // 6. Detach 物体
    detachObject(object);

    geometry_msgs::msg::Pose final_pose = grasp_pose.getObjectOffsetPose(placepoce);

    // 更新物体的坐标
    object.setPosition(final_pose.position.x, final_pose.position.y, final_pose.position.z);
    RCLCPP_INFO(node_->get_logger(), "物体坐标已更新到 (%f, %f, %f)", final_pose.position.x, final_pose.position.y, final_pose.position.z);
}
void ManipulatorAction::execution_hmpc(Object &object, TargetPose &target_pose, GraspPose &grasp_pose) {
    geometry_msgs::msg::Pose placepoce=target_pose.getTargetPose();

    geometry_msgs::msg::Pose grasp_offset_pose = grasp_pose.getGraspOffsetPose(object.getCollisionObject().primitive_poses[0]);

    if(!mpc_planning(grasp_offset_pose,true)) {
        return;
    }
    // 2. 闭合夹爪
    closeGripper();

    // 3. Attach 物体
    attachObject(object);

    if(!mpc_planning(placepoce,false)) {
        return;
    }

    // 5. 打开夹爪
    openGripper();

    // 6. Detach 物体
    detachObject(object);

    geometry_msgs::msg::Pose final_pose = grasp_pose.getObjectOffsetPose(placepoce);

    // 更新物体的坐标
    object.setPosition(final_pose.position.x, final_pose.position.y, final_pose.position.z);
    RCLCPP_INFO(node_->get_logger(), "物体坐标已更新到 (%f, %f, %f)", final_pose.position.x, final_pose.position.y, final_pose.position.z);
}

void ManipulatorAction::executeAction_mpc_trajectory(Object &object, TargetPose &target_pose, GraspPose &grasp_pose) {
    geometry_msgs::msg::Pose placepoce=target_pose.getTargetPose();

    geometry_msgs::msg::Pose grasp_offset_pose = grasp_pose.getGraspOffsetPose(object.getCollisionObject().primitive_poses[0]);

    mpc_planning_follow(object,grasp_offset_pose);
    // 5. 打开夹爪
    openGripper();

    // 6. Detach 物体
    detachObject(object);
    geometry_msgs::msg::Pose final_pose = grasp_pose.getObjectOffsetPose(placepoce);

    // 更新物体的坐标
    object.setPosition(final_pose.position.x, final_pose.position.y, final_pose.position.z);
    RCLCPP_INFO(node_->get_logger(), "物体坐标已更新到 (%f, %f, %f)", final_pose.position.x, final_pose.position.y, final_pose.position.z);
}


// 检查文件是否存在的函数
bool fileExists(const std::string& filename) {
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}
void ManipulatorAction::moveToPose_compare(const geometry_msgs::msg::Pose &pose) {
    bool success = false;
    int attempt_count = 0;
    const int max_attempts = 100;
    std::string planner_id = move_group_.getPlannerId();
    RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "====== 当前使用的规划器是: [%s] ======", planner_id.c_str());

    RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "当前使用的规划器是: %s", planner_id.c_str());

    // 设置目标位姿
    move_group_.setPoseTarget(pose);

    while (attempt_count < max_attempts && !success) {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        // 尝试规划
        if (move_group_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "规划成功，尝试执行动作");

            // ----------------- 加上轨迹碰撞检测 -----------------
            // 构造 RobotTrajectory 对象
            robot_trajectory::RobotTrajectory robot_traj(move_group_.getRobotModel(), move_group_.getName());
            robot_traj.setRobotTrajectoryMsg(*move_group_.getCurrentState(), my_plan.trajectory_.joint_trajectory);

            auto planning_scene = planning_scene_monitor_->getPlanningScene();
            if (!planning_scene) {
                RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "PlanningScene is NULL! 取消执行");
                return;
            }

            bool trajectory_valid = true;
            moveit::core::RobotStatePtr temp_state(new moveit::core::RobotState(*move_group_.getCurrentState()));

            for (std::size_t i = 0; i < robot_traj.getWayPointCount(); ++i) {
                *temp_state = robot_traj.getWayPoint(i);

                if (planning_scene->isStateColliding(*temp_state, move_group_.getName())) {
                    RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "碰撞检测失败，轨迹第%zu个点发生碰撞！", i);
                    trajectory_valid = false;
                    break;
                }
            }
            // ---------------------------------------------------

            if (!trajectory_valid) {
                RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "轨迹无效，重新尝试规划");
                attempt_count++;
                continue; // 不执行，直接重新规划
            }

            // 如果轨迹有效，才执行动作
            RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "轨迹无碰撞，尝试执行动作");

            if (move_group_.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "执行成功");
                success = true;

                // 可视化轨迹
                visual_tools_->publishTrajectoryLine(my_plan.trajectory_, move_group_.getCurrentState()->getJointModelGroup(move_group_.getName()));
                visual_tools_->trigger();
            } else {
                RCLCPP_WARN(rclcpp::get_logger("move_group_interface"), "执行失败");
            }
        } else {
            RCLCPP_WARN(rclcpp::get_logger("move_group_interface"), "规划失败，第%d次尝试", attempt_count+1);
        }

        attempt_count++;
    }

    if (!success) {
        RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "规划和执行失败，尝试了%d次", max_attempts);

        // 失败后回零位
        std::vector<double> joint_values(6, 0.0); // 假设6个关节
        move_group_.setJointValueTarget(joint_values);

        moveit::planning_interface::MoveGroupInterface::Plan reset_plan;
        if (move_group_.plan(reset_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            move_group_.execute(reset_plan);
            RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "返回初始位置成功");
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "返回初始位置失败");
        }
    }
}


// 闭合夹爪
void ManipulatorAction::closeGripper() {
    RCLCPP_INFO(node_->get_logger(), "Closing gripper.");
    gripper_group_.setNamedTarget("close");
    gripper_group_.move();
    // 发布夹爪闭合的状态
    // std_msgs::msg::Float32 msg;
    // msg.data = 0.0;  // 夹爪闭合状态，发布 0
    // gripper_state_publisher_->publish(msg);
}

// 打开夹爪
void ManipulatorAction::openGripper() {
    RCLCPP_INFO(node_->get_logger(), "Opening gripper.");
    gripper_group_.setNamedTarget("open");
    gripper_group_.move();
    // 发布夹爪打开的状态
    // std_msgs::msg::Float32 msg;
    // msg.data = 1.0;  // 夹爪打开状态，发布 1
    // gripper_state_publisher_->publish(msg);
}
void ManipulatorAction::openGripper_max() {
    RCLCPP_INFO(node_->get_logger(), "Opening gripper.");
    gripper_group_.setNamedTarget("open_max");
    gripper_group_.move();
    // 发布夹爪打开的状态
    // std_msgs::msg::Float32 msg;
    // msg.data = 1.0;  // 夹爪打开状态，发布 1
    // gripper_state_publisher_->publish(msg);
}

// Attach 物体
void ManipulatorAction::attachObject(Object& object) {
    std::vector<std::string> touch_links;
    touch_links.push_back("link_right");
    touch_links.push_back("link_left");

    gripper_group_.attachObject(object.getCollisionObject().id, "link_hand", touch_links);
    RCLCPP_INFO(node_->get_logger(), "Object attached.");
}



// Detach 物体
    void ManipulatorAction::detachObject(Object& object) {
        gripper_group_.detachObject(object.getCollisionObject().id);
        RCLCPP_INFO(node_->get_logger(), "Object detached.");
    }


bool ManipulatorAction::mpc_planning(const geometry_msgs::msg::Pose &pose,bool cylinder) {
    RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Starting MPC planning...");

    // 构造命令行参数
    if(cylinder) {
        std::ostringstream cmd_stream;
        cmd_stream << "python3 /home/fins/github/hmpc/src/hmpc_python/moveit_mpc_planner_task1_v1.py"
                   << " "
                   << pose.position.x << " " << pose.position.y << " " << pose.position.z
                   << " "
                   << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z << " " << pose.orientation.w
                   << " &";
        std::string command = cmd_stream.str();
        RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Running command: %s", command.c_str());
        int ret = std::system(command.c_str());

        if (ret != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("mpc_planning"), "Failed to start Python MPC planner.");
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Python MPC planner started.");
    }else {
        std::ostringstream cmd_stream;
        cmd_stream << "python3 /home/fins/github/hmpc/src/hmpc_python/moveit_mpc_planner_task1_v2.py"
                   << " "
                   << pose.position.x << " " << pose.position.y << " " << pose.position.z
                   << " "
                   << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z << " " << pose.orientation.w
                   << " &";
        std::string command = cmd_stream.str();
        RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Running command: %s", command.c_str());
        int ret = std::system(command.c_str());

        if (ret != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("mpc_planning"), "Failed to start Python MPC planner.");
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Python MPC planner started.");
    }





    // 创建 ROS 2 节点
    auto node = rclcpp::Node::make_shared("mpc_joint_trajectory_subscriber");

    // 存储接收到的轨迹点
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> received_trajectory_points;
    std::vector<std::string> joint_names;

    // 记录最后接收到消息的时间
    auto last_received_time = std::chrono::steady_clock::now();

    auto callback = [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
        RCLCPP_INFO(rclcpp::get_logger("mpc_joint_trajectory_subscriber"), "Received JointState message.");

        if (msg->name.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("mpc_joint_trajectory_subscriber"), "No joint data received!");
            return;
        }

        // 获取关节名称（仅在第一次接收时存储）
        if (joint_names.empty()) {
            joint_names = msg->name;
        }

        // 解析关节数据
        trajectory_msgs::msg::JointTrajectoryPoint joint_point;
        joint_point.positions = msg->position;
        joint_point.velocities = msg->velocity;
        joint_point.effort = msg->effort;

        // 设置时间间隔
        if (received_trajectory_points.empty()) {
            joint_point.time_from_start = rclcpp::Duration(0, 0);
        } else {
            joint_point.time_from_start = rclcpp::Duration::from_nanoseconds(received_trajectory_points.size() * 100'000'000); // 每 0.1s 一个点
        }

        received_trajectory_points.push_back(joint_point);

        // 更新最后接收到消息的时间
        last_received_time = std::chrono::steady_clock::now();
    };

    // 订阅 MPC 轨迹
    auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
        "/mpc_joint_trajectory",
        100,
        callback
    );

    // 运行订阅器
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto start = std::chrono::steady_clock::now();

    // 设置超时阈值为0.01秒
    std::chrono::milliseconds timeout(15000);

    while (rclcpp::ok()) {
        executor.spin_some();

        // 如果队列中没有消息且没有收到新的消息超过0.01秒，停止订阅
        if (!received_trajectory_points.empty() &&
            std::chrono::steady_clock::now() - last_received_time > timeout) {
            RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "No new messages for 0.01 seconds and queue is empty, stopping subscription.");
            break;
            }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }


    RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Finished receiving JointStates.");

    // **执行接收到的轨迹**
    if (!received_trajectory_points.empty()) {
        RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Executing MPC trajectory...");

        // 构造 MoveIt 轨迹消息
        moveit_msgs::msg::RobotTrajectory msg_trajectory;
        trajectory_msgs::msg::JointTrajectory& joint_trajectory = msg_trajectory.joint_trajectory;

        // 设置关节名称
        joint_trajectory.joint_names = joint_names;

        // 添加轨迹点
        joint_trajectory.points = received_trajectory_points;

        // 构造规划 plan
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        move_group_.plan(plan);  // dummy plan
        plan.trajectory_.joint_trajectory = joint_trajectory;

        // 使用封装好的碰撞检测和执行函数
        if (!collisionCheckAndExecute(plan, "MPC")) {
            RCLCPP_ERROR(rclcpp::get_logger("mpc_planning"), "Trajectory execution aborted due to collision.");
            return false;
        }

        // **计算最终的末端执行器位置**
        RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Calculating final end-effector position...");

        moveit::core::RobotStatePtr current_state = move_group_.getCurrentState(10.0);
        if (!current_state) {
            RCLCPP_ERROR(rclcpp::get_logger("mpc_planning"), "Failed to get current robot state.");
            return false;
        }
        return true;
        // const std::string end_effector_link = move_group_.getEndEffectorLink();
        // geometry_msgs::msg::Pose final_pose;
        //
        // if (!current_state->satisfiesBounds()) {
        //     RCLCPP_ERROR(rclcpp::get_logger("mpc_planning"), "Robot state is out of bounds.");
        // }
        //
        // if (!current_state->hasAttachedBody(end_effector_link)) {
        //     RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Fetching transform for end-effector.");
        //     final_pose = move_group_.getCurrentPose().pose;
        // } else {
        //     RCLCPP_ERROR(rclcpp::get_logger("mpc_planning"), "End-effector link not found.");
        //     return false;
        // }

    }else {
        RCLCPP_WARN(rclcpp::get_logger("mpc_planning"), "No trajectory received. Execution skipped.");
    }
}

void ManipulatorAction::mpc_planning_follow(Object& object,const geometry_msgs::msg::Pose &pose) {
    RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Starting MPC planning...");

    // 构造命令行参数
    std::ostringstream cmd_stream;
    cmd_stream << "python3 /home/fins/myrobot_move/src/mpc_python/moveit_mpc_planner_task2.py"
               << " "
               << pose.position.x << " " << pose.position.y << " " << pose.position.z
               << " "
               << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z << " " << pose.orientation.w
               << " &";

    std::string command = cmd_stream.str();
    RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Running command: %s", command.c_str());

    int ret = std::system(command.c_str());

    if (ret != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("mpc_planning"), "Failed to start Python MPC planner.");
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Python MPC planner started.");

    // 创建 ROS 2 节点
    auto node = rclcpp::Node::make_shared("mpc_joint_trajectory_subscriber");

    // 存储接收到的轨迹点
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> received_trajectory_points;
    std::vector<std::string> joint_names;

    // 记录最后接收到消息的时间
    auto last_received_time = std::chrono::steady_clock::now();

    auto callback = [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
        RCLCPP_INFO(rclcpp::get_logger("mpc_joint_trajectory_subscriber"), "Received JointState message.");

        if (msg->name.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("mpc_joint_trajectory_subscriber"), "No joint data received!");
            return;
        }

        // 获取关节名称（仅在第一次接收时存储）
        if (joint_names.empty()) {
            joint_names = msg->name;
        }

        // 解析关节数据
        trajectory_msgs::msg::JointTrajectoryPoint joint_point;
        joint_point.positions = msg->position;
        joint_point.velocities = msg->velocity;
        joint_point.effort = msg->effort;

        // 设置时间间隔
        if (received_trajectory_points.empty()) {
            joint_point.time_from_start = rclcpp::Duration(0, 0);
        } else {
            joint_point.time_from_start = rclcpp::Duration::from_nanoseconds(received_trajectory_points.size() * 100'000'000); // 每 0.1s 一个点
        }

        received_trajectory_points.push_back(joint_point);

        // 更新最后接收到消息的时间
        last_received_time = std::chrono::steady_clock::now();
    };

    // 订阅 MPC 轨迹
    auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
        "/mpc_joint_trajectory",
        100,
        callback
    );

    // 运行订阅器
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto start = std::chrono::steady_clock::now();

    // 设置超时阈值为0.01秒
    std::chrono::milliseconds timeout(1000);

    while (rclcpp::ok()) {
        executor.spin_some();

        // 如果队列中没有消息且没有收到新的消息超过0.01秒，停止订阅
        if (!received_trajectory_points.empty() &&
            std::chrono::steady_clock::now() - last_received_time > timeout) {
            RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "No new messages for 0.01 seconds and queue is empty, stopping subscription.");
            break;
            }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }


    RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Finished receiving JointStates.");
    if (!received_trajectory_points.empty()) {
        // 分割轨迹为第一段和第二段
        size_t second_trajectory_size = 30;
        std::vector<trajectory_msgs::msg::JointTrajectoryPoint> second_trajectory_points(
            received_trajectory_points.end() - second_trajectory_size - 1,
            received_trajectory_points.end());
        std::vector<trajectory_msgs::msg::JointTrajectoryPoint> first_trajectory_points(
            received_trajectory_points.begin(),
            received_trajectory_points.end() - second_trajectory_size);

        RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "First trajectory size: %zu", first_trajectory_points.size());
        RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Second trajectory size: %zu", second_trajectory_points.size());

        // 重设第二段轨迹的时间
        rclcpp::Duration time_offset(0, 0);
        for (auto& point : second_trajectory_points) {
            point.time_from_start = time_offset;
            time_offset = rclcpp::Duration(0, time_offset.nanoseconds() + 100'000'000);
        }

        // 第一段规划
        moveit_msgs::msg::RobotTrajectory msg_trajectory;
        trajectory_msgs::msg::JointTrajectory& joint_trajectory = msg_trajectory.joint_trajectory;
        joint_trajectory.joint_names = joint_names;
        joint_trajectory.points = first_trajectory_points;

        moveit::planning_interface::MoveGroupInterface::Plan plan1;
        move_group_.plan(plan1);
        plan1.trajectory_.joint_trajectory = joint_trajectory;

        // 第二段规划
        joint_trajectory.points = second_trajectory_points;
        moveit::planning_interface::MoveGroupInterface::Plan plan2;
        move_group_.plan(plan2);
        plan2.trajectory_.joint_trajectory = joint_trajectory;

        // 第一段规划 & 执行
        RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Executing pre-grasp trajectory...");
        if (!collisionCheckAndExecute(plan1, "pre_grasp")) {
            RCLCPP_ERROR(rclcpp::get_logger("mpc_planning"), "Pre-grasp execution failed. Aborting.");
            return;
        }

        // 夹爪操作
        RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Closing gripper and attaching object...");
        closeGripper();
        attachObject(object);

        // 第二段规划 & 执行
        RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Executing post-grasp trajectory...");
        if (!collisionCheckAndExecute(plan2, "post_grasp")) {
            RCLCPP_ERROR(rclcpp::get_logger("mpc_planning"), "Post-grasp execution failed.");
        }
    }

    // // **执行接收到的轨迹**
    // if (!received_trajectory_points.empty()) {
    //     // 分割轨迹为第一段和第二段
    //     size_t second_trajectory_size = 30;  // 假设后30个是第二段轨迹
    //     std::vector<trajectory_msgs::msg::JointTrajectoryPoint> second_trajectory_points(received_trajectory_points.end() - second_trajectory_size-1, received_trajectory_points.end());
    //     std::vector<trajectory_msgs::msg::JointTrajectoryPoint> first_trajectory_points(received_trajectory_points.begin(), received_trajectory_points.end() - second_trajectory_size);
    //
    //     RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "First trajectory size: %zu", first_trajectory_points.size());
    //     RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Second trajectory size: %zu", second_trajectory_points.size());
    //
    //     // 重设第二段轨迹的时间
    //     rclcpp::Duration time_offset(0, 0);  // 从0开始
    //
    //     for (auto& point : second_trajectory_points) {
    //         point.time_from_start = time_offset;
    //         time_offset = rclcpp::Duration(0, time_offset.nanoseconds() + 100'000'000);  // 每个点增加 0.1s (100ms)
    //     }
    //
    //     // 执行第一段轨迹
    //     RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Executing first trajectory...");
    //     moveit_msgs::msg::RobotTrajectory msg_trajectory;
    //     trajectory_msgs::msg::JointTrajectory& joint_trajectory = msg_trajectory.joint_trajectory;
    //
    //     // 设置关节名称
    //     joint_trajectory.joint_names = joint_names;
    //
    //     // 添加第一段轨迹点
    //     joint_trajectory.points = first_trajectory_points;
    //
    //     // 使用 MoveGroupInterface::Plan 规划
    //     moveit::planning_interface::MoveGroupInterface::Plan plan,plan2;
    //     move_group_.plan(plan);
    //     plan.trajectory_.joint_trajectory = joint_trajectory;
    //
    //     // 添加第二段轨迹点
    //     joint_trajectory.points = second_trajectory_points;
    //
    //     // 使用 MoveGroupInterface::Plan 规划
    //     move_group_.plan(plan2);
    //     plan2.trajectory_.joint_trajectory = joint_trajectory;
    //
    //     // ----------------- 加上轨迹碰撞检测 -----------------
    //     // 构造 RobotTrajectory 对象
    //     robot_trajectory::RobotTrajectory robot_traj(move_group_.getRobotModel(), move_group_.getName());
    //     robot_traj.setRobotTrajectoryMsg(*move_group_.getCurrentState(), plan.trajectory_.joint_trajectory);
    //
    //     auto planning_scene = planning_scene_monitor_->getPlanningScene();
    //     if (!planning_scene) {
    //         RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "PlanningScene is NULL! 取消执行");
    //         return;
    //     }
    //
    //     bool trajectory_valid = true;
    //     moveit::core::RobotStatePtr temp_state(new moveit::core::RobotState(*move_group_.getCurrentState()));
    //     std::size_t collision_index = robot_traj.getWayPointCount();  // 初始化为最大值
    //     for (std::size_t i = 0; i < robot_traj.getWayPointCount(); ++i) {
    //         *temp_state = robot_traj.getWayPoint(i);
    //
    //         if (planning_scene->isStateColliding(*temp_state, move_group_.getName())) {
    //             RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "碰撞检测失败，轨迹第%zu个点发生碰撞！", i);
    //             trajectory_valid = false;
    //             collision_index = i;
    //             break;
    //         }
    //     }
    //     // ---------------------------------------------------
    //     // 可视化轨迹
    //     // visual_tools_->publishTrajectoryLine(plan.trajectory_, move_group_.getCurrentState()->getJointModelGroup(move_group_.getName()));
    //     // visual_tools_->trigger();
    //     // visual_tools_->publishTrajectoryLine(plan2.trajectory_, move_group_.getCurrentState()->getJointModelGroup(move_group_.getName()));
    //     // visual_tools_->trigger();
    //     // 可视化轨迹，分段处理颜色
    //     auto joint_model_group = move_group_.getCurrentState()->getJointModelGroup(move_group_.getName());
    //
    //     if (!trajectory_valid) {
    //         // 获取完整轨迹
    //         moveit_msgs::msg::RobotTrajectory full_trajectory_msg;
    //         robot_traj.getRobotTrajectoryMsg(full_trajectory_msg);
    //         auto& full_joint_trajectory = full_trajectory_msg.joint_trajectory;
    //
    //         // 分段轨迹：绿色 + 红色
    //         moveit_msgs::msg::RobotTrajectory green_traj_msg;
    //         green_traj_msg.joint_trajectory.joint_names = full_joint_trajectory.joint_names;
    //         green_traj_msg.joint_trajectory.points.insert(
    //             green_traj_msg.joint_trajectory.points.end(),
    //             full_joint_trajectory.points.begin(),
    //             full_joint_trajectory.points.begin() + collision_index
    //         );
    //
    //         moveit_msgs::msg::RobotTrajectory red_traj_msg;
    //         red_traj_msg.joint_trajectory.joint_names = full_joint_trajectory.joint_names;
    //         red_traj_msg.joint_trajectory.points.insert(
    //             red_traj_msg.joint_trajectory.points.end(),
    //             full_joint_trajectory.points.begin() + collision_index,
    //             full_joint_trajectory.points.end()
    //         );
    //
    //         // 绿色部分
    //         visual_tools_->publishTrajectoryLine(green_traj_msg, joint_model_group, rviz_visual_tools::GREEN);
    //         visual_tools_->trigger();
    //
    //         // 红色部分
    //         visual_tools_->publishTrajectoryLine(red_traj_msg, joint_model_group, rviz_visual_tools::RED);
    //         visual_tools_->trigger();
    //     } else {
    //         // 整条轨迹绿色
    //         visual_tools_->publishTrajectoryLine(plan.trajectory_, joint_model_group, rviz_visual_tools::GREEN);
    //         visual_tools_->trigger();
    //         visual_tools_->publishTrajectoryLine(plan2.trajectory_, joint_model_group, rviz_visual_tools::GREEN);
    //         visual_tools_->trigger();
    //     }
    //
    //     if (!trajectory_valid) {
    //         RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "轨迹无效，只执行到发生碰撞的点！");
    //
    //         // 先定义一个RobotTrajectoryMsg
    //         moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
    //
    //         // 正确调用getRobotTrajectoryMsg()，填进去
    //         robot_traj.getRobotTrajectoryMsg(robot_trajectory_msg);
    //
    //         // 再拿joint_trajectory
    //         auto& full_joint_trajectory = robot_trajectory_msg.joint_trajectory;
    //
    //         // 裁剪轨迹，只保留到碰撞发生点
    //         std::vector<trajectory_msgs::msg::JointTrajectoryPoint> safe_trajectory_points(
    //             full_joint_trajectory.points.begin(),
    //             full_joint_trajectory.points.begin() + collision_index
    //         );
    //
    //         // 重建新的轨迹
    //         moveit_msgs::msg::RobotTrajectory safe_msg_trajectory;
    //         trajectory_msgs::msg::JointTrajectory& safe_joint_trajectory = safe_msg_trajectory.joint_trajectory;
    //         safe_joint_trajectory.joint_names = full_joint_trajectory.joint_names;
    //         safe_joint_trajectory.points = safe_trajectory_points;
    //
    //         // 创建新的计划并执行
    //         moveit::planning_interface::MoveGroupInterface::Plan safe_plan;
    //         move_group_.plan(safe_plan);  // Dummy骨架
    //         safe_plan.trajectory_ = safe_msg_trajectory;
    //
    //         moveit::core::MoveItErrorCode execution_result = move_group_.execute(safe_plan);
    //         if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
    //             RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "执行裁剪后的轨迹失败！");
    //         } else {
    //             RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "成功执行裁剪后的轨迹！");
    //         }
    //
    //         return;
    //     }
    //     else {
    //         // 如果轨迹有效，才执行动作
    //         RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "轨迹无碰撞，尝试执行动作");
    //     }
    //
    //     // 执行第一段轨迹
    //     moveit::core::MoveItErrorCode execution_result = move_group_.execute(plan);
    //     if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
    //         RCLCPP_ERROR(rclcpp::get_logger("mpc_planning"), "Execution of first trajectory failed.");
    //         return;
    //     } else {
    //         RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "First trajectory executed successfully.");
    //     }
    //
    //     // **执行 open_gripper 和 attach 物体操作**
    //     RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Opening gripper and attaching object...");
    //     closeGripper();
    //     attachObject(object);
    //     // 执行第二段轨迹
    //     RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Executing second trajectory...");
    //
    //     // 执行第二段轨迹
    //     execution_result = move_group_.execute(plan2);
    //     if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
    //         RCLCPP_ERROR(rclcpp::get_logger("mpc_planning"), "Execution of second trajectory failed.");
    //     } else {
    //         RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Second trajectory executed successfully.");
    //     }
    // }
}

void ManipulatorAction::dynamic_grasping(Object &object, TargetPose &target_pose, GraspPose &grasp_pose,bool isFixed) {

    const double position_tolerance = 0.03;  // 位置误差容差（米）
    const double orientation_tolerance = 0.05;  // 姿态误差容差（四元数点乘接近1）
    const int max_iterations = 300;  // 防止死循环

    geometry_msgs::msg::Pose target_pose_;
    geometry_msgs::msg::Pose current_pose;
    geometry_msgs::msg::Pose grasp_offset_pose;

    int iteration = 0;
    bool within_tolerance = false;
    bool isfirst=true;
    subscribeTopositionUpdate();
    // 等待接收到更新
    while (!isupdate && rclcpp::ok()) {
        rclcpp::sleep_for(std::chrono::milliseconds(1));  // 等一下
    }
    isupdate=false;
    // 1. 获取关节值（包含底盘的 x, y, theta）
    // std::vector<double> joint_values = move_group_.getCurrentJointValues();
    //
    // double car_x = joint_values[0];      // position_base_x
    // double car_y = joint_values[1];      // position_base_y
    // double car_theta = joint_values[2];  // position_base_theta
    //
    // double x1 =target_position_.x1; // 例如视觉检测得到
    // double y1 = target_position_.y1;
    // target_pose_.position.x = car_x + std::cos(car_theta) * x1 - std::sin(car_theta) * y1;
    // target_pose_.position.y = car_y + std::sin(car_theta) * x1 + std::cos(car_theta) * y1;
    target_pose_.position.x = target_position_.x1;
    target_pose_.position.y = target_position_.y1;
    target_pose_.position.z = 0.115;
    target_pose_.orientation.x = 0.0;
    target_pose_.orientation.y = 1.0;
    target_pose_.orientation.z = 0.0;
    target_pose_.orientation.w = 0.0;

    // 获取抓取偏移位姿
    grasp_offset_pose = grasp_pose.getGraspOffsetPose(target_pose_);
    geometry_msgs::msg::Pose placepoce=target_pose.getTargetPose();
    while (!within_tolerance && iteration < max_iterations) {
        RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Iteration %d: Planning grasp...", iteration + 1);
        // 执行动态路径规划
        if(isfirst) {
            dynamic_mpc_planning(grasp_offset_pose,isFixed);
            execute_trajectory();

            isfirst=false;
        }
        else {
            publish_to_python(grasp_offset_pose);
            execute_trajectory();


        }

        // 获取当前机器人状态
        moveit::core::RobotStatePtr current_state = move_group_.getCurrentState(10.0);
        if (!current_state) {
            RCLCPP_ERROR(rclcpp::get_logger("mpc_planning"), "Failed to get current robot state.");
            return;
        }
        const std::string end_effector_link = move_group_.getEndEffectorLink();
        // 获取末端执行器当前位姿（将 Eigen::Isometry3d 转换为 geometry_msgs::msg::Pose）
        const Eigen::Isometry3d& ee_state = current_state->getGlobalLinkTransform(end_effector_link);

        current_pose.position.x = ee_state.translation().x();
        current_pose.position.y = ee_state.translation().y();
        current_pose.position.z = ee_state.translation().z();

        Eigen::Quaterniond quat(ee_state.rotation());
        current_pose.orientation.x = quat.x();
        current_pose.orientation.y = quat.y();
        current_pose.orientation.z = quat.z();
        current_pose.orientation.w = quat.w();
        //在计算之前 再次更新坐标
        subscribeTopositionUpdate();
        while (!isupdate && rclcpp::ok()) {
            rclcpp::sleep_for(std::chrono::milliseconds(10));  // 等一下
        }

        isupdate=false;
        // joint_values = move_group_.getCurrentJointValues();
        //
        // car_x = joint_values[0];      // position_base_x
        // car_y = joint_values[1];      // position_base_y
        // car_theta = joint_values[2];  // position_base_theta
        // // <<<<------ 添加打印 ↓↓↓
        // RCLCPP_INFO(rclcpp::get_logger("mpc_planning"),
        //     "Current Car Pose: x=%.4f, y=%.4f, theta=%.4f", car_x, car_y, car_theta);
        // // <<<<------ 添加打印 ↑↑↑
        //
        // x1 =target_position_.x1; // 例如视觉检测得到
        // y1 = target_position_.y1;
        target_pose_.position.x = target_position_.x1;
        target_pose_.position.y = target_position_.y1;
        target_pose_.position.z = 0.115;
        target_pose_.orientation.x = 0.0;
        target_pose_.orientation.y = 1.0;
        target_pose_.orientation.z = 0.0;
        target_pose_.orientation.w = 0.0;

        // 获取抓取偏移位姿
        grasp_offset_pose = grasp_pose.getGraspOffsetPose(target_pose_);
        // 计算位置误差
        // 计算位置误差
        double dx = grasp_offset_pose.position.x - current_pose.position.x;
        double dy = grasp_offset_pose.position.y - current_pose.position.y;
        double dz = grasp_offset_pose.position.z - current_pose.position.z;

        // 拆解误差
        double xy_error = std::sqrt(dx * dx + dy * dy);
        double z_error = fabs(dz);  // 如果你只关心绝对差值

        // 计算姿态误差（四元数点乘）
        tf2::Quaternion q1, q2;
        tf2::fromMsg(grasp_offset_pose.orientation, q1);
        tf2::fromMsg(current_pose.orientation, q2);
        double orientation_similarity = fabs(q1.dot(q2));  // 接近 1 表示姿态相近
        float xy_tolerance=0.005;
        float z_tolerance=0.01;
        // 误差容差判断
        within_tolerance = (xy_error < xy_tolerance) &&
                           (z_error < z_tolerance) &&
                           (orientation_similarity > (1.0 - orientation_tolerance));


        RCLCPP_INFO(rclcpp::get_logger("mpc_planning"),
                    "Position Error: %.4f, Orientation Similarity: %.4f",
                    xy_error, orientation_similarity);

        iteration++;
    }

    if (within_tolerance) {
        std_msgs::msg::Float32 msg;
        msg.data = 3.0;  // 夹爪闭合状态，发布 0
        gripper_state_publisher_->publish(msg);

        publish_stop_to_python();
        closeGripper();
        attachObject(object);
        moveToPose_compare(placepoce);
        // mpc_planning_task2(placepoce);
        // moveToPose(placepoce);
        openGripper();
        detachObject(object);
        RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Grasp pose reached successfully within tolerance.");
    } else {
        publish_stop_to_python();
        RCLCPP_WARN(rclcpp::get_logger("mpc_planning"), "Max iterations reached. Final pose may be inaccurate.");
    }

    // 输出最终目标与实际位姿
    RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Final Target Position: [%.4f, %.4f, %.4f]",
                grasp_offset_pose.position.x, grasp_offset_pose.position.y, grasp_offset_pose.position.z);
    RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Final Actual Position: [%.4f, %.4f, %.4f]",
                current_pose.position.x, current_pose.position.y, current_pose.position.z);

    RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Final Target Orientation: [%.4f, %.4f, %.4f, %.4f]",
                grasp_offset_pose.orientation.x, grasp_offset_pose.orientation.y,
                grasp_offset_pose.orientation.z, grasp_offset_pose.orientation.w);
    RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Final Actual Orientation: [%.4f, %.4f, %.4f, %.4f]",
                current_pose.orientation.x, current_pose.orientation.y,
                current_pose.orientation.z, current_pose.orientation.w);
}

void ManipulatorAction::dynamic_mpc_planning(const geometry_msgs::msg::Pose &pose,bool isFixed) {
    RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Starting MPC planning...");

    if(!isFixed) {
        // 构造命令行参数
        std::ostringstream cmd_stream;
        cmd_stream << "python3 /home/fins/github/hmpc/src/hmpc_python/moveit_hmpc_planner_task3.py"
                   << " "
                   << pose.position.x << " " << pose.position.y << " " << pose.position.z
                   << " "
                   << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z << " " << pose.orientation.w
                   << " &";

        std::string command = cmd_stream.str();
        RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Running command: %s", command.c_str());

        int ret = std::system(command.c_str());

        if (ret != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("mpc_planning"), "Failed to start Python MPC planner.");
            return;
        }

        RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Python MPC planner started.");
    }
    else {
        // 构造命令行参数
        std::ostringstream cmd_stream;
        cmd_stream << "python3 /home/fins/github/hmpc/src/hmpc_python/moveit_hmpc_planner_task3_compare.py"
                   << " "
                   << pose.position.x << " " << pose.position.y << " " << pose.position.z
                   << " "
                   << pose.orientation.x << " " << pose.orientation.y << " " << pose.orientation.z << " " << pose.orientation.w
                   << " &";

        std::string command = cmd_stream.str();
        RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Running command: %s", command.c_str());

        int ret = std::system(command.c_str());

        if (ret != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("mpc_planning"), "Failed to start Python MPC planner.");
            return;
        }

        RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Python MPC planner started.");
    }
}

// 负责执行接收到的轨迹
void ManipulatorAction::execute_trajectory() {
    // 创建 ROS 2 节点
    auto node = rclcpp::Node::make_shared("mpc_joint_trajectory_subscriber");

    // 存储接收到的轨迹点
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> received_trajectory_points;
    std::vector<std::string> joint_names;

    // 记录最后接收到消息的时间
    auto last_received_time = std::chrono::steady_clock::now();

    auto callback = [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
        RCLCPP_INFO(rclcpp::get_logger("mpc_joint_trajectory_subscriber"), "Received JointState message.");

        if (msg->name.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("mpc_joint_trajectory_subscriber"), "No joint data received!");
            return;
        }

        // 获取关节名称（仅在第一次接收时存储）
        if (joint_names.empty()) {
            joint_names = msg->name;
        }

        // 解析关节数据
        trajectory_msgs::msg::JointTrajectoryPoint joint_point;
        joint_point.positions = msg->position;
        joint_point.velocities = msg->velocity;
        joint_point.effort = msg->effort;

        // 设置时间间隔
        if (received_trajectory_points.empty()) {
            joint_point.time_from_start = rclcpp::Duration(0, 0);
        } else {
            joint_point.time_from_start = rclcpp::Duration::from_nanoseconds(received_trajectory_points.size() * 100'000'000); // 每 0.1s 一个点
        }

        received_trajectory_points.push_back(joint_point);

        // 更新最后接收到消息的时间
        last_received_time = std::chrono::steady_clock::now();
    };

    // 订阅 MPC 轨迹
    auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
        "/mpc_joint_trajectory",
        100,
        callback
    );

    // 运行订阅器
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    auto start = std::chrono::steady_clock::now();

    // 设置超时阈值为0.01秒
    std::chrono::milliseconds timeout(10);
    while (rclcpp::ok()) {
        executor.spin_some();

        // 如果队列中没有消息且没有收到新的消息超过0.01秒，停止订阅
        if (!received_trajectory_points.empty() &&
            std::chrono::steady_clock::now() - last_received_time > timeout) {
            RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "No new messages for 0.01 seconds and queue is empty, stopping subscription.");
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // **执行接收到的轨迹**
    if (!received_trajectory_points.empty()) {
        RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Executing MPC trajectory...");

        // 构造 MoveIt 轨迹消息
        moveit_msgs::msg::RobotTrajectory msg_trajectory;
        trajectory_msgs::msg::JointTrajectory& joint_trajectory = msg_trajectory.joint_trajectory;

        // 设置关节名称
        joint_trajectory.joint_names = joint_names;

        // 添加轨迹点
        joint_trajectory.points = received_trajectory_points;

        // 使用 MoveGroupInterface::Plan 规划
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        move_group_.plan(plan);
        plan.trajectory_.joint_trajectory = joint_trajectory;
        // ----------------- 加上轨迹碰撞检测 -----------------
        // 构造 RobotTrajectory 对象
        robot_trajectory::RobotTrajectory robot_traj(move_group_.getRobotModel(), move_group_.getName());
        robot_traj.setRobotTrajectoryMsg(*move_group_.getCurrentState(), plan.trajectory_.joint_trajectory);

        auto planning_scene = planning_scene_monitor_->getPlanningScene();
        if (!planning_scene) {
            RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "PlanningScene is NULL! 取消执行");
            return;
        }

        bool trajectory_valid = true;
        moveit::core::RobotStatePtr temp_state(new moveit::core::RobotState(*move_group_.getCurrentState()));
        std::size_t collision_index = robot_traj.getWayPointCount();  // 初始化为最大值
        for (std::size_t i = 0; i < robot_traj.getWayPointCount(); ++i) {
            *temp_state = robot_traj.getWayPoint(i);

            if (planning_scene->isStateColliding(*temp_state, move_group_.getName())) {
                RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "碰撞检测失败，轨迹第%zu个点发生碰撞！", i);
                trajectory_valid = false;
                collision_index = i;
                break;
            }
        }
        // ---------------------------------------------------
        // 可视化轨迹
        visual_tools_->publishTrajectoryLine(plan.trajectory_, move_group_.getCurrentState()->getJointModelGroup(move_group_.getName()));
        visual_tools_->trigger();
        if(collision_index==0) {
            RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "无法执行，直接返回");
            return;
        }
        if (!trajectory_valid && collision_index!=0) {
            RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "轨迹无效，只执行到发生碰撞的点！");

            // 先定义一个RobotTrajectoryMsg
            moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;

            // 正确调用getRobotTrajectoryMsg()，填进去
            robot_traj.getRobotTrajectoryMsg(robot_trajectory_msg);

            // 再拿joint_trajectory
            auto& full_joint_trajectory = robot_trajectory_msg.joint_trajectory;

            // 裁剪轨迹，只保留到碰撞发生点
            std::vector<trajectory_msgs::msg::JointTrajectoryPoint> safe_trajectory_points(
                full_joint_trajectory.points.begin(),
                full_joint_trajectory.points.begin() + collision_index
            );

            // 重建新的轨迹
            moveit_msgs::msg::RobotTrajectory safe_msg_trajectory;
            trajectory_msgs::msg::JointTrajectory& safe_joint_trajectory = safe_msg_trajectory.joint_trajectory;
            safe_joint_trajectory.joint_names = full_joint_trajectory.joint_names;
            safe_joint_trajectory.points = safe_trajectory_points;

            // 创建新的计划并执行
            moveit::planning_interface::MoveGroupInterface::Plan safe_plan;
            move_group_.plan(safe_plan);  // Dummy骨架
            safe_plan.trajectory_ = safe_msg_trajectory;

            moveit::core::MoveItErrorCode execution_result = move_group_.execute(safe_plan);
            if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "执行裁剪后的轨迹失败！");
            } else {
                RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "成功执行裁剪后的轨迹！");
            }

            return;
        }

        // 如果轨迹有效，才执行动作
        RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "轨迹无碰撞，尝试执行动作");


        // 执行轨迹
        moveit::core::MoveItErrorCode execution_result = move_group_.execute(plan);
        if (execution_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(rclcpp::get_logger("mpc_planning"), "Execution failed.");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "MPC trajectory executed successfully.");
        }

    } else {
        RCLCPP_WARN(rclcpp::get_logger("mpc_planning"), "No trajectory received. Execution skipped.");
    }
}

void ManipulatorAction::poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {


    // 更新结构体内的坐标
    target_position_.x1 = msg->poses[0].position.x;
    target_position_.y1 = msg->poses[0].position.y;
    target_position_.order1=msg->poses[0].position.z;
    target_position_.x2 = msg->poses[1].position.x;
    target_position_.y2 = msg->poses[1].position.y;
    target_position_.order2=msg->poses[1].position.z;
    target_position_.x3 = msg->poses[2].position.x;
    target_position_.y3 = msg->poses[2].position.y;
    target_position_.order3 = msg->poses[2].position.z;
    // 输出更新日志
    RCLCPP_INFO(node_->get_logger(),
        "接收到新的坐标: x1=%.6f, y1=%.6f, x2=%.6f, y2=%.6f, x3=%.6f, y3=%.6f",
        target_position_.x1, target_position_.y1,
        target_position_.x2, target_position_.y2,
        target_position_.x3, target_position_.y3);
    // 取消订阅
    pose_subscriber.reset();  // 取消订阅
    isupdate=true;
}

void ManipulatorAction::subscribeTopositionUpdate() {
    // 检查是否已经有订阅者存在，如果有则不再创建新的订阅者
    // if (pose_subscriber) {
    //     RCLCPP_INFO(node_->get_logger(), "订阅者已存在，不再重新创建");
    //     return;  // 如果已经订阅过了，直接返回
    // }
    RCLCPP_INFO(node_->get_logger(),
    "触发subsribe");
    pose_subscriber = node_->create_subscription<geometry_msgs::msg::PoseArray>(
    "arm_pose", 10, std::bind(&ManipulatorAction::poseArrayCallback, this, std::placeholders::_1)
);
}
void ManipulatorAction::publish_to_python(const geometry_msgs::msg::Pose &pose) {
    // 发布 Pose 数据到 Python 程序
    mpc_pose_publisher_->publish(pose);
    RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Published pose to Python: [%.4f, %.4f, %.4f]",
                pose.position.x, pose.position.y, pose.position.z);
}

void ManipulatorAction::publish_stop_to_python() {
    // 创建一个 Pose 消息，并将其 7 个数设置为 999
    geometry_msgs::msg::Pose stop_pose;
    stop_pose.position.x = 999.0;
    stop_pose.position.y = 999.0;
    stop_pose.position.z = 999.0;

    stop_pose.orientation.x = 999.0;
    stop_pose.orientation.y = 999.0;
    stop_pose.orientation.z = 999.0;
    stop_pose.orientation.w = 999.0;

    // 发布 Pose 数据到 Python 程序
    mpc_pose_publisher_->publish(stop_pose);
    RCLCPP_INFO(rclcpp::get_logger("mpc_planning"), "Published stop pose (999s) to Python: [%.4f, %.4f, %.4f]",
                stop_pose.position.x, stop_pose.position.y, stop_pose.position.z);
}

bool ManipulatorAction::collisionCheckAndExecute(
    moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string& label)
{
    auto joint_model_group = move_group_.getCurrentState()->getJointModelGroup(move_group_.getName());

    // 转换为RobotTrajectory
    robot_trajectory::RobotTrajectory robot_traj(move_group_.getRobotModel(), move_group_.getName());
    robot_traj.setRobotTrajectoryMsg(*move_group_.getCurrentState(), plan.trajectory_.joint_trajectory);

    auto planning_scene = planning_scene_monitor_->getPlanningScene();
    if (!planning_scene) {
        RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "[%s] PlanningScene is NULL!", label.c_str());
        return false;
    }

    // 碰撞检测
    bool trajectory_valid = true;
    std::size_t collision_index = robot_traj.getWayPointCount();
    moveit::core::RobotStatePtr temp_state(new moveit::core::RobotState(*move_group_.getCurrentState()));

    // for (std::size_t i = 0; i < robot_traj.getWayPointCount(); ++i) {
    //     *temp_state = robot_traj.getWayPoint(i);
    //     if (planning_scene->isStateColliding(*temp_state, move_group_.getName())) {
    //         RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "[%s] 轨迹第 %zu 个点发生碰撞！", label.c_str(), i);
    //         trajectory_valid = false;
    //         collision_index = i;
    //         break;
    //     }
    // }

    for (std::size_t i = 0; i < robot_traj.getWayPointCount(); ++i) {
        *temp_state = robot_traj.getWayPoint(i);

        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        collision_request.contacts = true;
        collision_request.max_contacts = 100;
        collision_request.max_contacts_per_pair = 1;
        collision_request.verbose = false;

        planning_scene->checkCollision(collision_request, collision_result, *temp_state);

        if (collision_result.collision) {
            RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "[%s] 轨迹第 %zu 个点发生碰撞！", label.c_str(), i);
            for (const auto& contact : collision_result.contacts) {
                RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"),
                             "碰撞对象: %s 与 %s",
                             contact.first.first.c_str(),
                             contact.first.second.c_str());
            }
            trajectory_valid = false;
            collision_index = i;
            break;
        }
    }


    // 可视化轨迹（分段红绿）
    moveit_msgs::msg::RobotTrajectory full_msg;
    robot_traj.getRobotTrajectoryMsg(full_msg);
    const auto& traj = full_msg.joint_trajectory;

    if (!trajectory_valid) {
        // 分成绿色 + 红色
        moveit_msgs::msg::RobotTrajectory green_traj, red_traj;
        green_traj.joint_trajectory.joint_names = traj.joint_names;
        red_traj.joint_trajectory.joint_names = traj.joint_names;

        green_traj.joint_trajectory.points.assign(traj.points.begin(), traj.points.begin() + collision_index);
        red_traj.joint_trajectory.points.assign(traj.points.begin() + collision_index, traj.points.end());

        visual_tools_->publishTrajectoryLine(green_traj, joint_model_group, rviz_visual_tools::GREEN);
        visual_tools_->trigger();
        visual_tools_->publishTrajectoryLine(red_traj, joint_model_group, rviz_visual_tools::RED);
        visual_tools_->trigger();
    } else {
        visual_tools_->publishTrajectoryLine(full_msg, joint_model_group, rviz_visual_tools::GREEN);
        visual_tools_->trigger();
    }
    auto result = move_group_.execute(plan);
    if(trajectory_valid) {
        RCLCPP_INFO(rclcpp::get_logger("move_group_interface"),"没有发生碰撞");
        return true;
    }else {
        RCLCPP_INFO(rclcpp::get_logger("move_group_interface"),"发生了碰撞");
        return true;
    }

    //  // 执行轨迹
    //  if (!trajectory_valid) {
    //      RCLCPP_WARN(rclcpp::get_logger("move_group_interface"), "[%s] 执行裁剪轨迹", label.c_str());
    //
    //      moveit_msgs::msg::RobotTrajectory safe_traj;
    //      safe_traj.joint_trajectory.joint_names = traj.joint_names;
    //      safe_traj.joint_trajectory.points.assign(traj.points.begin(), traj.points.begin() + collision_index);
    //
    //      moveit::planning_interface::MoveGroupInterface::Plan safe_plan;
    //      move_group_.plan(safe_plan);  // dummy
    //      safe_plan.trajectory_ = safe_traj;
    //
    //      auto result = move_group_.execute(safe_plan);
    //      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    //          RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "[%s] 裁剪轨迹执行失败", label.c_str());
    //      } else {
    //          RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "[%s] 成功执行裁剪轨迹", label.c_str());
    //      }
    //  } else {
    //      RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "[%s] 执行完整轨迹", label.c_str());
    //      auto result = move_group_.execute(plan);
    //      if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    //          RCLCPP_ERROR(rclcpp::get_logger("move_group_interface"), "[%s] 执行完整轨迹失败", label.c_str());
    //      } else {
    //          RCLCPP_INFO(rclcpp::get_logger("move_group_interface"), "[%s] 成功执行完整轨迹", label.c_str());
    //          return true;
    //      }
    // }
    //  return false;  // 裁剪失败或执行失败
}