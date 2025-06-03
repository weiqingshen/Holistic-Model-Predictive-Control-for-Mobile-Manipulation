import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 初始化 moveit_config（读取 URDF、SRDF 等）
    moveit_config = (
        MoveItConfigsBuilder("mybot")  # "robot name"
        .robot_description(file_path="config/robot_arm_description.urdf.xacro")
        .robot_description_semantic(file_path="config/robot_arm_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_planning_scene=True)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"])
        .to_moveit_configs()
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("mybot"),
        "config",
        "move_group.rviz"
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("mybot"),
        "config",
        "ros2_controllers.yaml"
    )

    # 参数广播节点（关键，支持你的 C++ 程序用 move_group_interface）
    param_broadcaster = Node(
        package="rclcpp_components",
        executable="component_container_mt",
        name="parameter_broadcaster",
        parameters=[moveit_config.to_dict()],
        output="screen"
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # static tf world -> base_link
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"]
    )

    # move_group 节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()]
    )

    # rviz2 节点
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ]
    )

    # ros2_control
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
    )

    # controllers
    controller_spawners = []
    for controller in [
        "joint_state_broadcaster",
        "arm_move_controller",
        "hand_controller",
    ]:
        controller_spawners.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            param_broadcaster,
            robot_state_publisher,
            static_tf,
            ros2_control_node,
            move_group_node,
            rviz_node,
        ] + controller_spawners
    )
