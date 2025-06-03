from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("mybot")
        .robot_description(file_path="config/robot_arm_description.urdf.xacro")
        .robot_description_semantic(file_path="config/robot_arm_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"])
        .to_moveit_configs()
    )

    return LaunchDescription([
        Node(
            package="mybot",
            executable="rrt_base",
            output="screen",
            parameters=[moveit_config.to_dict()],
        )
    ])

