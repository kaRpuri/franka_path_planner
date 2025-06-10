from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Robot description URDF
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("franka_description"),
            "robots", "fr3", "fr3.urdf.xacro"
        ]),
        " hand:=true",
        " arm_id:=fr3"
    ])
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    
    # Robot description semantic SRDF - CORRECTED FILENAME
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("franka_fr3_moveit_config"),
            "srdf", "fr3_arm.srdf.xacro"  # ← CORRECTED PATH
        ]),
        " hand:=true",
        " arm_id:=fr3"
    ])
    
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }
    
    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"publish_robot_description_semantic": True},
        ]
    )
    
    # Your test path node
    test_path_node = Node(
        package="franka_path_planner",
        executable="test_path",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
        ]
    )
    
    return LaunchDescription([
        move_group_node,
        test_path_node,
    ])
