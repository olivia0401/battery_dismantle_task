#!/usr/bin/env python3
"""
LLM Agent Launch File - Complete setup for battery disassembly

Launches:
1. robot_state_publisher - Robot model
2. move_group - MoveIt motion planning
3. rviz2 - Visualization
4. skill_server - ROS2 skill execution server (接收Python LLM Agent的命令)

Usage:
    ros2 launch battery_dismantle_task llm_agent.launch.py

Then in another terminal:
    cd battery_dismantle_task/LLM_Robot_Agent
    python3 main.py
"""

import os
import yaml
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time"
    )

    # Package paths
    moveit_pkg_name = "battery_dismantle_task"
    moveit_share = Path(get_package_share_directory(moveit_pkg_name))
    kortex_description_share = Path(get_package_share_directory("kortex_description"))

    # File paths
    xacro_file = str(kortex_description_share / "robots" / "gen3_robotiq_2f_85.xacro")
    srdf_file = str(moveit_share / "config" / "gen3_robotiq_2f_85.srdf")
    rviz_file = str(moveit_share / "config" / "moveit.rviz")
    waypoints_file = str(moveit_share / "config" / "waypoints.json")
    qos_params_file = str(moveit_share / "config" / "robot_state_publisher_qos.yaml")
    controllers_file = str(moveit_share / "config" / "moveit_controllers.yaml")

    # Home pose (non-colliding start configuration)
    home_pose_str = '''{'joint_1': 0.0, 'joint_2': 0.2618, 'joint_3': 3.14159, 'joint_4': -2.2689, 'joint_5': 0.0, 'joint_6': 0.9599, 'joint_7': 1.5708}'''

    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder(robot_name="kinova_gen3_6dof_robotiq_2f_85")
        .robot_description(
            file_path=xacro_file,
            mappings={"use_fake_hardware": "true", "initial_positions": home_pose_str},
        )
        .robot_description_semantic(file_path=srdf_file)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Kinematics parameters
    kinematics_params = {
        "robot_description_kinematics": {
            "manipulator": {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            },
            "gripper": {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            }
        }
    }

    # Load controller configuration
    with open(controllers_file, 'r') as f:
        controllers_config = yaml.safe_load(f)

    controllers_params = {
        "moveit_simple_controller_manager": controllers_config
    }

    # ============================================================
    # Node 1: robot_state_publisher
    # ============================================================
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            qos_params_file,
        ],
    )

    # ============================================================
    # Node 2: move_group
    # ============================================================
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            controllers_params,
            kinematics_params,
        ],
    )

    # ============================================================
    # Node 3: rviz2
    # ============================================================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    # ============================================================
    # Node 4: skill_server (THE MISSING PIECE!)
    # ============================================================
    # This node receives commands from Python LLM Agent via /llm_commands topic
    # and publishes feedback via /llm_feedback topic
    skill_server_node = Node(
        package="battery_dismantle_task",
        executable="skill_server",
        name="skill_server",
        output="screen",
        parameters=[
            {
                "waypoints_path": waypoints_file,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    # Delay skill_server to ensure move_group is ready
    delayed_skill_server = TimerAction(
        period=5.0,  # Wait 5 seconds for move_group to initialize
        actions=[skill_server_node]
    )

    # ============================================================
    # Assemble Launch Description
    # ============================================================
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
        delayed_skill_server,  # Start skill_server after 5 seconds
    ])
