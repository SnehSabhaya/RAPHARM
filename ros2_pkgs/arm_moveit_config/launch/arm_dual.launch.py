import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import Command
import launch_ros.descriptions
import launch_ros.substitutions
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    robot_model = 'arm'
    package_name = 'arm'
    urdf_file_path = 'urdf/arm.xacro'
    world_file_path = 'worlds/empty_world.world'

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model = os.path.join(pkg_share, urdf_file_path)
    world_path = os.path.join(pkg_share, world_file_path)



    moveit_config = (
        MoveItConfigsBuilder("arm")
        .robot_description(file_path="config/arm.urdf.xacro")
        .robot_description_semantic(file_path="config/arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("arm_moveit_config"), "config"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[moveit_config.to_dict()],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )


	# Spawn the robot in Gazebo
    spawn_entity_robot = Node(package     ='gazebo_ros', 
							  executable  ='spawn_entity.py', 
							  arguments   = ['-entity', 'arm', '-topic', 'robot_description',"-x", "0.0", "-y", "0.0", "-z", "0.0"],
							  output      ='screen')

	# Gazebo   
    gazebo_node = ExecuteProcess(cmd=['gazebo', '--verbose', world_path,'-s', 'libgazebo_ros_factory.so'], output='screen')


    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("arm_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "joint_state_broadcaster",
    #         "--controller-manager",
    #         "/controller_manager",
    #     ],
    # )

    # ur_manipiulator_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "ur_manipulator_controller",
    #         "--controller-manager",
    #         "/controller_manager",
    #     ],
    # )
    # gripper_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "gripper_controller",  # Controller name from ros2_controllers.yaml
    #         "--controller-manager",
    #         "/controller_manager",  # Controller manager namespace from ros2_controllers.yaml
    #     ],
    # )

    load_joint_state_broadcaster = ExecuteProcess(
										cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','joint_state_broadcaster'],
										output='screen')

	
    load_joint_trajectory_controller = ExecuteProcess( 
										cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'ur_manipulator_controller'], 
										output='screen')
    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            spawn_entity_robot,
            gazebo_node,
            ros2_control_node,
            # joint_state_broadcaster_spawner,
            # ur_manipiulator_controller_spawner,
            # gripper_controller_spawner,
            load_joint_state_broadcaster,
            load_joint_trajectory_controller
        ]
    )
