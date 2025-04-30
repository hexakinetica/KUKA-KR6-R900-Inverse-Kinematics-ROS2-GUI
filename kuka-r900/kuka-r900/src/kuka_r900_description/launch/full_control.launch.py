from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("kuka_r900_description"),
            "urdf",
            "kuka_r900.urdf.xacro"
        ])
    ])

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("kuka_r900_description"),
        "config",
        "position_controllers.yaml"
    ])

    return LaunchDescription([
        # Controller manager node (ros2_control)
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": robot_description_content}, controllers_yaml],
            output="screen"
        ),

        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description_content}],
            output="screen"
        ),

        # Spawner: joint_state_broadcaster
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen"
        ),

        # Spawner: position_trajectory_controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["position_trajectory_controller", "--controller-manager", "/controller_manager"],
            output="screen"
        ),

        Node(
            package="kuka_r900_kinematics",
            executable="kdl_ik_node",
            name="kdl_ik_node",
            parameters=[{"robot_description": robot_description_content}],
            output="screen"
        ),

        Node(
            package="kuka_r900_kinematics",
            executable="kdl_fk_node",
            name="kdl_fk_node",
            parameters=[{"robot_description": robot_description_content}],
            output="screen"
        ),
        # RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen"
        )
    ])
