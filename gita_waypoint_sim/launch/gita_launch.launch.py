from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    simulator = Node(
        package="gita_waypoint_sim",
        executable="simulator_node",
        name="simulator",
        output="screen"
    )

    controller = Node(
        package="gita_waypoint_sim",
        executable="controller_node",
        name="controller",
        output="screen"
    )

    maze_planner = Node(
        package="gita_waypoint_sim",
        executable="waypoint_listener",
        name="waypoint_listener",
        output="screen"
    )

    return LaunchDescription([
        simulator,
        controller,
        maze_planner
    ])