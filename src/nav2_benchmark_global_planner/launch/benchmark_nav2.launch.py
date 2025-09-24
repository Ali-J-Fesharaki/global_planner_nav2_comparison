from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_base_path=get_package_share_directory('nav2_benchmark_global_planner')
    nav2_config_path=PathJoinSubstitution([pkg_base_path, 'config', 'nav2_params.yaml'])
    map_path=PathJoinSubstitution([pkg_base_path, 'maps', 'maze_orthogonal.yaml'])
    map_server_cmd=Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="log",
            parameters=[{"yaml_filename": map_path}]
        )
    planner_server_cmd=Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_config_path],
            arguments=['--ros-args', '--log-level', 'planner_server:=debug']

        )
    lifecycle_manager_cmd=Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_planning",
            output="log",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["map_server", "planner_server"]
            }]
        )
    map_odom_tf_pub_cmd=Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "base_link"],
        name="map_odom_tf_pub"
    )
    odom_base_link_tf_pub_cmd=Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "odom", "base_link"],
        name="odom_base_link_tf_pub"
    )
    path_service_node_cmd=Node(
        package="nav2_benchmark_global_planner",
        executable="path_service_node.py",
        name="path_service_node",
        output="log"
    )
    benchmark_client_cmd=Node(
        package="nav2_benchmark_global_planner",
        executable="benchmark_client.py",
        name="benchmark_client",
        output="screen"
    )
    return LaunchDescription([
        map_server_cmd,
        planner_server_cmd,
        lifecycle_manager_cmd,
        map_odom_tf_pub_cmd,
        # odom_base_link_tf_pub_cmd,
        # path_service_node_cmd,
        # benchmark_client_cmd
    ])
