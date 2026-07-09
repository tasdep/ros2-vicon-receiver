from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    hostname = LaunchConfiguration("hostname")
    buffer_size = LaunchConfiguration("buffer_size")
    topic_namespace = LaunchConfiguration("topic_namespace")
    world_frame = LaunchConfiguration("world_frame")
    vicon_frame = LaunchConfiguration("vicon_frame")
    map_xyz = LaunchConfiguration("map_xyz")
    map_rpy = LaunchConfiguration("map_rpy")
    map_rpy_in_degrees = LaunchConfiguration("map_rpy_in_degrees")
    pose_topic = LaunchConfiguration("pose_topic")
    velocity_topic = LaunchConfiguration("velocity_topic")
    method = LaunchConfiguration("method")
    alpha = LaunchConfiguration("alpha")
    window_size = LaunchConfiguration("window_size")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "hostname",
                default_value="192.168.200.120",
                description="Vicon server hostname or IP address.",
            ),
            DeclareLaunchArgument(
                "buffer_size",
                default_value="200",
                description="Vicon receiver buffer size.",
            ),
            DeclareLaunchArgument(
                "topic_namespace",
                default_value="vicon",
                description="Topic namespace for Vicon messages.",
            ),
            DeclareLaunchArgument(
                "world_frame",
                default_value="map",
                description="World frame for Vicon transforms.",
            ),
            DeclareLaunchArgument(
                "vicon_frame",
                default_value="vicon",
                description="Vicon frame for Vicon transforms.",
            ),
            DeclareLaunchArgument(
                "map_xyz",
                default_value="[0.0, 0.0, 0.0]",
                description="XYZ translation for coordinate frame mapping.",
            ),
            DeclareLaunchArgument(
                "map_rpy",
                default_value="[0.0, 0.0, 0.0]",
                description="RPY rotation for coordinate frame mapping.",
            ),
            DeclareLaunchArgument(
                "map_rpy_in_degrees",
                default_value="false",
                description="Whether map_rpy values are in degrees.",
            ),
            DeclareLaunchArgument(
                "pose_topic",
                default_value="/vicon/tas/tas",
                description="Pose topic consumed by the velocity estimator.",
            ),
            DeclareLaunchArgument(
                "velocity_topic",
                default_value="/vicon/tas/velocity",
                description="TwistStamped topic published by the velocity estimator.",
            ),
            DeclareLaunchArgument(
                "method",
                default_value="lowpass_fd",
                description="Velocity estimation method.",
            ),
            DeclareLaunchArgument(
                "alpha",
                default_value="0.35",
                description="Low-pass smoothing coefficient for lowpass_fd.",
            ),
            DeclareLaunchArgument(
                "window_size",
                default_value="7",
                description="Estimator sample window size.",
            ),
            Node(
                package="vicon_receiver",
                executable="vicon_client",
                name="vicon_client",
                output="screen",
                parameters=[
                    {
                        "hostname": hostname,
                        "buffer_size": buffer_size,
                        "namespace": topic_namespace,
                        "world_frame": world_frame,
                        "vicon_frame": vicon_frame,
                        "map_xyz": map_xyz,
                        "map_rpy": map_rpy,
                        "map_rpy_in_degrees": map_rpy_in_degrees,
                    }
                ],
            ),
            Node(
                package="vicon_velocity_estimator",
                executable="pose-velocity-estimator",
                name="pose_velocity_estimator",
                output="screen",
                parameters=[
                    {
                        "pose_topic": pose_topic,
                        "velocity_topic": velocity_topic,
                        "method": method,
                        "alpha": alpha,
                        "window_size": window_size,
                    }
                ],
            ),
        ]
    )
