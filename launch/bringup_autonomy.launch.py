from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():

    os.environ["TURTLEBOT3_MODEL"] = "waffle"
    os.environ["GAZEBO_MODEL_PATH"] = (
        "/opt/ros/humble/share/turtlebot3_gazebo/models:"
        + os.environ.get("GAZEBO_MODEL_PATH", "")
    )

    pkg_path = os.path.join(os.getenv("HOME"), "turtlebot3_ws", "src", "projet_robotique_i2")
    world_file  = os.path.join(pkg_path, "worlds", "207.world")
    maps_dir    = os.path.join(pkg_path, "maps", "map")
    nav2_params = os.path.join(pkg_path, "config", "nav2_params.yaml")
    slam_params = os.path.join(pkg_path, "config", "slam_toolbox_params.yaml")

    declare_world = DeclareLaunchArgument("world", default_value=world_file)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource("/opt/ros/humble/share/gazebo_ros/launch/gazebo.launch.py"),
        launch_arguments={"world": LaunchConfiguration("world")}.items()
    )

    urdf_xacro = "/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_waffle.urdf.xacro"
    urdf_plain = "/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_waffle.urdf"
    robot_description = ParameterValue(
        Command(["xacro ", urdf_xacro] if os.path.exists(urdf_xacro)
                else ["sed 's/\\${namespace}//g' ", urdf_plain]),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "robot_description": robot_description}]
    )

    spawn_robot = TimerAction(period=3.0, actions=[
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-file", "/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf",
                "-entity", "waffle_ros",
                "-x", "-9.07", "-y", "4.31", "-z", "0.01",
            ],
            output="screen"
        )
    ])

    slam_args = {"use_sim_time": "true"}
    if os.path.exists(slam_params):
        slam_args["slam_params_file"] = slam_params

    slam = TimerAction(period=6.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                "/opt/ros/humble/share/slam_toolbox/launch/online_async_launch.py"),
            launch_arguments=slam_args.items()
        )
    ])

    nav2_args = {"use_sim_time": "true"}
    if os.path.exists(nav2_params):
        nav2_args["params_file"] = nav2_params

    nav2 = TimerAction(period=10.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                "/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py"),
            launch_arguments=nav2_args.items()
        )
    ])

    rviz = TimerAction(period=10.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                "/opt/ros/humble/share/nav2_bringup/launch/rviz_launch.py")
        )
    ])


    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        declare_world,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        slam,
        nav2,
        rviz,
    ])