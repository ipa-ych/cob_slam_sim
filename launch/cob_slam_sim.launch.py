import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    ld = LaunchDescription()

    autorepeat_rate_arg = DeclareLaunchArgument(
        "autorepeat_rate", default_value=TextSubstitution(text="20.0")
    )
    ld.add_action(autorepeat_rate_arg)
    deadzone_arg = DeclareLaunchArgument(
        "deadzone", default_value=TextSubstitution(text="0.3")
    )
    ld.add_action(deadzone_arg)
    joy_dev_arg = DeclareLaunchArgument(
        "joy_dev", default_value=TextSubstitution(text="/dev/input/js0")
    )
    ld.add_action(joy_dev_arg)
    teleop_twist_joy_node_config = os.path.join(
        get_package_share_directory('cob_slam_sim'),
        'config',
        'teleop_twist_joy_node.yaml'
        )
    twist_mux_config = os.path.join(
        get_package_share_directory('cob_slam_sim'),
        'config',
        'twist_mux.yaml'
        )
    diagnostic_updater_period_arg = DeclareLaunchArgument(
        "diagnostic_updater_period", default_value=TextSubstitution(text="1.0")
    )
    ld.add_action(diagnostic_updater_period_arg)
    topic_joy_priotiry_arg = DeclareLaunchArgument(
        "topic_joy_priotiry", default_value=TextSubstitution(text="100")
    )
    ld.add_action(topic_joy_priotiry_arg)
    ros2_laserscan_merger_config = os.path.join(
        get_package_share_directory('cob_slam_sim'),
        'config',
        'ros2_laserscan_merger.yaml'
        )
    pointcloud_to_laserscan_config = os.path.join(
        get_package_share_directory('cob_slam_sim'),
        'config',
        'pointcloud_to_laserscan.yaml'
        )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        prefix = 'xterm -e',
        output='screen',
        name="joy_node",
parameters=[{
        "autorepeat_rate": LaunchConfiguration("autorepeat_rate"),
        "deadzone": LaunchConfiguration("deadzone"),
        "joy_dev": LaunchConfiguration("joy_dev"),}]
    )
    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        prefix = 'xterm -e',
        output='screen',
        name="teleop_twist_joy_node",
        parameters = [teleop_twist_joy_node_config]
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        prefix = 'xterm -e',
        output='screen',
        name="twist_mux",
        parameters = [twist_mux_config]
    )
    cmd_vel_bridge_node = Node(
        package="tricycle_sim",
        executable="cmd_vel_bridge_Twist_node",
        prefix = 'xterm -e',
        output='screen',
        name="cmd_vel_bridge_node",
parameters=[{
        "diagnostic_updater.period": LaunchConfiguration("diagnostic_updater_period"),
        "topics.joystick.priority": LaunchConfiguration("topic_joy_priotiry"),}]
    )
    ros2_laserscan_merger = Node(
        package="ros2_laser_scan_merger",
        executable="ros2_laser_scan_merger_3",
        prefix = 'xterm -e',
        output='screen',
        name="ros2_laserscan_merger",
        parameters = [ros2_laserscan_merger_config]
    )
    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        prefix = 'xterm -e',
        output='screen',
        name="pointcloud_to_laserscan",
        parameters = [pointcloud_to_laserscan_config]
    )
    include_cob_gazebo= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('cob_sim') + '/launch/cob_gazebo_slam_base.launch.py'])
    )

    ld.add_action(joy_node)
    ld.add_action(teleop_twist_joy_node)
    ld.add_action(twist_mux)
    ld.add_action(cmd_vel_bridge_node)
    ld.add_action(ros2_laserscan_merger)
    ld.add_action(pointcloud_to_laserscan)
    ld.add_action(include_cob_gazebo)

    return ld
