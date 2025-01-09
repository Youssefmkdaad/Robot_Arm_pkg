import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    bringup_dir = get_package_share_directory('robot_crafters')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Launch configuration variables
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    # use_joint_state_pub = LaunchConfiguration('use_joint_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    urdf_file = LaunchConfiguration('urdf_file')
    gazebo_world = LaunchConfiguration('gazebo_world')

    # Declare launch arguments
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'view.rviz'),
        description='Full path to the RVIZ config file to use')
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')
    # declare_use_joint_state_pub_cmd = DeclareLaunchArgument(
    #     'use_joint_state_pub',
    #     default_value='True',
    #     description='Whether to start the joint state publisher')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    declare_urdf_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(bringup_dir, 'urdf', 'crafters_urdf.urdf.xacro'),
        description='Path to the robot URDF file')
    declare_gazebo_world_cmd = DeclareLaunchArgument(
        'gazebo_world',
        default_value=os.path.join(bringup_dir, 'worlds', 'test_world.world'),
        description='Path to the Gazebo world file')

    # Nodes and processes
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_file])

    # start_joint_state_publisher_cmd = Node(
    #     condition=IfCondition(use_joint_state_pub),
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     output='screen',
    #     arguments=[urdf_file]
    #     )

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    gazebo_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': gazebo_world}.items()
    )

    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot_crafters'
        ],
        output='screen'
    )

    # # Add a small delay to ensure robot_description is loaded
    # delay_cmd = TimerAction(
    #     period=2.0,  # Delay of 2 seconds
    #     actions=[spawn_entity_cmd]
    # )

    # Create the launch description and populate
    ld = LaunchDescription()
    # motor_pins = [
    #     #step, dir
    #     (27, 17),  # Motor 1
    #     (5, 6),    # Motor 2
    #     (22, 12),  # Motor 3
    #     (13, 23)   # Motor 4
    # ]

    # motor_nodes = [
    #     Node(
    #         package="motors",
    #         executable="motor_srv",
    #         name=f"motor_{i + 1}",
    #         parameters=[
    #             {"SrvName": f"move_motor_{i + 1}"},
    #             {"MsgName": f"motor_{i + 1}_position"},
    #             {"StepPin": step},
    #             {"DirPin": dir}
    #         ]
    #     )
    #     for i, (step, dir) in enumerate(motor_pins)
    # ]

    # motor_cli = Node(
    #     package="motors",
    #     executable="motor_cli",
    #     name="motor_client"
    # )

    # ld.add_action(motor_cli)
    # for node in motor_nodes:
    #     ld.add_action(node)


    # Declare the launch options
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_urdf_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    # ld.add_action(declare_use_joint_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_gazebo_world_cmd)

    # Add the nodes to the launch description
    # ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)

    # Add Gazebo and spawn entity actions
    ld.add_action(gazebo_launch_cmd)
    ld.add_action(spawn_entity_cmd)
    # ld.add_action(delay_cmd)

    return ld
