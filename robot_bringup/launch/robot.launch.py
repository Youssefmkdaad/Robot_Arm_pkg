from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    motor_pins = [
        #step, dir
        (27, 17),  # Motor 1
        (5, 6),    # Motor 2
        (22, 12),  # Motor 3
        (13, 23)   # Motor 4
    ]

    motor_nodes = [
        Node(
            package="motors",
            executable="motor_srv",
            name=f"motor_{i + 1}",
            parameters=[
                {"SrvName": f"move_motor_{i + 1}"},
                {"MsgName": f"motor_{i + 1}_position"},
                {"StepPin": step},
                {"DirPin": dir}
            ]
        )
        for i, (step, dir) in enumerate(motor_pins)
    ]

    motor_cli = Node(
        package="motors",
        executable="motor_cli",
        name="motor_client"
    )

    ld.add_action(motor_cli)
    for node in motor_nodes:
        ld.add_action(node)

    return ld
