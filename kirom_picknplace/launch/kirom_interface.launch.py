from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("gen3", package_name="kinova_gen3_6dof_robotiq_2f_85_moveit_config").to_dict()

    # MTC Demo node
    kirom_interface_node = Node(
        package="kirom_picknplace",
        executable="kirom_interface",
        output="screen",
        arguments=['--ros-args', '--log-level', 'INFO'],
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([kirom_interface_node])
