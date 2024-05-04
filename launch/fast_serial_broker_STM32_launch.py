from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            #prefix=['gdbserver localhost:3000'],
            package='krabi_fast_serial_broker',
            namespace='krabi_ns',
            executable='krabi_fast_serial_broker_node',
            name='fast_serial_broker_STM32'
        )
    ])