"""
Launch file for event bridge nodes.

Converts Prophesee event_camera_msgs to dvs_msgs for ESVO.

Convention:
  - master (cam_0) = left
  - slave (cam_1) = right

Usage:
  # Default (stereo):
  ros2 launch metavision_bridge metavision_bridge.launch.py

  # Single camera:
  ros2 launch metavision_bridge metavision_bridge.launch.py stereo:=false

  # Flip left/right (if master is physically on the right):
  ros2 launch metavision_bridge metavision_bridge.launch.py \
    left_input:=/event_cam_1/events right_input:=/event_cam_0/events
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Declare arguments
    stereo_arg = DeclareLaunchArgument(
        'stereo',
        default_value='true',
        description=
        'Launch both left and right bridges (set false for single camera)')

    left_input_arg = DeclareLaunchArgument(
        'left_input',
        default_value='/event_cam_0/events',
        description='Input topic for left (master) camera')

    right_input_arg = DeclareLaunchArgument(
        'right_input',
        default_value='/event_cam_1/events',
        description='Input topic for right (slave) camera')

    left_output_arg = DeclareLaunchArgument(
        'left_output',
        default_value='/evk/left/events',
        description='Output topic for left camera')

    right_output_arg = DeclareLaunchArgument(
        'right_output',
        default_value='/evk/right/events',
        description='Output topic for right camera')

    # Left camera bridge (master)
    left_bridge = ComposableNode(
        package='metavision_bridge',
        plugin='EventBridgeNode',
        parameters=[{
            'is_master': True
        }],
        name='metavision_bridge_left',
        remappings=[
            ('~/events_in', LaunchConfiguration('left_input')),
            ('~/events_out', LaunchConfiguration('left_output')),
        ],
        # output='screen',
        extra_arguments=[{
            "use_intra_process_comms": True
        }],
    )

    # Right camera bridge (slave) - only if stereo
    right_bridge = ComposableNode(
        package='metavision_bridge',
        plugin='EventBridgeNode',
        parameters=[{
            'is_master': False
        }],
        name='metavision_bridge_right',
        remappings=[
            ('~/events_in', LaunchConfiguration('right_input')),
            ('~/events_out', LaunchConfiguration('right_output')),
        ],
        # output='screen',
        extra_arguments=[{
            "use_intra_process_comms": True
        }],
    )

    return LaunchDescription([
        stereo_arg, left_input_arg, right_input_arg, left_output_arg,
        right_output_arg,
        LoadComposableNodes(target_container="metavision_driver_container",
                            composable_node_descriptions=[
                                left_bridge,
                            ]),
        LoadComposableNodes(
            target_container="metavision_driver_container",
            composable_node_descriptions=[
                right_bridge,
            ],
            condition=IfCondition(LaunchConfiguration('stereo')),
        )
    ])
