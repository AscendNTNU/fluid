from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fluid',
            executable='main',
#            parameters=[{
#                "ekf": False,
#                "use_perception": False,
#                "refresh_rate": 20,
#                "should_auto_arm": False,
#                "should_auto_offboard": False,
#                "distance_completion_threshold": 0.30,
#                "velocity_completion_threshold": 0.10,
#                "default_height": 2.0,
#                "launch_rviz": False,
#                "interaction_show_prints": False,
#                "interaction_max_vel": 0.30,
#                "interaction_max_acc": 0.23,
#                "travel_max_angle": 70.0,
#                "travel_speed": 15.0,
#                "travel_accel": 10.0,
#
#                "fh_offset_x": 0.42,
#                "fh_offset_y": 0.02,
#                "fh_offset_z": -0.10,
#
#            }],
            output="screen"
        ),
    ])