import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # xml_file_name = "model/xml/spot_mini/spot_mini.xml"
    xml_file_name = "model/xml/anymal_c/scene.xml"
    xml_file = os.path.join(get_package_share_path("description"), xml_file_name)

    return LaunchDescription(
        [
            Node(
                package="mujoco",
                executable="simulation",
                name="simulation_mujoco",
                output="screen",
                parameters=[
                    {"simulation/model_file": xml_file},
                    {"imu_timer":           2.5},
                    {"joint_timer":         2.},
                    {"odom_timer":          20.},
                    {"sensor_odom_timer":   2.},
                    {"touch_timer":         2.},
                    {"img_timer":           20.},
                    {"contacts_timer":      2.},
                ],
                emulate_tty=True,
                arguments=[("__log_level:=debug")],
                
            ),
        ]
    )
