import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    xml_file_name = "model/xml/anymal_c/scene_modified.xml"
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
                    {"imu_timer":           5.},
                    {"joint_timer":         5.},
                    {"odom_timer":          5.},
                    {"sensor_odom_timer":   5.},
                    {"touch_timer":         5.},
                    {"img_timer":           5.},
                    {"contacts_timer":      5.},
                    {"base_wrench_timer":   5.},
                ],
                emulate_tty=True,
                arguments=[("__log_level:=debug")],
                
            ),
        ]
    )
