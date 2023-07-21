from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os

def generate_launch_description():
    MYCAR_MODEL = os.environ['MYCAR_MODEL']
    return LaunchDescription([
        launch_ros.actions.Node(
            package="ros2_stm32_bridge",
            executable="base_controller",
            parameters=[os.path.join(get_package_share_directory("ros2_stm32_bridge"), "params", MYCAR_MODEL + ".yaml")],
        )
    ])
