
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Localizar o diretorio de instalacao do package
    package_dir = get_package_share_directory('tsb_control')

    # Construir o caminho para o ficheiro YAML
    pid_config = os.path.join(package_dir, 'config', 'pid_params.yaml')

    return LaunchDescription([
        Node(
            package='tsb_control',
            executable='sensor_processing_node',
            name='sensor_processing_node',
            output='screen'
        ),

        Node(
            package='tsb_control',
            executable='pid_node',
            name='pid_node',
            output='screen',
            parameters=[pid_config]
        ),

        Node(
            package='tsb_control',
            executable='los_node',
            name='los_node',
            output='screen'
        )
    ])
