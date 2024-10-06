import os, sys, yaml
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

sys.path.append(os.path.join(os.getcwd(), 'ac_bringup', 'launch'))
def get_params(name):
    return os.path.join(os.getcwd(), 'src', 'ac_bringup', 'config', '{}_params.yaml'.format(name))

from launch import LaunchDescription
from launch.actions import TimerAction, Shutdown
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node, SetParameter, PushRosNamespace

def generate_launch_description():
    camera_node = Node(
        package='ac_camera',
        executable='usb_cam_node',
        output='screen',
        # parameters=[get_params('camera')],
    )

    classify_node = Node(
        package='ac_classify',
        executable='classify_node',
        output='screen',
        parameters=[get_params('classify')],
    )

    solvepnp_node = Node(
        package='ac_solver',
        executable='solver_node',
        output='screen',
        parameters=[get_params('solver')],
    )

    launch_list = [
        camera_node,
        classify_node,
        solvepnp_node,
    ]

    return LaunchDescription(launch_list)