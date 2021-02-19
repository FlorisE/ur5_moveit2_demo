import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # planning_context
    robot_description_config = load_file('ur_e_description', 'urdf/ur5e_robot.urdf')
    robot_description = {'robot_description': robot_description_config}

    robot_description_semantic_config = load_file('ur5_e_moveit_config', 'config/ur5e.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('ur5_e_moveit_config', 'config/kinematics.yaml')

    # MoveGroupInterface demo executable
    run_move_group_demo = Node(name='ur5_run_move_group',
                               package='ur5_run_move_group',
                               executable='ur5_run_move_group',
                               prefix='xterm -e',
                               output='screen',
                               parameters=[robot_description,
                                           robot_description_semantic,
                                           kinematics_yaml])

    return LaunchDescription([run_move_group_demo])
