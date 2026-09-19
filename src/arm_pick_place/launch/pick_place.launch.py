import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    desc_pkg = get_package_share_directory('arm_description')
    moveit_pkg = get_package_share_directory('arm_moveit_config')

    xacro_file = os.path.join(desc_pkg, 'urdf', 'arm.urdf.xacro')
    robot_description = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    }

    srdf_file = os.path.join(moveit_pkg, 'config', 'arm.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    kinematics_yaml = load_yaml('arm_moveit_config', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('arm_moveit_config', 'config/joint_limits.yaml')

    ompl_planning_yaml = {
        'planning_pipelines': {'pipeline_names': ['ompl']},
        'ompl': {
            'planning_plugins': ['ompl_interface/OMPLPlanner'],
            'request_adapters': [
                'default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision',
            ],
            'response_adapters': [
                'default_planning_response_adapters/AddTimeOptimalParameterization',
                'default_planning_response_adapters/ValidateSolution',
                'default_planning_response_adapters/DisplayMotionPath',
            ],
        }
    }

    moveit_controllers_yaml = load_yaml('arm_moveit_config', 'config/moveit_controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_controllers_yaml['moveit_simple_controller_manager'],
        'moveit_controller_manager': moveit_controllers_yaml['moveit_controller_manager'],
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    moveit_config_dict = {
        **robot_description,
        **robot_description_semantic,
        'robot_description_kinematics': kinematics_yaml,
        'robot_description_planning': joint_limits_yaml,
        **ompl_planning_yaml,
        **trajectory_execution,
        **moveit_controllers,
        **planning_scene_monitor_parameters,
    }

    pick_place_node = Node(
        package='arm_pick_place',
        executable='pick_place_demo',
        output='screen',
        parameters=[moveit_config_dict],
    )

    return LaunchDescription([pick_place_node])
