# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_pal.arg_utils import read_launch_argument
from launch_pal.robot_utils import (get_robot_model,
                                    get_robot_name)
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from ari_description.ari_launch_utils import get_ari_hw_suffix


def declare_args(context, *args, **kwargs):

    robot_name = read_launch_argument('robot_name', context)

    # ari description arguments
    return [get_robot_model(robot_name)]


def launch_setup(context, *args, **kwargs):

    robot_model = read_launch_argument('robot_model', context)
    mappings = {
        'robot_model': robot_model,
    }

    robot_description_path = os.path.join(
        get_package_share_directory('ari_description'), 'robots', 'ari.urdf.xacro')

    robot_description_semantic = (
        f'config/ari{get_ari_hw_suffix(robot_model=robot_model)}.srdf')

    joint_limits = (
        f'config/joint_limits{get_ari_hw_suffix(robot_model=robot_model)}.yaml')

    # Trajectory Execution Functionality
    moveit_simple_controllers_path = (
        f'config/controllers/controllers{get_ari_hw_suffix(robot_model=robot_model)}.yaml')

    moveit_config = (
        MoveItConfigsBuilder('ari')
        .robot_description(file_path=robot_description_path, mappings=mappings)
        .robot_description_semantic(file_path=robot_description_semantic)
        .robot_description_kinematics(file_path=os.path.join('config', 'kinematics.yaml'))
        .joint_limits(file_path=joint_limits)
        .trajectory_execution(moveit_simple_controllers_path)
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    use_sim_time = {
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }

    # RViz
    rviz_base = os.path.join(get_package_share_directory(
        'ari_moveit_config'), 'config', 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        parameters=[
            use_sim_time,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    return [rviz_node]


def generate_launch_description():

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='False', description='Use sim time'
    )

    ld = LaunchDescription()

    # Declare arguments
    # we use OpaqueFunction so the callbacks have access to the context
    ld.add_action(get_robot_name('ari'))
    ld.add_action(OpaqueFunction(function=declare_args))
    ld.add_action(sim_time_arg)

    # Execute move_group node
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
