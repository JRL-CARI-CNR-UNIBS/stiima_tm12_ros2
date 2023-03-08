# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Denis Stogl

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")
    headless_mode = LaunchConfiguration("headless_mode")



    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("omron_app"), "urdf", description_file])
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )


    # define update rate
    # update_rate_config_file = PathJoinSubstitution(
    #     [
    #         FindPackageShare(runtime_config_package),
    #         "config",
    #         ur_type.perform(context) + "_update_rate.yaml",
    #     ]
    # )


    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace='omron',
        # parameters=[robot_description, update_rate_config_file, initial_joint_controllers],
        parameters=[robot_description, initial_joint_controllers],
        output="screen",
    )


    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="screen",
    #     parameters=[robot_description],
    # )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace='omron',
        arguments=["joint_state_broadcaster", "--controller-manager", "/omron/controller_manager"],
        output="screen",
    )

    # speed_scaling_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "speed_scaling_state_broadcaster",
    #         "--controller-manager",
    #         "/controller_manager",
    #     ],
    #     output="screen",
    # )

    # force_torque_sensor_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "force_torque_sensor_broadcaster",
    #         "--controller-manager",
    #         "/controller_manager",
    #     ],
    #     output="screen",
    # )

    # forward_position_controller_spawner_stopped = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     # arguments=["forward_position_controller", "-c", "/controller_manager", "--stopped"],
    #     arguments=["forward_position_controller", "-c", "/controller_manager --stopped"],
    #     output="screen",
    # )

    cart_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["my_cartesian_motion_controller", "-c", "omron/controller_manager"],
        output="screen",
    )

    # There may be other controllers of the joints, but this is the initially-started one
    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace='omron',
        arguments=["joint_trajectory_controller", "-c", "/omron/controller_manager"],
    )

    imm_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace='omron',
        arguments=["imm", "-c", "/omron/controller_manager"],
    )

    nodes_to_start = [
        control_node,
        # robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        # joint_trajectory_controller,
        imm_controller,
        # cart_position_controller_spawner,
        # forward_position_controller_spawner_stopped,
        # initial_joint_controller_spawner_stopped,
        # initial_joint_controller_spawner_started,
    ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="tm12_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="tm_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="system.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
