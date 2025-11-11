import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from mrs_lib.remappings_custom_config_parser import RemappingsCustomConfigParser
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
        LaunchConfiguration,
        IfElseSubstitution,
        PythonExpression,
        PathJoinSubstitution,
        EnvironmentVariable,
        )
import launch

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "livox_ros_driver2"

    this_pkg_path = get_package_share_directory(pkg_name)

    namespace='livox'

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    # #} end of custom_config

    # #{ container_name

    container_name = LaunchConfiguration('container_name')

    declare_container_name = DeclareLaunchArgument(
        'container_name',
        default_value='',
        description='Name of an existing container to load into (if standalone is false)'
    )

    ld.add_action(declare_container_name)

    # #} end of container_name

    # #{ standalone

    standalone = LaunchConfiguration('standalone')

    declare_standalone = DeclareLaunchArgument(
        'standalone',
        default_value='true',
        description='Whether to start a as a standalone or load into an existing container.'
    )

    ld.add_action(declare_standalone)

    # #} end of standalone

    # #{ custom_config

    config = LaunchConfiguration('config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'config',
        default_value=this_pkg_path+"/config/mid360.json",
        description="Path to the json configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        ))

    # behaviour:
    #     config == "" => config: ""
    #     config == "/<path>" => config: "/<path>"
    #     config == "<path>" => config: "$(pwd)/<path>"
    config = IfElseSubstitution(
            condition=PythonExpression(['"', config, '" != "" and ', 'not "', config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), config]),
            else_value=config
            )

    # #} end of custom_config

    # #{ node

    node = ComposableNode(
        package='livox_ros_driver2',
        plugin='livox_ros::DriverNode',
        name='livox',
        namespace=uav_name,
        parameters=[
            {"multi_topic": False},
            {"publish_freq": 10.0},
            {"frame_id": [uav_name,"/livox"]},
            {"user_config_path": config},
        ]
    )

    load_into_existing = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[node],
        condition=UnlessCondition(standalone)
    )

    ld.add_action(load_into_existing)

    # #} end of node

    # #{ standalone container

    standalone_container = ComposableNodeContainer(
        namespace=uav_name,
        name=namespace+'_livox_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output="screen",
        #prefix='xterm -e gdb -ex run --args',
        # prefix='gdb -ex run --args',
        # prefix='valgrind --tool=massif',
        # prefix=['debug_roslaunch ' + os.ttyname(sys.stdout.fileno())],
        composable_node_descriptions=[node],
        parameters=[
            {'use_intra_process_comms': True},
            {'thread_num': os.cpu_count()},
        ],
    )

    ld.add_action(standalone_container)

    # #} end of standalone container

    return ld
