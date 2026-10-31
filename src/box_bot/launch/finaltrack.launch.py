import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, AppendEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_share = get_package_share_directory('box_bot')
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('box_bot'),
            'urdf',
            'robot_og.urdf.xacro'
        ])
    ])

    set_env = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=pkg_share
    )

    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', 
        PathJoinSubstitution([
               FindPackageShare('box_bot'),
                'worlds',
                'final_grading.sdf'])],
        output='screen'
    )

    gazebo_gui = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['gz', 'sim', '-g'],
                output='screen'
            )
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }
        ]
    )

    spawn_robot = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'robot'
                ],
                output='screen'
            )
        ]
    )

    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=[
            '/model/robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        remappings=[
            ('/model/robot/cmd_vel', '/cmd_vel')
        ],
        output='screen'
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        output='screen'
    )

    camera_viewer = Node(
    package='box_bot',
    executable='camera_viewer',
    name='camera_viewer',
    output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),
        set_env,
        gazebo_server,
        gazebo_gui,
        robot_state_publisher,
        spawn_robot,
        cmd_vel_bridge,
        camera_bridge,
        camera_viewer
    ])
