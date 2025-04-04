import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_path = os.path.join('/home/road2022/mingue/src', 'jumping_robot')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot_for_sim.urdf')
    # world_file = os.path.join(pkg_path, 'include', 'world.sdf')

    # (2) URDF 파일 읽어서 robot_description에 담기
    with open(urdf_file, 'r') as f:
        robot_description_config = f.read()

    robot_description = {'robot_description': robot_description_config}

# Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'verbose': 'false',
            'pause': 'false'
        }.items()
    )

#robot_state_publisher는 이제 익숙하네요 
    node_robot_state_publisher = Node( 
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

#gazebo 실행 및 모델 스폰 
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'robot',
                                   '-z', '1'],
                        output='screen')

#joint 상태를 확인하는 노드(아마도 service를 사용하는 것 같습니다)
    joint_state_broadcaster_spawner = Node( 
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

#controller 설정인데 여기선 forward_position_controller를 사용하여 position 제어를 합니다
#여기 부분을 수정한다면 yaml 파일도 수정해야하는 것 같아요 주요 수정부분이 되는 것 같습니다. 
    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["effort_controller", "-c", "/controller_manager"],
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[effort_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[position_controller_spawner],
            )
        ),
        gazebo,
        spawn_entity,
        node_robot_state_publisher,
    ])