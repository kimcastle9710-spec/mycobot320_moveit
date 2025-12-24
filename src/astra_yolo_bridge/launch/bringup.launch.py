import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 패키지 경로 찾기
    moveit_pkg_path = get_package_share_directory('adaptive_gripper_config')
    yolo_pkg_path = get_package_share_directory('astra_yolo_bridge')

    # 2. 파라미터 정의 (모델 경로 등)
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/han/robot/models/best2.pt',
        description='Absolute path to YOLO model file'
    )

    # 3. MoveIt Demo Launch 포함 (Rviz + MoveGroup)
    # setup_assistant로 만든 demo.launch.py를 여기서 실행시킵니다.
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg_path, 'launch', 'demo.launch.py')
        )
    )

    # 4. YOLO 노드 실행
    yolo_node = Node(
        package='astra_yolo_bridge',
        executable='astra_yolo_node',
        name='astra_yolo_node',
        output='screen',
        parameters=[{
            'yolo_model_path': LaunchConfiguration('model_path'),
            # 'serial_port': '/dev/ttyUSB0', # 필요시 수정
            # 'color_topic': '/camera/color/image_raw',
            # 'depth_topic': '/camera/depth/image_raw'
        }]
    )

    # 5. Cube Picker 노드 실행 (뇌)
    # picker_node = Node(
    #     package='astra_yolo_bridge',
    #     executable='cube_picker',
    #     name='cube_picker',
    #     output='screen'
    # )
    # 5. [수정] 컨트롤러 노드 (로봇을 움직이는 핵심!)
    controller_node = Node(
        package='astra_yolo_bridge',
        executable='cube_controller',
        name='cube_controller',
        output='screen'
    )
    # camera_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='camera_tf_publisher',
    #     arguments=['0.35', '0.0', '0.58', '0.0', '0.0', '0.0', 'base', 'camera_link']
    # )
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        # X=0.35, Y=0, Z=0.60
        # Yaw=0, Pitch=1.57(90도 숙임), Roll=0
        # Parent=base, Child=camera_link
       # [수정 후] 8cm 당겨줌
        arguments=['-0.27', '0.0', '0.60', '0.0', '1.57', '0.0', 'base', 'camera_link']
    )
    return LaunchDescription([
        model_path_arg,
        moveit_demo,
        yolo_node,
        controller_node,
        # picker_node,
        camera_tf
    ])
