from setuptools import setup

package_name = 'astra_yolo_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=[
        'setuptools',
        # YOLOv8 쓰면 필요 (pip로 설치)
        # 'ultralytics',
        'opencv-python',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='han',
    maintainer_email='kimcastle9710@gmail.com',
    description='Bridge node: Astra(ROS2) + YOLO + ESP32 serial',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astra_yolo_node = astra_yolo_bridge.astra_yolo_node:main',
            'cube_picker = astra_yolo_bridge.cube_picker:main',
            'cube_controller = astra_yolo_bridge.cube_controller:main',
        ],
    },
)

