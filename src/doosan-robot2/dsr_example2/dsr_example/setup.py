import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dsr_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ★ weights 폴더의 pt + onnx 모두 포함
        (os.path.join('share', package_name, 'weights'),
        glob(os.path.join(package_name, 'weights', '*.pt')) +
        glob(os.path.join(package_name, 'weights', '*.onnx'))),

        ('share/dsr_example/launch', ['launch/smart_refueler_bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gossi',
    maintainer_email='mincheol710313@gmail.com',
    description='TODO: Package description',
    license='BSD',
    # tests_require=['pytest'],
    entry_points={
            'console_scripts': [
                'example_gripper = dsr_example.simple.example_gripper:main',
                'example_move = dsr_example.simple.example_move:main',
                'fuel_task_manager = dsr_example.fuel_task_manager:main',
                'fuel_listener_node = dsr_example.fuel_listener_node:main',
                'webcam_manager_ros = dsr_example.webcam_manager_ros:main',
                'realsense_manager_ros = dsr_example.realsense_manager_ros:main',
                'vision_target_node = dsr_example.vision_target_node:main',
                'plate_recognizer = dsr_example.plate_recognizer:main',
                'car_auto_announce_node = dsr_example.car_auto_announce_node:main',
                'face_payment_manager = dsr_example.face_payment_manager:main',
                'motion_controller = dsr_example.motion_controller:main',
                'test_task_manager = dsr_example.test_task_manager:main',
                'test_listener_node = dsr_example.test_listener_node:main',
                'realsense_streamer = dsr_example.realsense_streamer:main',
                'webcam_streamer = dsr_example.webcam_streamer:main',
                'ocr_manager_ros = dsr_example.ocr_manager_ros:main',
                'ocr_streamer = dsr_example.ocr_streamer:main'
            ],
        },
    )
