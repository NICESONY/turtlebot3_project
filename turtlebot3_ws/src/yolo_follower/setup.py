from setuptools import setup

package_name = 'yolo_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/'+package_name]),
        ('share/'+package_name, ['package.xml']),
        ('share/'+package_name+'/launch', ['launch/follow_yolo.launch.py']),
        ('share/'+package_name+'/config', ['config/params.yaml']),
        ('share/'+package_name+'/models', ['models/yolov8n.pt']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
        # ultralytics, torch 계열은 미리 pip로 설치할 것이므로 여기서 제거
    ],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='YOLO follower for TurtleBot3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = yolo_follower.detector_node:main',
            'follower_node = yolo_follower.follower_node:main',
        ],
    },
)
