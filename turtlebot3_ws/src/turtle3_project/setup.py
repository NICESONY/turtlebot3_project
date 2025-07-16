from setuptools import find_packages, setup
import os 
import sys
from glob import glob
package_name = 'turtle3_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 패키지 등록용
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # 패키지 XML 파일
        ('share/' + package_name, ['package.xml']),
    
        # launch 폴더 안의 모든 .launch.py 파일 
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
        # maps 폴더 안의 모든 .yaml 파일
        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*.*')
        ),
        # config  폴더 안의 모든 .yaml 파일
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
        # worlds folder .world file
                (
            os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')
        ),

    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samson',
    maintainer_email='songunhee5426@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = turtle3_project.test_node:main',
            'turtlebot3_sim_code = turtle3_project.turtlebot3_sim_code:main',
            'turtlebot3_robot_code = turtle3_project.turtlebot3_robot_code:main',
            'turtlebot3_robot_video = turtle3_project.turtlebot3_robot_video:main',
            'turtlebot3_sim_video = turtle3_project.turtlebot3_sim_video:main',
            'logic_flow = turtle3_project.logic_flow:main',
        ],
    },
)
