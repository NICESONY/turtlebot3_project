from setuptools import find_packages, setup

package_name = 'aruco_detector_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinu',
    maintainer_email='wlsdnsla2568@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slow_aruco_detect_turtle = aruco_detector_pkg.slow_aruco_detect_turtle:main',
            'fast_aruco_detect_turtle = aruco_detector_pkg.fast_aruco_detect_turtle:main',
            'motion_aruco_detect_turtle = aruco_detector_pkg.motion_aruco_detect_turtle:main',
            'follow_aruco_detect_turtle = aruco_detector_pkg.follow_aruco_detect_turtle:main',
        ],
    },
)
