from setuptools import setup

package_name = 'parking_enforcement'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (
            f'share/{package_name}/launch',
            [
                'launch/parking_demo.launch.py',
                'launch/parking_first_pass.launch.py',
                'launch/parking_revisit.launch.py',
            ],
        ),
        (f'share/{package_name}', ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenAI',
    maintainer_email='support@example.com',
    description='ROS2 nodes for ROB498 parking-enforcement demo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parking_mission_node = parking_enforcement.parking_mission_node:main',
            'parking_revisit_node = parking_enforcement.parking_revisit_node:main',
            'stereo_depth_node = parking_enforcement.stereo_depth_node:main',
            'local_costmap_node = parking_enforcement.local_costmap_node:main',
            'plate_reader_node = parking_enforcement.plate_reader_node:main',
        ],
    },
)
