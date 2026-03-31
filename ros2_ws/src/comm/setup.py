from setuptools import setup

package_name = 'comm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='piroomira@live.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'comm_node = comm.comm_node:main',
            'occupancy_node = comm.occupancy_node:main',
            'local_planner_node = comm.local_planner_node:main',
            'plate_reader_node = comm.plate_reader_node:main'
        ],
    },
)
