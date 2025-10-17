from setuptools import setup

package_name = 'obstacle_monitor'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/obstacle_monitor.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='auto',
    maintainer_email='auto@example.com',
    description='Obstacle monitor node for 2D LiDAR (Phase2).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_monitor = obstacle_monitor.obstacle_monitor_node:main',
        ],
    },
)
