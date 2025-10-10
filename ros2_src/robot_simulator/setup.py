from setuptools import setup

package_name = 'robot_simulator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_simulator']),
        ('share/' + package_name + '/launch', ['launch/robot_simulator.launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Codex Agent',
    maintainer_email='codex@example.com',
    description='robot_simulator ノードを提供するパッケージ',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_simulator = robot_simulator.robot_simulator:main',
        ],
    },
)
