from setuptools import setup
from glob import glob

package_name = 'route_follower'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/route_follower.launch.py']),
        ('share/' + package_name + '/routes', glob('routes/**/*', recursive=True)),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kazuki',
    maintainer_email='kaz.ogata1988@gmail.com',
    description='経路追従ロジックを提供する route_follower ノード',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'route_follower = route_follower.route_follower_node:main',
        ],
    },
)
