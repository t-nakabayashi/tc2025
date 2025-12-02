import os

from setuptools import setup
from typing import List, Tuple

package_name = 'route_manager'

def list_data_files(target_dir: str) -> List[Tuple[str, List[str]]]:
    entries = []
    for root, _, files in os.walk(target_dir):
        if not files:
            continue
        install_dir = os.path.join('share', package_name, os.path.relpath(root, '.'))
        src_files = [os.path.join(root, file_name) for file_name in files]
        entries.append((install_dir, src_files))
    return entries

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    (
        'share/' + package_name + '/launch',
        ['launch/route_manager.launch.py', 'launch/active_route_marker.launch.py'],
    ),
    ('share/' + package_name, ['package.xml']),
]

if os.path.isdir('params'):
    data_files.extend(list_data_files('params'))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kazuki',
    maintainer_email='kaz.ogata1988@gmail.com',
    description='経路サービスと状態配信を行う route_manager ノード',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'route_manager = route_manager.route_manager_node:main',
            'active_route_marker = route_manager.route_markers_node:main',
        ],
    },
)
