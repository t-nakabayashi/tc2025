from glob import glob
import os
from setuptools import setup

package_name = 'route_planner'

def list_data_files(target_dir):
    data_entries = []
    for root, _, files in os.walk(target_dir):
        if not files:
            continue
        src_files = [os.path.join(root, f) for f in files]
        # ルートからの相対パスを維持したコピー先ディレクトリを作成
        rel_path = os.path.relpath(root, '.')
        install_dir = os.path.join('share', package_name, rel_path)
        data_entries.append((install_dir, src_files))
    return data_entries

map_files = [f for f in glob('map/*') if os.path.isfile(f)]

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]
for subdir in ['routes', 'launch']:
    if os.path.isdir(subdir):
        data_files.extend(list_data_files(subdir))

if map_files:
    data_files.append(('share/' + package_name + '/map', map_files))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kazuki',
    maintainer_email='kaz.ogata1988@gmail.com',
    description='経路生成と再計画サービスを提供する route_planner ノード',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'route_planner = route_planner.route_planner:main',
        ],
    },
)
