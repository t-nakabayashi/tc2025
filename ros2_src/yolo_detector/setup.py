import os
from glob import glob
from typing import List, Tuple

from setuptools import setup

package_name = 'yolo_detector'


def list_data_files(target_dir: str) -> List[Tuple[str, List[str]]]:
    """指定ディレクトリ配下のファイルを構造を保ったままインストールする。"""

    data_entries: List[Tuple[str, List[str]]] = []
    for root, _, files in os.walk(target_dir):
        if not files:
            continue

        rel_path = os.path.relpath(root, '.')
        install_dir = os.path.join('share', package_name, rel_path)
        src_files = [os.path.join(root, file_name) for file_name in files]
        data_entries.append((install_dir, src_files))

    return data_entries


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

for subdir in ['models', 'params', 'launch']:
    if os.path.isdir(subdir):
        data_files.extend(list_data_files(subdir))

# scriptsディレクトリのファイルも含める
script_files = glob('scripts/*.py')
if script_files:
    data_files.append((os.path.join('share', package_name, 'scripts'), script_files))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='YOLO11n object detection for ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_node = yolo_detector.yolo_node:main',
            'road_blockage_detector = yolo_detector.road_blockage_detector_node:main',
            'yolo_ncnn_node = yolo_detector.yolo_ncnn_node:main',
            'camera_simulator_node = yolo_detector.camera_simulator_node:main',
        ],
    },
)
