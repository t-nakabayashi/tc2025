from glob import glob
from pathlib import Path
from typing import List

from setuptools import find_packages, setup

package_name = 'robot_console'


def collect_data_files(directory: str) -> List[str]:
    """指定ディレクトリ配下のファイル一覧を返す。"""

    base_path = Path(directory)
    if not base_path.exists():
        return []
    return [str(path) for path in base_path.rglob('*') if path.is_file()]


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/config/node_params',
         glob('config/node_params/**/*.yaml', recursive=True)),
        (f'share/{package_name}/rviz', glob('rviz/*.rviz')),
        (f'share/{package_name}/docs', collect_data_files('docs')),
        (f'share/{package_name}/tools', collect_data_files('tools')),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools', 'Pillow>=9.0', 'opencv-python>=4.5'],
    zip_safe=True,
    maintainer='robot_console maintainers',
    maintainer_email='maintainer@example.com',
    description='Route monitoring console with tkinter GUI and ROS2 integrations.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_console = robot_console.robot_console_node:main',
        ],
    },
)
