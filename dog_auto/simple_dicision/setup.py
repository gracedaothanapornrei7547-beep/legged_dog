import os
from glob import glob
from setuptools import setup

package_name = 'simple_dicision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rum',
    maintainer_email='rum@todo.todo',
    description='简化决策机：手动输入颜色 → gap12抓取 → 顺序放置',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'simple_mission_node = '
            'simple_dicision.simple_mission_node:main',
        ],
    },
)
