import os
from glob import glob
from setuptools import setup

package_name = 'mission_director'

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
    description='任务赛决策机：arm_link + nav_client + strategy + mission',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'mission_director_node = '
            'mission_director.mission_director_node:main',
        ],
    },
)
