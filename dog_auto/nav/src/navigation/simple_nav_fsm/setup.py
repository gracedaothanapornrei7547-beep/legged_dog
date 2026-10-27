from setuptools import find_packages, setup

package_name = 'simple_nav_fsm'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml', 'config/params_dog.yaml']),
        ('share/' + package_name + '/launch', ['launch/simple_nav.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='极简导航状态机：直线 + 原地转向，替代 Nav2 规划/控制/costmap 层',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fsm_node = simple_nav_fsm.fsm_node:main',
            'record_waypoint = simple_nav_fsm.record_waypoint:main',
            'run_route = simple_nav_fsm.run_route:main',
        ],
    },
)
