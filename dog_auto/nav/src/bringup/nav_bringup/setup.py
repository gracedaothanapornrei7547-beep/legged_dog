import os
from glob import glob

from setuptools import find_packages, setup

package_name = "nav_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xiaocheng",
    maintainer_email="xiaocheng@todo.todo",
    description="MID360 导航总入口 (建图自动保存 / 重定位 / 规划 / 导航)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "run_mapping = nav_bringup.run_mapping:main",
            "pcd_to_grid = nav_bringup.pcd_to_grid:main",
            # 遗留(Nav2自主导航时期)，与决策机+FSM模型冲突，停用；
            # 需单独测Nav2过缝直行时再取消注释。文件保留见 waypoint_gap_navigator.py。
            # "waypoint_gap_navigator = nav_bringup.waypoint_gap_navigator:main",
        ],
    },
)
