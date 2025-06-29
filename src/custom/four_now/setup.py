from setuptools import find_packages, setup
from glob import glob
import os

package_name = "four_now"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
    ],
    package_data={
        'four_now': ['srv/*.srv'],
    },
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="itsikelis",
    maintainer_email="tsikelis.i@protonmail.com",
    description="TODO: Package description",
    license="BSD-2-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "navigate_detect = four_now.navigate_detect:main",
            "waypoint_publisher = four_now.waypoint_publisher:main",
            'robot_to_map_node = four_now.robot_to_map_node:main',
            'map_logger_node = four_now.map_save:main',
            'save_map_node = four_now.save_map_node:main',
            'move_a_bit_node = four_now.move_a_bit_node:main',
            'give_up_exploration_node = four_now.give_up_exploration_node:main',
        ],
    },
)
