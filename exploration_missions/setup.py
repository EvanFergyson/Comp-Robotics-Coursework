import os
from glob import glob
from setuptools import find_packages, setup

package_name = "exploration_missions"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # added launch files for execution from ROS2
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="TODO@cardiff.ac.uk",
    description="Mission handler to explore and map an environment using random waypoints",
    license="Apache-2.0",
    entry_points={
        # added node as console script for execution
        "console_scripts": [
            "mission_handler = exploration_missions.mission_handler:main",
        ],
    },
)
