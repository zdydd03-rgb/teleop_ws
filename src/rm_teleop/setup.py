from glob import glob
from setuptools import find_packages, setup

package_name = "rm_teleop"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RM Teleop Developer",
    maintainer_email="user@example.com",
    description="Single-path PICO to RM65 CANFD high-follow teleoperation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pico_receiver_node = rm_teleop.pico_receiver_node:main",
            "target_generator_node = rm_teleop.target_generator_node:main",
            "rm65_canfd_high_follow_node = rm_teleop.rm65_canfd_high_follow_node:main",
        ],
    },
)
