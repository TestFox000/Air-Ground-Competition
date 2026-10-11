from setuptools import find_packages, setup
import os
from glob import glob

package_name = "car_localization"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="duzhong",
    maintainer_email="duzhong@todo.todo",
    description="Car localization package",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "serial_reader_node = car_localization.serial_reader:main"
        ],
    },
)
