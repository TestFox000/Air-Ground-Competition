import os
from glob import glob

from setuptools import setup

package_name = "ag_integrate"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="team",
    maintainer_email="team@example.com",
    description="官方框架车端薄集成：dzuav 二进制情报 - 世界模型 - path_executor",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "integrate_node = ag_integrate.integrate_node:main",
        ],
    },
)
