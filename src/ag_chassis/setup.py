from setuptools import setup

package_name = "ag_chassis"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="team",
    maintainer_email="team@example.com",
    description="底盘执行节点",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "chassis_node = ag_chassis.chassis_node:main",
        ],
    },
)
