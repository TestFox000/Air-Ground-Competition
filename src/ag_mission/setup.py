from setuptools import setup

package_name = "ag_mission"

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
    description="任务规划与敌我识别决策节点",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mission_node = ag_mission.mission_node:main",
            "target_node = ag_mission.target_node:main",
        ],
    },
)
