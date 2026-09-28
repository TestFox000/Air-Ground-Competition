from setuptools import setup

package_name = "ag_referee"

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
    description="车载裁判系统串口节点",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "referee_node = ag_referee.referee_node:main",
        ],
    },
)
