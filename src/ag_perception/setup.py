from setuptools import setup

package_name = "ag_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="team",
    maintainer_email="team@example.com",
    description="视觉推理节点",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "vision_node = ag_perception.vision_node:main",
            "intel_fusion_node = ag_perception.intel_fusion_node:main",
        ],
    },
)
