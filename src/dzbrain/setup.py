# -*- coding: utf-8 -*-
"""dzbrain 打包脚本（ament_python）。

注意：本包是**纯 Python 决策层**，改代码不需编译（车上 colcon build 只是为了装入口点与
install 树里的副本）。真正需要编译的是 4 个 C++ 硬件包。
"""

from setuptools import find_packages, setup

package_name = "dzbrain"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests", "tests.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="duzhong",
    maintainer_email="duzhong@local",
    description="dzrobot 新决策层：多线程任务架构 + 通用纯追踪循迹",
    license="MIT",
    entry_points={
        "console_scripts": [
            # 主进程：单进程多线程任务架构
            "dzbrain_node = dzbrain.main:main",
            # 离线工具（本地可跑，不需要 ROS）
            "dzbrain_sim = dzbrain.tools.sim_cli:main",
        ],
    },
)
