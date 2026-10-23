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
            "serial_reader_node = car_localization.serial_reader:main",
            # ★2026-10-03 补(P2)：此前 path_executor 只能"python3 源码直跑"，
            #   `ros2 run car_localization ...` 不可用，体检/调试时无法用统一入口。
            #   它本身就有 main(args=None)，直接注册即可。
            #   ★注意：start_all.sh 仍沿用直跑方式(已实车验证)，此处只作补充入口，
            #     不要贸然把启动脚本改成 ros2 run，避免影响已稳定的启动时序。
            "path_executor_node = car_localization.path_executor:main",
        ],
    },
)
