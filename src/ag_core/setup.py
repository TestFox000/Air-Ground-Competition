from setuptools import setup

package_name = "ag_core"

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
    description="空地协同侦排挑战赛纯算法核心",
    license="Apache-2.0",
    tests_require=["pytest"],
)
