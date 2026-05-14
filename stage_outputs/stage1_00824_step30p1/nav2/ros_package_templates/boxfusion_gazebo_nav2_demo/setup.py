from setuptools import setup
from glob import glob

package_name = "boxfusion_gazebo_nav2_demo"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/params", glob("params/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="BoxFusion",
    maintainer_email="boxfusion@example.invalid",
    description="Step 17 downstream Gazebo/Nav2 skeleton for BoxFusion artifacts.",
    license="Research artifact",
)
