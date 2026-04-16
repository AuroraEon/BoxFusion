from setuptools import find_packages, setup


setup(
    name="boxfusion_ros_query_server",
    version="0.0.1",
    packages=find_packages(where="..", include=["boxfusion", "boxfusion.*"]),
    package_dir={"": ".."},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/boxfusion_ros_query_server"]),
        ("share/boxfusion_ros_query_server", ["package.xml"]),
        ("share/boxfusion_ros_query_server/launch", ["launch/publication_diagnostics.launch.py"]),
        ("share/boxfusion_ros_query_server/launch", ["launch/query_server.launch.py"]),
        ("share/boxfusion_ros_query_server/launch", ["launch/simulation_replay_source.launch.py"]),
        ("share/boxfusion_ros_query_server/launch", ["launch/runtime_export_coordinator.launch.py"]),
        ("share/boxfusion_ros_query_server/launch", ["launch/simulation_ingress.launch.py"]),
    ],
    install_requires=["setuptools"],
    include_package_data=True,
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "boxfusion_runtime_export_coordinator = boxfusion.runtime_export_coordinator:main",
            "boxfusion_simulation_ingress_node = boxfusion.ros_simulation_ingress:main",
            "boxfusion_simulation_replay_source_node = boxfusion.ros_simulation_replay_source:main",
            "boxfusion_publication_diagnostics_node = boxfusion.ros_publication_diagnostics_server:main",
            "boxfusion_query_server_node = boxfusion.ros_query_server:main",
        ],
    },
)
