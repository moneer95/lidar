from setuptools import find_packages, setup

package_name = "lidar_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="user@example.com",
    description="Read YDLidar G4 scan data with timestamps and plot graphs",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scan_reader_node = lidar_tools.scan_reader_node:main",
            "scan_plot_node = lidar_tools.scan_plot_node:main",
            "export_scan_node = lidar_tools.export_scan_node:main",
        ],
    },
)
