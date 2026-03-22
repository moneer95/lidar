from setuptools import find_packages, setup

package_name = "tb3_tools"

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
    description="TurtleBot3 helper nodes (ROS2)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tb3_gap_nav = tb3_tools.gap_nav:main",
            "tb3_arrow_teleop = tb3_tools.arrow_teleop:main",
            "tb3_forward_test = tb3_tools.forward_test:main",
            "tb3_single_wheel_test = tb3_tools.single_wheel_test:main",
        ],
    },
)
