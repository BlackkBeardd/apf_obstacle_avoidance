from setuptools import find_packages, setup

package_name = "diff_drive_description"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/display.launch.py", "launch/gazebo.launch.py"],
        ),
        (
            "share/" + package_name + "/urdf",
            [
                "urdf/diff_drive.xacro",
                "urdf/diff_drive_macro.xacro",
                "urdf/lidar.xacro",
                "urdf/gazebo_control.xacro",
                "urdf/ros2_control.xacro",
            ],
        ),
        ("share/" + package_name + "/rviz", ["rviz/diff_drive.rviz"]),
        ("share/" + package_name + "/worlds", ["worlds/my_world.sdf"]),
        ("share/" + package_name + "/config", ["config/gazebo_bridge.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="blackbeard",
    maintainer_email="abubakarmughal92@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
