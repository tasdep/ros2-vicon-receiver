from setuptools import setup

package_name = "vicon_velocity_estimator"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tasman de Pury",
    maintainer_email="tdepury@gmail.com",
    description="Estimate linear and angular velocity from Vicon PoseStamped messages.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pose-velocity-estimator = vicon_velocity_estimator.pose_velocity_estimator:main",
        ],
    },
)
