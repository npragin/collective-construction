from setuptools import find_packages, setup

package_name = "cc_localization"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/gopro_calib.npz"]),
        ("share/" + package_name + "/scripts", ["scripts/activate_gopro.sh"]),
        ("share/" + package_name + "/launch", ["launch/cc_localization_viz.launch.py"]),
        ("share/" + package_name + "/rviz", ["rviz/aruco_tf.rviz"]),
    ],
    package_data={"": ["py.typed"]},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Noah Pragin",
    maintainer_email="npragin@gmail.com",
    description="ArUco-based localization of nested coordinate frames for the construction cell, intended to publish tf\
        transforms for an outer world frame and an inner workspace frame.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "aruco_detect = cc_localization.aruco_detect:main",
            "cc_localization = cc_localization.cc_localization_node:main",
        ],
    },
)
