from glob import glob

from setuptools import find_packages, setup

package_name = "cc_planner"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/scenarios", glob("test/scenarios/*.yaml")),
    ],
    package_data={"": ["py.typed"]},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Noah Pragin",
    maintainer_email="npragin@gmail.com",
    description="Collective-Construction Planner: high-level task planner and mission state machine that sequences and\
        dispatches construction tasks to the robots.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "cc_planner = cc_planner.planner_node:main",
            "fake_retriever = cc_planner.fake_retriever:main",
            "test_harness = cc_planner.test_harness:main",
        ],
    },
)
