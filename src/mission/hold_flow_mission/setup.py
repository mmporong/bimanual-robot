from setuptools import find_packages, setup


package_name = "hold_flow_mission"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mmporong",
    maintainer_email="mmporong@users.noreply.github.com",
    description="Manipulation Action adapter and simulation-only execution servers",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "manipulation_action_client = hold_flow_mission.action_client:main",
            "mock_manipulation_server = hold_flow_mission.mock_action_server:main",
            "planned_artifact_server = hold_flow_mission.planned_artifact_server:main",
            "planned_ipc_server = hold_flow_mission.planned_ipc_server:main",
        ],
    },
)
