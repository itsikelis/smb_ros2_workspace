from setuptools import find_packages, setup

package_name = "four_now"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="itsikelis",
    maintainer_email="tsikelis.i@protonmail.com",
    description="TODO: Package description",
    license="BSD-2-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["navigate_detect = four_now.navigate_detect:main"],
    },
)
