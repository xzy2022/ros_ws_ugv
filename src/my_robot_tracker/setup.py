from setuptools import setup, find_packages

setup(
    name="my_robot_tracker",
    version="0.0.0",
    packages=find_packages(where="."),
    package_dir={"": "."},
)
