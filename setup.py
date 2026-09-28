# Only for catkin_python_setup(); do not run directly.
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(packages=["bms_ble"], package_dir={"": "src"}))
