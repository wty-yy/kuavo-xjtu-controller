from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['robot_state', 'utils', 'src', "config"],
    package_dir={'': ''}
)

setup(**d)