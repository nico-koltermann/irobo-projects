# DO NOT USE
# python setup.py install
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['turtlebot3_viz'],
    package_dir={'': 'src'}
)

setup(**d)
