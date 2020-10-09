# To install the module for import into main state machine

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['autovalet_goal_generation'],
    package_dir={'': 'src'}
)
setup(**d)
