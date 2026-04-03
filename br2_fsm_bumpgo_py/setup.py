from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['br2_fsm_bumpgo_py'],
    package_dir={'': '.'}
)

setup(**d)
