# setup.py
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['litter_destruction'],  # Replace with the package name
    package_dir={'': 'src'}
)

setup(**setup_args)
