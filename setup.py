from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
	version='0.1.0',
	scripts=['scripts'],
    packages=['erica'],
    package_dir={'': 'src'}
)

setup(**d)