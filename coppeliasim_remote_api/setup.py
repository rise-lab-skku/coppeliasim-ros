# -*- coding: utf-8 -*-
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

distuils_kwargs = generate_distutils_setup(
    packages=['coppeliasim_remote_api', 'coppeliasim_remote_api.bluezero', 'coppeliasim_remote_api.legacy'],
    package_dir={'': 'src'}
)

setup(**distuils_kwargs)
