#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
        ##  don't do this unless you want a globally visible script
        scripts=['scripts/iw_console',
                 'scripts/get_intention',
                 'scripts/load_desires',
                 'scripts/events_to_string.py'], 
        packages=['iw_tools'],
        package_dir={'': 'src'}
        )

setup(**d)
