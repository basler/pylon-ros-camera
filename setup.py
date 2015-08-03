#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    # #  don't do this unless you want a globally visible script
    scripts=['scripts/maru_frame_rate_tester.py'], 
    #packages=[],
    #package_dir={}
  )
maps = d
setup(**maps)
