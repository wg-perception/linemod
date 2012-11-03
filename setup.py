#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.package import parse_package_for_distutils

d = parse_package_for_distutils()
d['packages'] = ['object_recognition_linemod']
d['package_dir'] = {'': 'python'}
d['install_requires'] = []

setup(**d)
