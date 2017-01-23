#!/usr/bin/env python
"""Python log crunching for PX4.

This module allows you to do control and statistical analysis for the PX4.

"""

from __future__ import print_function
import os
import sys
import versioneer

from setuptools import setup, find_packages

DOCLINES = __doc__.split("\n")

CLASSIFIERS = """\
Development Status :: 1 - Planning
Intended Audience :: Science/Research
Intended Audience :: Developers
License :: OSI Approved :: BSD License
Programming Language :: Python
Programming Language :: Python :: 3
Programming Language :: Other
Topic :: Software Development
Topic :: Scientific/Engineering :: Artificial Intelligence
Topic :: Scientific/Engineering :: Mathematics
Topic :: Scientific/Engineering :: Physics
Operating System :: Microsoft :: Windows
Operating System :: POSIX
Operating System :: Unix
Operating System :: MacOS
"""

# pylint: disable=invalid-name


setup(
    name='px4tools',
    maintainer="James Goppert",
    maintainer_email="james.goppert@gmail.com",
    description=DOCLINES[0],
    long_description="\n".join(DOCLINES[2:]),
    url='https://github.com/dronecrew/px4tools',
    author='James Goppert',
    author_email='james.goppert@gmail.com',
    download_url='https://github.com/dronecrew/px4tools',
    license='BSD 3-Clause',
    classifiers=[_f for _f in CLASSIFIERS.split('\n') if _f],
    platforms=["Windows", "Linux", "Solaris", "Mac OS-X", "Unix"],
    install_requires=[
        'scipy', 'numpy',
        'pandas >= 0.19.2', 'transforms3d', 'pyulog'],
    tests_require=['nose'],
    test_suite='nose.collector',
    entry_points={
        'console_scripts': ['px42csv=px4tools.px42csv:main'],
    },
    packages=find_packages(),
    version=versioneer.get_version(),
    cmdclass=versioneer.get_cmdclass(),
)
