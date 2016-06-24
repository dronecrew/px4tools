#!/bin/bash
set -e
python --version
bdir=~/anaconda3/conda-bld
rm -rf $bdir
rm -rf dist
conda config --set anaconda_upload no
conda build .
conda convert --platform all $bdir/linux-64/px4tools*.tar.bz2 -o ~/anaconda3/conda-bld
anaconda upload -u dronecrew $bdir/linux-64/px4tools*.tar.bz2
anaconda upload -u dronecrew $bdir/win-32/px4tools*.tar.bz2
anaconda upload -u dronecrew $bdir/win-64/px4tools*.tar.bz2
anaconda upload -u dronecrew $bdir/osx-64/px4tools*.tar.bz2
python setup.py sdist upload
