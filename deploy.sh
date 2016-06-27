#!/bin/bash
root=$PWD
set -e
conda install conda-build anaconda-client anaconda-build
anaconda login
python --version
bdir=~/anaconda3/conda-bld
rm -rf $bdir
rm -rf dist
conda config --set anaconda_upload no
cd conda
conda build .
anaconda build submit .
conda convert --platform all $bdir/linux-64/px4tools*.tar.bz2 -o ~/anaconda3/conda-bld
anaconda upload -u dronecrew $bdir/linux-64/px4tools*.tar.bz2
anaconda upload -u dronecrew $bdir/win-32/px4tools*.tar.bz2
anaconda upload -u dronecrew $bdir/win-64/px4tools*.tar.bz2
anaconda upload -u dronecrew $bdir/osx-64/px4tools*.tar.bz2
cd $root
python setup.py sdist upload
