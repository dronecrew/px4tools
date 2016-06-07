#!/bin/bash
conda build .
conda convert --platform all ~/anaconda3/conda-bld/linux-64/px4tools*.tar.bz2 -o ~/anaconda3/conda-bld

