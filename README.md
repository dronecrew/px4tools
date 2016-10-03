# px4tools

Python tools for px4.

[![Coverage Status](https://coveralls.io/repos/github/dronecrew/px4tools/badge.svg?branch=master)](https://coveralls.io/github/dronecrew/px4tools?branch=master)
[![Build Status](https://travis-ci.org/dronecrew/px4tools.svg?branch=master)](https://travis-ci.org/dronecrew/px4tools)

Current build status
====================

Linux: [![Circle CI](https://circleci.com/gh/conda-forge/px4tools-feedstock.svg?style=shield)](https://circleci.com/gh/conda-forge/px4tools-feedstock)
OSX: [![TravisCI](https://travis-ci.org/conda-forge/px4tools-feedstock.svg?branch=master)](https://travis-ci.org/conda-forge/px4tools-feedstock)
Windows: [![AppVeyor](https://ci.appveyor.com/api/projects/status/github/conda-forge/px4tools-feedstock?svg=True)](https://ci.appveyor.com/project/conda-forge/px4tools-feedstock/branch/master)

Current release info
====================
Version: [![Anaconda-Server Badge](https://anaconda.org/conda-forge/px4tools/badges/version.svg)](https://anaconda.org/conda-forge/px4tools)
Downloads: [![Anaconda-Server Badge](https://anaconda.org/conda-forge/px4tools/badges/downloads.svg)](https://anaconda.org/conda-forge/px4tools)

## Install

### Dependencies

1. See requirements.txt

### Using Anaconda (Recommended)

1. [Install anaconda](http://docs.continuum.io/anaconda/install)

	* Python 3 version recommended
	* Do not select add to path if you use ROS. ROS expects the standard python to be installed. You can create a script to source and add anaconda to your path. This is similar to setup.bash for ROS users.

	~/bin/conda_env:

	```bash
	#!/bin/bash
	export PATH=$HOME/anaconda3/bin:$PATH
	```

	Now you can source the script to start using anaconda instead of the sytem python:

	```bash
	. conda_env
	```

2. Install via conda

```bash
conda config --add channels conda-forge
conda install px4tools jupyter
```

### Using PyPi
```bash
pip install px4tools jupyter --user
```

## Usage

First use the sdlog2_dumpy.py program to convert the px4log to csv:

```bash
wget https://github.com/PX4/Firmware/raw/master/Tools/sdlog2/sdlog2_dump.py
python sdlog2_dumpy.py your_log.px4log > your_log.csv
```

Now start jupyter notebook in the directoy of your_log.csv:

```bash
jupyter notebook
```

## Example:

https://github.com/jgoppert/lpe-analysis/blob/master/15-09-30%20Kabir%20Log.ipynb
