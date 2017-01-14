# px4tools
[![Build Status](https://travis-ci.org/dronecrew/px4tools.svg?branch=master)](https://travis-ci.org/dronecrew/px4tools)
[![Coverage Status](https://coveralls.io/repos/github/dronecrew/px4tools/badge.svg?branch=master)](https://coveralls.io/github/dronecrew/px4tools?branch=master)
[![Documentation Status](https://readthedocs.org/projects/px4tools/badge/?version=latest)](http://px4tools.readthedocs.io/en/latest/?badge=latest)

A log analysis toolbox for the [PX4 autopilot](http://px4.io/) written in python.

## Features

* **Flight Plotting** using standard python libraries.
* Automatic **System Identification** from log data.
* Automatic **Control Design** from log data.
* **Cross-Platform** deployment, testing, and support (Windows/OSX/Linux).
* Well integrated with **Jupyter** notebook and **Pandas**.
* Natively uses pandas **CSV format** so easily integrated with all log formats.

### Anaconda
Linux: [![Circle CI](https://circleci.com/gh/conda-forge/px4tools-feedstock.svg?style=shield)](https://circleci.com/gh/conda-forge/px4tools-feedstock)
OSX: [![TravisCI](https://travis-ci.org/conda-forge/px4tools-feedstock.svg?branch=master)](https://travis-ci.org/conda-forge/px4tools-feedstock)
Windows: [![AppVeyor](https://ci.appveyor.com/api/projects/status/github/conda-forge/px4tools-feedstock?svg=True)](https://ci.appveyor.com/project/conda-forge/px4tools-feedstock/branch/master)
Version: [![Anaconda-Server Badge](https://anaconda.org/conda-forge/px4tools/badges/version.svg)](https://anaconda.org/conda-forge/px4tools)
Downloads: [![Anaconda-Server Badge](https://anaconda.org/conda-forge/px4tools/badges/downloads.svg)](https://anaconda.org/conda-forge/px4tools)

## Install

### Dependencies

1. See requirements.txt

	For pandas, to fix time series plotting with time delta index you need this branch:

	```bash
	https://github.com/jgoppert/pandas/tree/tdi_plot_fix
	```

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

3. Upgrading using pip in conda

	The conda-forge px4tools package usually lags behind master. If you need the latest code, use pip within your conda env.

	```bash
	. conda_env
	pip install px4tools
	```

4. Building form source and installed to conda

	If you want to do development and edit some of the source, follow this example:

	```bash
	. conda_env
	git clone git@github.com:dronecrew/px4tools.git
	cd px4tools
	python setup.py build install
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

## Examples:

1. [Automatic System Identification and Control Design](https://github.com/dronecrew/px4tools/blob/master/examples/Log%20based%20System%20Identification%20and%20Control%20Design.ipynb)
1. [General Flight Data Plotting](https://github.com/jgoppert/lpe-analysis/blob/master/15-09-30%20Kabir%20Log.ipynb)
