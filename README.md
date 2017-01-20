# px4tools
[![Build Status](https://travis-ci.org/dronecrew/px4tools.svg?branch=master)](https://travis-ci.org/dronecrew/px4tools)
[![Coverage Status](https://coveralls.io/repos/github/dronecrew/px4tools/badge.svg?branch=master)](https://coveralls.io/github/dronecrew/px4tools?branch=master)
[![Documentation Status](https://readthedocs.org/projects/px4tools/badge/?version=latest)](http://px4tools.readthedocs.io/en/latest/?badge=latest)

### Anaconda
Linux: [![Circle CI](https://circleci.com/gh/conda-forge/px4tools-feedstock.svg?style=shield)](https://circleci.com/gh/conda-forge/px4tools-feedstock)
OSX: [![TravisCI](https://travis-ci.org/conda-forge/px4tools-feedstock.svg?branch=master)](https://travis-ci.org/conda-forge/px4tools-feedstock)
Windows: [![AppVeyor](https://ci.appveyor.com/api/projects/status/github/conda-forge/px4tools-feedstock?svg=True)](https://ci.appveyor.com/project/conda-forge/px4tools-feedstock/branch/master)
Version: [![Anaconda-Server Badge](https://anaconda.org/conda-forge/px4tools/badges/version.svg)](https://anaconda.org/conda-forge/px4tools)
Downloads: [![Anaconda-Server Badge](https://anaconda.org/conda-forge/px4tools/badges/downloads.svg)](https://anaconda.org/conda-forge/px4tools)

A log analysis toolbox for the [PX4 autopilot](http://px4.io/) written in python.

## Features

* **Flight Plotting** using standard python libraries.
* Automatic **System Identification** from log data.
* Automatic **Control Design** from log data.
* **Cross-Platform** deployment, testing, and support (Windows/OSX/Linux).
* Well integrated with **Jupyter** notebook and **Pandas**.
* Natively uses pandas **CSV format** so easily integrated with all log formats.

## Usage

1. px4log format

	First use the sdlog2_dumpy.py program to convert the px4log to csv:

	```bash
	wget https://github.com/PX4/Firmware/raw/master/Tools/sdlog2/sdlog2_dump.py
	python sdlog2_dumpy.py your_log.px4log > your_log.csv
	```

	Now start jupyter notebook in the directoy of your_log.csv:

	```bash
	jupyter notebook
	```

2. ulog format

	No preprocessing required, see [ulog example](https://github.com/dronecrew/px4tools/blob/master/examples/ulog%20analysis.ipynb).
	

## Examples:

1. [Automatic System Identification and Control Design](https://github.com/dronecrew/px4tools/blob/master/examples/Log%20based%20System%20Identification%20and%20Control%20Design.ipynb)
2. [General Flight Data Plotting](https://github.com/jgoppert/lpe-analysis/blob/master/15-09-30%20Kabir%20Log.ipynb)
3. [ULOG basic use](https://github.com/dronecrew/px4tools/blob/master/examples/ulog%20analysis.ipynb).
4. [ULOG noise analysis](https://github.com/dronecrew/px4tools/blob/master/examples/ulog%20noise%20analysis.ipynb).


## Install

### Dependencies

1. See requirements.txt

	For pandas, to fix time series plotting memory 
	[issue](https://github.com/pandas-dev/pandas/pull/15067) with time delta index you need this branch:


	```bash
	. conda_env (See instructions below for setting up conda_env script)
	git clone git@github.com:jgoppert/pandas.git
	cd pandas
	git checkout tdi_plot_fix
	python setup.py install
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

2. Install px4tools via conda

	```bash
	conda config --add channels conda-forge
	conda install px4tools jupyter
	```

3. Upgrading px4tools using pip in conda

	The conda-forge px4tools package usually lags behind master. If you need the latest code, use pip within your conda env.

	```bash
	. conda_env
	pip install px4tools
	```

4. Building px4tools form source and installing to conda environment

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
