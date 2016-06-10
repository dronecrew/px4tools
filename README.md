
# px4tools

[![Binstar Badge](https://anaconda.org/dronecrew/px4tools/badges/build.svg)](https://anaconda.org/dronecrew/px4tools/builds)
[![Binstar Badge](https://anaconda.org/dronecrew/px4tools/badges/version.svg)](https://anaconda.org/dronecrew/px4tools)
[![Binstar Badge](https://anaconda.org/dronecrew/px4tools/badges/license.svg)](https://anaconda.org/dronecrew/px4tools)
[![Binstar Badge](https://anaconda.org/dronecrew/px4tools/badges/installer/conda.svg)](https://conda.anaconda.org/dronecrew)

Python tools for px4.

## Install

### Using Anaconda

1. [Install anaconda](http://docs.continuum.io/anaconda/install)

2. Install via conda (only linux 64-bit built)
```bash
conda config --add channels http://conda.anaconda.org/dronecrew
conda install px4tools
```

### Using PyPi
```bash
pip install px4tools --user
```


## Usage

First use the sdlog2_dumpy.py program to convert the px4log to csv:

```bash
wget https://github.com/PX4/Firmware/raw/master/Tools/sdlog2/sdlog2_dump.py
python sdlog2_dumpy.py your_log.px4log > your_log.csv
```

Install ipython notebook:

```
sudo apt-get install ipython-notebook
```

Start ipython notebook:

```
ipython notebook
```

## Example:

https://github.com/jgoppert/lpe-analysis/blob/master/15-09-30%20Kabir%20Log.ipynb
