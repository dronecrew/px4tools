from __future__ import print_function

try:
    from .mapping import *
except ImportError as ex:
    print(ex)
    print('''
WARNING: mapping module requires basemap
    sudo apt-get install python-basemap
    or:
    conda install basemap
''')

try:
    from .analysis import *
except ImportError as ex:
    print(ex)

try:
    from .ulog import *
except ImportError as ex:
    print(ex)
    print('''
WARNING: ulog module requires pyulog

please install:
    pip install pyulog
''')

try:
    from .logsysid import *
except ImportError as ex:
    print(ex)
    print('''
WARNING: logsysid module requires python-control

please install:
    pip install control
''')
