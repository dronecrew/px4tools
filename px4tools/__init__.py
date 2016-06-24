from __future__ import print_function

try:
    from .mapping import *
    from .analysis import *
except ImportError as e:
    print(e)
