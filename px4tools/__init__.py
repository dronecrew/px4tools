from __future__ import print_function
from .analysis import *

try:
    from .mapping import *
except ImportError as e:
    print(e)
