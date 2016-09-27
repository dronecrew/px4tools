import os
import inspect
import unittest
from px4tools import px42csv
import sys

TEST_PATH = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))

class Test(unittest.TestCase):

    def test_px42csv(self):
        sdlog2_path = os.path.join(TEST_PATH, 'sdlog2_dump.py')
        file_path = os.path.join(TEST_PATH, 'log', '01_07_59.px4log')
        sys.argv = [
            os.path.join(TEST_PATH, 'log', '01_07_59.px4log'),
            '--sdlog2',
            '{:s}'.format(sdlog2_path)
        ]
        px42csv.main()

# vim: set et fenc=utf-8 ft=python ff=unix sts=4 sw=4 ts=4 : 
