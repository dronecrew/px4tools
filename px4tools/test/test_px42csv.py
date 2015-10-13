import unittest
from .. import px42csv
import sys


class Test(unittest.TestCase):

    def test_px42csv(self):
        sys.argv = ['./log/01_07_59.px4log']
        px42csv.main()

# vim: set et fenc=utf-8 ft=python ff=unix sts=4 sw=4 ts=4 : 
