import pandas 
import os
import unittest
import inspect

from px4tools import analysis

TEST_PATH = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))


class Test(unittest.TestCase):

    def test_process_data(self):
        filename = os.path.join(TEST_PATH, 'log', '01_07_59.csv')
        print("filename: {:s}".format(filename))
        with open(filename, 'r') as f:
            data = analysis.process_data(pandas.read_csv(f))
