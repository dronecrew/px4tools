import pandas 
import unittest

from .. import analysis


class Test(unittest.TestCase):

    def test_process_data(self):
        with open('./log/01_07_59.csv', 'r') as f:
            data = analysis.process_data(pandas.read_csv(f))
