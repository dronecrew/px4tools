import os
import pandas 
import unittest
import inspect
from px4tools.analysis import *
from px4tools.mapping import *
have_control = False
try:
    from px4tools.logsysid import *
    have_control = True
except ImportError as e:
    print(e)

TEST_PATH = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))


class Test(unittest.TestCase):

    def test_process_data(self):
        filename = os.path.join(TEST_PATH, 'log', '01_07_59.csv')
        print("filename: {:s}".format(filename))
        with open(filename, 'r') as f:
            data = process_data(pandas.read_csv(f))
            data = process_lpe_health(data)
            data = project_lat_lon(data)
            data = process_lpe_health(data)
            find_meas_period(data['LPOS_VX'])
            #all_new_sample(data['LPOS_VX'])
            new_sample(data['LPOS_VX'])
            find_lpe_gains(data, printing=True)
            set_time_series(data)
            get_auto_data(data)
            get_float_data(data)
            isfloatarray(data['LPOS_VX'])
            octa_cox_data_to_ss(data)
            # fails on miniconda, windows
            # filter_finite(data)

    def test_logsysid(self):
        if not have_control:
            return
        filename = os.path.join(TEST_PATH, 'log', '01_07_59.csv')
        print("filename: {:s}".format(filename))
        with open(filename, 'r') as f:
            log_data = pandas.read_csv(f)
            gains = control_design(log_data)

    @unittest.skip("skip plotting test for CI")
    def test_plotting(self):
        filename = os.path.join(TEST_PATH, 'log', '01_07_59.csv')
        print("filename: {:s}".format(filename))
        with open(filename, 'r') as f:
            data = process_data(pandas.read_csv(f))
            data = process_lpe_health(data)
            data = project_lat_lon(data)
            alt_analysis(data)
            statistics(data, ['LPOS_VX'], plot=True)
            data = process_lpe_health(data)
            plot_modes(data)
            find_meas_period(data['LPOS_VX'])
            plot_control_loops(data)
            plot_position_loops(data)
            plot_velocity_loops(data)
            plot_attitude_rate_loops(data)
            plot_attitude_loops(data)
            plot_faults(data)
            pos_analysis(data)


