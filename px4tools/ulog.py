"""
ulog2pandas converter
"""

#pylint: disable=no-member, invalid-name, broad-except, too-many-locals

import os
import tempfile
import re
import glob

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pyulog

try:
    import transforms3d.taitbryan as tf
except ImportError as ex:
    print(ex)
    print('please install transforms3d: pip install transforms3d')

class PX4DataFrame(object):

    """
    This is the main structure users use to interact with resampled log files
    """

    def __init__(self, dataframe, t1=None, t2=None):
        armed_index = dataframe.index[dataframe.t_vehicle_status_0__f_arming_state == 2]
        if t1 is None:
            t1 = armed_index[0]
        if t2 is None:
            t2 = armed_index[-1]
        self.df = dataframe[t1:t2]
        self.compute_data()

    def compute_data(self):
        """
        This computes useful data from analysis from the raw log data,
        converting quaternions to euler angles etc.
        """
        df = self.df
        series = [df]
        msg = 't_vehicle_attitude_0'
        roll, pitch, yaw = self.series_quat2euler(
            df.t_vehicle_attitude_0__f_q_0_,
            df.t_vehicle_attitude_0__f_q_1_,
            df.t_vehicle_attitude_0__f_q_2_,
            df.t_vehicle_attitude_0__f_q_3_, msg)
        series += [roll, pitch, yaw]

        try:
            msg_gt = 't_vehicle_attitude_groundtruth_0'
            roll_gt, pitch_gt, yaw_gt = self.series_quat2euler(
                df.t_vehicle_attitude_groundtruth_0__f_q_0_,
                df.t_vehicle_attitude_groundtruth_0__f_q_1_,
                df.t_vehicle_attitude_groundtruth_0__f_q_2_,
                df.t_vehicle_attitude_groundtruth_0__f_q_3_, msg_gt)

            e_roll = pd.Series(self.angle_wrap(roll - roll_gt), name=msg + '__f_roll_error')
            e_pitch = pd.Series(self.angle_wrap(pitch - pitch_gt), name=msg + '__f_pitch_error')
            e_yaw = pd.Series(self.angle_wrap(yaw - yaw_gt), name=msg + '__f_yaw_error')

            series += [roll_gt, pitch_gt, yaw_gt, e_roll, e_pitch, e_yaw]
        except Exception as ex:
            print(ex)

        self.df = pd.concat(series, axis=1)

    @staticmethod
    def angle_wrap(x):
        """wrap angle betwe -pi and pi"""
        return np.arcsin(np.sin(x))

    def plot_altitude(self, plot_groundtruth):
        """
        Plot altitude
        """
        plt.title('altitude')
        self.df.t_vehicle_global_position_0__f_alt.plot(label='alt', style='b-')
        if plot_groundtruth:
            self.df.t_vehicle_global_position_groundtruth_0__f_alt.plot(
                label='alt-true', style='b--')
        plt.grid()
        plt.legend(loc='best', ncol=3)
        plt.xlabel('t, sec')
        plt.ylabel('m')

    def plot_local_position(self, plot_groundtruth):
        plt.title('local position')
        plt.plot(self.df.t_vehicle_local_position_0__f_x,
                         self.df.t_vehicle_local_position_0__f_y, label='estimate')
        if plot_groundtruth:
            plt.plot(self.df.t_vehicle_local_position_groundtruth_0__f_x,
                             self.df.t_vehicle_local_position_groundtruth_0__f_y, 'r--',
                             label='true')
        plt.grid()
        plt.xlabel('N, m')
        plt.ylabel('E, m')
        plt.legend(loc='best')

    def plot_euler(self, plot_groundtruth):
        """
        Plot euler angles
        """
        plt.title('euler angles')
        np.rad2deg(self.df.t_vehicle_attitude_0__f_roll).plot(label='roll', style='r-')
        if plot_groundtruth:
            np.rad2deg(self.df.t_vehicle_attitude_groundtruth_0__f_roll).plot(
                label='roll true', style='r--')
        np.rad2deg(self.df.t_vehicle_attitude_0__f_pitch).plot(label='pitch', style='g-')
        if plot_groundtruth:
            np.rad2deg(self.df.t_vehicle_attitude_groundtruth_0__f_pitch).plot(
                label='pitch true', style='g--')
        np.rad2deg(self.df.t_vehicle_attitude_0__f_yaw).plot(label='yaw', style='b-')
        if plot_groundtruth:
            np.rad2deg(self.df.t_vehicle_attitude_groundtruth_0__f_yaw).plot(
                label='yaw true', style='b--')
        plt.grid()
        plt.legend(loc='best', ncol=3)
        plt.xlabel('t, sec')
        plt.ylabel('deg')

    def plot_euler_error(self):
        """
        Plot error between euler angles and ground truth
        """
        plt.title('euler angle errors')
        np.rad2deg(self.df.t_vehicle_attitude_0__f_roll_error).plot(
            label='roll error', style='r-')
        np.rad2deg(self.df.t_vehicle_attitude_0__f_pitch_error).plot(
            label='pitch error', style='g-')
        np.rad2deg(self.df.t_vehicle_attitude_0__f_yaw_error).plot(
            label='yaw error', style='b-')
        plt.grid()
        plt.legend(loc='best', ncol=3)
        plt.xlabel('t, sec')
        plt.ylabel('deg')

    def plot_velocity(self, plot_groundtruth):
        """
        Plot velocity
        """
        plt.title('velocity')
        self.df.t_vehicle_global_position_0__f_vel_n.plot(label='vel_n', style='r-')
        if plot_groundtruth:
            self.df.t_vehicle_global_position_groundtruth_0__f_vel_n.plot(
                label='vel_n-true', style='r--')
        self.df.t_vehicle_global_position_0__f_vel_e.plot(
            label='vel_e', style='g-')
        if plot_groundtruth:
            self.df.t_vehicle_global_position_groundtruth_0__f_vel_e.plot(
                label='vel_e-true', style='g--')
        self.df.t_vehicle_global_position_0__f_vel_d.plot(
            label='vel_d', style='b-')
        if plot_groundtruth:
            self.df.t_vehicle_global_position_groundtruth_0__f_vel_d.plot(
                label='vel_d-true', style='b--')
        plt.grid()
        plt.legend(loc='best', ncol=3)
        plt.xlabel('t, sec')
        plt.ylabel('m/s')

    @staticmethod
    def series_quat2euler(q0, q1, q2, q3, msg_name):
        """
        Given pandas series q0-q4, compute series roll, pitch, yaw
        """
        yaw, pitch, roll = np.array([
            tf.quat2euler([q0i, q1i, q2i, q3i]) for
            q0i, q1i, q2i, q3i in zip(q0, q1, q2, q3)]).T
        yaw = pd.Series(name=msg_name + '__f_yaw', data=yaw, index=q0.index)
        pitch = pd.Series(name=msg_name + '__f_pitch', data=pitch, index=q0.index)
        roll = pd.Series(name=msg_name + '__f_roll', data=roll, index=q0.index)
        return roll, pitch, yaw

    def estimator_analysis(self):
        """
        Evaluates estimator performance
        """
        #pylint: disable=unused-variable
        roll_error_mean = np.rad2deg(
            self.df.t_vehicle_attitude_0__f_roll_error.mean())
        pitch_error_mean = np.rad2deg(
            self.df.t_vehicle_attitude_0__f_pitch_error.mean())
        yaw_error_mean = np.rad2deg(
            self.df.t_vehicle_attitude_0__f_yaw_error.mean())
        roll_error_std = np.rad2deg(np.sqrt(
            self.df.t_vehicle_attitude_0__f_roll_error.var()))
        pitch_error_std = np.rad2deg(np.sqrt(
            self.df.t_vehicle_attitude_0__f_pitch_error.var()))
        yaw_error_std = np.rad2deg(np.sqrt(
            self.df.t_vehicle_attitude_0__f_yaw_error.var()))
        print('''
ESTIMATOR ANALYSIS
-----------------------------------
mean euler error:
\t{roll_error_mean:10f}\t deg
\t{pitch_error_mean:10f}\t deg
\t{yaw_error_mean:10f}\t deg

standard deviation euler error:
\t{roll_error_std:10f}\t deg
\t{pitch_error_std:10f}\t deg
\t{yaw_error_std:10f}\t deg
'''.format(**locals()))

        plt.figure()
        self.plot_altitude(plot_groundtruth=True)

        plt.figure()
        self.plot_euler(plot_groundtruth=True)

        plt.figure()
        self.plot_euler_error()

        plt.figure()
        self.plot_local_position(plot_groundtruth=True)

        plt.figure()
        self.plot_velocity(plot_groundtruth=True)

    def __repr__(self):
        """
        To make display work
        """
        return self.df.__repr__()


class PX4MessageDict(dict):

    """
    PX4 has several data frames in a log and they don't all have the same
    index, so this structure is used to manipulate the resulting dictionary
    of dataframes that a log produces.
    """

    def __init__(self, d):
        super(PX4MessageDict, self).__init__(d)

    def resample(self, period):
        """
        @param period, '1L' (1 millisecond), '100L', (100 miiliseconds), @see pandas
        """
        msg_dict_rs = {}
        for key in self.keys():
            try:
                msg_dict_rs[key] = self[key].resample(period).ffill().bfill()
            except ValueError as ex:
                print(key, ex)
        return PX4MessageDict(msg_dict_rs)

    def concat(self):
        """
        If all of the dataframes have the same inde (through resample method),
        this concatentates all of the dataframes into one pandas DataFrame object.
        """
        data = PX4DataFrame(pd.concat([
            pd.DataFrame(data=self[msg].values,
                         index=self[msg].index,
                         columns=[msg + '__' + key for key in self[msg].columns])
            for msg in self.keys()], axis=1))
        data.df.index = [
            (data.df.index[i] - data.df.index[0]).total_seconds()
            for i in range(len(data.df.index))]
        return data

def read_ulog(ulog_filename, verbose=False):
    """
    Convert ulog to pandas dataframe.
    """

    pyulog.ulog2csv.convert_ulog2csv(
        ulog_filename, '', tempfile.tempdir, ',')
    log_name = os.path.splitext(os.path.basename(ulog_filename))[0]
    data = {}
    glob_expr = '{:s}*.csv'.format(
        os.path.join(tempfile.gettempdir(), log_name))

    # column naming
    d_col_rename = {
        '[': '_',
        ']': '_',
    }
    col_rename_pattern = re.compile(
        r'(' + '|'.join([re.escape(key) for key in d_col_rename.keys()]) + r')')

    for file in sorted(glob.glob(glob_expr)):
        if verbose:
            print('processing', file)
        file_name = os.path.splitext(os.path.basename(file))[0]
        topic_name = file_name.replace(log_name + '_', '')

        # read data
        data_new = pd.read_csv(file, index_col=0)
        data_new.index = pd.to_datetime(data_new.index, unit='us')
        data_new.columns = [
            'f_' + col_rename_pattern.sub(
                lambda x: d_col_rename[x.group()], col)
            for col in data_new.columns
        ]

        data['t_' + topic_name] = data_new

    if verbose:
        print(log_name, 'data loaded')

    return PX4MessageDict(data)


#  vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 :
