"""
ulog2pandas converter
"""

#pylint: disable=no-member, invalid-name, broad-except, too-many-locals

import os
import tempfile
import re
import glob
import collections

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

    def __init__(self, dataframe):
        self.df = dataframe
        self.compute_data()
        
    def compute_data(self):
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

            e_roll = pd.Series(roll - roll_gt, name=msg + '__f_roll_error')
            e_pitch = pd.Series(pitch - pitch_gt, name=msg + '__f_pitch_error')
            e_yaw = pd.Series(yaw - yaw_gt, name=msg + '__f_yaw_error')

            series += [roll_gt, pitch_gt, yaw_gt, e_roll, e_pitch, e_yaw]
        except Exception as ex:
            print(ex)

        self.df = pd.concat(series, axis=1)

    def plot_euler(self):
        self.df.t_vehicle_attitude_0__f_roll.plot(label='roll', style='r-')
        self.df.t_vehicle_attitude_0__f_pitch.plot(label='pitch', style='g-')
        self.df.t_vehicle_attitude_0__f_yaw.plot(label='yaw', style='b-')
    
    def plot_euler_groundtruth(self):
        self.df.t_vehicle_attitude_groundtruth_0__f_roll.plot(label='roll true', style='r--')
        self.df.t_vehicle_attitude_groundtruth_0__f_pitch.plot(label='pitch true', style='g--')
        self.df.t_vehicle_attitude_groundtruth_0__f_yaw.plot(label='yaw true', style='b--')

    def plot_euler_error(self):
        self.df.t_vehicle_attitude_0__f_roll_error.plot(label='roll error', style='r-')
        self.df.t_vehicle_attitude_0__f_pitch_error.plot(label='pitch error', style='g-')
        self.df.t_vehicle_attitude_0__f_yaw_error.plot(label='yaw error', style='b-')
   
    @staticmethod
    def series_quat2euler(q0, q1, q2, q3, msg_name):
        yaw, pitch, roll = np.array([ tf.quat2euler([q0i, q1i, q2i, q3i]) for
            q0i, q1i, q2i, q3i in zip(q0, q1, q2, q3) ]).T
        yaw = pd.Series(name=msg_name + '__f_yaw', data=yaw, index=q0.index)
        pitch = pd.Series(name=msg_name + '__f_pitch', data=pitch, index=q0.index)
        roll = pd.Series(name=msg_name + '__f_roll', data=roll, index=q0.index)
        return roll, pitch, yaw
    
    def __repr__(self):
        return self.df.__repr__()


class PX4MessageDict(dict):

    def __init__(self, d):
        super(PX4MessageDict, self).__init__(d)
    
    def resample(self, period):
        msg_dict_rs = {}
        for key in self.keys():
            try:
                msg_dict_rs[key] = self[key].resample(period).ffill().bfill()
            except ValueError as ex:
                print(key, ex)
        return PX4MessageDict(msg_dict_rs)

    def concat(self):
        return PX4DataFrame(pd.concat([
            pd.DataFrame(data=self[msg].values,
                         index=self[msg].index,
                         columns=[ msg + '__' + key for key in self[msg].columns ])
            for msg in self.keys() ], axis=1))


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
