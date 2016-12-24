"""
ulog2pandas converter
"""

#pylint: disable=no-member, invalid-name, broad-except, too-many-locals

from __future__ import print_function

import os
import tempfile
import re
import glob
import shutil

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pyulog

try:
    import transforms3d.taitbryan as tf
except ImportError as ex:
    print(ex)
    print('please install transforms3d: pip install transforms3d')

IEKF_STATES = {
    0: 'q_nb_0',
    1: 'q_nb_1',
    2: 'q_nb_2',
    3: 'q_nb_3',
    4: 'vel_N',
    5: 'vel_E',
    6: 'vel_D',
    7: 'gyro_bias_bx',
    8: 'gyro_bias_by',
    9: 'gyro_bias_bz',
    10: 'accel_scale',
    11: 'pos_N',
    12: 'pos_E',
    13: 'pos_D',
    14: 'terrain_alt',
    15: 'baro_bias',
    16: 'wind_N',
    17: 'wind_E',
    18: 'wind_D',
}

IEKF_ERROR_STATES = {
    0: 'rot_N',
    1: 'rot_E',
    2: 'rot_D',
    3: 'vel_N',
    4: 'vel_E',
    5: 'vel_D',
    6: 'gyro_bias_N',
    7: 'gyro_bias_E',
    8: 'gyro_bias_D',
    9: 'accel_scale',
    10: 'pos_N',
    11: 'pos_E',
    12: 'pos_D',
    13: 'terrain_alt',
    14: 'baro_bias',
    15: 'wind_N',
    16: 'wind_E',
    17: 'wind_D',
}

def compute_data(df):
    """
    This computes useful data from analysis from the raw log data,
    converting quaternions to euler angles etc.
    """
    series = [df]
    msg = 't_vehicle_attitude_0'
    roll, pitch, yaw = series_quat2euler(
        df.t_vehicle_attitude_0__f_q_0_,
        df.t_vehicle_attitude_0__f_q_1_,
        df.t_vehicle_attitude_0__f_q_2_,
        df.t_vehicle_attitude_0__f_q_3_, msg)
    series += [roll, pitch, yaw]

    try:
        msg_gt = 't_vehicle_attitude_groundtruth_0'
        roll_gt, pitch_gt, yaw_gt = series_quat2euler(
            df.t_vehicle_attitude_groundtruth_0__f_q_0_,
            df.t_vehicle_attitude_groundtruth_0__f_q_1_,
            df.t_vehicle_attitude_groundtruth_0__f_q_2_,
            df.t_vehicle_attitude_groundtruth_0__f_q_3_, msg_gt)

        e_roll = pd.Series(angle_wrap(roll - roll_gt), name=msg + '__f_roll_error')
        e_pitch = pd.Series(angle_wrap(pitch - pitch_gt), name=msg + '__f_pitch_error')
        e_yaw = pd.Series(angle_wrap(yaw - yaw_gt), name=msg + '__f_yaw_error')

        series += [roll_gt, pitch_gt, yaw_gt, e_roll, e_pitch, e_yaw]
    except Exception as ex:
        print(ex)

    return pd.concat(series, axis=1)

def angle_wrap(x):
    """wrap angle betwe -pi and pi"""
    return np.arcsin(np.sin(x))

def plot_altitude(df, plot_groundtruth=False):
    """
    Plot altitude
    """
    plt.title('altitude')
    df.t_vehicle_global_position_0__f_alt.plot(label='alt', style='b-')
    if plot_groundtruth:
        df.t_vehicle_global_position_groundtruth_0__f_alt.plot(
            label='alt-true', style='b--')
    plt.grid()
    plt.legend(loc='best', ncol=3)
    plt.xlabel('t, sec')
    plt.ylabel('m')

def plot_iekf_std_dev(df):
    """
    Plot IEKF standard deviation.
    """
    for i in range(len(IEKF_ERROR_STATES)):
        exec('np.sqrt(df.t_estimator_status_0__f_covariances_{:d}_).plot(label=IEKF_ERROR_STATES[{:d}])'.format(i, i))
    plt.gca().set_ylim(0, 4)
    plt.legend(ncol=3, loc='best')
    plt.title('IEKF est std. dev.')
    plt.grid()

def plot_iekf_states(df):
    """
    Plot IEKF states
    """
    for i in range(len(IEKF_STATES)):
        exec('df.t_estimator_status_0__f_states_{:d}_.plot(label=IEKF_STATES[{:d}])'.format(i, i))
    plt.legend(ncol=3, loc='best')
    plt.title('IEKF states')
    plt.grid()

def plot_local_position(df, plot_groundtruth=False):
    """
    Plot local position
    """
    plt.title('local position')
    plt.plot(df.t_vehicle_local_position_0__f_x,
             df.t_vehicle_local_position_0__f_y, label='estimate')
    if plot_groundtruth:
        plt.plot(df.t_vehicle_local_position_groundtruth_0__f_x,
                 df.t_vehicle_local_position_groundtruth_0__f_y, 'r--',
                 label='true')
    plt.grid()
    plt.xlabel('N, m')
    plt.ylabel('E, m')
    plt.legend(loc='best')

def plot_euler(df, plot_groundtruth=False):
    """
    Plot euler angles
    """
    plt.title('euler angles')
    np.rad2deg(df.t_vehicle_attitude_0__f_roll).plot(label='roll', style='r-')
    if plot_groundtruth:
        np.rad2deg(df.t_vehicle_attitude_groundtruth_0__f_roll).plot(
            label='roll true', style='r--')
    np.rad2deg(df.t_vehicle_attitude_0__f_pitch).plot(label='pitch', style='g-')
    if plot_groundtruth:
        np.rad2deg(df.t_vehicle_attitude_groundtruth_0__f_pitch).plot(
            label='pitch true', style='g--')
    np.rad2deg(df.t_vehicle_attitude_0__f_yaw).plot(label='yaw', style='b-')
    if plot_groundtruth:
        np.rad2deg(df.t_vehicle_attitude_groundtruth_0__f_yaw).plot(
            label='yaw true', style='b--')
    plt.grid()
    plt.legend(loc='best', ncol=3)
    plt.xlabel('t, sec')
    plt.ylabel('deg')

def plot_euler_error(df):
    """
    Plot error between euler angles and ground truth
    """
    plt.title('euler angle errors')
    np.rad2deg(df.t_vehicle_attitude_0__f_roll_error).plot(
        label='roll error', style='r-')
    np.rad2deg(df.t_vehicle_attitude_0__f_pitch_error).plot(
        label='pitch error', style='g-')
    np.rad2deg(df.t_vehicle_attitude_0__f_yaw_error).plot(
        label='yaw error', style='b-')
    plt.grid()
    plt.legend(loc='best', ncol=3)
    plt.xlabel('t, sec')
    plt.ylabel('deg')

def plot_velocity(df, plot_groundtruth=False):
    """
    Plot velocity
    """
    plt.title('velocity')
    df.t_vehicle_global_position_0__f_vel_n.plot(label='vel_n', style='r-')
    if plot_groundtruth:
        df.t_vehicle_global_position_groundtruth_0__f_vel_n.plot(
            label='vel_n-true', style='r--')
    df.t_vehicle_global_position_0__f_vel_e.plot(
        label='vel_e', style='g-')
    if plot_groundtruth:
        df.t_vehicle_global_position_groundtruth_0__f_vel_e.plot(
            label='vel_e-true', style='g--')
    df.t_vehicle_global_position_0__f_vel_d.plot(
        label='vel_d', style='b-')
    if plot_groundtruth:
        df.t_vehicle_global_position_groundtruth_0__f_vel_d.plot(
            label='vel_d-true', style='b--')
    plt.grid()
    plt.legend(loc='best', ncol=3)
    plt.xlabel('t, sec')
    plt.ylabel('m/s')

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

def estimator_analysis(df, plot=True):
    """
    Evaluates estimator performance
    """
    #pylint: disable=unused-variable
    data = {
        'roll_error_mean' : np.rad2deg(
            df.t_vehicle_attitude_0__f_roll_error.mean()),
        'pitch_error_mean' : np.rad2deg(
            df.t_vehicle_attitude_0__f_pitch_error.mean()),
        'yaw_error_mean' : np.rad2deg(
            df.t_vehicle_attitude_0__f_yaw_error.mean()),
        'roll_error_std' : np.rad2deg(np.sqrt(
            df.t_vehicle_attitude_0__f_roll_error.var())),
        'pitch_error_std' : np.rad2deg(np.sqrt(
            df.t_vehicle_attitude_0__f_pitch_error.var())),
        'yaw_error_std' : np.rad2deg(np.sqrt(
            df.t_vehicle_attitude_0__f_yaw_error.var())),
    }
    if plot:
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
'''.format(**data))

        plt.figure()
        plot_altitude(df, plot_groundtruth=True)

        plt.figure()
        plot_euler(df, plot_groundtruth=True)

        plt.figure()
        plot_euler_error(df)

        plt.figure()
        plot_local_position(df, plot_groundtruth=True)

        plt.figure()
        plot_velocity(df, plot_groundtruth=True)

    return data

class PX4MessageDict(dict):

    """
    PX4 has several data frames in a log and they don't all have the same
    index, so this structure is used to manipulate the resulting dictionary
    of dataframes that a log produces.
    """

    def __init__(self, d):
        super(PX4MessageDict, self).__init__(d)

    def resample_and_concat(self, dt):
        """
        Resample at dt and concatenate all data frames.
        """
        act_ctrl = self['actuator_controls_0_0']
        # build empty data frame with the index we want
        m = pd.DataFrame(
            data=np.arange(
                int(1e3*act_ctrl.timestamp.values[0]),
                int(1e3*act_ctrl.timestamp.values[-1]),
                int(1e9*dt)), columns=['timestamp'])
        # populate dataframe with other data frames merging
        # as of our defined index
        for topic in sorted(self.keys()):
            new_cols = {}
            for col in self[topic].columns:
                if col == 'timestamp':
                    new_cols[col] = col
                else:
                    new_cols[col] = 't_' + topic + '__f_' + col
            df = self[topic].rename(columns=new_cols)
            # estimator status uses nano-seconds, the
            # rest use micro-seconds
            if topic != 'estimator_status_0':
                df.timestamp = np.array(df.timestamp*1e3, dtype=np.int)
            m = pd.merge_asof(m, df, on='timestamp')
            m = m.drop_duplicates()
        m.index = pd.Index(m.timestamp/1e9, name='time, sec')
        return compute_data(m)

def read_ulog(ulog_filename, messages='', verbose=False):
    """
    Convert ulog to pandas dataframe.
    """

    tmp_dir = tempfile.mkdtemp()
    pyulog.ulog2csv.convert_ulog2csv(
        ulog_filename, messages, tmp_dir, ',')
    log_name = os.path.splitext(os.path.basename(ulog_filename))[0]
    data = {}
    glob_expr = '{:s}*.csv'.format(
        os.path.join(tmp_dir, log_name))

    # column naming
    d_col_rename = {
        '[': '_',
        ']': '_',
    }
    col_rename_pattern = re.compile(
        r'(' + '|'.join([re.escape(key) for key in d_col_rename.keys()]) + r')')

    for filename in sorted(glob.glob(glob_expr)):
        if verbose:
            print('processing', filename)
        file_name = os.path.splitext(os.path.basename(filename))[0]
        topic_name = file_name.replace(log_name + '_', '')

        # read data
        data_new = pd.read_csv(filename)
        data_new.columns = [
            col_rename_pattern.sub(
                lambda x: d_col_rename[x.group()], col)
            for col in data_new.columns
        ]

        data[topic_name] = data_new

    if verbose:
        print(log_name, 'data loaded')

    shutil.rmtree(tmp_dir)
    return PX4MessageDict(data)

#  vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 :
