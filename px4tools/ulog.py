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
import transforms3d.taitbryan as tf
from scipy.interpolate import interp1d

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
        sensor_comb = self['sensor_combined_0']
        # build empty data frame with the index we want
        m = pd.DataFrame(
            data=np.arange(
                int(1e3*sensor_comb.timestamp.values[0]),
                int(1e3*sensor_comb.timestamp.values[-1]),
                int(1e9*dt)), columns=['timestamp'])
        # populate dataframe with other data frames merging
        # as of our defined index
        for topic in sorted(self.keys()):
            new_cols = {}
            print('merging {:s} as of timestamp'.format(topic))
            for col in self[topic].columns:
                if col == 'timestamp':
                    new_cols[col] = col
                else:
                    new_cols[col] = 't_' + topic + '__f_' + col
            df = self[topic].rename(columns=new_cols)
            df.timestamp = np.array(df.timestamp*1e3, dtype=np.int)
            df = df.sort_values('timestamp')
            m = pd.merge_asof(m, df, on='timestamp')
        m.index = pd.Index(m.timestamp/1e9, name='time, sec')
        try:
            return compute_data(m)
        except AttributeError as e:
            print(e)
            return m

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

def plot_allan_variance(data, dt, plot=True):
    """
    Given a dataset of a stationary vehicle on the ground,
    this compute the alan variance plot for the noise.
    The intersection at 1 is the noise power.
    """


    data.index = pd.TimedeltaIndex(data.index, unit='s')
    data_vals = []
    dt_vals = []
    c = 0
    c_vals = []
    while 2**c < len(data.index)/2**6:
        c_vals += [2**c]
        c += 1
    for i in c_vals:
        std = float(np.sqrt(data.resample(
            '{:d}L'.format(int(i*dt*1000))).agg('mean').var()))
        data_vals += [std]
        dt_vals += [(i*dt)]

    p = np.polyfit(np.log10(dt_vals), np.log10(data_vals), 4)

    try:
        noise_power = float(10**np.polyval(p, 0.0))
    except Exception as e:
        print(e)
        noise_power = 0.0

    if plot:
        plt.title('Allan variance plot')
        plt.loglog(dt_vals, data_vals, '.-')
        plt.xlabel('Averaging Time, $\\tau$, sec')
        plt.ylabel('Allan Deviation $\\sigma(\\tau)$')
        plt.grid(True, which='both')
        plt.minorticks_on()
    return noise_power

def plot_autocorrelation(data, dt, plot=True):
    """
    Given a dataset of a stationary vehicle on the ground,
    this compute the autocorrellation. The intersection with
    the horizontal line at 0.348 represents the correllation
    time constant. If the intersection does not occur, it
    indicates that the random walk process is not significant
    and is over powered by wide-band noise.
    """
    data_vals = []
    dt_vals = []
    lag_n = min(int(len(data.index)/2), int(1000/dt))
    lag_max = dt*lag_n
    for i in range(lag_n):
        data_vals += [data.autocorr(lag=i)]
        dt_vals += [i*dt]

    # polynomial fits
    p = np.polynomial.Polynomial.fit(dt_vals, data_vals, 4)

    if plot:
        x = np.linspace(0, lag_max)
        y = p(x)
        plt.title('autocorrelation plot')
        plt.plot(dt_vals, data_vals, '.-')
        plt.xlabel('lag, sec')
        plt.ylabel('autocorrelation')
        plt.hlines(1/np.e, 0, lag_max)
        plt.plot(x, y)
        plt.grid(True)

    # grab first positive, real root
    correlation_time = 0.0
    for root in sorted((p - 1/np.e).roots()):
        if np.isreal(root) and root > 0 and root < lag_max:
            correlation_time = float(np.real(root))
            break
    return correlation_time

def noise_analysis(df, dt_sample, plot=True):
    """
    Given a dataset of a stationary vehicle on the ground, this compute the noise statistics.
    """

    r = {}

    # gyroscope
    plt.figure()
    t_g_x = plot_autocorrelation(df.t_sensor_combined_0__f_gyro_rad_0_, dt_sample, plot)
    t_g_y = plot_autocorrelation(df.t_sensor_combined_0__f_gyro_rad_1_, dt_sample, plot)
    t_g_z = plot_autocorrelation(df.t_sensor_combined_0__f_gyro_rad_2_, dt_sample, plot)
    plt.title('autocorrelation - gyroscope')
    r['gyroscope_randomwalk_correlation_time'] = np.mean([t_g_x, t_g_y, t_g_z])

    plt.figure()
    n_g_x = plot_allan_variance(df.t_sensor_combined_0__f_gyro_rad_0_, dt_sample, plot)
    n_g_y = plot_allan_variance(df.t_sensor_combined_0__f_gyro_rad_1_, dt_sample, plot)
    n_g_z = plot_allan_variance(df.t_sensor_combined_0__f_gyro_rad_2_, dt_sample, plot)
    plt.title('Alan variance plot - gyroscope')
    r['gyroscope_noise_density'] = np.mean([n_g_x, n_g_y, n_g_z])

    # accelerometer
    plt.figure()
    t_a_x = plot_autocorrelation(
        df.t_sensor_combined_0__f_accelerometer_m_s2_0_,
        dt_sample, plot)
    t_a_y = plot_autocorrelation(
        df.t_sensor_combined_0__f_accelerometer_m_s2_1_,
        dt_sample, plot)
    t_a_z = plot_autocorrelation(
        df.t_sensor_combined_0__f_accelerometer_m_s2_2_,
        dt_sample)
    r['accelerometer_randomwalk_correlation_time'] = np.mean([t_a_x, t_a_y, t_a_z])
    plt.title('autocorrelation - accelerometer')

    plt.figure()
    n_a_x = plot_allan_variance(
        df.t_sensor_combined_0__f_accelerometer_m_s2_0_,
        dt_sample, plot)
    n_a_y = plot_allan_variance(
        df.t_sensor_combined_0__f_accelerometer_m_s2_1_,
        dt_sample, plot)
    n_a_z = plot_allan_variance(
        df.t_sensor_combined_0__f_accelerometer_m_s2_2_,
        dt_sample, plot)
    plt.title('Alan variance plot - accelerometer')
    r['accelerometer_noise_density'] = np.mean([n_a_x, n_a_y, n_a_z])

    # magnetometer
    plt.figure()
    t_m_x = plot_autocorrelation(
        df.t_sensor_combined_0__f_magnetometer_ga_0_,
        dt_sample, plot)
    t_m_y = plot_autocorrelation(
        df.t_sensor_combined_0__f_magnetometer_ga_1_,
        dt_sample, plot)
    t_m_z = plot_autocorrelation(
        df.t_sensor_combined_0__f_magnetometer_ga_2_,
        dt_sample, plot)
    r['magnetometer_randomwalk_correlation_time'] = np.mean([t_m_x, t_m_y, t_m_z])
    plt.title('autocorrelation - magnetometer')

    plt.figure()
    n_m_x = plot_allan_variance(
        df.t_sensor_combined_0__f_magnetometer_ga_0_,
        dt_sample, plot)
    n_m_y = plot_allan_variance(
        df.t_sensor_combined_0__f_magnetometer_ga_1_,
        dt_sample, plot)
    n_m_z = plot_allan_variance(
        df.t_sensor_combined_0__f_magnetometer_ga_2_,
        dt_sample, plot)
    plt.title('Alan variance plot - magnetometer')
    r['magnetometer_noise_density'] = np.mean([n_m_x, n_m_y, n_m_z])

    # baro
    plt.figure()
    t_b = plot_autocorrelation(
        df.t_sensor_combined_0__f_baro_alt_meter,
        dt_sample, plot)
    r['baro_randomwalk_correlation_time'] = t_b
    plt.title('autocorrelation - barometric altimeter')

    plt.figure()
    n_b = plot_allan_variance(
        df.t_sensor_combined_0__f_baro_alt_meter,
        dt_sample, plot)
    plt.title('Alan variance plot - barometric altimeter')
    r['baro_noise_desnity'] = n_b

    return r

#  vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 :
