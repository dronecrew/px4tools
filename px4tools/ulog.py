"""
ulog2pandas converter
"""

# pylint: disable=no-member, invalid-name, broad-except, too-many-locals

from __future__ import print_function

import os
import pickle
import re

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pyulog
import scipy.signal
import transforms3d.quaternions as quat
import transforms3d.taitbryan as tf


# create index to state lookup for estimators
EST_NAME = {
    'iekf': [
        'q_nb_0', 'q_nb_1', 'q_nb_2', 'q_nb_3',
        'vel_N', 'vel_E', 'vel_D',
        'gyro_bias_bx', 'gyro_bias_by', 'gyro_bias_bz',
        'accel_bias_bx', 'accel_bias_by', 'accel_bias_bz',
        'pos_N', 'pos_E', 'pos_D',
        'terrain_alt',
        'baro_bias',
        # 'wind_N', 'wind_E', 'wind_D',
        ],
    'iekf_error': [
        'rot_N', 'rot_E', 'rot_D',
        'vel_N', 'vel_E', 'vel_D',
        'gyro_bias_N', 'gyro_bias_E', 'gyro_bias_D',
        'accel_bias_N', 'accel_bias_E', 'accel_bias_D',
        'pos_N', 'pos_E', 'pos_D',
        'terrain_alt',
        'baro_bias',
        # 'wind_N', 'wind_E', 'wind_D',
        ],
    'ekf2': [
        'q_nb_0', 'q_nb_1', 'q_nb_2', 'q_nb_3',
        'vel_N', 'vel_E', 'vel_D',
        'pos_N', 'pos_E', 'pos_D',
        'gyro_bias_bx', 'gyro_bias_by', 'gyro_bias_bz',
        'accel_bias_bx', 'accel_bias_by', 'accel_bias_bz',
        'mag_N', 'mag_E', 'mag_D',
        'wind_N', 'wind_E', 'wind_D'
        ],
    'ekf2_error': [
        'rot_N', 'rot_E', 'rot_D',
        'vel_N', 'vel_E', 'vel_D',
        'pos_N', 'pos_E', 'pos_D',
        'gyro_bias_bx', 'gyro_bias_by', 'gyro_bias_bz',
        'accel_bias_bx', 'accel_bias_by', 'accel_bias_bz',
        'mag_N', 'mag_E', 'mag_D',
        'wind_N', 'wind_E', 'wind_D'
        ]
}


def state_to_index(l):
    """
    Given 
    """
    return {l[i]: i for i, name in enumerate(l)}


def compute_data(df):
    """
    This computes useful data from analysis from the raw log data,
    converting quaternions to euler angles etc.
    """
    series = [df]
    msg_att = 't_vehicle_attitude_0'
    roll, pitch, yaw = series_quat2euler(
        df.t_vehicle_attitude_0__f_q_0_,
        df.t_vehicle_attitude_0__f_q_1_,
        df.t_vehicle_attitude_0__f_q_2_,
        df.t_vehicle_attitude_0__f_q_3_, msg_att)
    series += [roll, pitch, yaw]

    try:
        msg_gt = 't_vehicle_attitude_groundtruth_0'
        roll_gt, pitch_gt, yaw_gt = series_quat2euler(
            df.t_vehicle_attitude_groundtruth_0__f_q_0_,
            df.t_vehicle_attitude_groundtruth_0__f_q_1_,
            df.t_vehicle_attitude_groundtruth_0__f_q_2_,
            df.t_vehicle_attitude_groundtruth_0__f_q_3_, msg_gt)

        e_roll = pd.Series(angle_wrap(roll - roll_gt),
                           name=msg_att + '__f_roll_error')
        e_pitch = pd.Series(angle_wrap(pitch - pitch_gt),
                            name=msg_att + '__f_pitch_error')
        e_yaw = pd.Series(angle_wrap(yaw - yaw_gt),
                          name=msg_att + '__f_yaw_error')

        msg_lpos = 't_vehicle_local_position_0'
        msg_lpos_gt = 't_vehicle_local_position_groundtruth_0'

        e_x = pd.Series(
            df.t_vehicle_local_position_0__f_x -
            df.t_vehicle_local_position_groundtruth_0__f_x,
            name=msg_lpos + '__f_x_error')

        e_y = pd.Series(
            df.t_vehicle_local_position_0__f_y -
            df.t_vehicle_local_position_groundtruth_0__f_y,
            name=msg_lpos + '__f_y_error')

        e_z = pd.Series(
            df.t_vehicle_local_position_0__f_z -
            df.t_vehicle_local_position_groundtruth_0__f_z,
            name=msg_lpos + '__f_z_error')

        e_vx = pd.Series(
            df.t_vehicle_local_position_0__f_vx -
            df.t_vehicle_local_position_groundtruth_0__f_vx,
            name=msg_lpos + '__f_vx_error')

        e_vy = pd.Series(
            df.t_vehicle_local_position_0__f_vy -
            df.t_vehicle_local_position_groundtruth_0__f_vy,
            name=msg_lpos + '__f_vy_error')

        e_vz = pd.Series(
            df.t_vehicle_local_position_0__f_vz -
            df.t_vehicle_local_position_groundtruth_0__f_vz,
            name=msg_lpos + '__f_vz_error')

        speed = pd.Series(np.sqrt(
            df.t_vehicle_local_position_0__f_vx ** 2 +
            df.t_vehicle_local_position_0__f_vy ** 2 +
            df.t_vehicle_local_position_0__f_vz ** 2
            ), name=msg_lpos + '__f_speed')

        speed_gt = pd.Series(np.sqrt(
            df.t_vehicle_local_position_groundtruth_0__f_vx ** 2 +
            df.t_vehicle_local_position_groundtruth_0__f_vy ** 2 +
            df.t_vehicle_local_position_groundtruth_0__f_vz ** 2
            ), name=msg_lpos_gt + '__f_speed')

        e_speed = pd.Series(
            speed - speed_gt,
            name=msg_lpos + '__f_speed_error')

        series += [
            roll_gt, pitch_gt, yaw_gt,
            e_roll, e_pitch, e_yaw,
            e_x, e_y, e_z,
            e_vx, e_vy, e_vz, speed, speed_gt, e_speed]
    except Exception as ex:
        print(ex)

    return pd.concat(series, axis=1)


def angle_wrap(x):
    """wrap angle between -pi and pi"""
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
    plt.gcf().autofmt_xdate()


def extract_P(df, msg_name='t_estimator_status_0__f_covariances_', num_states=19):
    '''
    Extract the covariance matrices P for at each time step
    of the log data df. P is a diagonal matrix sized by the
    number of states num_states. Data is extracted from the
    t_estimator_status topic for num_states topics.
    '''
    states = np.arange(0, num_states, 1)
    # initialize list of covariances for each state at each time step
    estimator_status_list = []
    for k in range(len(states)):
        estimator_name = msg_name + states.astype('unicode')[k] + '_'
        attribute = np.array([getattr(df, estimator_name).values]).T
        estimator_status_list += [attribute]
    # covariance n-dimensional array of size (num_states/no.(df points),1)

    covariance_nd_array = np.ascontiguousarray(estimator_status_list)
    # print(covariance_nd_array.shape)
    # List of covariance matrices
    P_list = []
    # for the ith time-step
    for i in range(len(df.t_estimator_status_0__f_covariances_0_.values)):
        # initialize P as a num_states x num_states zero matrix
        P = np.zeros((num_states, num_states))
        #  cycle through each states at the ith time step
        for j in range(len(states)):
            # update the diagonal element for the jth state of the ith P matrix
            P[j, j] = np.ascontiguousarray(covariance_nd_array[j, i, :])
        # list of i P(jxj) matrices
        P_list += [P]
    return P_list


def plot_estimator_state(df, est, states=()):
    # type: (pandas.DataFrame, list, list) -> None
    """
    Plot States with a give set of labels
    :param df: pandas DataFrame
    :param est: name of estmator (iekf, ekf2)
    :param states: tuple of states to plot, see EST_NAME
    :return: None
    """
    state_list = EST_NAME[est]
    if len(states) == 0:
        plot_states = state_list
    else:
        plot_states = states
    name_to_index = {plot_states[i]: i for i, state in enumerate(plot_states)}
    for state in plot_states:
        d = df['t_estimator_status_0__f_states_{:d}_'.format(name_to_index[state])]
        d.plot(label=state)
    plt.legend(ncol=3, loc='best')
    plt.title('estimator state')
    plt.grid()
    plt.gcf().autofmt_xdate()


def plot_estimator_state_uncertainty(df, est, states=()):
    # type: (pandas.DataFrame, list, list) -> None
    """
    Plot States with a give set of labels
    :param df: pandas DataFrame
    :param est: name of estmator (iekf, ekf2)
    :param states: tuple of states to plot, see EST_NAME
    :return: None
    """
    state_list = EST_NAME[est + '_error']
    if len(states) == 0:
        plot_states = state_list
    else:
        plot_states = states
    name_to_index = {plot_states[i]: i for i, state in enumerate(plot_states)}
    for state in plot_states:
        d = df['t_estimator_status_0__f_covariances_{:d}_'.format(name_to_index[state])]
        np.sqrt(d).plot(label=state)

    plt.legend(ncol=3, loc='best')
    plt.title('estimator state uncertainty')
    plt.grid()
    plt.gcf().autofmt_xdate()


def plot_local_position(df, plot_groundtruth=False):
    """
    Plot local position
    """
    plt.title('local position')
    plt.plot(df.t_vehicle_local_position_0__f_y,
             df.t_vehicle_local_position_0__f_x, label='estimate')
    if plot_groundtruth:
        plt.plot(df.t_vehicle_local_position_groundtruth_0__f_y,
                 df.t_vehicle_local_position_groundtruth_0__f_x, 'r--',
                 label='true')
    plt.grid()
    plt.xlabel('E, m')
    plt.ylabel('N, m')
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
    np.rad2deg(df.t_vehicle_attitude_0__f_pitch).plot(
        label='pitch', style='g-')
    if plot_groundtruth:
        np.rad2deg(df.t_vehicle_attitude_groundtruth_0__f_pitch).plot(
            label='pitch true', style='g--')
    np.rad2deg(df.t_vehicle_attitude_0__f_yaw).plot(label='yaw', style='b-')
    if plot_groundtruth:
        np.rad2deg(df.t_vehicle_attitude_groundtruth_0__f_yaw).plot(
            label='yaw true', style='b--')
    plt.grid()
    plt.legend(loc='best', ncol=3)
    plt.xlabel('time')
    plt.ylabel('deg')
    plt.gcf().autofmt_xdate()


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
    plt.xlabel('time')
    plt.ylabel('deg')
    plt.gcf().autofmt_xdate()


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
    plt.xlabel('time')
    plt.ylabel('m/s')
    plt.gcf().autofmt_xdate()


def plot_speed(df):
    """
    Plot speed.
    """
    df.t_vehicle_local_position_0__f_speed.plot()
    plt.gcf().autofmt_xdate()
    plt.ylabel('speed, m/s')
    plt.xlabel('time')
    plt.grid()


def series_quatrot(x, y, z, q0, q1, q2, q3, rot_name):
    """
    Given pandas series x-z and quaternion q0-q4,
    compute rotated vector x_r, y_r, z_r
    """
    vec = np.array([
        quat.rotate_vector([xi, yi, zi], [q0i, q1i, q2i, q3i])
        for xi, yi, zi, q0i, q1i, q2i, q3i in zip(x, y, z, q0, q1, q2, q3)
    ])
    x_r = pd.Series(name=x.name + '_' + rot_name, data=vec[:, 0], index=x.index)
    y_r = pd.Series(name=y.name + '_' + rot_name, data=vec[:, 1], index=y.index)
    z_r = pd.Series(name=z.name + '_' + rot_name, data=vec[:, 2], index=z.index)
    return x_r, y_r, z_r


def series_quatrot_inverse(x, y, z, q0, q1, q2, q3, rot_name):
    """
    Given pandas series x-z and quaternion q0-q4,
    compute reversed rotated vector x_r, y_r, z_r
    """
    return series_quatrot(x, y, z, q0, -q1, -q2, -q3, rot_name)


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
    # pylint: disable=unused-variable
    data = {
        'roll_error_mean': np.rad2deg(
            df.t_vehicle_attitude_0__f_roll_error.mean()),
        'pitch_error_mean': np.rad2deg(
            df.t_vehicle_attitude_0__f_pitch_error.mean()),
        'yaw_error_mean': np.rad2deg(
            df.t_vehicle_attitude_0__f_yaw_error.mean()),
        'roll_error_std': np.rad2deg(
            df.t_vehicle_attitude_0__f_roll_error.std()),
        'pitch_error_std': np.rad2deg(
            df.t_vehicle_attitude_0__f_pitch_error.std()),
        'yaw_error_std': np.rad2deg(
            df.t_vehicle_attitude_0__f_yaw_error.std()),
        'x_error_mean':
            df.t_vehicle_local_position_0__f_x_error.mean(),
        'y_error_mean':
            df.t_vehicle_local_position_0__f_y_error.mean(),
        'z_error_mean':
            df.t_vehicle_local_position_0__f_z_error.mean(),
        'vx_error_mean':
            df.t_vehicle_local_position_0__f_vx_error.mean(),
        'vy_error_mean':
            df.t_vehicle_local_position_0__f_vy_error.mean(),
        'vz_error_mean':
            df.t_vehicle_local_position_0__f_vz_error.mean(),
        'x_error_std':
            df.t_vehicle_local_position_0__f_x_error.std(),
        'y_error_std':
            df.t_vehicle_local_position_0__f_y_error.std(),
        'z_error_std':
            df.t_vehicle_local_position_0__f_z_error.std(),
        'vx_error_std':
            df.t_vehicle_local_position_0__f_vx_error.std(),
        'vy_error_std':
            df.t_vehicle_local_position_0__f_vy_error.std(),
        'vz_error_std':
            df.t_vehicle_local_position_0__f_vz_error.std(),
    }
    if plot:
        print('''
ESTIMATOR ANALYSIS
-----------------------------------

attitude error:
\troll  mean: {roll_error_mean:10f}\tstd:\t{roll_error_std:10f}\tdeg
\tpitch mean: {pitch_error_mean:10f}\tstd:\t{pitch_error_std:10f}\tdeg
\tyaw   mean: {yaw_error_mean:10f}\tstd:\t{yaw_error_std:10f}\tdeg

position error:
\tx mean: {x_error_mean:10f}\tstd:\t{x_error_std:10f}\tm
\ty mean: {y_error_mean:10f}\tstd:\t{y_error_std:10f}\tm
\tz mean: {z_error_mean:10f}\tstd:\t{z_error_std:10f}\tm

velocity error:
\tx mean: {vx_error_mean:10f}\tstd:\t{vx_error_std:10f}\tm/s
\ty mean: {vy_error_mean:10f}\tstd:\t{vy_error_std:10f}\tm/s
\tz mean: {vz_error_mean:10f}\tstd:\t{vz_error_std:10f}\tm/s
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
        for topic in self.keys():
            new_cols = {}
            for col in self[topic].columns:
                if col == 'timestamp':
                    new_cols[col] = col
                elif col[:2] != 'f_':
                    new_cols[col] = 'f_' + col
            df = self[topic].rename(columns=new_cols)
            df.index = pd.TimedeltaIndex(df.timestamp * 1e3, unit='ns')
            self[topic] = df

    def concat(self, topics=None, on=None, dt=None, verbose=False):
        """

        @param topics: topics to merge on, unspecified merges all

        timestamp options
        @param on: the topic whose timestamp will define the merge timestamp,
            if specified, dt cannot be specified
        @param dt: if specified, on cannot be specified

        @param verbose: show status
        """

        d = PX4MessageDict(self)

        for topic in d.keys():
            new_cols = {}
            for col in d[topic].columns:
                if col == 'timestamp':
                    new_cols[col] = col
                else:
                    new_cols[col] = topic + '__' + col
            df = d[topic].rename(columns=new_cols)
            d[topic] = df

        # define timestamps to merge on
        if dt is not None:
            ts_min = None
            ts_max = None
            for topic in sorted(d.keys()):
                ts_t_min = d[topic]['timestamp'].min()
                ts_t_max = d[topic]['timestamp'].max()
                if ts_min is None or ts_t_min < ts_min:
                    ts_min = ts_t_min
                if ts_max is None or ts_t_max > ts_max:
                    ts_max = ts_t_max
            timestamps = np.arange(
                ts_min, ts_max - dt * 1e6, dt * 1e6, dtype=np.uint64)
        elif on is not None:
            timestamps = d[on].timestamp
        else:
            raise IOError('must pass dt or on')

        # concat
        m = pd.DataFrame(data=timestamps, columns=['timestamp'])
        if topics is None:
            topics = d.keys()
        for topic in topics:
            if verbose:
                print('merging {:s} as of timestamp'.format(topic))
            df = d[topic]
            df.sort_values(by='timestamp', inplace=True)
            m = pd.merge_asof(m, df, 'timestamp')
        m.index = pd.TimedeltaIndex(m.timestamp * 1e3, unit='ns')
        return m

    def __getattr__(self, attr):
        # Fake a __getstate__ method that returns None
        if attr == "__getstate__":
            return lambda: None
        return self[attr]

    def __setattr__(self, attr, value):
        self[attr] = value

    def set_with_dict(self, D):
        """ set attributes with a dict """
        for k in D.keys():
            self.__setattr__(k, D[k])

    def __dir__(self):
        return super(PX4MessageDict, self).__dir__() + list(self.keys())


def read_ulog(ulog_filename, messages=None):
    """
    Convert ulog to pandas dataframe.
    """
    log = pyulog.ULog(ulog_filename, messages)

    # column naming
    d_col_rename = {
        '[': '_',
        ']': '_',
        '.': '_',
    }
    col_rename_pattern = re.compile(
        r'(' + '|'.join([
            re.escape(key)
            for key in d_col_rename.keys()]) + r')')

    data = {}
    for msg in log.data_list:
        msg_data = pd.DataFrame.from_dict(msg.data)
        msg_data.columns = [
            col_rename_pattern.sub(
                lambda x: d_col_rename[x.group()], col)
            for col in msg_data.columns
        ]
        msg_data.index = pd.TimedeltaIndex(msg_data['timestamp'] * 1e3, unit='ns')
        data['t_{:s}_{:d}'.format(msg.name, msg.multi_id)] = msg_data

    return PX4MessageDict(data)


def _smallest_positive_real_root(roots, min_val=0, max_val=1e6):
    """
    Find smallest positive real root in list
    """
    res = np.nan
    if len(roots) > 0:
        cond = np.logical_and(
            np.isreal(roots),
            np.real(roots) > min_val)
        posreal = sorted(np.real(roots[cond]))
        if len(posreal) > 0:
            res = posreal[0]
    if not np.isfinite(res) or res < min_val or res > max_val:
        res = np.nan
    return res


def plot_allan_std_dev(
        data, plot=True, plot_deriv=False, min_intervals=9, poly_order=2):
    """
    Given a dataset of a stationary vehicle on the ground,
    this compute the Allan standard deviation plot for the noise.
    """
    # pylint: disable=too-many-statements

    data.index = pd.TimedeltaIndex(data.index, unit='s')
    dt = float(np.diff(data.index.values).mean()) / 1e9

    data_vals = []
    dt_vals = []
    c = int(np.ceil(np.log10(dt)))
    c_vals = []
    # require at least 9 clusters for < 25% error:
    # source:
    # http://www.afahc.ro/ro/afases/2014/mecanica/marinov_rov_allan.pdf
    t_len = (data.index.values[-1] - data.index.values[0]) / 1e9
    while 10 ** c < float(t_len / min_intervals):
        c_vals += [10 ** c]
        c += 0.2
    for c_i in c_vals:
        allan_std = float(np.sqrt(data.resample(
            '{:d}L'.format(int(c_i * 1000))).agg('mean').diff().var() / 2))
        if not np.isfinite(allan_std):
            break
        data_vals += [allan_std]
        dt_vals += [c_i]

    x = np.log10(dt_vals)
    y = np.log10(data_vals)

    p = np.polynomial.Polynomial.fit(x, y, poly_order)
    pdiff = p.deriv()

    log_tau_0 = _smallest_positive_real_root((pdiff + 0.5).roots(), -5, 5)
    tau_0 = 10 ** log_tau_0
    if tau_0 > 0 and np.isfinite(tau_0):
        sig_rw = 10 ** p(log_tau_0) * np.sqrt(tau_0)
    else:
        # if intersect fails, evaluate slope at tau=1
        tau_0 = 1
        sig_rw = 10 ** p(np.log10(tau_0)) * np.sqrt(tau_0)

    log_tau_1 = _smallest_positive_real_root((pdiff).roots(), log_tau_0, 5)
    tau_1 = 10 ** log_tau_1
    if tau_1 > 0 and np.isfinite(tau_1):
        sig_bi = (10 ** p(log_tau_1)) * np.sqrt(np.pi / (2 * np.log(2)))
    else:
        sig_bi = 0

    log_tau_2 = _smallest_positive_real_root(
        (pdiff - 0.5).roots(), log_tau_1, 5)
    tau_2 = 10 ** log_tau_2
    if tau_2 > 0 and np.isfinite(tau_2):
        sig_rrw = (10 ** p(log_tau_2)) * np.sqrt(3 / tau_2)
    else:
        sig_rrw = 0

    if plot:
        x2 = np.linspace(
            min([x[0], log_tau_0, log_tau_1, log_tau_2]),
            max([x[-1], log_tau_0, log_tau_1, log_tau_2]))
        y2 = p(x2)
        plt.title('Frequency Stability')
        plt.loglog(dt_vals, data_vals, '.', label='raw')
        plt.xlabel('Averaging Time, $\\tau$, sec')
        plt.ylabel('Allan Deviation $\\sigma(\\tau)$')
        plt.loglog(10 ** x2, 10 ** y2, '-', label='fit')
        if plot_deriv:
            ydiff = pdiff(x2)
            plt.loglog(10 ** x2, 10 ** ydiff, '--', label='fit deriv')
        plt.plot(
            tau_0, 10 ** p(log_tau_0), 'rx',
            label='$\\sigma_{rw}$', markeredgewidth=3)
        plt.plot(
            tau_1, 10 ** p(log_tau_1), 'bx',
            label='$\\sigma_{bi}$', markeredgewidth=3)
        plt.plot(
            tau_2, 10 ** p(log_tau_2), 'gx',
            label='$\\sigma_{rrw}$', markeredgewidth=3)
        plt.grid(True, which='both')
        plt.minorticks_on()

    return {
        'sig_rw': sig_rw,
        'sig_bi': sig_bi,
        'sig_rrw': sig_rrw,
        'tau_0': tau_0,
        'tau_1': tau_1,
        'tau_2': tau_2
    }


def plot_autocorrelation(data, plot=True, poly_order=1):
    """
    Given a dataset of a stationary vehicle on the ground,
    this compute the autocorrellation. The intersection with
    the horizontal line at 0.348 represents the correllation
    time constant. If the intersection does not occur, it
    indicates that the random walk process is not significant
    and is over powered by wide-band noise.
    """
    data.index = pd.TimedeltaIndex(data.index, unit='s')
    data_vals = []
    dt_vals = []
    dt = 1
    # downsample every dt seconds, taking mean
    data = data.resample(
        '{:d}L'.format(int(dt * 1000))).agg('mean')
    lag_max = int(dt * len(data.index) / 2)
    for i in range(1, int(lag_max / dt)):
        data_vals += [data.autocorr(i)]
        dt_vals += [i * dt]

    # polynomial fits
    p = np.polynomial.Polynomial.fit(dt_vals, data_vals, poly_order)

    # normalize by max of fit polynomial
    x = np.linspace(0, lag_max)
    y = p(x)
    y_max = np.max(np.abs(y))
    data_vals /= y_max
    y /= y_max
    p /= y_max

    if plot:
        plt.title('normalized autocorrelation')
        plt.plot(dt_vals[1:], data_vals[1:], '.', alpha=0.2, label='raw')
        plt.xlabel('lag, sec')
        plt.ylabel('corr/ max(corr)')
        plt.plot(x, y, linewidth=2, label='fit')
        plt.hlines(1 / np.e, 0, lag_max, linewidth=2)
        plt.gca().set_ylim(-1, 1)
        plt.gca().set_xlim(0, lag_max)
        plt.grid(True)

    correlation_time = _smallest_positive_real_root((p - 1 / np.e).roots())
    return correlation_time


def noise_analysis_sensor(
        df, topic='sensor_gyro_0', plot=True, allan_args=None,
        corr_args=None):
    """
    Given a sensor gyro dataset of a stationary vehicle on the ground,
    this compute the noise statistics.
    """

    if allan_args is None:
        allan_args = {}

    if corr_args is None:
        corr_args = {}

    r = {}

    # gyroscope
    plt.figure()
    gx = df['t_{:s}__f_x'.format(topic)]
    gy = df['t_{:s}__f_y'.format(topic)]
    gz = df['t_{:s}__f_z'.format(topic)]
    tau1 = plot_autocorrelation(gx, plot, **corr_args)
    handles, labels = plt.gca().get_legend_handles_labels()
    tau2 = plot_autocorrelation(gy, plot, **corr_args)
    tau3 = plot_autocorrelation(gz, plot, **corr_args)
    plt.title('normalized autocorrelation - {:s}'.format(topic))
    plt.legend(handles, labels, loc='best', ncol=3)
    r['{:s}_randomwalk_correlation_time'.format(topic)] = [tau1, tau2, tau3]

    plt.figure()
    res1 = plot_allan_std_dev(gx, plot, **allan_args)
    handles, labels = plt.gca().get_legend_handles_labels()
    res2 = plot_allan_std_dev(gy, plot, **allan_args)
    res3 = plot_allan_std_dev(gz, plot, **allan_args)
    plt.title('Allan variance plot - {:s}'.format(topic))
    plt.legend(handles, labels, loc='best', ncol=3)
    for key in res1.keys():
        r['{:s}_{:s}'.format(topic, key)] = \
            [d[key] for d in [res1, res2, res3]]
    return r


def noise_analysis_sensor_combined(df, plot=True):
    """
    Given a sensor combined dataset of a stationary vehicle on the ground,
    this compute the noise statistics.
    """
    # pylint: disable=too-many-statements

    r = {}

    # gyroscope
    plt.figure()
    tau1 = plot_autocorrelation(df.t_sensor_combined_0__f_gyro_rad_0_, plot)
    handles, labels = plt.gca().get_legend_handles_labels()
    tau2 = plot_autocorrelation(df.t_sensor_combined_0__f_gyro_rad_1_, plot)
    tau3 = plot_autocorrelation(df.t_sensor_combined_0__f_gyro_rad_2_, plot)
    plt.title('normalized autocorrelation - gyroscope')
    plt.legend(handles, labels, loc='best', ncol=3)
    r['gyroscope_randomwalk_correlation_time'] = [tau1, tau2, tau3]

    plt.figure()
    res1 = plot_allan_std_dev(df.t_sensor_combined_0__f_gyro_rad_0_, plot)
    handles, labels = plt.gca().get_legend_handles_labels()
    res2 = plot_allan_std_dev(df.t_sensor_combined_0__f_gyro_rad_1_, plot)
    res3 = plot_allan_std_dev(df.t_sensor_combined_0__f_gyro_rad_2_, plot)
    plt.title('Allan variance plot - gyroscope')
    plt.legend(handles, labels, loc='best', ncol=3)
    for key in res1.keys():
        r['gyroscope_' + key] = [d[key] for d in [res1, res2, res3]]

    # accelerometer
    plt.figure()
    tau1 = plot_autocorrelation(
        df.t_sensor_combined_0__f_accelerometer_m_s2_0_, plot)
    handles, labels = plt.gca().get_legend_handles_labels()
    tau2 = plot_autocorrelation(
        df.t_sensor_combined_0__f_accelerometer_m_s2_1_, plot)
    tau3 = plot_autocorrelation(
        df.t_sensor_combined_0__f_accelerometer_m_s2_2_, plot)
    plt.title('normalized autocorrelation - accelerometer')
    plt.legend(handles, labels, loc='best', ncol=3)
    r['accelerometer_randomwalk_correlation_time'] = [tau1, tau2, tau3]

    plt.figure()
    res1 = plot_allan_std_dev(
        df.t_sensor_combined_0__f_accelerometer_m_s2_0_, plot)
    handles, labels = plt.gca().get_legend_handles_labels()
    res2 = plot_allan_std_dev(
        df.t_sensor_combined_0__f_accelerometer_m_s2_1_, plot)
    res3 = plot_allan_std_dev(
        df.t_sensor_combined_0__f_accelerometer_m_s2_2_, plot)
    # res = np.array([res1, res2, res3])
    plt.title('Allan variance plot - accelerometer')
    plt.legend(handles, labels, loc='best', ncol=3)
    for key in res1.keys():
        r['accelerometer_' + key] = [d[key] for d in [res1, res2, res3]]

    # magnetometer
    plt.figure()
    tau1 = plot_autocorrelation(
        df.t_sensor_combined_0__f_magnetometer_ga_0_, plot)
    handles, labels = plt.gca().get_legend_handles_labels()
    tau2 = plot_autocorrelation(
        df.t_sensor_combined_0__f_magnetometer_ga_1_, plot)
    tau3 = plot_autocorrelation(
        df.t_sensor_combined_0__f_magnetometer_ga_2_, plot)
    plt.title('normalized autocorrelation - magnetometer')
    plt.legend(handles, labels, loc='best', ncol=3)
    r['magnetometer_randomwalk_correlation_time'] = [tau1, tau2, tau3]

    plt.figure()
    res1 = plot_allan_std_dev(
        df.t_sensor_combined_0__f_magnetometer_ga_0_, plot)
    handles, labels = plt.gca().get_legend_handles_labels()
    res2 = plot_allan_std_dev(
        df.t_sensor_combined_0__f_magnetometer_ga_1_, plot)
    res3 = plot_allan_std_dev(
        df.t_sensor_combined_0__f_magnetometer_ga_2_, plot)
    # res = np.array([res1, res2, res3])
    plt.title('Allan variance plot - magnetometer')
    plt.legend(handles, labels, loc='best', ncol=3)
    for key in res1.keys():
        r['magnetometer_' + key] = [d[key] for d in [res1, res2, res3]]

    # baro
    plt.figure()
    tau = plot_autocorrelation(
        df.t_sensor_combined_0__f_baro_alt_meter,
        plot)
    plt.title('normalized autocorrelation - barometric altimeter')
    plt.legend(loc='best', ncol=3)
    r['baro_randomwalk_correlation_time'] = float(tau)

    plt.figure()
    res = plot_allan_std_dev(
        df.t_sensor_combined_0__f_baro_alt_meter,
        plot)
    plt.title('Allan variance plot - barometric altimeter')
    plt.legend(loc='best', ncol=3)
    for key in res1.keys():
        r['baro_' + key] = res[key]

    return r


def cached_log_processing(
        log, processing_func, msg_filter='',
        save_path='',
        force_processing=False, verbose=False):
    """
    Downsamples a log using mean aggregation and stores as a
    pkl for later loading.

    @param log: ulog to process
    @param processing_func: data = f(data0), how to process data
    @param msg_filter: filter to pass to ulog, '' is all topics
    @param save_path: file path for saved pkl file
    @param force_processing: force processing
    @param verbose: show status messages
    """
    # pylint: disable=too-many-arguments

    if not force_processing and os.path.exists(save_path):
        if verbose:
            print('loading pickle', save_path)
        with open(save_path, 'rb') as f:
            d = pickle.load(f)
    else:
        if verbose:
            print('creating pickle', save_path)
        d0 = read_ulog(log, msg_filter)
        d = processing_func(d0)
        with open(save_path, 'wb') as f:
            pickle.dump(d, f)
    return d


def power_spectrum(x, cross_points=(-1, 0, 1), poly_order=4, freq_max=0.1):
    """
    Plot the power spectrum and print the intersection at various log slopes
    @param x, the data
    @param cross_points, the slopes to print intersections for
    @param freq_max, the max frequency to plot
    @param poly_order, the max order to use when fitting the plot
    """
    data = {}
    x -= x.mean()
    freq, power = scipy.signal.periodogram(
        x.resample('1 s').mean().ffill())
    freq_range = np.logical_and(freq > 1e-4, freq < freq_max)
    power = power[freq_range]
    freq = freq[freq_range]
    log_freq = np.log10(freq)
    log_power = np.log10(power)

    plt.loglog(freq, power)

    p = np.polynomial.Polynomial.fit(log_freq, log_power, poly_order)
    dpdf = p.deriv()
    freq_lin = np.logspace(log_freq.min(), log_freq.max())
    plt.loglog(freq_lin, 10 ** p(np.log10(freq_lin)))

    for cross in cross_points:
        data_cross = []
        roots = (dpdf - cross).roots()
        roots = np.real(roots[np.isreal(roots)])
        vals = p(roots)
        # print('crossing point', cross)
        # print('\troots', 10**roots)
        # print('\tvals', 10**vals)
        for root, val in zip(roots, vals):
            plt.loglog(10 ** root, 10 ** val, 'rx', markeredgewidth=2)
            data_cross += [{'root': 10 ** root, 'val': 10 ** val}]
        data[cross] = data_cross

    # gca().set_xlim([freq.min(), freq.max()])
    # gca().set_ylim([power.min(), power.max()])
    plt.grid()
    plt.title('Power spectrum')
    plt.xlabel('Hz')
    plt.ylabel('Power')

    return data

#  vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 :
