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
import pandas
import pyulog

try:
    import transforms3d.taitbryan as tf
except ImportError as ex:
    print(ex)
    print('please install transforms3d: pip install transforms3d')

def ulog2pandas(ulog_filename, verbose=False):
    """
    Convert ulog to pandas dataframe.
    """

    pyulog.ulog2csv.convert_ulog2csv(
        ulog_filename, '', tempfile.tempdir, ',')
    log_name = os.path.splitext(os.path.basename(ulog_filename))[0]
    data = []
    topic_names = []
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
        data_new = pandas.read_csv(file, index_col=0)
        data_new.index = pandas.to_datetime(data_new.index, unit='us')
        data_new.columns = [
            'f_' + col_rename_pattern.sub(
                lambda x: d_col_rename[x.group()], col)
            for col in data_new.columns
        ]

        data += [data_new]
        topic_names += [topic_name]

    if verbose:
        print(log_name, 'data loaded')

    return collections.namedtuple(
        'px4log',
        ['t_' + tn for tn in topic_names])(*data)

def plot_euler_angles(df):
    """
    Plot euler angles
    """

    q0 = df.t_vehicle_attitude_0.f_q_0_
    q1 = df.t_vehicle_attitude_0.f_q_1_
    q2 = df.t_vehicle_attitude_0.f_q_2_
    q3 = df.t_vehicle_attitude_0.f_q_3_

    have_gt = False
    try:
        q0_gt = df.t_vehicle_attitude_groundtruth_0.f_q_0_
        q1_gt = df.t_vehicle_attitude_groundtruth_0.f_q_1_
        q2_gt = df.t_vehicle_attitude_groundtruth_0.f_q_2_
        q3_gt = df.t_vehicle_attitude_groundtruth_0.f_q_3_
        psi_gt, theta_gt, phi_gt = np.array([
            tf.quat2euler([q0i, q1i, q2i, q3i]) for
            q0i, q1i, q2i, q3i in zip(q0_gt, q1_gt, q2_gt, q3_gt)]).T
        have_gt = True
    except Exception:
        pass

    t = q0.index
    psi, theta, phi = np.array([
        tf.quat2euler([q0i, q1i, q2i, q3i]) for
        q0i, q1i, q2i, q3i in zip(q0, q1, q2, q3)]).T

    plt.plot(t, np.rad2deg(phi), label='roll', color='r')
    if have_gt:
        plt.plot(t, np.rad2deg(phi_gt), label='roll-true', color='r', linestyle='--')
    plt.plot(t, np.rad2deg(theta), label='pitch', color='g')
    if have_gt:
        plt.plot(t, np.rad2deg(theta_gt), label='pitch-true', color='g', linestyle='--')
    plt.plot(t, np.rad2deg(psi), label='yaw', color='b')
    if have_gt:
        plt.plot(t, np.rad2deg(psi_gt), label='yaw-true', color='b', linestyle='--')
    plt.legend(ncol=3, loc='best')
    plt.grid()

def plot_quaternions(df):
    """
    Plot quaternions and groundtruth
    """

    q0 = df.t_vehicle_attitude_0.f_q_0_
    q0.plot(color='r', label='q0')
    try:
        q0_gt = df.t_vehicle_attitude_groundtruth_0.f_q_0_
        q0_gt.plot(color='r', style='--', label='q0-true')
    except Exception:
        pass

    q1 = df.t_vehicle_attitude_0.f_q_1_
    q1.plot(color='g', label='q1')
    try:
        q1_gt = df.t_vehicle_attitude_groundtruth_0.f_q_1_
        q1_gt.plot(color='g', style='--', label='q1-true')
    except Exception:
        pass

    q2 = df.t_vehicle_attitude_0.f_q_2_
    q2.plot(color='b', label='q2')
    try:
        q2_gt = df.t_vehicle_attitude_groundtruth_0.f_q_2_
        q2_gt.plot(color='b', style='--', label='q2-true')
    except Exception:
        pass

    q3 = df.t_vehicle_attitude_0.f_q_3_
    q3.plot(color='k', label='q3')
    try:
        q3_gt = df.t_vehicle_attitude_groundtruth_0.f_q_3_
        q3_gt.plot(color='k', style='--', label='q3-true')
    except Exception:
        pass

    plt.legend(ncol=4, loc='best')
    plt.xlabel('time')
    plt.ylabel('q')
    plt.title('quaternions')
    plt.grid()

def plot_quaternion_normal(df):
    """
    plot the quaternion normal
    """
    q0 = df.t_vehicle_attitude_0.f_q_0_
    q1 = df.t_vehicle_attitude_0.f_q_1_
    q2 = df.t_vehicle_attitude_0.f_q_2_
    q3 = df.t_vehicle_attitude_0.f_q_3_

    plt.figure(figsize=(10, 10))
    np.sqrt(q0**2 + q1**2 + q2**2 + q3**2).plot()
    plt.title('quaternion norm')
    plt.grid()
    plt.xlabel('time')
    plt.ylabel('magnitude')

#  vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 :
