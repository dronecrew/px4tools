"""
Analysis of px4 logs
"""

#pylint: disable=invalid-name, missing-docstring, no-member, broad-except
#pylint: disable=wildcard-import, unused-wildcard-import

from __future__ import print_function
import pandas
import numpy as np
import matplotlib.pyplot as plt

try:
    from . import mapping
except ImportError as e:
    print(e)

FLIGHT_MODES = [
    'MANUAL', 'ALTCTL', 'POSCTL',
    'AUTO_MISSION', 'AUTO_LOITER', 'AUTO_RTL',
    'ACRO', 'OFFBOARD', 'STAB', 'RATTITUDE',
    'AUTO_TAKEOFF', 'AUTO_LAND',
    'AUTO_FOLLOW_TARGET']

FLIGHT_MODE_COLOR = {
    'MANUAL': 'white',
    'ALTCTL': 'red',
    'POSCTL' : 'green',
    'AUTO_MISSION' : 'blue',
    'AUTO_LOITER' : 'cyan',
    'AUTO_RTL': 'magenta',
    'ACRO' : 'yellow',
    'OFFBOARD' : 'black',
    'STAB' : 'grey',
    'RATTITUDE' : 'brown',
    'AUTO_TAKEOFF' : 'lime',
    'AUTO_LAND' : 'violet',
    'AUTO_FOLLOW_TARGET' : 'orange',
}


def filter_finite(data):

    return np.all(data[np.isfinite(data.values.astype(float))], axis=1)

def octa_cox_data_to_ss(data):
    """
    Extracts state space model data from octa cox log.
    """
    t = pandas.Series((
        data['TIME_StartTime'] -
        data['TIME_StartTime'].values[0])/1.0e6, name='t, sec')
    xh = pandas.DataFrame(
        data[[
            'LPOS_X', 'LPOS_Y', 'LPOS_Z',
            'LPOS_VX', 'LPOS_VY', 'LPOS_VZ',
            'ATT_Roll', 'ATT_Pitch', 'ATT_Yaw',
            'ATT_RollRate', 'ATT_PitchRate', 'ATT_YawRate']].values,
        columns=[
            'X', 'Y', 'Z', 'V_X', 'V_Y', 'V_Z',
            'Phi', 'Theta', 'Psi',
            'P', 'Q', 'R'], index=t)
    y = pandas.DataFrame(
        data[[
            'GPS_Lat', 'GPS_Lon', 'GPS_Alt',
            'SENS_BaroAlt',
            'IMU1_AccX', 'IMU1_AccY', 'IMU1_AccZ',
            'IMU1_GyroX', 'IMU1_GyroY', 'IMU1_GyroZ',
            'IMU1_MagX', 'IMU1_MagY', 'IMU1_MagZ']].values,
        columns=[
            'GPS_Lat', 'GPS_Lon', 'GPS_Alt',
            'Baro_Alt',
            'Acc_X', 'Acc_Y', 'Acc_Z',
            'Gyro_X', 'Gyro_Y', 'Gyro_Z',
            'Mag_X', 'Mag_Y', 'Mag_Z'], index=t)
    u_raw = pandas.DataFrame(
        ((data[[
            'OUT0_Out0', 'OUT0_Out1', 'OUT0_Out2',
            'OUT0_Out3', 'OUT0_Out4', 'OUT0_Out5', 'OUT0_Out6',
            'OUT0_Out7']] - 1000.0)/1000.0).values,
        columns=['1', '2', '3', '4', '5', '6', '7', '8'], index=t)
    C_mix_octo = np.array([
        [1, 1, 1, 1, 1, 1, 1, 1], # thrust
        [-1, 1, 1, -1, -1, 1, 1, -1], # roll
        [-1, -1, 1, 1, -1, -1, 1, 1], # pitch
        [1, -1, 1, -1, 1, -1, 1, -1], # yaw
        ])/8.0
    u = pandas.DataFrame(
        C_mix_octo.dot(u_raw.T).T,
        columns=['thrust', 'roll', 'pitch', 'yaw'],
        index=t)
    return t, xh, u, y, u_raw

def alt_analysis(data, min_alt=None, max_alt=None):
    """
    Altitude analysis.
    """

    i0_Baro = data.SENS_BaroAlt.first_valid_index()
    i0_GPS = data.GPS_Alt.first_valid_index()

    try:
        data.DIST_Distance.plot(legend=True)
    except AttributeError:
        data.DIST_Bottom.plot(legend=True)
    except Exception:
        pass
    (-1*data.LPSP_Z).plot(legend=True)
    (data.SENS_BaroAlt - data.SENS_BaroAlt[i0_Baro]).plot(legend=True)
    (data.GPS_Alt - data.GPS_Alt[i0_GPS]).plot(legend=True)
    (-1*data).LPOS_Z.plot(legend=True)

    import matplotlib.pyplot as plt

    if min_alt is not None and max_alt is not None:
        plt.axis([data.index[0], data.index[-1], min_alt, max_alt])
    plt.grid()
    plt.xlabel('t, sec')
    plt.ylabel('altitude, m')
    background_flight_modes(data)
    plt.legend(loc='best', ncol=3)

def pos_analysis(data):
    """
    Analyze position.
    """
    tmerc_map = mapping.create_map(data.GPS_Lon.values, data.GPS_Lat.values)
    gps_y, gps_x = tmerc_map(data.GPS_Lon.values, data.GPS_Lat.values)
    gpos_y, gpos_x = tmerc_map(data.GPOS_Lon.values, data.GPOS_Lat.values)
    gpsp_y, gpsp_x = tmerc_map(
        data.GPSP_Lon[np.isfinite(data.GPSP_Lon.values)].values,
        data.GPSP_Lat[np.isfinite(data.GPSP_Lat.values)].values)

    import matplotlib.pyplot as plt
    plt.plot(gpos_y, gpos_x, '.', label='est')

    plt.plot(gps_y, gps_x, 'x', label='GPS')

    plt.plot(gpsp_y, gpsp_x, 'ro', label='cmd')

    plt.xlabel('E, m')
    plt.ylabel('N, m')
    plt.grid()
    plt.autoscale(True, 'both', True)
    plt.legend(loc='best')
    return locals()

def isfloatarray(cell):
    """
    Convert cell to float if possible.
    """
    try:
        cell.astype(float)
        return True
    except ValueError:
        return False

def get_float_data(dataframe):
    """
    Get float data out of dataframe.
    """
    dataframe = dataframe[np.isfinite(dataframe.TIME_StartTime)]
    float_cols = [isfloatarray(col) for col in dataframe.values.T]
    return (dataframe.T[float_cols].T).astype(float)

def get_auto_data(data):
    """
    Extract auto data.
    """
    data = data[data['STAT_MainState'] == 3]
    data = data[np.isfinite(data['GPSP_Lat'].astype(float))]
    data = data[np.isfinite(data['GPSP_Lon'].astype(float))]
    if len(data) == 0:
        raise RuntimeError('no auto mode detected')
    return data

def set_time_series(data):
    """
    Set data to use time series
    """
    t = pandas.Series(
        (data['TIME_StartTime'] -
         data['TIME_StartTime'].values[0])/1.0e6, name='t, sec')
    data = pandas.DataFrame(
        data.values,
        columns=data.columns, index=t)
    return data

def process_data(data):
    return set_time_series(get_float_data(data))

def plot_attitude_loops(data):
    """
    Plot attitude loops.
    """
    import matplotlib.pyplot as plt
    plt.subplot(311)
    plt.title('attitude control')
    np.rad2deg(data.ATT_Roll).plot(legend=True)
    np.rad2deg(data.ATSP_RollSP).plot()
    plt.ylabel('roll, deg')
    plt.grid(True)
    background_flight_modes(data)

    plt.subplot(312)
    np.rad2deg(data.ATT_Pitch).plot(legend=True)
    np.rad2deg(data.ATSP_PitchSP).plot()
    plt.grid(True)
    plt.ylabel('pitch, deg')
    background_flight_modes(data)

    plt.subplot(313)
    np.rad2deg(data.ATT_Yaw).plot(legend=True)
    np.rad2deg(data.ATSP_YawSP).plot()
    plt.grid(True)
    plt.ylabel('yaw, deg')
    background_flight_modes(data)

def plot_attitude_rate_loops(data):
    """
    Plot attitude rate control loops.
    """
    import matplotlib.pyplot as plt
    plt.title('attitude rate control')

    plt.subplot(311)
    np.rad2deg(data.ATT_RollRate).plot(legend=True)
    np.rad2deg(data.ARSP_RollRateSP).plot()
    plt.ylabel('roll, deg/s')
    plt.grid(True)
    background_flight_modes(data)

    plt.subplot(312)
    np.rad2deg(data.ATT_PitchRate).plot(legend=True)
    np.rad2deg(data.ARSP_PitchRateSP).plot()
    plt.ylabel('pitch, deg/s')
    plt.grid(True)
    background_flight_modes(data)

    plt.subplot(313)
    np.rad2deg(data.ATT_YawRate).plot(legend=True)
    np.rad2deg(data.ARSP_YawRateSP).plot()
    plt.xlabel('t, sec')
    plt.ylabel('yaw, deg/s')
    plt.grid(True)
    background_flight_modes(data)

def plot_velocity_loops(data):
    """
    Plot velocity loops.
    """
    import matplotlib.pyplot as plt
    plt.subplot(311)
    plt.title('velocity control')
    data.LPOS_VX.plot(legend=True)
    data.LPSP_VX.plot()
    plt.ylabel('x, m/s')
    plt.grid(True)
    background_flight_modes(data)

    plt.subplot(312)
    data.LPOS_VY.plot(legend=True)
    data.LPSP_VY.plot()
    plt.ylabel('y, m/s')
    plt.grid(True)
    background_flight_modes(data)

    plt.subplot(313)
    data.LPOS_VZ.plot(legend=True)
    data.LPSP_VZ.plot()
    plt.xlabel('t, sec')
    plt.ylabel('z, m/s')
    plt.grid(True)
    background_flight_modes(data)

def plot_position_loops(data):
    """
    Plot position loops.
    """
    import matplotlib.pyplot as plt
    plt.subplot(311)
    plt.title('position control')
    data.LPOS_X.plot(legend=True)
    data.LPSP_X.plot()
    plt.ylabel('x, m')
    plt.grid(True)
    background_flight_modes(data)

    plt.subplot(312)
    data.LPOS_Y.plot(legend=True)
    data.LPSP_Y.plot()
    plt.ylabel('y, m')
    plt.grid(True)
    background_flight_modes(data)

    plt.subplot(313)
    data.LPOS_Z.plot(legend=True)
    data.LPSP_Z.plot()
    plt.ylabel('z, m')
    plt.grid(True)
    background_flight_modes(data)


def plot_control_loops(data):
    """
    Plot all control loops.
    """
    plot_attitude_rate_loops(data)
    plot_attitude_loops(data)
    plot_velocity_loops(data)
    plot_position_loops(data)

def statistics(df, keys=None, plot=False):
    data = {}
    for key in keys:
        try:
            df_meas = new_sample(df[key])
            dt = find_meas_period(df_meas)
            stddev = np.sqrt(df_meas.var())
            mean = df_meas.mean()
            variance = stddev**2
            noise_power = variance*dt
        except Exception:
            df_meas = 0
            dt = 0
            stddev = 0
            mean = 0
            noise_power = 0
            variance = 0
        data[key+'_stddev'] = stddev
        data[key+'_variance'] = variance
        data[key+'_mean'] = mean
        data[key+'_dt'] = dt
        data[key+'_noise_power'] = noise_power
        if plot:
            import matplotlib.pyplot as plt
            plt.figure()
            plt.title(key + ' statistics')
            df[key].plot(alpha=0.5)
            plt.hlines(mean, df.index[0], df.index[-1], 'k')
            plt.hlines([mean + stddev, mean - stddev],
                       df.index[0], df.index[-1], 'r')
    return data

def find_lpe_gains(df, printing=False):
    keys = ['GPS_X', 'GPS_Y', 'GPS_Z', 'GPS_VelN', 'GPS_VelE', 'GPS_VelD',
            'DIST_Distance',
            'IMU1_AccX', 'IMU1_AccY', 'IMU1_AccZ', 'SENS_BaroAlt']
    stats = statistics(df, keys)

    params = {
        'LPE_LDR_Z': stats['DIST_Distance_stddev'],
        'LPE_BAR_Z': stats['SENS_BaroAlt_stddev'],
        'LPE_ACC_XY': np.array([
            stats['IMU1_AccX_stddev'],
            stats['IMU1_AccX_noise_power']]).max(),
        'LPE_ACC_Z': stats['IMU1_AccZ_stddev'],
        'LPE_GPS_XY': np.array([
            stats['GPS_X_stddev'],
            stats['GPS_Y_stddev']]).max(),
        'LPE_GPS_Z': stats['GPS_Z_stddev'],
        'LPE_GPS_VXY':  np.array([
            stats['GPS_VelN_stddev'],
            stats['GPS_VelE_stddev']]).max(),
        'LPE_GPS_VZ':  stats['GPS_VelD_stddev']
    }

    if printing:
        for key in params.keys():
            print("{:s}\t=\t{:0.3g}".format(key, params[key]))
    return params

def new_sample(series):
    return series[np.absolute(series.diff()) > 0]

def all_new_sample(df):
    df_new = pandas.DataFrame(df)
    for key in df.keys():
        df_new[key] = new_sample(df[key])
    return df_new

def find_meas_period(series):
    new = new_sample(series)
    return ((new.index[-1] - new.index[0])/
            len(new))

def plot_modes(data):
    import matplotlib.pyplot as plt
    data.STAT_MainState.plot()
    plt.ylim([0, 13])
    plt.yticks(range(0, 13), FLIGHT_MODES)
    background_flight_modes(data)

def background_flight_modes(data):
    """
    Overlays a background color for each flight mode. Can be called to style a graph.
    """
    import matplotlib.pyplot as plt
    modes = np.array(data.STAT_MainState.unique(), np.uint8)
    for m in modes:
        mode_data = data.STAT_MainState[data.STAT_MainState == m]
        mode_name = FLIGHT_MODES[m]
        mode_color = FLIGHT_MODE_COLOR[mode_name]
        t_min = mode_data.index[0]
        t_max = mode_data.index[mode_data.count() - 1]
        plt.axvspan(
            t_min, t_max, alpha=0.1, color=mode_color,
            label=mode_name, zorder=0)

def process_all(data_frame, project_lat_lon=True, lpe_health=True):
    data = process_data(data_frame)
    if project_lat_lon:
        data = mapping.project_lat_lon(data)
    if lpe_health:
        data = process_lpe_health(data)
    return data

def process_lpe_health(data):
    names = ['baro', 'gps', 'lidar', 'flow', 'sonar', 'vision', 'mocap']
    try:
        faults = np.array([[1 if (int(data.EST2_fHealth.values[i]) & 2**j) else 0
                            for j in range(7)]
                           for i in range(len(data.EST2_fHealth.notnull().index))])
        for i in range(7):
            data['fault_' + names[i]] = pandas.Series(
                data=faults[:, i], index=data.index, name='fault ' + names[i])
    except Exception as e:
        print(e)
    try:
        timeouts = np.array([[0 if (int(data.EST0_fTOut.values[i]) & 2**j) else 1
                              for j in range(7)]
                             for i in range(len(data.EST0_fTOut.notnull().index))])
        for i in range(7):
            data['timeout_' + names[i]] = pandas.Series(
                data=timeouts[:, i], index=data.index, name='timeout ' + names[i])
    except Exception as e:
        print(e)
    return data

def plot_faults(data):
    try:
        data.fault_sonar.plot()
        data.fault_baro.plot()
        data.fault_gps.plot()
        data.fault_flow.plot()
        data.fault_vision.plot()
        data.fault_mocap.plot()
        data.fault_lidar.plot()
    except AttributeError as e:
        print(e)
    plt.gca().set_ylim(-1, 2)

def plot_timeouts(data):
    try:
        data.timeout_sonar.plot()
        data.timeout_baro.plot()
        data.timeout_gps.plot()
        data.timeout_flow.plot()
        data.timeout_vision.plot()
        data.timeout_mocap.plot()
        data.timeout_lidar.plot()
    except AttributeError as e:
        print(e)
    plt.gca().set_ylim(-1, 2)


# vim: set et fenc= ft=python ff=unix sts=0 sw=4 ts=4 :
