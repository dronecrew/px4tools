"""
Analyze a PX4 log to perform sysid and control design.
"""
from __future__ import print_function
from collections import OrderedDict

import pandas
import control
import scipy.optimize
import scipy.signal
import numpy as np

import px4tools

# pylint: disable=invalid-name, no-member, too-many-locals, too-many-arguments

def setup_data(df):
    """
    Resample a dataframe at 1 ms to prep for sysid.
    @df pandas DataFrame of px4log
    @return (df_rs, dt) resample dataframe and period (1 ms)
    """
    df = px4tools.get_float_data(df)

    t = pandas.to_timedelta(
        df.TIME_StartTime.values - df.TIME_StartTime.values[0],
        unit='us')
    t += pandas.Timestamp('2016-01-01')
    t = pandas.DatetimeIndex(t)
    df = pandas.DataFrame(df.values,
                          columns=df.columns, index=t, dtype=float)

    # resample dataframe at 1 millisecond
    df_rs = df.resample('1L').mean().interpolate()

    dt = 1.0/1000.0 # resampled at 1 millisecond
    df_rs.index = [i/1.0e3 for i in range(len(df_rs.index))]
    return df_rs, dt

def delay_and_gain_fit_fun(x, y, u, dt):
    """
    Fitness function for dealy_and_gain_sysid
    @x state (k, delay)
    @y output
    @u input
    @dt period (sec)
    @return the fitness cost
    """
    k = x[0]
    delay = x[1]

    # cost at start of interval
    delay1_periods = int(np.floor(delay/dt))
    delay1 = delay1_periods*dt
    uf1 = k*u.shift(delay1_periods)
    err1 = (y - uf1)
    fit1 = (err1**2).sum()

    # cost at end of interval
    delay2_periods = delay1_periods + 1
    delay2 = delay2_periods*dt
    uf2 = k*u.shift(delay2_periods)
    err2 = (y - uf2)
    fit2 = (err2**2).sum()

    # interpolate
    fit = fit1 + (delay - delay1)*(fit2 - fit1)/(delay2 - delay1)
    return fit

def delay_and_gain_sysid(y, u, verbose=False):
    """
    Finds gain and time dellay to best fit output y to input u
    @y output
    @u input
    @return (k, delay)
    """
    dt = 0.001
    k_guess = 154.45
    delay_guess = 0.039
    res = scipy.optimize.minimize(
        delay_and_gain_fit_fun,
        x0=[k_guess, delay_guess],
        bounds=[[0, 1000], [0.001, 0.050]],
        args=(y, u, dt))
    if res['success'] != True:
        print(res)
        raise RuntimeError('optimization failed')
    elif verbose:
        print(res)

    k = res['x'][0]
    delay = res['x'][1]
    fit = calculate_fitness(k, delay, y, u, dt)
    print('fit quality', round(fit*100, 2), '%')
    if fit < 0.75:
        print('WARNING: poor sysid fit')
    return k, delay

def calculate_fitness(k, delay, y, u, dt):
    """
    Find how well the function fits the data
    """
    delay_periods = int(delay/dt)
    e = y - k*u.shift(delay_periods)
    # print('e var', e.var())
    # print('y var', y.var())
    fit = 1 - (e.var()/y.var())**2
    return fit

def plot_delay_and_gain_fit(k, delay, y, u, dt=0.001):
    """
    Plot the delay and gain fit vs the actual output.
    """
    delay_periods = int(delay/dt)
    uf = k*u.shift(delay_periods)
    y.plot()
    uf.plot()

def lqr_ofb_con(K, R, Q, X, ss_o):
    """
    Constraint for LQR output feedback optimization.
    This asserts that all eignvalues are negative so that
    the system is stable.
    @K gain matrix
    @Q process noise covariance matrix
    @X initial state covariance matrix
    @ss_o open loop state space system
    @return constraint
    """
    #pylint: disable=unused-argument
    K = np.matrix(K).T
    A = np.matrix(ss_o.A)
    B = np.matrix(ss_o.B)
    C = np.matrix(ss_o.C)
    A_c = A - B*K*C
    return -np.real(np.linalg.eig(A_c)[0])

def lqr_ofb_cost(K, R, Q, X, ss_o):
    """
    Cost for LQR output feedback optimization.
    @K gain matrix
    @Q process noise covariance matrix
    @X initial state covariance matrix
    @ss_o open loop state space system
    @return cost
    """
    K = np.matrix(K).T
    A = np.matrix(ss_o.A)
    B = np.matrix(ss_o.B)
    C = np.matrix(ss_o.C)
    A_c = A - B*K*C
    Q_c = C.T*K.T*R*K*C + Q
    P = scipy.linalg.solve_lyapunov(A_c.T, -Q_c)
    J = np.trace(P*X)
    return J

def lqr_ofb_jac(K, R, Q, X, ss_o):
    """
    Jacobian for LQR Output feedback optimization.
    TODO: might be an error here, doesn't not help optim
    """
    K = np.matrix(K).T
    A = np.matrix(ss_o.A)
    B = np.matrix(ss_o.B)
    C = np.matrix(ss_o.C)
    A_c = A - B*K*C
    Q_c = C.T*K.T*R*K*C + Q
    P = scipy.linalg.solve_lyapunov(A_c.T, -Q_c)
    S = scipy.linalg.solve_lyapunov(A_c, -X)
    J = 2*(R*K*C*S*C.T - B.T*P*S*C.T)
    J = np.array(J)[:, 0]
    return J

def lqr_ofb_design(K_guess, ss_o, verbose=False):
    """
    LQR output feedback controller design.
    @K_guess initial stabilizing gains
    @ss_o open loop state space system
    @return gain matrix
    """
    n_x = ss_o.A.shape[0]
    n_u = ss_o.B.shape[1]
    R = 1e-6*np.eye(n_u)
    Q = np.eye(n_x)
    X = 1e-3*np.eye(n_x)

    constraints = [
        {'type':'ineq',
         'fun':lqr_ofb_con,
         'args':(R, Q, X, ss_o),
        }
    ]

    res = scipy.optimize.minimize(
        fun=lqr_ofb_cost,
        method='SLSQP',
        args=(R, Q, X, ss_o),
        x0=K_guess,
        # jac=lqr_ofb_jac,
        bounds=[[1e-6, 100], [1e-6, 100], [1e-6, 100]],
        constraints=constraints
        )
    K = np.matrix(res['x'])

    if verbose:
        print(res)
    if  res['success'] != True:
        print('cost', lqr_ofb_cost(K, R, Q, X, ss_o))
        print('jac', lqr_ofb_jac(K, R, Q, X, ss_o))
        print('constraint', lqr_ofb_con(K, R, Q, X, ss_o))
        raise RuntimeError('optimization failed')

    return K.T


def attitude_sysid(y_acc, u_mix, verbose=False):
    """
    roll/pitch system id assuming delay/gain model
    @y_acc (roll or pitch acceleration)
    @u_mix (roll or pitch acceleration command)

    G_ol: open loop plant
    delay: time delay (sec)
    k: gain
    @return (G_ol, delay, k)
    """
    if verbose:
        print('solving for plant model ', end='')
    k, delay = delay_and_gain_sysid(y_acc, u_mix, verbose)
    if verbose:
        print('done')


    # open loop, rate output, mix input plant
    tf_acc = k*control.tf(*control.pade(delay, 1)) # order 1 approx
    # can neglect motor, pole too fast to matter compared to delay, not used in sysid either
    tf_integrator = control.tf((1), (1, 0))
    G_ol = tf_acc * tf_integrator

    return G_ol, delay, k


def attitude_rate_design(G, K_guess, d_tc, verbose=False):
    """
    @G transfer function
    @K_guess gain matrix guess
    @d_tc time constant for derivative

    K: gain matrix
    G_comp: open loop compensated plant
    Gc_comp: closed loop compensated plant
    @return (K, G_comp, Gc_comp)
    """
    # compensator transfer function
    H = np.array([[
        control.tf(1, 1),
        control.tf((1), (1, 0)), control.tf((1, 0), (d_tc, 1))]]).T
    H_num = [[H[i][j].num[0][0] for i in range(H.shape[0])] for j in range(H.shape[1])]
    H_den = [[H[i][j].den[0][0] for i in range(H.shape[0])] for j in range(H.shape[1])]
    H = control.tf(H_num, H_den)

    ss_open = control.tf2ss(G*H)

    if verbose:
        print('optimizing controller')
    K = lqr_ofb_design(K_guess, ss_open, verbose)
    if verbose:
        print('done')

    G_comp = control.series(G, H*K)
    Gc_comp = G_comp/(1 + G_comp)

    return K, G_comp, Gc_comp

def plot_attitude_rate_design(name, G_ol, G_cl):
    """
    Plot attitude rate design
    @name Name of axis (e.g. roll/pitch)
    @G_ol open loop transfer function
    @G_cl closed loop transfer function
    """
    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(*control.step_response(G_cl, np.linspace(0, 1, 1000)))
    plt.title(name + ' step resposne')
    plt.grid()

    plt.figure()
    control.bode(G_ol)
    print('margins', control.margin(G_ol))
    plt.subplot(211)
    plt.title(name + ' open loop bode plot')

    plt.figure()
    control.rlocus(G_ol, np.logspace(-2, 0, 1000))
    for pole in G_cl.pole():
        plt.plot(np.real(pole), np.imag(pole), 'rs')
    plt.title(name + ' root locus')
    plt.grid()

def attitude_control_design(
        name, y, u, rolling_mean_window=100,
        do_plot=False, verbose=False):
    """
    Do sysid and control design for roll/pitch rate loops.
    """
    K_guess = np.matrix([[0.01, 0.01, 0.001]]).T

    d_tc = 1.0/125 # nyquist frequency of derivative in PID, (250 Hz/2)

    # remove bias
    y_bias = y.rolling(rolling_mean_window).mean()
    y_debiased = y - y_bias
    u_bias = u.rolling(rolling_mean_window).mean()
    u_debiased = u - u_bias

    G_ol, delay, k = attitude_sysid(
        y_debiased, u_debiased, verbose)

    K, G_ol_rate, G_cl_rate = attitude_rate_design(
        G_ol, K_guess, d_tc, verbose)

    if do_plot:
        import matplotlib.pyplot as plt

        plt.figure()
        u_debiased.plot(label='debiased input')
        u.plot(label='input')
        plt.legend()
        plt.xlabel('t, sec')
        plt.title(name + ' input')
        plt.grid()

        plt.figure()
        y_debiased.plot(label='debiased output')
        y.plot(label='output')
        plt.legend()
        plt.xlabel('t, sec')
        plt.title(name + ' output')
        plt.grid()

        plt.figure()
        plot_delay_and_gain_fit(k, delay, y_debiased, u_debiased)
        plt.title(name + ' fit')
        plt.grid()

        plt.figure()
        plot_attitude_rate_design(name, G_ol_rate, G_cl_rate)

    return K

def control_design(raw_data, do_plot=False, rolling_mean_window=100):
    """
    Design a PID controller from log file.
    """
    data, dt = setup_data(raw_data)

    roll_acc = data.ATT_RollRate.diff()/dt
    K_roll = attitude_control_design(
        'roll rate', roll_acc, data.ATTC_Roll,
        rolling_mean_window=rolling_mean_window, do_plot=do_plot)

    pitch_acc = data.ATT_PitchRate.diff()/dt
    K_pitch = attitude_control_design(
        'pitch rate', pitch_acc, data.ATTC_Pitch,
        rolling_mean_window=rolling_mean_window, do_plot=do_plot)

    return OrderedDict([
        ('MC_ROLLRATE_P', round(K_roll[0, 0], 3)),
        ('MC_ROLLRATE_I', round(K_roll[1, 0], 3)),
        ('MC_ROLLRATE_D', round(K_roll[2, 0], 3)),
        ('MC_PITCHRATE_P', round(K_pitch[0, 0], 3)),
        ('MC_PITCHRATE_I', round(K_pitch[1, 0], 3)),
        ('MC_PITCHRATE_D', round(K_pitch[2, 0], 3)),
    ])
