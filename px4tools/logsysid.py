from __future__ import print_function
import pandas
import px4tools
import control
import scipy.optimize
import scipy.signal
import time
import numpy as np
from datetime import date

def setup_data(df):
    df = px4tools.get_float_data(df)

    t = pandas.to_timedelta(
        df.TIME_StartTime.values - df.TIME_StartTime.values[0],
        unit='us')
    t += pandas.Timestamp('2016-01-01')
    t = pandas.DatetimeIndex(t)
    df = pandas.DataFrame(df.values,
                          columns=df.columns, index=t, dtype=float)
    
    # resample dataframe at 1 millisecond
    df_rs = df['2016-01-01 00:00:00':
               '2016-01-01 00:01:04'].resample('1L').mean().interpolate()

    dt = 1.0/1000.0 # resampled at 1 millisecond
    df_rs.index = [ i/1.0e3 for i in range(len(df_rs.index))]
    return df_rs, dt

def delay_and_gain_fit_fun(x, y, u, dt):
    k = x[0]
    delay = x[1]

    # cost at start of interval
    delay1_periods = np.floor(delay/dt)
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

def delay_and_gain_sysid(y, u):
    dt = 0.001
    k_guess = 154.45
    delay_guess = 0.039
    res = scipy.optimize.minimize(delay_and_gain_fit_fun,
            x0=[k_guess, delay_guess],
            bounds=[[0, 1000], [0.001, 0.050]],
            args=(y, u, dt))
    if res['success'] != True:
        print(res)
    k = res['x'][0]
    delay = res['x'][1]
    return k, delay

def plot_delay_and_gain_fit(k, delay, y, u, dt=0.001):
    delay_periods = delay/dt
    uf = k*u.shift(delay_periods)
    y.plot()
    uf.plot()

def lqr_ofb_con(K, R, Q, X, ss_o):
    K = np.matrix(K).T
    A = np.matrix(ss_o.A)
    B = np.matrix(ss_o.B)
    C = np.matrix(ss_o.C)
    A_c = A - B*K*C
    return -np.real(np.linalg.eig(A_c)[0])

def lqr_ofb_cost(K, R, Q, X, ss_o):
    K = np.matrix(K).T
    A = np.matrix(ss_o.A)
    B = np.matrix(ss_o.B)
    C = np.matrix(ss_o.C)
    A_c = A - B*K*C
    Q_c = C.T*K.T*R*K*C + Q
    P = scipy.linalg.solve_lyapunov(A_c.T, -Q_c)
    S = scipy.linalg.solve_lyapunov(A_c, -X)
    J = np.trace(P*X)
    return J

def lqr_ofb_jac(K, R, Q, X, ss_o):
    K = np.matrix(K).T
    A = np.matrix(ss_o.A)
    B = np.matrix(ss_o.B)
    C = np.matrix(ss_o.C)
    A_c = A - B*K*C
    Q_c = C.T*K.T*R*K*C + Q
    P = scipy.linalg.solve_lyapunov(A_c.T, -Q_c)
    S = scipy.linalg.solve_lyapunov(A_c, -X)
    J = 2*(R*K*C*S*C.T - B.T*P*S*C.T)
    J = np.array(J)[:,0]
    return J

def lqr_ofb_design(K_guess, ss_o, i_max=2000):
    K_k = K_guess
    n_x = ss_o.A.shape[0]
    n_u = ss_o.B.shape[1]
    R = 1e-6*np.eye(n_u)
    Q = np.eye(n_x)
    X = 1e-3*np.eye(n_x)

    constraints=[
        {'type': 'ineq',
         'fun': lqr_ofb_con,
         'args': (R, Q, X, ss_o),
        }
    ]

    res = scipy.optimize.minimize(fun=lqr_ofb_cost,
        method='SLSQP',
        args=(R, Q, X, ss_o),
        x0=K_guess,
        # jac=lqr_ofb_jac,
        bounds=[[1e-6, 100], [1e-6, 100], [1e-6, 100]],
        constraints=constraints
        )
    K = np.matrix(res['x'])

    print('cost', lqr_ofb_cost(K, R, Q, X, ss_o))
    print('jac', lqr_ofb_jac(K, R, Q, X, ss_o))
    print('constraint', lqr_ofb_con(K, R, Q, X, ss_o))

    if  res['success'] != True:
        print('optimization failed')
        print(res)

    return np.matrix(res['x']).T


def attitude_sysid(y_acc, u_mix):
    print('solving for plant model ', end='')
    k, delay = delay_and_gain_sysid(y_acc, u_mix)
    print('done')


    # open loop, rate output, mix input plant
    tf_acc = k*control.tf(*control.pade(delay, 1)) # order 1 approx
    # can neglect motor, pole too fast to matter compared to delay, not used in sysid either
    tf_integrator = control.tf((1), (1, 0))
    G_ol = tf_acc * tf_integrator

    return G_ol, delay, k


def attitude_rate_design(G, K_guess, d_tc):
    # compensator transfer function
    H = np.array([[control.tf(1, 1),
        control.tf((1), (1, 0)), control.tf((1, 0), (d_tc, 1))]]).T
    H_num = [[ H[i][j].num[0][0] for i in range(H.shape[0])] for j in range(H.shape[1])]
    H_den = [[ H[i][j].den[0][0] for i in range(H.shape[0])] for j in range(H.shape[1])]
    H = control.tf(H_num, H_den)

    ss_open = control.tf2ss(G*H)

    print('optimizing controller')
    K = lqr_ofb_design(K_guess, ss_open)
    print('done')

    G_comp = control.series(G, H*K)
    Gc_comp = G_comp/(1 + G_comp)

    return K, G_comp, Gc_comp

def plot_attitude_rate_design(name, G_ol, G_cl):
    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(*control.step_response(G_cl, np.linspace(0, 1, 1000)))
    plt.title(name + ' rate step resposne')

    plt.figure()
    control.bode(G_ol);
    print(control.margin(G_ol))

    plt.figure()
    control.rlocus(G_ol, np.logspace(-2, 0, 1000));
    for pole in G_cl.pole():
        plt.plot(np.real(pole), np.imag(pole), 'rs')
    plt.title(name + ' rate step root locus')

def control_design():


    # load log file data, resample at 1 millisecond
    data, dt = setup_data(pandas.read_csv('./sess052/log001.csv'))

    K_roll, G_ol_roll_rate, G_cl_roll_rate = attitude_rate_design(
        data.ATT_RollRate.diff()/dt, data.ATTC_Roll)
    plot_attitude_rate_design('roll', G_ol_roll_rate, G_cl_roll_rate)

    K_pitch, G_ol_pitch_rate, G_cl_pitch_rate = attitude_rate_design(
        data.ATT_PitchRate.diff()/dt, data.ATTC_Pitch)
    plot_attitude_rate_design('pitch', G_ol_pitch_rate, G_cl_pitch_rate)
