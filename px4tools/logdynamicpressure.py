"""
Analyze ulog for dynamic pressure
"""
from __future__ import print_function

import scipy.optimize
from scipy.interpolate import interp1d
import pandas as pd
import numpy as np

import px4tools

POSCTRL = 2
px4topics = ['vehicle_local_position','vehicle_local_position_setpoint',
                 'vehicle_attitude','sensor_combined','commander_state']

GRAVITY = 9.80665 # earth gravity m/s/s
DENSITY = 1.225  #air density kg/m^3
BC_data = [14, 14, 14, 14, 14] # ballistic coefficient lookup table
AoS_bp_data = [-np.pi, -np.pi/2, 0, np.pi/2, np.pi] # Angle of sideslip breakpoint


def load_data(f):
    """
    Read log data from a .ulg file and prepare data for dynamic pressure analysis
    @f global path to file .BulgarianLangModel
    @retun dataframe to do dynamic pressure on  
    """
    # ulog topics needed
    px4topics = ['vehicle_local_position','vehicle_local_position_setpoint',
                 'vehicle_attitude','sensor_combined','commander_state']
    data = px4tools.read_ulog(f, px4topics)
    
    #do resample on setpoint
    df = data.concat(on='t_vehicle_local_position_setpoint_0') 
    
    # to get roll, pitch and yaw
    series = [df]
    msg_att = 't_vehicle_attitude_0'
    roll, pitch, yaw = px4tools.series_quat2euler(
        df.t_vehicle_attitude_0__f_q_0_,
        df.t_vehicle_attitude_0__f_q_1_,
        df.t_vehicle_attitude_0__f_q_2_,
        df.t_vehicle_attitude_0__f_q_3_, msg_att)
    series += [roll, pitch, yaw]
    df = pd.concat(series, axis=1)  
    return df


def prepare_dynamicaltitude_data(df):
    # only consider position control
    df= df[df.t_commander_state_0__f_main_state == 2] 
    
    # z acceleration estimated from vz                           
    acc_z_approx = pd.Series(
            df.t_vehicle_local_position_0__f_vz.diff(), name='t_vehicle_local_position_0__f_az_approx')
    
    # consider only constant altitude
    df = df[df.t_vehicle_local_position_setpoint_0__f_z.diff() == 0] 
       
    # only consider parts where acceleration in z is low
    df = df[acc_z_approx.abs() < 2.0]
    
    # reset index since we dont care about time anymore
    df.index = list(range(len(df))) 
    
    # group along constant altitude to compute static altitude
    group = df.groupby(df.t_vehicle_local_position_setpoint_0__f_z) 
    
    # subtract "static" altitude with assumption that at low speed no dynamic pressure change is present
    for g, data in group: 
            m = data[data.t_vehicle_local_position__0__f_vxy_mag < 5.0].t_sensor_combined_0__f_baro_alt_meter.mean()
            df.ix[df.t_vehicle_local_position_setpoint_0__f_z == g, 't_sensor_combined_0__f_baro_alt_meter'] = \
                df.ix[df.t_vehicle_local_position_setpoint_0__f_z == g, 't_sensor_combined_0__f_baro_alt_meter'].subtract(m)
    
    # split data frame into airspeed body x and y
    dfx = df[np.abs(df.t_vehicle_altitude_hold_0__f_v_airx_body) >= np.abs(df.t_vehicle_altitude_hold_0__f_v_airy_body)] 
    dfy = df[np.abs(df.t_vehicle_altitude_hold_0__f_v_airy+body) >= np.abs(df.t_vehicle_altitude_hold_0__f_v_airx_body)]   
    
    return dfx, dfy

def estimate_airspeed(df):
    """
    Estimates airspeed from log
    @df dataframe
    @return dataframe with airspeed estimate
    """
    series = [df]
    msg_name = 't_vehicle_altitude_hold_0'
    accZ = accZ_altitudehold(df.t_vehicle_attitude_0__f_roll,
                                  df.t_vehicle_attitude_0__f_pitch, msg_name)
    # rotate to NED frame and take horizontal components
    accX = accY = accZ * 0
    accN, accE, accD =  px4tools.series_quatrot(accX, accY, accZ, df.t_vehicle_attitude_0__f_q_0_, 
                                df.t_vehicle_attitude_0__f_q_1_,
                                df.t_vehicle_attitude_0__f_q_2_, 
                                df.t_vehicle_attitude_0__f_q_3_,msg_name)    
    accN.name = msg_name + '__f_accx'
    accE.name = msg_name + '__f_accy'
    
    # add new messages to dataframe
    series += [accN, accE, accD]
    df = pd.concat(series, axis=1)
    df = df.dropna()
    
    velN = np.zeros(len(df))
    velE = np.zeros(len(df))
    
    itp = interp1d(AoS_bp_data,BC_data)
    for idx in range(1,len(df)):
      
      dt = (df.timestamp[idx] - df.timestamp[idx-1]) * 1e-6
      # angle of sideslip
      AoS = px4tools.angle_wrap(np.arctan2(velE[idx-1], velN[idx-1]) - df.t_vehicle_attitude_0__f_yaw[idx -1])
    
      # calculate ballistic coeff
      BC = itp(AoS)
      
      # calucalte the drag coefficient
      coef = DENSITY / (2 * BC)
      dragN = - np.sign(velN[idx-1]) * coef * velN[idx-1]**2
      dragE = - np.sign(velE[idx-1]) * coef * velE[idx-1]**2
      
      # calculate the net accerati which is thrust and drag contribution
      netAccelN = df.t_vehicle_altitude_hold_0__f_accx[idx] + dragN
      netAccelE = df.t_vehicle_altitude_hold_0__f_accy[idx] + dragE
      
      # integrate acceleration to calculate velocity
      velN[idx] = velN[idx-1] + dt * netAccelN
      velE[idx] = velE[idx-1] + dt * netAccelE
    
    # add new messages to dataframe
    df['t_vehicle_altitude_hold_0__f_v_airx'] = pd.Series(velN, index = df.index )
    df['t_vehicle_altitude_hold_0__f_v_airy'] = pd.Series(velE, index = df.index )
    
    #rotate airspeed into frame which corresponds to body yaw
    ax0 = ax1 = df.t_vehicle_local_position_0__f_z * 0
    ax2 = ax0 + 1
    qax0, qax1, qax2, qax3 = px4tools.series_axangle2quat(ax0, ax1, ax2, df.t_vehicle_attitude_0__f_yaw, 'quat_axangle') 
    vx_air_b, vy_air_b, vz_air_b = px4tools.series_quatrot_inverse( 
        df.t_vehicle_altitude_hold_0__f_v_airx,
        df.t_vehicle_altitude_hold_0__f_v_airy,
        df.t_vehicle_local_position_0__f_vz * 0,
        qax0,qax1,qax2,qax3,'body') 
    v_air_mag_hor = pd.Series(np.sqrt(df.t_vehicle_altitude_hold_0__f_v_airx**2 +
                                      df.t_vehicle_altitude_hold_0__f_v_airy**2),name='t_vehicle_altitude_hold_0__f_vxy_mag')
    
    # rotate body 
    vx = df.t_vehicle_local_position_0__f_vx
    vy = df.t_vehicle_local_position_0__f_vy
    vz = df.t_vehicle_local_position_0__f_vz
    q0 = df.t_vehicle_attitude_0__f_q_0_
    q1 = df.t_vehicle_attitude_0__f_q_1_
    q2 = df.t_vehicle_attitude_0__f_q_2_
    q3 = df.t_vehicle_attitude_0__f_q_3_
    vx_r, vy_r, vz_r = px4tools.series_quatrot_inverse(vx, vy, vz, q0, q1, q2, q3, 'body')
    v_mag_hor = pd.Series(np.sqrt(vx**2 + vy**2), name='t_vehicle_local_position__0__f_vxy_mag')
    
    # consider only messages that are needed
    series = [vx, vy, vz, v_mag_hor, vx_r, vy_r, vz_r, 
              df.t_vehicle_altitude_hold_0__f_v_airx,
              df.t_vehicle_altitude_hold_0__f_v_airy, v_air_mag_hor, vx_air_b, vy_air_b,
              df.t_vehicle_local_position_setpoint_0__f_z,
              df.t_sensor_combined_0__f_baro_alt_meter] 
    df = pd.concat(series, axis=1)
    
    return df
    
   
def accZ_altitudehold(roll, pitch, msg_name):   
    """
    calculate the z body acceleration required to cancel gravity
    """
    accZ = -GRAVITY / (np.cos(roll) * np.cos(pitch))
    accZ.name = msg_name + '__f_z_body'
    return accZ
  
    
if __name__ == "__main__":  

    file = '/home/dennis/src/px4Analysis/log/altitude/12_52_01.ulg'
    df = load_data(file)
    df = estimate_airspeed(df)
    
    