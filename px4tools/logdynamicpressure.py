"""
Analyze ulog for dynamic pressure
"""
from __future__ import print_function

from px4tools import logdynamicpressure as ldp
import pandas
from bokeh.plotting import figure, output_notebook, show
from bokeh.palettes import Spectral11
output_notebook()
import pandas as pd
import numpy as np
from scipy.optimize import curve_fit
import os
import glob


import scipy.optimize
from scipy.interpolate import interp1d
import pandas as pd
import numpy as np

import px4tools

POSCTRL = 2

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
    
    # only consider dataseries of interest
    series = [df.t_sensor_combined_0__f_baro_alt_meter,
              df.t_vehicle_attitude_0__f_q_0_,
              df.t_vehicle_attitude_0__f_q_1_,
              df.t_vehicle_attitude_0__f_q_2_,
              df.t_vehicle_attitude_0__f_q_3_,
              roll, pitch, yaw,
              df.t_vehicle_local_position_0__f_z,
              df.t_vehicle_local_position_0__f_vx,
              df.t_vehicle_local_position_0__f_vy,
              df.t_vehicle_local_position_0__f_vz,
              df.t_vehicle_local_position_setpoint_0__f_z,
              df.t_commander_state_0__f_main_state]
    df = pd.concat(series, axis=1)  
    df = df.dropna()
    return df


def prepare_dynamicaltitude_data(df):
    """
    Only consider data needed for the analysis
    Subtract static baro altitude
    @df dataframe
    @return dataframe dfx, dfy 
    """
    # only consider position control
    df= df[df.t_commander_state_0__f_main_state == 2] 
    
     # consider only constant altitude
    df = df[df.t_vehicle_local_position_setpoint_0__f_z.diff() == 0] 
          
    # only consider parts where acceleration in z is low
    df = df[df.t_vehicle_local_position_0__f_vz.diff().abs() < 2.0]
    
    # we are not interested in time series, shorter names
    d = {'vx': df.t_vehicle_local_position_0__f_vx.values,
         'vy': df.t_vehicle_local_position_0__f_vy.values,
         'vx_b_yaw': df.t_vehicle_local_position_0__f_vx_body_yaw.values,
         'vy_b_yaw': df.t_vehicle_local_position_0__f_vy_body_yaw.values,
         'v_airx': df.t_vehicle_altitude_hold_0__f_v_airx.values,
         'v_airy': df.t_vehicle_altitude_hold_0__f_v_airy.values,
         'v_airx_b_yaw': df.t_vehicle_altitude_hold_0__f_v_airx_body_yaw.values,
         'v_airy_b_yaw': df.t_vehicle_altitude_hold_0__f_v_airy_body_yaw.values,
         'baro_alt': df.t_sensor_combined_0__f_baro_alt_meter.values,
         'z_sp': df.t_vehicle_local_position_setpoint_0__f_z.values}
    df = pd.DataFrame(d)
    
    # group along constant altitude to compute static altitude
    group = df.groupby(df.z_sp) 
    # subtract "static" altitude with assumption that at low speed no dynamic pressure change is present
    for g, data in group:
        df.loc[df.z_sp == g,'baro_alt'] -=  data.baro_alt.mean()   
    return df

def estimate_airspeed(df):
    """
    Estimates airspeed from log
    @df dataframe
    @return dataframe with airspeed estimate
    """
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
        
    velN = np.zeros(len(df))
    velE = np.zeros(len(df))
    
    itp = interp1d(AoS_bp_data,BC_data)
    for idx in range(1,len(df)):
      
      dt = (df.index[idx] - df.index[idx-1]).total_seconds()
      # angle of sideslip
      AoS = angle_wrap(np.arctan2(velE[idx-1], velN[idx-1]) - df.t_vehicle_attitude_0__f_yaw[idx-1])

      # calculate ballistic coeff
      BC = itp(AoS)
       
      # calucalte the drag coefficient
      coef = DENSITY / (2 * BC)
      dragN = - np.sign(velN[idx-1]) * coef * velN[idx-1]**2
      dragE = - np.sign(velE[idx-1]) * coef * velE[idx-1]**2
      
      # calculate the net accerati which is thrust and drag contribution
      netAccelN = accN[idx] + dragN
      netAccelE = accE[idx] + dragE
      
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
    v_airx_b_yaw, v_airy_b_yaw, v_airz_b_yaw = px4tools.series_quatrot_inverse( 
        df.t_vehicle_altitude_hold_0__f_v_airx,
        df.t_vehicle_altitude_hold_0__f_v_airy,
        df.t_vehicle_local_position_0__f_vz * 0,
        qax0,qax1,qax2,qax3,'body_yaw') 
   
    # rotate NED velocity into frame which corresponds to body yaw
    vx_b_yaw, vy_b_yaw, vz_b_yaw = px4tools.series_quatrot_inverse(
         df.t_vehicle_local_position_0__f_vx, 
         df.t_vehicle_local_position_0__f_vy, 
         df.t_vehicle_local_position_0__f_vz,
         qax0,qax1,qax2,qax3,'body_yaw')
    
    # add message
    series = [df]
    series += [vx_b_yaw, vy_b_yaw, v_airx_b_yaw, v_airy_b_yaw]
    df = pd.concat(series, axis=1)
    return df
    
   
def accZ_altitudehold(roll, pitch, msg_name):   
    """
    calculate the z body acceleration required to cancel gravity
    """
    accZ = -GRAVITY / (np.cos(roll) * np.cos(pitch))
    accZ.name = msg_name + '__f_z_body'
    return accZ

def angle_wrap(angle):
    if angle > np.pi:
        return  angle - 2*np.pi;
    if angle < -np.pi:
        return angle + 2*np.pi;
    return angle

  
    
if __name__ == "__main__":  
    
    # ulog filename
    path = 'path to folder with log files'
    files = glob.glob(os.path.join(path,"*.ulg"))
    
    # loop through all files
    df = pd.DataFrame()
    for file in files:
        data = ldp.load_data(file)
        data = ldp.estimate_airspeed(data)
        data = ldp.prepare_dynamicaltitude_data(data)
        df = df.append(data)
    df.index = list(range(len(df))) 
 


   
   

   
   