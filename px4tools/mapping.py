"""
Mapping functions
"""

#pylint: disable=invalid-name, missing-docstring, no-member

from __future__ import print_function
from mpl_toolkits.basemap import Basemap
import pandas


def create_map(lon, lat):
    """
    Create a map projection.
    """
    lon_center = lon[0]
    lat_center = lat[0]
    return Basemap(
        lon_0=lon_center,
        lat_0=lat_center, projection='tmerc',
        width=1e-5, height=1e-5)


def project_lat_lon(df):
    gps_map = Basemap(lat_0=df.GPS_Lat.values[0],
            lon_0=df.GPS_Lon.values[0],
            width=11e-5, height=1e-5, projection='tmerc')
    gps_y, gps_x = gps_map(df.GPS_Lon.values, df.GPS_Lat.values)
    gps_z = df.GPS_Alt - df.GPS_Alt.values[0]
    df_new = pandas.DataFrame(pandas.DataFrame({
        'GPS_X': gps_x, 'GPS_Y': gps_y, 'GPS_Z': gps_z}, index=df.index))
    return pandas.concat([df, df_new], axis=1)

# vim: set et fenc= ff=unix sts=0 sw=4 ts=4 :
