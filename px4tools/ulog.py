"""
ulog2pandas converter
"""

import os
import tempfile
import re
import glob
import numpy as np

import pandas
import pyulog

def ulog2pandas(ulog_filename, verbose=False):
    """
    Convert ulog to pandas dataframe.
    """

    pyulog.ulog2csv.convert_ulog2csv(
        ulog_filename, '', tempfile.tempdir, ',')
    log_name = os.path.splitext(os.path.basename(ulog_filename))[0]
    df = None
    glob_expr = '{:s}*.csv'.format(
        os.path.join(tempfile.gettempdir(), log_name))
    
    # column naming
    d_col_rename = {
        '[': '_',
        ']': '_',
    }
    col_rename_pattern = re.compile(
        r'(' + '|'.join([ re.escape(key) for key in d_col_rename.keys()]) + r')')

    for file in sorted(glob.glob(glob_expr)):
        if verbose:
            print('processing', file)
        file_name = os.path.splitext(os.path.basename(file))[0]
        topic_name = file_name.replace(log_name + '_', '')
        try:
            # read data
            df_new = pandas.read_csv(file, index_col=0)
            df_new.columns = [ topic_name + '_' + col_rename_pattern.sub(
                    lambda x: d_col_rename[x.group()], col)
                for col in df_new.columns]

            # indexing
            #df_new.index = pandas.Index(data=[float(f)/1.0e6 for f in df_new.index], dtype=float)
            #df_new.index.name = 't, sec'

            # append
            if df is None:
                df = df_new
            else:
                df = df.append(df_new)
        except Exception as e:
            print(e)

    df.index = np.array(df.index/1.0e6, dtype=np.float32)
    df.index_name = 't, sec'
    if verbose:
        print(log_name, 'data loaded')
    return df

def time_range(data, t1, t2):
    d1 = data[data.index >= t1]
    return d1[d1.index <= t2]

#  vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 : 
