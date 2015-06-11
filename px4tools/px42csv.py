#!/usr/bin/env python
"""
Converts px4logs to csv.
"""

import os
import subprocess
import re
import argparse


def run_sdlog_dump(file_path):
    """
    Coverts a xp4log to a csv.
    """
    print file_path
    file_out = re.sub('.px4log', '.csv', file_path)
    cmd = 'python sdlog2_dump.py {file_path:s}'\
        ' -f {file_out:s} -e'.format(**locals())
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE)
    out, err = p.communicate()
    if p.returncode != 0:
        raise RuntimeError(err)

def main():
    """
    Entry point for commandline.
    """
    parser = argparse.ArgumentParser(description='Convert all px4logs to csv')
    parser.add_argument('--rm',
            help='remove px4log files if conversion successful')
    args = parser.parse_args()
    for root, dirs, files in os.walk(os.curdir):
        for filename in files:
            if filename.endswith('.px4log'):
                try:
                    file_path = os.path.abspath(os.path.join(root,
                        filename))
                    run_sdlog_dump(file_path)
                    if args.rm:
                        os.remove(file_path)
                except Exception as e:
                    print e


if __name__ == "__main__":
    main()
