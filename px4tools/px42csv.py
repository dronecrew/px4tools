#!/usr/bin/env python
"""
Converts px4logs to csv.
"""

from __future__ import print_function
import os
import subprocess
import re
import argparse


def run_sdlog_dump(file_path, sdlog2_path="sdlog2_dump.py"):
    """
    Coverts a xp4log to a csv.
    """
    file_out = re.sub('.px4log', '.csv', file_path)
    cmd = 'python {sdlog2_path:s} {file_path:s}'\
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
            help='remove px4log files if conversion successful',
            action='store_true')
    parser.add_argument('--sdlog2',
            help='path to sdlog2_dump.py')
    parser.set_defaults(rm=False, sdlog2='sdlog2_dump.py')
    args = parser.parse_args()
    for root, dirs, files in os.walk(os.curdir):
        for filename in files:
            if filename.endswith('.px4log'):
                try:
                    file_path = os.path.abspath(os.path.join(root,
                        filename))
                    run_sdlog_dump(file_path, args.sdlog2)
                    if args.rm:
                        os.remove(file_path)
                except Exception as e:
                    print(e)


if __name__ == "__main__":
    main()
