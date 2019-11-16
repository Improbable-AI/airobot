#!/usr/bin/env python

from __future__ import print_function

import argparse
import os
import getpass
import shutil

if __name__ == '__main__':

    user_name = getpass.getuser()
    default_image_name = "airobot-dev"

    parser = argparse.ArgumentParser()

    parser.add_argument('--image', type=str,
                        help="name for new docker image",
                        default=default_image_name)

    parser.add_argument('--cache', type=int, default=1,
                        help="0 if should build without using cache")

    args = parser.parse_args()

    # copy requirements file from parent into docker folder
    cwd = os.getcwd()
    shutil.copy(cwd+'/../requirements.txt', cwd)

    cmd = 'DOCKER_BUILDKIT=1 docker build '
    cmd += '-t %s ' % args.image
    cmd += '--ssh default '
    cmd += '--network=host '
    if not args.cache:
        cmd += '--no-cache '
    cmd += '.'

    print('command = \n\n', cmd)

    os.system(cmd)
