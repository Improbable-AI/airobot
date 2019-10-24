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

    args = parser.parse_args()

    # copy requirements file from parent into docker folder
    cwd = os.getcwd()
    shutil.copy(cwd+'/../requirements.txt', cwd)

    cmd = 'DOCKER_BUILDKIT=1 docker build '
    cmd += '-t %s ' % args.image
    cmd += '--ssh default .'

    print('command = \n\n', cmd)

    os.system(cmd)
