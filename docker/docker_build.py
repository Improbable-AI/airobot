#!/usr/bin/env python

from __future__ import print_function

import argparse
import os
import shutil


def execute_build(args):
    """
    Create docker build command based on user input arguments
    and execute the command

    Args:
        args (argparse.Namespace): Build arguments
    """
    if not os.path.exists(args.docker_file):
        print('Dockerfile %s not found! Exiting' % args.docker_file)
        return

    # copy requirements file from parent into docker folder
    cwd = os.getcwd()
    shutil.copy(cwd+'/../requirements.txt', cwd)

    cmd = 'DOCKER_BUILDKIT=1 docker build '
    cmd += '--ssh default '
    cmd += '--network=host '
    if not args.cache:
        cmd += '--no-cache '
    cmd += '-t %s -f %s .' % (args.image, args.docker_file)

    print('command = \n\n', cmd)

    if not args.dry_run:
        os.system(cmd)

    # removing copied requirements file from docker/ directory
    os.remove('requirements.txt')


if __name__ == '__main__':

    default_image_name = "airobot-dev"

    parser = argparse.ArgumentParser()

    parser.add_argument('-i', '--image', type=str,
                        default=default_image_name,
                        help='name for new docker image')

    parser.add_argument('-c', '--cache', type=int,
                        default=1,
                        help='0 if should build without using cache')

    parser.add_argument('-f', '--docker_file', type=str,
                        default='ur5e_2f140.dockerfile',
                        help='which Dockerfile to build from')

    parser.add_argument('-d', '--dry_run', type=int,
                        default=0,
                        help='1 if we should only print the build command '
                             'without executing')

    args = parser.parse_args()
    execute_build(args)
