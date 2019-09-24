from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


def print_red(skk):
    """
    print the text in red color

    Args:
        skk (str): text to be printed
    """
    print("\033[91m {}\033[00m".format(skk))


def print_green(skk):
    """
    print the text in green color

    Args:
        skk (str): text to be printed
    """
    print("\033[92m {}\033[00m".format(skk))


def print_yellow(skk):
    """
    print the text in yellow color

    Args:
        skk (str): text to be printed
    """
    print("\033[93m {}\033[00m".format(skk))


def print_blue(skk):
    """
    print the text in blue color

    Args:
        skk (str): text to be printed
    """
    print("\033[94m {}\033[00m".format(skk))


def print_purple(skk):
    """
    print the text in purple color

    Args:
        skk (str): text to be printed
    """
    print("\033[95m {}\033[00m".format(skk))


def print_cyan(skk):
    """
    print the text in cyan color

    Args:
        skk (str): text to be printed
    """
    print("\033[96m {}\033[00m".format(skk))
