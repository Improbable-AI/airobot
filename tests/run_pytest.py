import os
import shutil
import time
import webbrowser
from subprocess import Popen, PIPE, STDOUT

from absl import flags, app

FLAGS = flags.FLAGS

flags.DEFINE_string('out_dir', None, '')
flags.DEFINE_bool('nobrowser', False, 'if True, disable showing the coverage '
                                      'result (if exist) in a web browser')

FNULL = open(os.devnull, 'w')


def launch_ur5e_gazebo(args, wait_time=12):
    """
    Launch UR5e Gazebo simulation

    Args:
        args (str): Command line arguments
        wait_time (float): seconds to wait

    Returns:
        subprocess: the subprocess that runs the gazebo
    """
    print('Launching Gazebo ...')
    args = args.split()
    p = Popen(['roslaunch', 'ur5e_bringup', 'ur5e_start.launch'] + args,
              stdin=PIPE, stdout=FNULL, stderr=STDOUT)
    time.sleep(wait_time)
    return p


def run_test(testing_cmds, html_file=None, show_in_browser=True):
    """
    Run pytest

    Args:
        testing_cmds (list): list of testing commands to run
        html_file (str): filename of the html file that shows
            the pytest results
        show_in_browser (bool): if True, a web browser will be
            automatically opened to show the pytest results
            when tests are finished

    """
    if FLAGS.out_dir is not None:
        html_file = os.path.join(FLAGS.out_dir, html_file)
    for test_file in testing_cmds:
        cmd = ['pytest', '--cov=airobot', '-v']
        if html_file is not None:
            cmd += ['--html={:s}'.format(html_file), '--self-contained-html']
        cmd += ['--cov-append']
        t1 = Popen(cmd + test_file.split())
        t1.wait()
    if show_in_browser and not FLAGS.nobrowser:
        webbrowser.open(html_file)


def gen_html_anno(show_in_browser=True):
    """
    Generate the coverage report

    Args:
        show_in_browser (bool): if True, a web browser will be
            automatically opened to show the coverage report
    """
    if FLAGS.out_dir is not None:
        cov_dir = os.path.join(FLAGS.out_dir, 'coverage')
        p = Popen(['coverage', 'html', '-d', cov_dir])
        p.wait()
        if show_in_browser and not FLAGS.nobrowser:
            webbrowser.open(os.path.join(cov_dir, 'index.html'))
        print('Coverage report generation done!')


def exit_gazebo(gp):
    """

    Args:
        gp (subprocess): the gazebo subprocess

    """
    gp.terminate()
    print('Exiting Gazebo...')
    gp.wait()
    # # `rosnode cleanup` will give error: ERROR: Unable to communicate with master!
    # # if the gazebo is already shutdown correctly
    # # so this error is expected!
    p = Popen(['rosnode', 'cleanup'])
    p.wait()
    print('Gazebo exit successfully!')


def main(argv):
    """
    Run pytest on all test files and generate coverage report
    """
    # # delete old coverage reports
    # # all the coverage reports generated below
    # # will be appended
    cov_file = '.coverage'
    if os.path.exists(cov_file):
        os.remove(cov_file)
    if FLAGS.out_dir is not None:
        if os.path.exists(FLAGS.out_dir):
            shutil.rmtree(FLAGS.out_dir)
        os.makedirs(FLAGS.out_dir)

    p = Popen(['rosnode', 'cleanup'])
    p.wait()

    args = 'sim:=true'
    ur5e_p = launch_ur5e_gazebo(args)
    test_cmds = ['test_ur5e.py --robot_name=ur5e --sim_env=gazebo']
    run_test(test_cmds, 'ur5e_gazebo.html')
    exit_gazebo(ur5e_p)

    test_cmds = ['test_ur5e.py --robot_name=ur5e --sim_env=pybullet']
    run_test(test_cmds, 'ur5e_pybullet.html')

    # # Write put coverage results.
    gen_html_anno()


if __name__ == '__main__':
    app.run(main)
