# AIRobot

AIRobot is a python library to interface with robots. It follows a similar architecture from [PyRobot](https://pyrobot.org).
- [Citation](#citation)
- [Installation](#installation)
- [Supported Robots](#supported-robots)
- [Getting Started](#getting-started)
- [Credits](#credits)
- [Build API Docs](#build-api-docs)
- [Run Tests](#run-tests)
- [License](#license)
- [Coding Style](#coding-style)

## Citation

If you use AIRobot in your research, please use the following BibTeX entry.
```
@misc{airobot2019,
  author =       {Tao Chen and Anthony Simeonov and Pulkit Agrawal},
  title =        {{AIRobot}},
  howpublished = {\url{https://github.com/Improbable-AI/airobot}},
  year =         {2019}
}
```

## Installation

### Pre-installation steps for ROS 

If you want to use ROS to interface with robots, please install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) first.

If you want to use ROS for UR5e robots, please install [ur5e driver](https://github.com/Improbable-AI/ur5e_robotiq_2f140). 

If you are using RealSense cameras, please follow the instructions [here](https://github.com/IntelRealSense/realsense-ros#installation-instructions) or [here](https://github.com/Improbable-AI/camera_calibration/tree/qa).

### Install AIRobot

You might want to install it in a virtual environment. 

If you are using ROS, you can use [virtualenv](https://virtualenv.pypa.io/en/latest/installation/). Note that Anaconda doesn't work well with ROS. And only Python 2.7 is recommended for ROS at this point.

If you only want to use the robots in the PyBullet simulation environment, then you can use Python 2.7 or Python 3.7. And you can use either [virtualenv for Python 2.7](https://virtualenv.pypa.io/en/latest/installation/), [venv for Python 3.7](https://docs.python.org/3.7/tutorial/venv.html) or [Anaconda](https://docs.anaconda.com/anaconda/install/linux/).

```bash
git clone https://github.com/Improbable-AI/airobot.git
cd airobot
pip install -e .
```

## Supported Robots
* [UR5e](https://www.universal-robots.com/products/ur5-robot/) with ROS
* [UR5e](https://www.universal-robots.com/products/ur5-robot/) in PyBullet
* [ABB YuMi](https://new.abb.com/products/robotics/industrial-robots/irb-14000-yumi) in PyBullet

If you want to use [Sawyer](https://www.rethinkrobotics.com/sawyer) or [LoCoBot](https://locobot-website.netlify.com/), you can use [PyRobot](https://pyrobot.org).

## Getting Started
A sample script is shown below. You can find more examples [here](https://github.com/Improbable-AI/airobot/examples)

```python
from airobot import Robot
# create a UR5e robot in pybullet
robot = Robot('ur5e',
              pb=True,
              arm_cfg={'render': True})
robot.arm.go_home()
robot.arm.move_ee_xyz([0.1, 0.1, 0.1])
```

## Credits
**AIRobot** is maintained by the Improbable AI team at MIT. It follows a similar architecture in [PyRobot](https://pyrobot.org). Contributors include:
* [Tao Chen](https://taochenshh.github.io/)
* Anthony Simeonov
* [Pulkit Agrawal](http://people.csail.mit.edu/pulkitag/)

**AIRobot** is currently under development and used internally by Prof. Pulkit Agrawal's lab only. Please do not distribute the code publicly without permission from Prof. Pulkit Agrawal.


## Build API Docs

Run the following commands to build the API webpage.

```bash
cd docs
./make_api.sh
```

Then you can use any web browser to open the API doc (**`docs/build/html/index.html`**)

## Run tests

[pytest](https://docs.pytest.org/en/latest/) is used for unit tests. TODO: more test cases need to be added.
```bash
cd airobot/tests
./run_pytest.sh
```

## License
MIT license

## Coding Style

AIRobot uses [Google style](https://sphinxcontrib-napoleon.readthedocs.io/en/latest/example_google.html) for formatting docstrings. We use [Flake8](https://pypi.org/project/flake8/) to perform additional formatting and semantic checking of code.




