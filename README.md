# AIRobot

- [Installation](#installation)
- [Getting Started](#getting-started)
- [Credits](#credits)
- [License](#license)

## Installation
```bash
git clone https://github.com/Improbable-AI/airobot.git
cd airobot
pip install .
```

## Getting Started
```python
import airobot as ar
robot = ar.create_robot('ur5e',
                        robot_cfg={'render': True})
robot.go_home()
robot.move_ee_xyz([0.1, 0.1, 0.1])
```

## Credits
**AIRobot** is maintained by the Improbable AI team at MIT. Contributors include:
* [Tao Chen](https://taochenshh.github.io/)
* Anthony Simeonov
* [Pulkit Agrawal](https://www.linkedin.com/in/pulkit-agrawal-967a4218/)

**AIRobot** is currently under development and used internally by Prof. Pulkit Agrawal's lab only. Please do not distribute the code publicly without permission from Prof. Pulkit Agrawal.

## License
MIT license