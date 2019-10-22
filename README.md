# AIRobot

- [Installation](#installation)
- [Getting Started](#getting-started)
- [Credits](#credits)
- [Citation](#citation)
- [Run Tests](#run-tests)
- [License](#license)
- [Contributing](#coding-style)


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
**AIRobot** is maintained by the Improbable AI team at MIT. It follows the same architecture in [PyRobot](https://pyrobot.org). Contributors include:
* [Tao Chen](https://taochenshh.github.io/)
* Anthony Simeonov
* [Pulkit Agrawal](https://www.linkedin.com/in/pulkit-agrawal-967a4218/)

**AIRobot** is currently under development and used internally by Prof. Pulkit Agrawal's lab only. Please do not distribute the code publicly without permission from Prof. Pulkit Agrawal.

## Citation

If you use AIRobot in your research, please use the following BibTeX entry.
```
@misc{chen2019airobot,
  author =       {Tao Chen and Anthony Simeonov and Pulkit Agrawal},
  title =        {{AIRobot}},
  howpublished = {\url{https://github.com/Improbable-AI/airobot}},
  year =         {2019}
}
```

## Run tests
```bash
cd airobot/tests
./run_pytest.sh
```

## License
MIT license

## Coding Style

The python code uses [Flake8](https://pypi.org/project/flake8/) style.




