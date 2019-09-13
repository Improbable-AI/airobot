# airobot

- [Installation](#installation)
- [Getting Started](#getting-started)

## Installation
```bash
git clone https://github.com/Improbable-AI/airobot.git
cd airobot
python setup.py install
```

## Getting Started
```python
import airobot as ar
robot = ar.create_robot('ur5e', 
                      robot_cfg={'render':True})
robot.go_home()
robot.move_ee_xyz([0.1, 0.1, 0.1])
```