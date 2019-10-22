def pytest_addoption(parser):
    parser.addoption('--robot_name', metavar='robot_name',
                     default='ur5e', help="robot name")
    parser.addoption('--sim_env', metavar='sim_env',
                     default='gazebo',
                     help="simulation environment")
    print(parser)
