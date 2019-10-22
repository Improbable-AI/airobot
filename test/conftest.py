def pytest_addoption(parser):
    parser.addoption('--robot_name', metavar='robot_name',
                     default='ur5e', help="robot name")
    print(parser)
