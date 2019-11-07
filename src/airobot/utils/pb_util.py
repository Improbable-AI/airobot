import pkgutil
import threading
import time

import pybullet as p
import pybullet_data

PB_CLIENT = None
STEP_SIM_MODE = True
RENDER = False


def load_pb(render=False):
    """
    Load the pybullet client

    Args:
        render (bool): if GUI should be used for rendering or not
    """
    global PB_CLIENT, RENDER
    RENDER = render
    if render:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)
        # # using the eglRendererPlugin (hardware OpenGL acceleration)
        egl = pkgutil.get_loader('eglRenderer')
        if egl:
            p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
        else:
            p.loadPlugin("eglRendererPlugin")
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    PB_CLIENT = p
    if not render:
        th_sim = threading.Thread(target=step_rt_simulation)
        th_sim.daemon = True
        th_sim.start()
    else:
        PB_CLIENT.setRealTimeSimulation(1)


def step_rt_simulation():
    """
    Run step simulation all the time in backend
    """
    global STEP_SIM_MODE, PB_CLIENT
    while True:
        if not STEP_SIM_MODE:
            PB_CLIENT.stepSimulation()
        time.sleep(0.001)


def set_step_sim(step_mode=True):
    """
    Turn on/off the realtime simulation mode

    Args:
        step_mode (bool): run the simulation in step mode if
            it is True. Otherwise, the simulation will be
            in realtime
    """
    global STEP_SIM_MODE
    STEP_SIM_MODE = step_mode
    if RENDER:
        if step_mode:
            PB_CLIENT.setRealTimeSimulation(0)
        else:
            PB_CLIENT.setRealTimeSimulation(1)


def load_urdf(filename, base_pos, base_ori, scaling=1.0, **kwargs):
    """
    Load URDF into the pybullet client

    Args:
        filename (str): a relative or absolute path to the URDF
            file on the file system of the physics server.
        base_pos (list or np.ndarray): create the base of the object
            at the specified position in world space coordinates [X,Y,Z].
            Note that this position is of the URDF link position.
        base_ori (list or np.ndarray): create the base of the object
            at the specified orientation as world space
            quaternion [X,Y,Z,W].
        scaling (float): apply a scale factor to the URDF model

    Returns:
        int: a body unique id, a non-negative integer value.
        If the URDF file cannot be loaded, this integer will
        be negative and not a valid body unique id.

    """
    body_id = PB_CLIENT.loadURDF(filename,
                                 base_pos,
                                 base_ori,
                                 globalScaling=scaling,
                                 **kwargs)
    return body_id
