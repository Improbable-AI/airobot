import pkgutil

import pybullet as p
import pybullet_data

PB_CLIENT = None


def load_pb(render=False):
    global PB_CLIENT
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
    PB_CLIENT = p
