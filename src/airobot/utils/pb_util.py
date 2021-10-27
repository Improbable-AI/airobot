import functools
import inspect
import os
import pkgutil
import platform
import random
import threading
import time
from numbers import Number

import cv2
import numpy as np
import pybullet as p
import pybullet_data
import airobot
from airobot.utils.common import clamp

GRAVITY_CONST = -9.8


def create_pybullet_client(gui=True,
                           realtime=True,
                           opengl_render=True):
    """
    Create a pybullet simulation client.

    Args:
        gui (bool): use GUI mode or non-GUI mode.
        realtime: use realtime simulation or step simuation.
        opengl_render (bool): use OpenGL (hardware renderer) to render
            RGB images.
    """
    if gui:
        mode = p.GUI
    else:
        mode = p.DIRECT
    pb_client = BulletClient(connection_mode=mode,
                             realtime=realtime,
                             opengl_render=opengl_render)
    pb_client.setAdditionalSearchPath(pybullet_data.getDataPath())
    return pb_client


class BulletClient:
    """
    A wrapper for pybullet to manage different clients.

    Args:
        connection_mode (pybullet mode):
            `None` connects to an existing simulation or, if fails,
            creates a new headless simulation,
            `pybullet.GUI` creates a new simulation with a GUI,
            `pybullet.DIRECT` creates a headless simulation,
            `pybullet.SHARED_MEMORY` connects to an existing simulation.
        realtime (bool): whether to use realtime mode or not.
        opengl_render (bool): use OpenGL (hardware renderer) to render
            RGB images.

    """

    def __init__(self,
                 connection_mode=None,
                 realtime=False,
                 opengl_render=True,
                 options=''):
        self._in_realtime_mode = realtime
        self.opengl_render = opengl_render
        self._realtime_lock = threading.RLock()
        if connection_mode is None:
            self._client = p.connect(p.SHARED_MEMORY, options=options)
            if self._client >= 0:
                return
            else:
                connection_mode = p.DIRECT
        self._client = p.connect(connection_mode, options=options)
        is_linux = platform.system() == 'Linux'
        if connection_mode == p.DIRECT and is_linux and opengl_render:
            airobot.log_info('Load in OpenGL!')
            # # using the eglRendererPlugin (hardware OpenGL acceleration)
            egl = pkgutil.get_loader('eglRenderer')
            if egl:
                p.loadPlugin(egl.get_filename(), "_eglRendererPlugin",
                             physicsClientId=self._client)
            else:
                p.loadPlugin("eglRendererPlugin",
                             physicsClientId=self._client)
        self._gui_mode = connection_mode == p.GUI
        p.setGravity(0, 0, GRAVITY_CONST,
                     physicsClientId=self._client)
        self.set_step_sim(not self._in_realtime_mode)

    def __del__(self):
        """Clean up connection if not already done."""
        if self._client >= 0:
            try:
                p.disconnect(physicsClientId=self._client)
                self._client = -1
            except p.error:
                pass

    def __getattr__(self, name):
        """Inject the client id into Bullet functions."""
        attribute = getattr(p, name)
        if inspect.isbuiltin(attribute):
            attribute = functools.partial(attribute,
                                          physicsClientId=self._client)
        if name == "disconnect":
            self._client = -1
        return attribute

    def get_client_id(self):
        """
        Return the pybullet client id.

        Returns:
            int: pybullet client id.

        """
        return self._client

    def set_step_sim(self, step_mode=True):
        """
        Turn on/off the realtime simulation mode.

        Args:
            step_mode (bool): run the simulation in step mode if
                it is True. Otherwise, the simulation will be
                in realtime.
        """
        self._set_realtime_var(not step_mode)
        if self._gui_mode:
            if step_mode:
                self.setRealTimeSimulation(0)
            else:
                self.setRealTimeSimulation(1)
        else:
            if not step_mode and not hasattr(self, '_direct_real_th'):
                real_th = threading.Thread(target=self._step_rt_simulation)
                self._direct_real_th = real_th
                self._direct_real_th.daemon = True
                self._direct_real_th.start()

    def in_realtime_mode(self):
        """
        Check if the pybullet simulation is in step simulation
        mode or realtime simulation mode.

        Returns:
            bool: whether the pybullet simulation is in step simulation
            mode or realtime simulation mode.

        """
        realtime_mode = self._get_realtime_var()
        return realtime_mode

    def get_body_state(self, body_id):
        """
        Get the body state.

        Args:
            body_id (int): body index.

        Returns:
            4-element tuple containing

            - np.ndarray: x, y, z position of the body (shape: :math:`[3,]`).
            - np.ndarray: quaternion representation ([qx, qy, qz, qw]) of the
              body orientation (shape: :math:`[4,]`).
            - np.ndarray: linear velocity of the body (shape: :math:`[3,]`).
            - np.ndarray: angular velocity of the body (shape: :math:`[3,]`).

        """
        pos, quat = self.getBasePositionAndOrientation(body_id)
        pos = np.array(pos)
        quat = np.array(quat)
        linear_vel, angular_vel = self.getBaseVelocity(body_id)
        linear_vel = np.array(linear_vel)
        angular_vel = np.array(angular_vel)

        return pos, quat, linear_vel, angular_vel

    def reset_body(self, body_id, base_pos,
                   base_quat=None, lin_vel=None, ang_vel=None):
        """
        Reset body to the specified pose and specified initial velocity.

        Args:
            body_id (int): body index.
            base_pos (list or np.ndarray): position [x,y,z] of the body base.
            base_ori (list or np.ndarray): quaternion [qx, qy, qz, qw]
                of the body base.
            lin_vel (list or np.ndarray): initial linear velocity if provided.
            ang_vel (list or np.ndarray): initial angular velocity if provided.

        Returns:

        """
        if base_quat is None:
            base_quat = [0., 0., 0., 1.]
        self.resetBasePositionAndOrientation(body_id, base_pos, base_quat)
        if lin_vel is not None or ang_vel is not None:
            self.resetBaseVelocity(body_id,
                                   linearVelocity=lin_vel,
                                   angularVelocity=ang_vel)

    def remove_body(self, body_id):
        """
        Delete body from the simulation.

        Args:
            body_id (int): body index.

        Returns:
            bool: whether the body is removed.

        """
        self.removeBody(body_id)
        success = False
        try:
            self.getBodyInfo(body_id)
        except Exception:
            success = True
        return success

    def load_urdf(self, filename, base_pos=None,
                  base_ori=None, scaling=1.0, **kwargs):
        """
        Load URDF into the pybullet client.

        Args:
            filename (str): a relative or absolute path to the URDF
                file on the file system of the physics server.
            base_pos (list or np.ndarray): create the base of the object
                at the specified position in world space coordinates [X,Y,Z].
                Note that this position is of the URDF link position.
            base_ori (list or np.ndarray): create the base of the object
                at the specified orientation as world space
                quaternion [X,Y,Z,W].
            scaling (float): apply a scale factor to the URDF model.

        Returns:
            int: a body unique id, a non-negative integer value.
            If the URDF file cannot be loaded, this integer will
            be negative and not a valid body unique id.

        """
        if scaling <= 0:
            raise ValueError('Scaling should be a positive number.')
        if base_pos is None:
            base_pos = [0, 0, 0]
        if base_ori is None:
            base_ori = [0, 0, 0, 1]
        body_id = self.loadURDF(filename,
                                basePosition=base_pos,
                                baseOrientation=base_ori,
                                globalScaling=scaling,
                                **kwargs)
        self.setGravity(0, 0, GRAVITY_CONST)
        return body_id

    def load_sdf(self, filename, scaling=1.0, **kwargs):
        """
        Load SDF into the pybullet client.

        Args:
            filename (str): a relative or absolute path to the SDF
                file on the file system of the physics server.
            scaling (float): apply a scale factor to the SDF model.

        Returns:
            int: a body unique id, a non-negative integer value.
            If the SDF file cannot be loaded, this integer will
            be negative and not a valid body unique id.

        """
        if scaling <= 0:
            raise ValueError('Scaling should be a positive number.')
        body_id = self.loadSDF(filename,
                               globalScaling=scaling,
                               **kwargs)
        self.setGravity(0, 0, GRAVITY_CONST)
        return body_id

    def load_mjcf(self, filename, **kwargs):
        """
        Load SDF into the pybullet client.

        Args:
            filename (str): a relative or absolute path to the MJCF
                file on the file system of the physics server.

        Returns:
            int: a body unique id, a non-negative integer value.
            If the MJCF file cannot be loaded, this integer will
            be negative and not a valid body unique id.

        """
        body_id = self.loadMJCF(filename,
                                **kwargs)
        self.setGravity(0, 0, GRAVITY_CONST)
        return body_id

    def load_geom(self, shape_type, size=None, mass=0.5, visualfile=None,
                  collifile=None, mesh_scale=None, rgba=None,
                  specular=None, shift_pos=None, shift_ori=None,
                  base_pos=None, base_ori=None, no_collision=False, **kwargs):
        """
        Load a regular geometry (`sphere`, `box`,
        `capsule`, `cylinder`, `mesh`).

        Note:
            Please do not call **load_geom('capsule')** when you are using
            **robotiq gripper**. The capsule generated will be in wrong size
            if the mimicing thread (_th_mimic_gripper) in the robotiq
            gripper class starts running.
            This might be a PyBullet Bug (current version is 2.5.6).
            Note that other geometries(box, sphere, cylinder, etc.)
            are not affected by the threading in the robotiq gripper.

        Args:
            shape_type (str): one of [`sphere`, `box`, `capsule`, `cylinder`,
                `mesh`].

            size (float or list): Defaults to None.

                 If shape_type is sphere: size should be a float (radius).

                 If shape_type is capsule or cylinder: size should be
                 a 2-element list (radius, length).

                 If shape_type is box: size can be a float (same half
                 edge length for 3 dims) or a 3-element list
                 containing the half size of 3 edges

                 size doesn't take effect if shape_type is mesh.

            mass (float): mass of the object in kg.
                If mass=0, then the object is static.

            visualfile (str): path to the visual mesh file.
                only needed when the shape_type is mesh. If it's None, same
                collision mesh file will be used as the visual mesh file.

            collifile (str): path to the collision mesh file.
                only needed when the shape_type is mesh. If it's None, same
                viusal mesh file will be used as the collision mesh file.

            mesh_scale (float or list): scale the mesh. If it's a float number,
                the mesh will be scaled in same ratio along 3 dimensions.
                If it's a list, then it should contain 3 elements
                (scales along 3 dimensions).
            rgba (list): color components for red, green, blue and alpha,
                each in range [0, 1] (shape: :math:`[4,]`).
            specular(list): specular reflection color components
                for red, green, blue and alpha, each in
                range [0, 1] (shape: :math:`[4,]`).
            shift_pos (list): translational offset of collision
                shape, visual shape, and inertial frame (shape: :math:`[3,]`).
            shift_ori (list): rotational offset (quaternion [x, y, z, w])
                of collision shape, visual shape, and inertial
                frame (shape: :math:`[4,]`).
            base_pos (list): cartesian world position of
                the base (shape: :math:`[3,]`).
            base_ori (list): cartesian world orientation of the base as
                quaternion [x, y, z, w] (shape: :math:`[4,]`).

        Returns:
            int: a body unique id, a non-negative integer
            value or -1 for failure.

        """
        global GRAVITY_CONST
        pb_shape_types = {'sphere': p.GEOM_SPHERE,
                          'box': p.GEOM_BOX,
                          'capsule': p.GEOM_CAPSULE,
                          'cylinder': p.GEOM_CYLINDER,
                          'mesh': p.GEOM_MESH}
        if shape_type not in pb_shape_types.keys():
            raise TypeError('The following shape type is not '
                            'supported: %s' % shape_type)

        collision_args = {'shapeType': pb_shape_types[shape_type]}
        visual_args = {'shapeType': pb_shape_types[shape_type]}
        if shape_type == 'sphere':
            if size is not None and not (isinstance(size, float) and size > 0):
                raise TypeError('size should be a positive '
                                'float number for a sphere.')
            collision_args['radius'] = 0.5 if size is None else size
            visual_args['radius'] = collision_args['radius']
        elif shape_type == 'box':
            if isinstance(size, float):
                size = [size, size, size]
            elif isinstance(size, list):
                if len(size) != 3:
                    raise ValueError('If size is a list, its length'
                                     ' should be 3 for a box')
            elif size is not None:
                raise TypeError('size should be a float number, '
                                'or a 3-element list '
                                'for a box')
            collision_args['halfExtents'] = [1, 1, 1] if size is None else size
            visual_args['halfExtents'] = collision_args['halfExtents']
        elif shape_type in ['capsule', 'cylinder']:
            if size is not None:
                if not isinstance(size, list) or len(size) != 2:
                    raise TypeError('size should be a 2-element '
                                    'list (radius, length)'
                                    'for a capsule or a cylinder.')
                for si in size:
                    if not isinstance(si, Number) or si <= 0.0:
                        raise TypeError('size should be a list that '
                                        'contains 2 positive'
                                        'numbers (radius, length) for '
                                        'a capsule or '
                                        'a cylinder.')
            collision_args['radius'] = 0.5 if size is None else size[0]
            visual_args['radius'] = collision_args['radius']
            collision_args['height'] = 1.0 if size is None else size[1]
            visual_args['length'] = collision_args['height']
        elif shape_type == 'mesh':
            if visualfile is None and collifile is None:
                raise ValueError('At least one of the visualfile and collifile'
                                 'should be provided!')
            if visualfile is None:
                visualfile = collifile
            elif collifile is None:
                collifile = visualfile
            if not isinstance(visualfile, str):
                raise TypeError('visualfile should be the path to '
                                'the visual mesh file!')
            if not isinstance(collifile, str):
                raise TypeError('collifile should be the path to '
                                'the collision mesh file!')
            collision_args['fileName'] = collifile
            visual_args['fileName'] = visualfile
            if isinstance(mesh_scale, float):
                mesh_scale = [mesh_scale, mesh_scale, mesh_scale]
            elif isinstance(mesh_scale, list):
                if len(mesh_scale) != 3:
                    raise ValueError('If mesh_scale is a list, its length'
                                     ' should be 3.')
            elif mesh_scale is not None:
                raise TypeError('mesh_scale should be a float number'
                                ', or a 3-element list.')
            collision_args['meshScale'] = [1, 1, 1] if mesh_scale is None \
                else mesh_scale
            visual_args['meshScale'] = collision_args['meshScale']
        else:
            raise TypeError('The following shape type is not '
                            'supported: %s' % shape_type)

        visual_args['rgbaColor'] = rgba
        visual_args['specularColor'] = specular
        collision_args['collisionFramePosition'] = shift_pos
        collision_args['collisionFrameOrientation'] = shift_ori
        visual_args['visualFramePosition'] = shift_pos
        visual_args['visualFrameOrientation'] = shift_ori

        self.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        vs_id = self.createVisualShape(**visual_args)

        multi_body_kwargs = dict(
            baseMass=mass,
            baseInertialFramePosition=shift_pos,
            baseInertialFrameOrientation=shift_ori,
            baseVisualShapeIndex=vs_id,
            basePosition=base_pos,
            baseOrientation=base_ori,
            **kwargs
        )
        if not no_collision:
            cs_id = self.createCollisionShape(**collision_args)
            multi_body_kwargs['baseCollisionShapeIndex'] = cs_id
        body_id = self.createMultiBody(**multi_body_kwargs)
        self.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        self.setGravity(0, 0, GRAVITY_CONST)
        return body_id

    def _set_realtime_var(self, realtime_mode):
        with self._realtime_lock:
            self._in_realtime_mode = realtime_mode

    def _get_realtime_var(self):
        with self._realtime_lock:
            realtime_mode = self._in_realtime_mode
        return realtime_mode

    def _step_rt_simulation(self):
        """
        Run step simulation all the time in backend.
        This is only needed to run realtime simulation
        in DIRECT mode, ie, when `render=False`.
        """
        while True:
            realtime_mode = self._get_realtime_var()
            if realtime_mode:
                self.stepSimulation()
            time.sleep(0.001)


class TextureModder:
    """
    Modify textures in model.

    Args:
        pb_client_id (int): pybullet client id.

    Attributes:
        texture_dict (dict): a dictionary that tells the texture
            of a link on a body.
        texture_files (list): a list of texture files (usuallly images).
    """

    def __init__(self, pb_client_id):
        # {body_id: {link_id: [texture_id, height, width]}}
        self.texture_dict = {}
        self.texture_files = []
        self._pb_id = pb_client_id

    def set_texture(self, body_id, link_id, texture_file):
        """
        Apply texture to a link. You can download texture files from:
        https://www.robots.ox.ac.uk/~vgg/data/dtd/index.html.

        Args:
            body_id (int): body index.
            link_id (int): link index in the body.
            texture_file (str): path to the texture files (image, supported
                format: `jpg`, `png`, `jpeg`, `tga`, or `gif` etc.).

        """
        img = cv2.imread(texture_file)
        width = img.shape[1]
        height = img.shape[0]
        tex_id = p.loadTexture(texture_file)
        p.changeVisualShape(body_id, link_id,
                            textureUniqueId=tex_id,
                            physicsClientId=self._pb_id)
        if body_id not in self.texture_dict:
            self.texture_dict[body_id] = {}
        self.texture_dict[body_id][link_id] = [tex_id, height, width]

    def set_texture_path(self, path):
        """
        Set the root path to the texture files. It will search all the files
        in the folder (including subfolders), and find all files that end with
        `.png`, `jpg`, `jpeg`, `tga`, or `gif`.

        Args:
            path (str): root path to the texture files.

        """
        self.texture_files = []
        for root, dirs, files in os.walk(path):
            for name in files:
                if name.lower().endswith(('.png', '.jpg',
                                          '.jpeg', '.tga', '.gif')):
                    self.texture_files.append(os.path.join(root, name))
        print('Number of texture files found: %d' % len(self.texture_files))

    def rand_texture(self, body_id, link_id):
        """
        Randomly apply a texture to the link. Call `set_texture_path` first
        to set the root path to the texture files.

        Args:
            body_id (int): body index.
            link_id (int): link index in the body.

        """
        if len(self.texture_files) < 1:
            raise RuntimeError('Please call `set_texture_path` '
                               'first to set the '
                               'root path to the texture files')
        tex_file = random.choice(self.texture_files)
        self.set_texture(body_id, link_id, tex_file)

    def rand_rgb(self, body_id, link_id):
        """
        Randomize the color of the link.

        Args:
            body_id (int): body index.
            link_id (int): link index in the body.

        """
        rgb = np.random.uniform(size=3).flatten()
        rgba = np.append(rgb, 1)
        self.set_rgba(body_id, link_id, rgba=rgba)

    def rand_gradient(self, body_id, link_id):
        """
        Randomize the gradient of the color of the link.

        Args:
            body_id (int): body index.
            link_id (int): link index in the body.

        """
        rgb1, rgb2 = self._get_rand_rgb(2)
        vertical = bool(np.random.uniform() > 0.5)
        self.set_gradient(body_id, link_id, rgb1, rgb2, vertical)

    def rand_noise(self, body_id, link_id):
        """
        Randomly add noise to the foreground.

        Args:
            body_id (int): body index.
            link_id (int): link index in the body.

        """
        fraction = 0.1 + np.random.uniform() * 0.8
        rgb1, rgb2 = self._get_rand_rgb(2)
        self.set_noise(body_id, link_id, rgb1, rgb2, fraction)

    def rand_all(self, body_id, link_id):
        """
        Randomize color, gradient, noise, texture of the specified link.

        Args:
            body_id (int): body index.
            link_id (int): link index in the body.

        """
        choices = [
            self.rand_gradient,
            self.rand_noise,
            self.rand_rgb,
            # self.rand_texture,
        ]
        choice = np.random.randint(len(choices))
        choices[choice](body_id, link_id)

    def randomize(self, mode='all', exclude=None):
        """
        Randomize all the links in the scene.

        Args:
            mode (str): one of `all`, `rgb`, `noise`, `gradient`.
            exclude (dict): exclude bodies or links from randomization.
                `exclude` is a dict with body_id as the key,
                and a list of link ids as the value. If the value (link ids)
                is an empty list, then all links on the body will be excluded.

        """
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0,
                                   physicsClientId=self._pb_id)
        mode_to_func = {
            'all': self.rand_all,
            'rgb': self.rand_rgb,
            'noise': self.rand_noise,
            'gradient': self.rand_gradient,
            'texture': self.rand_texture,
        }
        body_num = p.getNumBodies(physicsClientId=self._pb_id)
        if exclude is None:
            sep_bodies = set()
        else:
            sep_bodies = set(exclude.keys())
        for body_idx in range(body_num):
            if not self._check_body_exist(body_idx):
                continue
            if body_idx in sep_bodies and not exclude[body_idx]:
                continue
            num_jnts = p.getNumJoints(body_idx,
                                      physicsClientId=self._pb_id)
            # start from -1 for urdf that has no joint but one link
            start = -1 if num_jnts == 0 else 0
            for link_idx in range(start, num_jnts):
                if body_idx in sep_bodies and link_idx in exclude[body_idx]:
                    continue
                mode_to_func[mode](body_idx, link_idx)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1,
                                   physicsClientId=self._pb_id)

    def set_rgba(self, body_id, link_id, rgba):
        """
        Set color to the link.

        Args:
            body_id (int): body index.
            link_id (int): link index in the body.
            rgba (list or np.ndarray): red, green, blue, alpha channel
                (opacity of the color), (shape: :math:`[4,]`).

        """
        p.changeVisualShape(body_id, link_id, rgbaColor=rgba,
                            physicsClientId=self._pb_id)

    def set_gradient(self, body_id, link_id, rgb1, rgb2, vertical=True):
        """
        Creates a linear gradient from rgb1 to rgb2.

        Args:
            body_id (int): body index.
            link_id (int): link index in the body.
            rgb1 (list or np.ndarray): first rgb color (shape: :math:`[3,]`).
            rgb2 (list or np.ndarray): second rgb color (shape: :math:`[3,]`).
            vertical (bool): if True, the gradient in the vertical direction,
                if False it's in the horizontal direction.
        """
        rgb1 = np.array(rgb1).reshape(1, 3)
        rgb2 = np.array(rgb2).reshape(1, 3)
        if not self._check_link_has_tex(body_id, link_id):
            return
        tex_id, height, width = self.texture_dict[body_id][link_id]
        if vertical:
            intp = np.linspace(0, 1, height)
            comp_intp = 1.0 - intp
            rgb_intp = np.multiply(rgb1, intp[:, None])
            rgb_intp += np.multiply(rgb2, comp_intp[:, None])
            new_color = np.repeat(rgb_intp, width, axis=0).flatten()
        else:
            intp = np.linspace(0, 1, width)
            comp_intp = 1.0 - intp
            rgb_intp = np.multiply(rgb1, intp[:, None])
            rgb_intp += np.multiply(rgb2, comp_intp[:, None])
            new_color = np.repeat(rgb_intp[None, :, :], height,
                                  axis=0).flatten()

        new_color = new_color.astype(np.uint8)
        p.changeTexture(tex_id,
                        new_color,
                        width,
                        height,
                        physicsClientId=self._pb_id)

    def set_noise(self, body_id, link_id, rgb1, rgb2, fraction=0.9):
        """
        Apply noise to the texture.

        Args:
            body_id (int): body index.
            link_id (int): link index in the body.
            rgb1 (list or np.ndarray): background rgb color
                (shape: :math:`[3,]`).
            rgb2 (list or np.ndarray): color of random noise
                foreground color (shape: :math:`[3,]`).
            fraction (float): fraction of pixels with
                foreground color.

        """
        if not self._check_link_has_tex(body_id, link_id):
            return
        rgb1 = np.array(rgb1).flatten()
        rgb2 = np.array(rgb2).flatten()
        tex_id, height, width = self.texture_dict[body_id][link_id]
        fraction = clamp(fraction, 0.0, 1.0)

        mask = np.random.uniform(size=(height, width)) < fraction
        new_color = np.tile(rgb1, (height, width, 1))
        new_color[mask, :] = rgb2
        p.changeTexture(tex_id,
                        new_color.flatten(),
                        width,
                        height,
                        physicsClientId=self._pb_id)

    def whiten_materials(self, body_id=None, link_id=None):
        """
        Helper method for setting all material colors to white.

        Args:
            body_id (int): unique body id when you load the body. If body_id
                is not provided, all the bodies will be whitened.
            link_id (int): the index of the link on the body. If link_id is not
                provided and body_id is provided, all the links of the body
                will be whitened.

        """
        if body_id is None:
            body_num = p.getNumBodies(physicsClientId=self._pb_id)
            for body_idx in range(body_num):
                if not self._check_body_exist(body_idx):
                    continue
                num_jnts = p.getNumJoints(body_idx,
                                          physicsClientId=self._pb_id)
                # start from -1 for urdf that has no joint but one link
                start = -1 if num_jnts == 0 else 0
                for i in range(start, num_jnts):
                    self.set_rgba(body_idx, i, rgba=[1, 1, 1, 1])
        else:
            if link_id is None:
                num_jnts = p.getNumJoints(body_id,
                                          physicsClientId=self._pb_id)
                # start from -1 for urdf that has no joint but one link
                start = -1 if num_jnts == 0 else 0
                for i in range(start, num_jnts):
                    self.set_rgba(body_id, i, rgba=[1, 1, 1, 1])
            else:
                self.set_rgba(body_id, link_id, rgba=[1, 1, 1, 1])

    def _get_rand_rgb(self, n=1):
        """
        Get random rgb color in range of [0, 255).

        Args:
            n (int): number of rgb color combinations to be returned.

        Returns:
            One of the following

            - np.ndarray: rgb color (shape: :math:`[3]`).
            - tuple: tuple containing n groups of rgb colors.

        """

        def _rand_rgb():
            return np.array(np.random.uniform(size=3) * 255,
                            dtype=np.uint8)

        if n == 1:
            return _rand_rgb()
        else:
            return tuple(_rand_rgb() for _ in range(n))

    def _check_link_has_tex(self, body_id, link_id):
        """
        Check if the link has texture.

        Args:
            body_id (int): body index.
            link_id (int): link index in the body.

        Returns:
            bool: True if the link has texture, False otherwise.

        """
        if body_id not in self.texture_dict or \
                link_id not in self.texture_dict[body_id]:
            return False
        return True

    def _check_body_exist(self, body_id):
        """
        Check if the body exist in the pybullet client.

        Args:
            body_id (int): body index.

        Returns:
            bool: True if the body exists, False otherwise.

        """
        exist = True
        try:
            p.getBodyInfo(body_id, physicsClientId=self._pb_id)
        except Exception:
            exist = False
        return exist
