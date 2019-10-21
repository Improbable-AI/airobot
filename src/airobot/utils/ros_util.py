import rospy
import tf
import PyKDL as kdl
import numpy as np

def kdl_array_to_numpy(kdl_data):
    """
    Convert KDL array data into numpy array

    Args:
        kdl_data: data in KDL format

    Returns:
        numpy array
    """
    np_array = np.zeros((kdl_data.rows(), kdl_data.columns()))
    for i in range(kdl_data.rows()):
        for j in range(kdl_data.columns()):
            np_array[i, j] = kdl_data[i, j]
    return np_array


def kdl_frame_to_numpy(frame):
    """
    Convert KDL Frame data into numpy array

    Args:
        frame: data of KDL Frame

    Returns:
        transformation matrix in numpy [4x4]
    """
    p = frame.p
    M = frame.M
    return np.array([[M[0, 0], M[0, 1], M[0, 2], p.x()],
                     [M[1, 0], M[1, 1], M[1, 2], p.y()],
                     [M[2, 0], M[2, 1], M[2, 2], p.z()],
                     [0, 0, 0, 1]])


def joints_to_kdl(joint_values):
    """
    Convert the numpy array into KDL data format

    Args:
        joint_values (np.ndarray): values for the joints

    Returns:

    """
    num_jts = joint_values.size
    kdl_array = kdl.JntArray(num_jts)
    for idx in range(num_jts):
        kdl_array[idx] = joint_values[idx]
    return kdl_array


def get_tf_transform(tf_listener, tgt_frame, src_frame):
    """
    Uses ROS TF to lookup the current transform from tgt_frame to src_frame,
    If the returned transform is applied to data, it will transform data in
    the src_frame into the tgt_frame

    Args:
        tf_listener (tf.TransformListener): ros tf transformlistener
        tgt_frame (str): target frame
        src_frame: source frame

    Returns:
        list: translation [x, y, z]
        list: quaternion [qx, qy, qz, qw]
    """

    try:
        tf_listener.waitForTransform(tgt_frame, src_frame,
                                     rospy.Time.now(),
                                     rospy.Duration(3))
        (trans, quat) = tf_listener.lookupTransform(tgt_frame,
                                                    src_frame,
                                                    rospy.Time(0))
    except (tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException):
        raise RuntimeError('Cannot fetch the transform from'
                           ' {0:s} to {1:s}'.format(tgt_frame, src_frame))
    return list(trans), list(quat)
