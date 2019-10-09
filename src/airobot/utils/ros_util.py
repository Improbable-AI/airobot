import rospy
import tf


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
