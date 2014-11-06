#!/usr/bin/env python
import rospy
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Empty
from pymouse import PyMouse
import numpy as np

initialized = False
tf_listener = None
yaw, pitch, roll = None, None, None
left, right, top, bottom = None, None, None, None
mouse = PyMouse()

def pose_cb(pose):
    pose.header.stamp = rospy.Time(0)
    pose_transformed = tf_listener.transformPose('face_detection', pose)
    global yaw, pitch, roll
    roll, pitch, yaw = euler_from_quaternion((
        pose_transformed.pose.orientation.x,
        pose_transformed.pose.orientation.y,
        pose_transformed.pose.orientation.z,
        pose_transformed.pose.orientation.w
    ))
    yaw = np.unwrap([yaw])[0]
    pitch = np.unwrap([pitch])[0]
    if initialized:
        px = int(angle_to_pixel(yaw, left, right, 0, mouse.screen_size()[0]))
        py = int(angle_to_pixel(pitch, top, bottom, 0, mouse.screen_size()[1]))
        print px, py
        mouse.move(px, py)


def click_cb(empty):
    mouse.click(*mouse.position())

def angle_to_pixel(ang, ang_min, ang_max, px_min, px_max):
    frac = abs((ang - ang_min) / abs(ang_max - ang_min))
    return px_min + frac * (px_max - px_min)

if __name__ == '__main__':
    rospy.init_node('pose_to_mouse')
    tf_listener = tf.TransformListener()
    fixed_frame = rospy.wait_for_message('/face_detector/faces_cloud', PointCloud).header.frame_id
    rospy.Subscriber('/head_pose', PoseStamped, pose_cb)

    print 'Move your head to the top left corner of the screen and click'
    rospy.wait_for_message('click', Empty)
    left, top = yaw, pitch
    print 'Move your head to the bottom right corner of the screen and click'
    rospy.wait_for_message('click', Empty)
    right, bottom = yaw, pitch
    print 'Done.'
    print left, right
    initialized = True

    rospy.Subscriber('click', Empty, click_cb)

    rospy.spin()