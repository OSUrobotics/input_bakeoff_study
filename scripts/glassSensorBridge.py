#!/usr/bin/env python
import SocketServer, time
import struct
import sys
import rospy, tf, numpy
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Imu, Illuminance
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty

# See enum values at http://developer.android.com/reference/android/hardware/Sensor.html
TYPE_ACCELEROMETER       = 1
TYPE_ORIENTATION         = 3
TYPE_GYROSCOPE           = 4
TYPE_LIGHT               = 5
TYPE_LINEAR_ACCELERATION = 10
TYPE_ROTATION_VECTOR     = 11
TYPE_WINK                = 98
TYPE_TAP                 = 99

class SocketHandler(SocketServer.BaseRequestHandler):
    child_frame_id = 'glass'

    def handle(self):
        print "Handle"
        self.br = tf.TransformBroadcaster()
        self.imu_pub = rospy.Publisher('/android/imu', Imu)
        self.pose_pub = rospy.Publisher('/head_pose', PoseStamped)
        self.light_pub = rospy.Publisher('/android/light', Illuminance)
        self.click_pub = rospy.Publisher('/click', Empty)
        self.glass_base_frame = '/face_detection'
        self.imu = Imu()
        self.imu.orientation_covariance = numpy.eye(3).flatten().tolist()
        self.imu.angular_velocity_covariance = numpy.eye(3).flatten().tolist()
        self.imu.linear_acceleration_covariance = numpy.eye(3).flatten().tolist()
        self.imu_ready = [False, False, False]
        print "Made broadcaster"
        self.data = self.request.recv(16)
        while not rospy.is_shutdown() and self.data:
            if self.data:
                try:
                    sensor = struct.unpack('>i', self.data[:4])[0]
                    if sensor == TYPE_ORIENTATION:
                        rpy = numpy.radians(struct.unpack('>3f', self.data[4:]))
                        quat = quaternion_from_euler(*rpy)
                        print 'Orientation', quat
                        stamp = rospy.Time.now()
                        pose_msg = PoseStamped()
                        pose_msg.header.frame_id = self.child_frame_id
                        pose_msg.header.stamp = stamp
                        pose_msg.pose.orientation.w = 1
                        self.pose_pub.publish(pose_msg)
                        self.br.sendTransform((0,0,0), quat, stamp, self.child_frame_id, self.glass_base_frame)
                    elif sensor == TYPE_LINEAR_ACCELERATION:
                        ax, ay, az = struct.unpack('>3f', self.data[4:])
                        print 'IMU', ax, ay, az
                        
                        self.imu.header.frame_id = self.glass_base_frame
                        self.imu.header.stamp = rospy.Time.now()
                        self.imu.orientation.w = 1
                        self.imu.linear_acceleration.x = ax
                        self.imu.linear_acceleration.y = ay
                        self.imu.linear_acceleration.z = az

                        self.imu_ready[0] = True

                        if all(self.imu_ready):
                            self.imu_ready = [False, False, False]
                            self.imu_pub.publish(self.imu)

                    elif sensor == TYPE_GYROSCOPE:
                        self.imu.header.frame_id = self.glass_base_frame
                        self.imu.header.stamp = rospy.Time.now()

                        vx, vy, vz = struct.unpack('>3f', self.data[4:])

                        self.imu.angular_velocity.x = vx
                        self.imu.angular_velocity.y = vy
                        self.imu.angular_velocity.z = vz

                        self.imu_ready[1] = True

                        if all(self.imu_ready):
                            self.imu_ready = [False, False, False]
                            self.imu_pub.publish(self.imu)

                    elif sensor == TYPE_ROTATION_VECTOR:
                        self.imu.header.frame_id = self.glass_base_frame
                        self.imu.header.stamp = rospy.Time.now()

                        rpy = numpy.radians(struct.unpack('>3f', self.data[4:]))
                        quat = quaternion_from_euler(*rpy)
                        self.imu.orientation.x = quat[0]
                        self.imu.orientation.y = quat[1]
                        self.imu.orientation.z = quat[2]
                        self.imu.orientation.w = quat[3]

                        self.imu_ready[2] = True

                        if all(self.imu_ready):
                            self.imu_ready = [False, False, False]
                            self.imu_pub.publish(self.imu)

                    elif sensor == TYPE_LIGHT:
                        l, _, _ = struct.unpack('>3f', self.data[4:])
                        print 'Light', l
                        ill = Illuminance()
                        ill.header.frame_id = self.child_frame_id
                        ill.header.stamp = rospy.Time.now()
                        ill.illuminance = l

                        self.light_pub.publish(ill)

                    elif sensor == TYPE_TAP:
                        print 'Click'
                        self.click_pub.publish()

                    elif sensor == TYPE_WINK:
                        print 'Click (wink)'
                        self.click_pub.publish()

                    else:
                        print 'Unknown sensor:', sensor

                except Exception, e:
                    if type(e) == struct.error:
                        print "Couldn't unpack ", self.data.__repr__(), 'with length', len(self.data)
                    else:
                        print e
            self.data = self.request.recv(16)
        print 'Disconnect'

def _maybe_shutdown(self):
    if rospy.is_shutdown():
        server.shutdown()

if __name__ == "__main__":
    HOST, PORT = "0.0.0.0", 9999 # default to listening on all interfaces (INADDR_ANY)
    argv = rospy.myargv()
    if len(argv) > 1:
        HOST, PORT = argv[1], int(argv[2])
    print 'Listening on %s:%s' % (HOST if HOST != "0.0.0.0" else "INADDR_ANY", PORT)

    # Create the server, HOST:PORT
    server = SocketServer.TCPServer((HOST, PORT), SocketHandler)

    rospy.init_node('glass_sensor_bridge')
    print 'ROS Node started. Listening for messages.'

    rospy.Timer(rospy.Duration(1.0), _maybe_shutdown)

    server.serve_forever()
