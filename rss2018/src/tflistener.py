#!/usr/bin/env python
import roslib
#roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
#import turtlesim.srv
if __name__ == '__main__':
    rospy.init_node('husky_tf_listener')
    listener = tf.TransformListener()
#    rospy.wait_for_service('spawn')
#    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
#    spawner(4, 2, 0, 'turtle2')
#    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/object_7', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        quaternion = (rot[0], rot[1], rot[2], rot[3])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

  #      print("trans x,y,z is ", trans[0], trans[1], trans[2])
  #      print("rot x,y,z is ", rot[0], rot[1], rot[2], rot[3])
        print("roll, pitch, yaw = ", roll, pitch, yaw)
        rate.sleep()
