#!/usr/bin/env python
import roslib
#roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(trans,q):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = trans[0]
    goal.target_pose.pose.position.y = trans[1]
#    goal.target_pose.pose.position.z = trans[2]
    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = 0
    goal.target_pose.pose.orientation.w = 1
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

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
            (trans,rot) = listener.lookupTransform('/map', '/goal', rospy.Time(0))
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
        try:
#            rospy.init_node('movebase_client_py')
            result = movebase_client(trans, quaternion)
            if result:
                rospy.loginfo("Goal execution done!")
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")

        rate.sleep()
