#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_py_demo_2', anonymous=True)

scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()
interpreter = moveit_commander.MoveGroupCommandInterpreter()
interpreter.execute("use left_arm")
group = interpreter.get_active_group()
joints = group.get_joints()

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path_josh', moveit_msgs.msg.DisplayTrajectory)

print "============ Reference frame: %s" % group.get_planning_frame()
print "============ Reference frame: %s" % group.get_end_effector_link()

#print "============ Printing robot state"
#print robot.get_current_state()
#print "============"
#print "============ Robot Groups:"
#print robot.get_group_names()
print "============ Waiting for RVIZ..."

#interpreter.execute("go " + "up" + " " + str(0.05))

rate = rospy.Rate(1)
listener = tf.TransformListener()
while not rospy.is_shutdown():
 try:
  (trans,rot) = listener.lookupTransform('/base_link', '/armgoal', rospy.Time(0))
 except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
  continue
 else:
  pose_target = geometry_msgs.msg.Pose()
  pose_target.orientation.x = 0.286
  pose_target.orientation.y = -0.275
  pose_target.orientation.z = -0.639
  pose_target.orientation.w = 0.659
  pose_target.position.x = trans[0]
  pose_target.position.y = trans[1]
  pose_target.position.z = trans[2]
  group.set_pose_target(pose_target)

  plan1 = group.plan()
  print "============ Visualizing plan1"
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()

  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan1)
  display_trajectory_publisher.publish(display_trajectory);
  #rospy.sleep(2)
  display_trajectory_publisher.publish(display_trajectory);
  #rospy.sleep(2)
  display_trajectory_publisher.publish(display_trajectory);
  confirmcheck = raw_input('type go to continue: ')
  if confirmcheck == 'go': group.go()
 rate.sleep
exit()
