#!/usr/bin/env python




#TODO ADD APROACH STEP, APROACH, LIFT AWAY, MOVE AWAY



#wheel imports
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#arm imports
import sys
import copy
import moveit_commander
import moveit_msgs.msg

#finger imports
from robotiq_s_model_articulated_msgs.msg import SModelRobotOutput

#wheel thang
def movebase_client(trans,q):
 client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 client.wait_for_server()

 goal = MoveBaseGoal()
 goal.target_pose.header.frame_id = "map"
 goal.target_pose.header.stamp = rospy.Time.now()
 goal.target_pose.pose.position.x = trans[0]
 goal.target_pose.pose.position.y = trans[1]
 #goal.target_pose.pose.position.z = trans[2]
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


#arm thang
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

#finger thang
gripper_publisher = rospy.Publisher("/l_gripper/SModelRobotOutput", SModelRobotOutput)
#activate_gripper = SModelRobotOutput()
#activate_gripper.rACT = 1
#activate_gripper.rGTO = 1
#activate_gripper.rSPA = 255
#activate_gripper.rFRA = 150
#open_gripper = activate_gripper
#close_gripper = activate_gripper
#open_gripper.rPRA = 0
#close_gripper.rPRA = 255

gcommand = SModelRobotOutput()
gcommand.rACT = 1
gcommand.rGTO = 1
gcommand.rSPA = 255
gcommand.rFRA = 150
gripper_publisher.publish(gcommand)


#all the other shizzle
rate = rospy.Rate(1)
listener = tf.TransformListener()
stage = 0
retry = 1
while not rospy.is_shutdown():
 if stage == 0:
  rospy.loginfo("time to go in:")
  rospy.loginfo("3")
  rospy.sleep(1)
  rospy.loginfo("2")
  rospy.sleep(1)
  rospy.loginfo("1")
  rospy.sleep(1)
  rospy.loginfo("lets go!!")
  rospy.loginfo("starting the gripper")
  gcommand.rPRA = 0
  gripper_publisher.publish(gcommand)
  stage = 1

 if stage == 1:
  rospy.loginfo("getting into position")
  try:
   (trans,rot) = listener.lookupTransform('/map', '/goal', rospy.Time(0))
# NEW STUFF==================================================================================
   #(atrans,arot) = listener.lookupTransform('/base_link', '/arm_goal', rospy.Time(0))
  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
   continue
  quaternion = (rot[0], rot[1], rot[2], rot[3])
  euler = tf.transformations.euler_from_quaternion(quaternion)
  roll = euler[0]
  pitch = euler[1]
  yaw = euler[2]

  #print("trans x,y,z is ", trans[0], trans[1], trans[2])
  #print("rot x,y,z is ", rot[0], rot[1], rot[2], rot[3])
#  rospy.loginfo("roll, pitch, yaw = ", roll, pitch, yaw)
  try:
  #rospy.init_node('movebase_client_py')
   result = movebase_client(trans, quaternion)
   if result:
    rospy.loginfo("Goal execution done!")
    stage = 2
    retry = 1
  except rospy.ROSInterruptException:
   rospy.loginfo("Navigation test finished.")
  # QUICK FIX
  stage = 2
  retry = 1
  #rospy.sleep(1)

 if stage == 2:
  try:
   if retry == 1: (atrans,arot) = listener.lookupTransform('/base_link', '/arm_goal', rospy.Time(0))
   else: rospy.loginfo("skipping arm relocate")
  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
   continue
  else:
   rospy.loginfo("reaching for object")
   rospy.sleep(1) #small pause
   pose_target = geometry_msgs.msg.Pose()
   pose_target.orientation.x = 0.286
   pose_target.orientation.y = -0.275
   pose_target.orientation.z = -0.639
   pose_target.orientation.w = 0.659
   pose_target.position.x = atrans[0]
   pose_target.position.y = atrans[1] + 0.5
#   pose_target.position.y = 0.0
   pose_target.position.z = atrans[2] + 0
   group.set_pose_target(pose_target)

   plan1 = group.plan()
   rospy.loginfo("============ Visualizing plan1")
   display_trajectory = moveit_msgs.msg.DisplayTrajectory()

   display_trajectory.trajectory_start = robot.get_current_state()
   display_trajectory.trajectory.append(plan1)
   display_trajectory_publisher.publish(display_trajectory);
   rospy.sleep(2)
   display_trajectory_publisher.publish(display_trajectory);
   rospy.sleep(2)
   display_trajectory_publisher.publish(display_trajectory);
   confirmcheck = raw_input('type go to continue: ')
   if confirmcheck == 'go':
    group.go()
    stage = 3
   else:
    retry = 1

  if stage == 3:
   interpreter.execute("go " + "right" + " " + str(0.1))
   stage = 4

  if stage == 4:
   rospy.loginfo("Grabbing time!!")
   rospy.sleep(1) #small pause
   gcommand.rPRA = 255
   gripper_publisher.publish(gcommand)
   rospy.sleep(1)
   stage = 5

  if stage == 5:
   interpreter.execute("go " + "up" + " " + str(0.1))
   rospy.sleep(1)
   interpreter.execute("go " + "down" + " " + str(0.1))
   rospy.sleep(1)
   gcommand.rPRA = 0
   gripper_publisher.publish(gcommand)
   rospy.sleep(1)
# STOP
   exit()
   stage = 6

  if stage == 6:
   rospy.loginfo("reaching for object")
   rospy.sleep(1) #small pause
   pose_target = geometry_msgs.msg.Pose()
   pose_target.orientation.x = 0.286
   pose_target.orientation.y = -0.275
   pose_target.orientation.z = -0.639
   pose_target.orientation.w = 0.659
   pose_target.position.x = 0.7
   pose_target.position.y = 0.7
#   pose_target.position.y = 0.0
   pose_target.position.z = 0.7
   group.set_pose_target(pose_target)

   plan1 = group.plan()
   rospy.loginfo("============ Visualizing plan1")
   display_trajectory = moveit_msgs.msg.DisplayTrajectory()

   display_trajectory.trajectory_start = robot.get_current_state()
   display_trajectory.trajectory.append(plan1)
   display_trajectory_publisher.publish(display_trajectory);
   rospy.sleep(2)
   display_trajectory_publisher.publish(display_trajectory);
   rospy.sleep(2)
   display_trajectory_publisher.publish(display_trajectory);
   confirmcheck = raw_input('type go to continue: ')
   if confirmcheck == 'go':
    group.go()
    exit()
   else:
    retry = 1

 rate.sleep
exit()
