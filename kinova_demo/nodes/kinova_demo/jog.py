#! /usr/bin/env python

import roslib; roslib.load_manifest('kinova_demo')
import rospy
import sys
import math
import actionlib
import kinova_msgs.msg
import kinova_msgs.srv
import sensor_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import tty

if len(sys.argv) < 2:
	topic_prefix = '/jaco2_arm_driver'
else:
	topic_prefix = sys.argv[1]
print 'topic prefix will be: ' + topic_prefix
print 'Specify an alternate prefix on command line to select a different node.'
print '    e.g.: jog.py /left_jaco2'

def home_arm():
  servicename = topic_prefix + '/in/home_arm'
  rospy.wait_for_service(servicename)
  home = rospy.ServiceProxy(servicename, kinova_msgs.srv.HomeArm)
  try:
    r = home()
    print 'called home_arm service.'
  except rospy.ServiceException as exc:
    print("Service did not process request: " + str(exc))

def send_tool_pos(pos):
  action_address = topic_prefix + '/arm_pose/arm_pose'
  client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
  client.wait_for_server()
  goal = kinova_msgs.msg.ArmPoseGoal()
  goal.pose.pose = pos
  goal.pose.header = std_msgs.msg.Header(frame_id = 'jaco_api_origin')
  print '--------------->'
  print('goal: {}'.format(goal))
  print '--------------->'
  client.send_goal(goal)
  if client.wait_for_result(rospy.Duration(20.0)):
    print 'action result:'
    print client.get_result()
    return client.get_result()
  else:
    print('        the arm pose action timed-out')
    client.cancel_all_goals()
    return None

def send_fingers(fingers):
  action_address = topic_prefix + '/fingers/finger_positions'
  client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.SetFingersPositionAction)
  client.wait_for_server()
  goal = kinova_msgs.msg.SetFingersPositionGoal()
  goal.fingers.finger1 = fingers[0]
  goal.fingers.finger2 = fingers[1]
  goal.fingers.finger3 = fingers[2]
  print '--------------->'
  print('goal: {}'.format(goal))
  print '--------------->'
  client.send_goal(goal)
  if client.wait_for_result(rospy.Duration(20.0)):
    print 'action result:'
    print client.get_result()
    return client.get_result()
  else:
    print('        the finger position action timed-out')
    client.cancel_all_goals()
    return None


def send_joint_angles(angle_set):
    """Send a joint angle goal to the action server."""
    action_address = topic_prefix + '/joint_angles/arm_joint_angles'
    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.ArmJointAnglesAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmJointAnglesGoal()

    goal.angles.joint1 = angle_set[0]
    goal.angles.joint2 = angle_set[1]
    goal.angles.joint3 = angle_set[2]
    goal.angles.joint4 = angle_set[3]
    goal.angles.joint5 = angle_set[4]
    goal.angles.joint6 = angle_set[5]

    print '--------------->'
    print('goal: {}'.format(goal))
    print '--------------->'

    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(20.0)):
        print 'action result:'
        print client.get_result()
        return client.get_result()
    else:
        print('        the joint angle action timed-out')
        client.cancel_all_goals()
        return None

def choose_canned_state():
  # TODO could use cartesian end effector position instead
  print 'Choose an arm pose:'
  print '1: ready front over table'
  print '2: stretch forward'
  print '3: right arm ready towards middle from side'
  print '4: left arm stretch'
  print '5: right arm stretch'
  print '6: down in front'
  k = sys.stdin.read(1)
  if k == '1':
      send_joint_angles([-1.692, -0.551, -0.073, -1.579, 3.057, 2.515])
  elif k == '2':
      send_joint_angles([-1.672, -0.046, -1.728, -1.579, 3.057, -1.035])
  elif k == '3':
      send_joint_angles([-2.020, -0.120, -0.952, -1.254, 1.344, -0.542])
  elif k == '4':
      send_joint_angles([-0.327, -0.079, -1.352, -2.589, -1.573, 3.255])
  elif k == '5':
      send_joint_angles([-3.043, -0.127, -1.337, -0.953, 1.092, 3.551])
  elif k == '6': 
      send_joint_angles([-1.690, 0.590, -0.570, -1.578, 3.055, 0.153])
  else:  
    return

#curr_joint_positions = ()
#def joint_state_cb(s):
#  curr_joint_positions = s.position
#  print curr_joint_positions


def help():
  print ''
  print '============================='
  print 'Jog position: x/X y/Y z/Z r/R p/P w/W'
  print 'Jog joint angles: 1/! 2/@ 3/# 4/$ 5/% 6/^'
  print 'Fingers: 7/& 8/* 9/(' #  (all) o/c'
  print 'Choose predefined pose: p'
  print 'Home: h'
  print 'Quit: q'

if __name__ == '__main__':
  angle_inc = math.pi/18; #radians
  dist_inc = 0.1 #meters
  print ''
  print 'This is a quick and dirty way to test the arm. Use the keys listed below to jog the joint angles of the arm by %d degrees or move the end effector by %d meters. Press q to quit.' % (angle_inc, dist_inc)
  print ''
  print 'Each keypress results in a new goal state/pos sent to the kinova arm node (thence to the arm itself), and we wait until that new goal state is reached.'
  print ''
  print 'Start by homing the arm with the h key.'
  print ''
  print 'WARNING: THERE IS NO COLLISION DETECTION WITH ARM ITSELF, ARM MOUNTING STRUCTURE, OR ANY OTHER OBJECT'
  print ''
  fd = sys.stdin.fileno()
  oldtc = tty.tcgetattr(fd)
  tty.setcbreak(fd)
  try:
    rospy.init_node('jaco_arm_jog_demo', sys.argv, True)

#    joint_state_sub = rospy.Subscriber(topic_prefix + '/out/joint_state', sensor_msgs.msg.JointState, joint_state_cb);
    while True:
      help()
      k = sys.stdin.read(1)

      print '<---------------------'
      # TODO subscribe asynchronously instead
      jointstate = rospy.wait_for_message(topic_prefix + '/out/joint_state', sensor_msgs.msg.JointState, 10)
      if jointstate == None:
        print 'did not receive current state of arm joints. try again.'
        continue
      jgoal = list(jointstate.position)[0:6]
      #fgoal = list(jointstate.position)[6:]
      fingerpos = rospy.wait_for_message(topic_prefix + '/out/finger_position', kinova_msgs.msg.FingerPosition)
      fgoal = [fingerpos.finger1, fingerpos.finger2, fingerpos.finger3]
      print 'current joint angles: %s' % (jgoal)
      print 'current fingers: %s' % (fgoal)
      toolpos = rospy.wait_for_message(topic_prefix + '/out/tool_position',
geometry_msgs.msg.PoseStamped, 5)
      if toolpos == None:
        print 'did not receive current tool pose of arm try again'
        continue
      cgoal = toolpos.pose
      print 'current end effector pose: %s' % (cgoal)
    
      print '<---------------------'
      
      #if len(jgoal) < 6:
      #  print 'don\'t have current position yet'
      #  continue
      if k == 'x':
        cgoal.position.x -= dist_inc
        send_tool_pos(cgoal)
      elif k == 'X':
        cgoal.position.x += dist_inc
        send_tool_pos(cgoal)
      elif k == 'y':
        cgoal.position.y -= dist_inc
        send_tool_pos(cgoal)
      elif k == 'Y':
        cgoal.position.y += dist_inc
        send_tool_pos(cgoal)
      elif k == 'z':
        cgoal.position.z -= dist_inc
        send_tool_pos(cgoal)
      elif k == 'Z':
        cgoal.position.z += dist_inc
        send_tool_pos(cgoal)
      elif k == '1':
        jgoal[0] -= angle_inc
        send_joint_angles(jgoal)
      elif k == '!':
        jgoal[0] += angle_inc
        send_joint_angles(jgoal)
      elif k == '2':
        jgoal[1] -= angle_inc
        send_joint_angles(jgoal)
      elif k == '@':
        jgoal[1] += angle_inc
        send_joint_angles(jgoal)
      elif k == '3':
        jgoal[2] -= angle_inc
        send_joint_angles(jgoal)
      elif k == '#':
        jgoal[2] += angle_inc
        send_joint_angles(jgoal)
      elif k == '4':
        jgoal[3] -= angle_inc
        send_joint_angles(jgoal)
      elif k == '$':
        jgoal[3] += angle_inc
        send_joint_angles(jgoal)
      elif k == '5':
        jgoal[4] -= angle_inc
        send_joint_angles(jgoal)
      elif k == '%':
        jgoal[4] += angle_inc
        send_joint_angles(jgoal)
      elif k == '6':
        jgoal[5] -= angle_inc
        send_joint_angles(jgoal)
      elif k == '^':
        jgoal[5] += angle_inc
        send_joint_angles(jgoal)
      elif k == '7':
        fgoal[0] -= angle_inc
        send_fingers(fgoal)
      elif k == '&':
        fgoal[0] += angle_inc
        send_fingers(fgoal)
      elif k == '8':
        fgoal[1] -= angle_inc
        send_fingers(fgoal)
      elif k == '*':
        fgoal[1] += angle_inc
        send_fingers(fgoal)
      elif k == '9':
        fgoal[2] -= angle_inc
        send_fingers(fgoal)
      elif k == '(':
        fgoal[2] += angle_inc
        send_fingers(fgoal)
      elif k == 'p':
        choose_canned_state()
      elif k == 'h':
        home_arm()
      elif k == 'q' or k == 'Q':
        sys.exit(0)
      else:
        print '?'
        continue

  except rospy.ROSInterruptException:
    print "program interrupted"
  finally:
      print 'quit'
      tty.tcsetattr(fd, tty.TCSAFLUSH, oldtc)


