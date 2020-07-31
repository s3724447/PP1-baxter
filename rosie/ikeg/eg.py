#!/usr/bin/python
# Baxter boogie REAM team 2018 Semester 1

import socket
import math
import time
import Vectors
import struct
import threading
import Queue
import rospy
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
from tf.transformations import *

import random

import tf2_ros
import tf2_geometry_msgs
tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)

# refer to http://sdk.rethinkrobotics.com/wiki/IK_Service_Example

from geometry_msgs.msg import (
        PoseStamped,
        Pose,
        Point,
        Quaternion,
        )
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
        SolvePositionIK,
        SolvePositionIKRequest,
        )
import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

direction = 1

mutex = threading.Lock()

last_data = None;
dz = 0.0;

rospy.init_node("marker_ik_example")
#time.sleep(2)

global posedebug
posedebug = rospy.Publisher('/red/pose_debug', PoseStamped, queue_size=2)
global marker_topic
marker_topic = rospy.Publisher('/red/ikeg/target_marker_pose', PoseStamped, queue_size=5)
global target_topic
target_topic = rospy.Publisher('/red/ikeg/target_gripper_pose', PoseStamped, queue_size=5)

global redebug
redebug = rospy.Publisher('/red/debug', String, queue_size=1)

# Something in here doesn't work unless Baxter is the master
#rs = baxter_interface.RobotEnable(CHECK_VERSION)
#init_state = rs.state().enabled
#print 'initial state', init_state
#rs.enable()

def ik_move(limb, pose, frame):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    #hdr = Header(stamp=rospy.Time.now(), frame_id='reference/right_hand_camera')
    hdr = Header(stamp=rospy.Time.now(), frame_id=frame)
    ps = PoseStamped(
            header=hdr,
            pose=pose,
            )
    global posedebug
    target_topic.publish(ps)
    ikreq.pose_stamp.append(ps)
    try:
        #print 'ik_move: waiting...'
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1
    #print 'ik_move: done...'

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
            resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                ikreq.SEED_USER: 'User Provided Seed',
                ikreq.SEED_CURRENT: 'Current Joint Angles',
                ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                }.get(resp_seeds[0], 'None')
        #print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" % (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print 'ik_move: solution found...'
        return resp
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        return None

def make_move(msg, limb, speed):
    SPEED_SCALE = 1
    speed = speed * SPEED_SCALE
    arm = baxter_interface.Limb(limb)
    print 'arm',arm
    lj = arm.joint_names()

    command = {}
    for i in range(0, len(msg.joints[0].name)):
        command[msg.joints[0].name[i]] = msg.joints[0].position[i]

    print 'current:',arm.joint_angles()

    print 'make_move: speed',speed
    arm.set_joint_position_speed(speed)
    print 'make_move: posns',command
    #arm.set_joint_positions(command)
    arm.move_to_joint_positions(command)
    print 'make_move: done'



def recv_data(data):
    '''
    Take udp data and break into component messages
    '''
    
    # split on token
    list_data = data.split(',')
    
    # Convert next beat time
    list_data[0] = float(list_data[0])

    # Convert and normalize energy
    list_data[1] = float(list_data[1])
    list_data[1] = min(3000, list_data[1])
    list_data[1] /= 3000

    list_data[2] = float(list_data[2]);

    dict_data = {'beat': list_data[0], 'energy': list_data[1], 'onset': list_data[2]}

    return dict_data

def decide_increment(data): 
    #TODO - decide based on data
    mutex.acquire()
    global direction
    
    global dz

    global last_data;
    if last_data is None:
        last_data = data;

    if (data is not None):
        wow = data['onset'];
        dwow = data['onset'] - last_data['onset'];
        if abs(dwow) > 0.01:
            dz = dwow / 10.0;
        print "dwow", dwow;

    print "dz: ", dz

    inc = Vectors.V4D(0.0, direction * 0.05, dz , 0.0)
    mutex.release()

    last_data = data;

    return inc;

def clamp(p, inner, outer):
    if p.x() > outer.x():
        p = Vectors.V4D(outer.x(), p.y(), p.z(), p.w())
    elif p.x() < inner.x():
        p = Vectors.V4D(inner.x(), p.y(), p.z(), p.w())
    if p.y() > outer.y():
        p = Vectors.V4D(p.x(), outer.y(), p.z(), p.w())
    elif p.y() < inner.y():
        p = Vectors.V4D(p.x(), inner.y(), p.z(), p.w())
    if p.z() > outer.z():
        p = Vectors.V4D(p.x(), p.y(), outer.z(), p.w())
    elif p.z() < inner.z():
        p = Vectors.V4D(p.x(), p.y(), inner.z(), p.w())
    if p.w() > outer.w():
        p = Vectors.V4D(p.x(), p.y(), p.z(), outer.w())
    elif p.w() < inner.w():
        p = Vectors.V4D(p.x(), p.y(), p.z(), inner.w())

    return p



def jitter():
    return random.uniform(-0.05, 0.05)

# Quaternion -> Quaternion
def normalize(quat):
 	quatNorm = math.sqrt(quat.x * quat.x + quat.y *
                        quat.y + quat.z * quat.z + quat.w * quat.w)
        normQuat = Quaternion(quat.x / quatNorm,
                              quat.y / quatNorm,
                              quat.z / quatNorm,
                              quat.w / quatNorm)
	return normQuat

def hack_pose(pose):
    quaternion = (
	pose.orientation.x,
	pose.orientation.y,
	pose.orientation.z,
	pose.orientation.w)
    # Leroy's guess
    #q_rot = quaternion_from_euler(math.pi/2, math.pi, 0.0, 'rzyx')
    q_rot1 = quaternion_from_euler(0.0, math.pi, 0.0, 'rzyx')
    #q_rot = quaternion_from_euler(math.pi/2, 0.0, 0.0, 'rzyx')
    q_new = quaternion_multiply(q_rot1, quaternion)
    #q_new = quaternion_from_euler(0, 0, 0)
    pose_rot = Pose(
            position=Point(
                x=pose.position.x,
                y=pose.position.y,
                z=pose.position.z
                ),
            orientation=normalize(Quaternion(
		x=q_new[0],
		y=q_new[1],
		z=q_new[2],
		w=q_new[3]
            ))
	    # This won't work in camera frame
	    #orientation=Quaternion(
            #                 x=-0.0249590815779,
            #                 y=0.999649402929,
            #                 z=0.00737916180073,
            #                 w=0.00486450832011)
    )
    return pose_rot

def right_arm(pos):
    '''
    Create goal Pose and call ik move
    '''
    pose_right = Pose(
            position=Point(
                x=pos.x(),
                y=pos.y(),
                z=pos.z(),
                ),
            orientation=Quaternion(
                x=0,
                y=1,
                z=0,
                w=0.5
                ),
            )
    return pose_right


def pose2(pos):
    '''
    Create goal Pose and call ik move
    '''
    pose_right = Pose(
            position=Point(
                x=pos.x(),
                y=pos.y(),
                z=pos.z(),
                ),
            orientation=Quaternion(
                x=0,
                y=1,
                z=0,
                w=0
                ),
            )
    return pose_right

#time.sleep(2)

init_pos = Vectors.V4D(0.8,
        -0.47,
        0.2, 0)

bound = Vectors.V4D(0.656982770038,
        -0.252598021641,
        0.5388609422173, 0)

init_pos2 = Vectors.V4D(0.656982770038,
        -0.35,
        0.1, 0)

init_pos3 = Vectors.V4D(0.656982770038,
        -0.35,
        0.4, 0)

# this works on command line but not in rospy!?
# rostopic pub /red/pose_debug geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
#time.sleep(2)
myps = PoseStamped(
			header=Header(stamp=rospy.Time.now(), frame_id='base'),
			#pose=Pose(position=Point(1.0,0.0,0.0),orientation=Quaternion(0.0,0.0,0.0,1.0))
			pose=pose2(init_pos),
)
#time.sleep(2)
print 'posedebug test'
posedebug.publish(myps)
print 'posedebug test done'

global moved
moved = False

#print 'initialisation move...'
#resp = ik_move('right', pose2(init_pos),'base')
#if resp is not None:
#    print 'moving...'
#    make_move(resp, 'right', 0.4)
#else:
#    print 'IK error'

time.sleep(1)

def callback(data):
  print 'callback'
  global moved
  global posedebug
  foundmarker = 0
  if not moved:
    if data.markers:
      for marker in data.markers:
        if marker.id != 255 and marker.id != 0:
          if not foundmarker:
		print 'markers:',
          foundmarker = 1
	  print marker.id,
	  if (marker.id == 4 or marker.id == 2 or marker.id == 5):
	      print 'found marker',marker.id
	      marker_pose = marker.pose.pose
	      marker_topic.publish(
		  PoseStamped(
			  header=marker.header,
			  pose=marker.pose.pose,
		  )
	      )
	      global tf_buffer
	      # https://answers.ros.org/question/222306/transform-a-pose-to-another-frame-with-tf2-in-python/
	      #transform = tf_buffer.lookup_transform('reference/torso',
	#	 marker.header.frame_id, #source frame
	#	 rospy.Time(0), #get the tf at first available time
	#	 rospy.Duration(2.0)) #wait for 1 second
	#      pose_transformed = tf2_geometry_msgs.do_transform_pose(marker.pose, transform)
	      #print 'pose in baxter frame',pose_transformed
	      pose = hack_pose(marker_pose)
	      #print 'desired pose',pose
	      #return;
	      resp = ik_move('right', pose, 'reference/right_hand_camera')
	      if resp is not None:
		  #print '**** MOVING ****'
		  print '**** (NOT) MOVING ****'
		  time.sleep(2)
		  moved = True
		  #make_move(resp, 'right', 0.2)
		  print '**** FINISHED ****'
		  time.sleep(1)
		  moved = False
	      else:
		  print 'IK error'
      if foundmarker:
          print

rospy.Subscriber('/ar_pose_marker', AlvarMarkers, callback, queue_size=1)

#time.sleep(240)
rospy.spin()
