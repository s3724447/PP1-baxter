#!/usr/bin/env python

import sys
import rospy
import roslib
import json
import time
from rosgraph_msgs.msg import Log

global close
close = False

msgs = {
  #"video": "GazeboRosVideo (gzserver, ns = /mobility_base/) has started",
  "video": "GazeboRosVideo (gzserver, ns = /",
  "simloaded": "Simulator is loaded and started successfully",
  "xcb": "[Wrn] [GuiIface.cc:120] QXcbConnection: XCB error"
}

def mylog(msg):
  global cmdname
  rospy.loginfo("**** " + cmdname+": "+msg+" ****")

def process_log(data):
  if mode=="video" and data.msg.startswith(msgs["video"]):
      mylog("**** await_simstart: "+data.msg+" ****")
      shutdown(0, data.msg)
  if mode=="simstart" and data.msg.startswith(msgs["simloaded"]):
      mylog("**** await_simstart: "+data.msg+" ****")
      shutdown(0, data.msg)
  if mode=="xcb_error" and data.msg.startswith(msgs["xcb"]):
      mylog("**** await_simstart: "+data.msg+" ****")
      shutdown(1, data.msg)

def mytimeout(event):
    rospy.loginfo("**** await_simstart: TIMEOUT ****")
    print("**** await_simstart: TIMEOUT ****")
    shutdown(1,'timeout')

global cmdname
cmdname = str(sys.argv[0])

global mode
mode = ""
print 'command name',cmdname

if cmdname.endswith('await_video_ready.py'):
  mode = "video"
if cmdname.endswith('await_simulator_is_loaded.py'):
  mode = "simstart"
if cmdname.endswith('await_xcb_error.py'):
  mode = "xcb_error"

print 'mode:',mode
if mode == "":
    mylog('assertion failure; no defined mode')
    sys.exit(1)

rospy.init_node('await_simstart', anonymous=True, disable_signals=True)
rospy.loginfo('**** await_simstart: initialized ****')
rospy.Subscriber('/rosout', Log, process_log)
#timer = rospy.Timer(rospy.Duration(5), mytimeout)

global exit_status
exit_status = 1

def shutdown(status=1,reason=''):
    global exit_status
    exit_status = status
    msg = "await_simstart: complete with status "+str(status)+" "+reason
    rospy.loginfo(msg)
    rospy.signal_shutdown(msg)
    while not rospy.is_shutdown():
      rospy.loginfo('await_simstart: waiting for ROS shutdown')
    rospy.sleep(1)
    sys.exit(status)

rospy.loginfo('**** await_simstart: sleeping ****')
# rospy.sleep relies on use_sim_time(!)

deadline_s = 100
def is_timeout():
    elapsed = time.time() - start_time
    print 'await_simstart: mode='+mode+' elapsed '+str(elapsed)
    return elapsed > deadline_s

start_time = time.time()
while (not rospy.is_shutdown()) and (not is_timeout()):
    time.sleep(1)

if rospy.is_shutdown():
    rospy.loginfo('await_simstart: shutdown')
if is_timeout():
    rospy.loginfo('await_simstart: de facto timeout')

# shutdown function does not seem to do this reliably(?)
rospy.loginfo('await_simstart: exiting with status '+str(exit_status))
time.sleep(5)
sys.exit(exit_status)
