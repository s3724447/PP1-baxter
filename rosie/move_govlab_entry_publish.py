#!/usr/bin/env python

import rospy
import argparse
from geometry_msgs.msg import PoseStamped

DOOR_X = 1.19961261749
DOOR_Y = 0.817299365997
ORIENT_Z = 0.0448849449134
ORIENT_W = 0.998992162992

parser = argparse.ArgumentParser(prog='move_to.py', description='Navigation to location')
#parser.add_argument("-p", '--pushButton', action="store_true", help="")
#args = parser.parse_args();

# apparently this has to be global for publishing to work!?
global pub
#pub = rospy.Publisher('/rosie/move_base_simple/goal', PoseStamped, queue_size=0)
pub = rospy.Publisher('/rosie/move_base_simple/goal', PoseStamped, latch=True)

def movebase_client():
    global pub

    target_pose = PoseStamped()
    target_pose.header.frame_id = "map"
    timestamp = rospy.Time.now()
    # "time==0": http://wiki.ros.org/rospy/Overview/Time
    while timestamp.secs==0 and timestamp.nsecs==0:
	    timestamp = rospy.Time.now()
    target_pose.header.stamp = timestamp
    target_pose.pose.position.x = DOOR_X
    target_pose.pose.position.y = DOOR_Y
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = ORIENT_Z
    target_pose.pose.orientation.w = ORIENT_W
    pub.publish(target_pose) 
    print 'target pose published'

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py',anonymous=True)
        print 'node initialized'
        result = movebase_client()
        print 'movebase_client finished'
        rospy.sleep(60)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
