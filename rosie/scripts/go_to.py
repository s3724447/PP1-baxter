#!/usr/bin/env python

import rospy
import actionlib
import argparse

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

parser = argparse.ArgumentParser(prog='move_to.py', description='Navigation to location')
parser.add_argument("robot", type=str, help="Robot name")
parser.add_argument("x", type=float, help="x coordinate")
parser.add_argument("y", type=float, help="y coordinate")
parser.add_argument("z", type=float, help="z orientation")
parser.add_argument("w", type=float, help="w orientation")
args = parser.parse_args()

def movebase_client():
    client = actionlib.SimpleActionClient('/%s/move_base' % args.robot, MoveBaseAction)
    rospy.loginfo("Waiting for server...")
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = args.x
    goal.target_pose.pose.position.y = args.y
    goal.target_pose.pose.orientation.z = args.z
    goal.target_pose.pose.orientation.w = args.w

    client.send_goal(goal)
    rospy.loginfo("Waiting for response...")
    result = client.wait_for_result()
    if not result:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py', anonymous=True)
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done")
        #    if args.pushButton:
        #         rospy.loginfo("Calling beerPusher.py to push the button...")
        #         beerPusher.main()
        else:
            rospy.loginfo("Goal execution unsuccessful!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")