#!/usr/bin/env python

import rospy
import actionlib
import multiprocessing
import time
import os

from random import randrange
from subprocess import check_output
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox

ROS_MASTER_IP = str(check_output("host vxlab-blue | sed 's/.* //g'", shell=True)).rstrip("\n")
ROS_MASTER_URI = "http://%s:11311" % ROS_MASTER_IP
os.environ["ROS_MASTER_IP"] = ROS_MASTER_IP
os.environ["ROS_MASTER_URI"] = ROS_MASTER_URI


class Goal:
    def __init__(self, location, x, y, z, w):
        self.location = location
        self.x = x
        self.y = y
        self.z = z
        self.w = w


static_goals = [Goal("Little Room", -5.5, -4.0, 0.0, 1.0), Goal("Window", -5.0, 3.0, 0.7, 0.7),
                Goal("Govlab", 4.5, -2.0, -0.7, 0.7),
                Goal("Door", 0.5, 5.0, 1.0, 0.0)]


def random_goal():
    return static_goals[randrange(len(static_goals))]


dynamic_goals = []
visited_goals = []


def callback(msg):
    for box in msg.boxes:
        new_goal = Goal("Human", box.pose.position.x, box.pose.position.y, box.pose.position.z,
                            box.pose.orientation.w)
        exists = False
        for d_g in dynamic_goals:
            print d_g.x
            print new_goal.x
            if abs(d_g.x - new_goal.x) < 1 and abs(d_g.y - new_goal.y) < 1:
                exists = True
                break
        if not exists:
            visited = False
            print len(visited_goals)
            for v_g in visited_goals:
                print v_g.x
                print new_goal.x
                print v_g.y
                print new_goal.y
                if abs(v_g.x - new_goal.x) < 1 and abs(v_g.y - new_goal.y) < 1:
                    visited = True
                    break
            if not visited:
                dynamic_goals.append(new_goal)
                rospy.loginfo("Goal added to dynamic collection.")
            else:
                rospy.loginfo("Goal has already been visited.")
        else:
            rospy.loginfo("Goal exists in dynamic collection.")


def movebase_client(goal):
    client = actionlib.SimpleActionClient('/blue/move_base', MoveBaseAction)
    rospy.loginfo("Waiting for server...")
    client.wait_for_server()

    move_goal = MoveBaseGoal()
    move_goal.target_pose.header.frame_id = "map"
    move_goal.target_pose.header.stamp = rospy.Time.now()
    move_goal.target_pose.pose.position.x = goal.x
    move_goal.target_pose.pose.position.y = goal.y
    move_goal.target_pose.pose.orientation.z = goal.z
    move_goal.target_pose.pose.orientation.w = goal.w

    rospy.loginfo("Target goal: %s" % goal.location)
    client.send_goal(move_goal)
    rospy.loginfo("Waiting for response...")
    result = client.wait_for_result()

    if not result:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


if __name__ == '__main__':
    goal = random_goal()
    rospy.init_node('delivery_client_py')
    # Subscribe to the human detector topic
    rospy.Subscriber("recognized_result", BoundingBoxArray, callback)

    while True:
        # Create goal process
        try:
            goal_result = movebase_client(goal)
            if goal_result:
                rospy.loginfo("Goal execution done")
            else:
                rospy.loginfo("Goal execution unsuccessful!")
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")

        if dynamic_goals:
            goal = dynamic_goals.pop()
            visited_goals.append(goal)
        else:
            goal = random_goal()
