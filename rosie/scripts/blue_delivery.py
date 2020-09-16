#!/usr/bin/env python

import rospy
import actionlib
import multiprocessing
import time
import os

from random import randrange
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from subprocess import check_output

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


goals = [Goal("Little Room", -5.5, -4.0, 0.0, 1.0), Goal("Window", -5.0, 3.0, 0.7, 0.7),
         Goal("Govlab", 4.5, -2.0, -0.7, 0.7),
         Goal("Door", 0.5, 5.0, 1.0, 0.0)]


class GoalProcess(multiprocessing.Process):
    def __init__(self, goal=None):
        multiprocessing.Process.__init__(self)
        if goal:
            self.goal = goal
        else:
            self.goal = self.random_goal()

    def run(self):
        try:
            rospy.init_node('movebase_client_py')
            result = movebase_client(self.goal)
            if result:
                rospy.loginfo("Goal execution done")
            else:
                rospy.loginfo("Goal execution unsuccessful!")
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")

    def random_goal(self):
        return goals[randrange(len(goals))]


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
    while True:
        goalProcess = GoalProcess()
        goalProcess.start()
        # While no humans detected - temporarily set to goal thread until human detector is working
        while goalProcess.is_alive():
            print "Separate process that can interrupt blue and redirect to human"
            time.sleep(5)
        goalProcess.terminate()
