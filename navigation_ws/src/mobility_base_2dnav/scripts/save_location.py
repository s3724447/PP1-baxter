#!/usr/bin/python

import rospy, json, yaml, os
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

global last_known_location

def store_position(data):
    global last_known_location
    last_known_location = data

def save_location(data):
    global last_known_location

    locations = {}

    locations = loadLocations()

    # if last_known_location == None:
    #     print "No pose stored, move_base likely not running"
    #     return

    if data.data not in locations:
        locations[data.data] = msg2json(last_known_location)

        print locations

    with open(os.path.dirname(os.path.abspath(__file__))+'/../locations/vxlab.json', 'w') as fp:
        json.dump(locations, fp)


def loadLocations():
    with open(os.path.dirname(os.path.abspath(__file__))+"/../locations/vxlab.json", "r") as fp:
        data = json.load(fp)
    return data

def msg2json(msg):
   ''' Convert a ROS message to JSON format'''
   y = yaml.load(str(msg))
   return json.dumps(y)


if __name__ == "__main__":
    last_known_location = None
    rospy.init_node("location_save")
    rospy.Subscriber("/red/save_location_request", String, save_location)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, store_position)
    rospy.spin()