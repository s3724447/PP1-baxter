#!/usr/bin/python

import rospy, json, yaml, os
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import String

global last_known_location

def stop():
    global last_known_location
    current_position = PoseStamped()

    current_position.header.stamp = rospy.get_rostime()
    current_position.header.frame_id = last_known_location.header.frame_id
    current_position.pose = last_known_location.pose.pose

    goal.publish(current_position)

def go_to(data):
    key = data.data

    if key == "stop":
        stop()
        return

    locations = loadLocations()
    if key not in locations:
        return
    
    pose_json = json.loads(locations[key])

    target = PoseStamped()

    target.header.stamp = rospy.get_rostime()
    target.header.frame_id = pose_json["header"]["frame_id"]
    target.pose.position.x = pose_json["pose"]["pose"]["position"]["x"]
    target.pose.position.y = pose_json["pose"]["pose"]["position"]["y"]
    target.pose.position.z = pose_json["pose"]["pose"]["position"]["z"]
    target.pose.orientation.x = pose_json["pose"]["pose"]["orientation"]["x"]
    target.pose.orientation.y = pose_json["pose"]["pose"]["orientation"]["y"]
    target.pose.orientation.z = pose_json["pose"]["pose"]["orientation"]["z"]
    target.pose.orientation.w = pose_json["pose"]["pose"]["orientation"]["w"]

    goal.publish(target)

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
    rospy.init_node("go_to_location")
    rospy.Subscriber("/red/go_to_location", String, go_to)
    rospy.Subscriber("/red/save_location_request", String, save_location)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, store_position)
    goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, latch=True, queue_size=1)
    rospy.spin()