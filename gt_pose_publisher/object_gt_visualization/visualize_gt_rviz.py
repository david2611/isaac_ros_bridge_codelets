import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import numpy as np
import json
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--gt_file", help="location of .json file which contains all gt poses in the world coordinate frame")
parser.add_argument("--ros_topic", help="rostopic to be publishing to")
parser.add_argument("--header_frame", help="name of the header frame that poses are relative to")
args = parser.parse_args()

_RED_VALUES = [1,1,1,0,1,0,0,0.5,1,0.5,1,0,0,0.5,0.5,1, 1, 1]
_GREEN_VALUES = [1,1,0,1,0,1,0,1,0.5,0,0,0.5,1,0.5,1,0.5, 0.5, 0.25]
_BLUE_VALUES = [1,0,1,1,0,0,1,0,0,1,0.5,1,0.5,1,0.5,0.5, 0.25, 0.5]

_CLASS_IDXS = {"potted plant": 0, "keyboard": 1, "bed": 2, "sofa": 3, "book": 4, "tv": 5, "mouse": 6,
               "toilet": 7, "bottle": 8, "bowl": 9, "cup": 10, "laptop": 11, "chair": 12, "table": 13, "spoon": 14, "refrigerator": 15, "orange": 16}


def create_marker(object_dict, object_id):
  """
  Function for creating a marker from a given object dictionary
  """
  marker = Marker()
  marker.header.frame_id = args.header_frame
  marker.type = marker.CUBE
  marker.action = marker.ADD
  marker.pose.position.x = object_dict["centroid"][0]
  marker.pose.position.y = object_dict["centroid"][1]
  marker.pose.position.z = object_dict["centroid"][2]
  marker.scale.x = object_dict["extent"][0]*2
  marker.scale.y = object_dict["extent"][1]*2
  marker.scale.z = object_dict["extent"][2]*2
  marker.id = object_id

  if object_dict["class"] not in _CLASS_IDXS:
    import sys
    sys.exit("ERROR! Class {} not in CLASS_IDXS".format(object_dict["class"]))
  else:
    marker.color.a = 0.75
    marker.color.r = _RED_VALUES[_CLASS_IDXS[object_dict["class"]]]
    marker.color.g = _GREEN_VALUES[_CLASS_IDXS[object_dict["class"]]]
    marker.color.b = _BLUE_VALUES[_CLASS_IDXS[object_dict["class"]]]
  
  return marker


# Read the json file of ground-truth and get all gt objects
with open(args.gt_file, "r") as f:
  gt_dicts = json.load(f)

# Setup ros publishing requirements
publisher = rospy.Publisher(args.ros_topic, MarkerArray)
rospy.init_node("register")

# Create MarkerArray to hold all markers
marker_array = MarkerArray()

# Populate marker array with all markers
for gt_idx, gt_dict in enumerate(gt_dicts.values()):
  marker_array.markers.append(create_marker(gt_dict, gt_idx))

# Publish the markers (currently constantly publishing just to play safe)
while not rospy.is_shutdown():
  publisher.publish(marker_array)
  rospy.sleep(0.01)