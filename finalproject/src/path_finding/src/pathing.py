#!/usr/bin/python
from pathcontroller import PathController
import numpy as np

import rospy
import tf2_ros
import tf
import sys

if __name__ == "__main__":
    rospy.init_node("pathing_node")

    paththing = PathController()
    if not paththing.Initialize():
        rospy.logerr("Failed to initialize the mapping node.")
        sys.exit(1)

    rospy.spin()