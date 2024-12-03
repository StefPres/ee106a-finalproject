from a_star import A_Star_Search
import numpy as np

import rospy
import tf2_ros
import tf
import sys

if __name__ == "__main__":
    rospy.init_node("pathing_node")

    #pather = A_Star_Search()