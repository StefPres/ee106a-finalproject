import rospy
import tf2_ros
import tf
import sys
import numpy as np

from a_star import A_Star_Search
from occupancy_grid_2d import OccupancyGrid2d

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# TODO: Add turtlebot commands, trajectory planning, recalculating path
class PathController:
    def __init__(self):
        self._intialized = False

        # Set up tf buffer and listener.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    def Initialize(self):
        self.og = OccupancyGrid2d()
        if not self.og.Initialize():
            return False

        if not self.SetupCallbacks():
            return False
        
        self.AStar = A_Star_Search(self.og.ExportGrid(), [0, 0], [0, 1])
        self.path = self.AStar.search()

        # set up relevant frames and topics
        self._sensor_frame = rospy.get_param("~frames/sensor")
        self._fixed_frame = rospy.get_param("~frames/fixed")
        #self._camera_frame = rospy.get_param("~frames/camera")

        self._sensor_topic = rospy.get_param("~topics/sensor")
        self._vis_topic = rospy.get_param("~topics/vis")
        #self._camera_topic = rospy.get_param("~topics/camera")

        return True
    
    # TODO: finish
    def SetupCallbacks(self):
        self.gridupdater = rospy.Subscriber(self.og._vis_topic,Marker,self.UpdateGrid,queue_size=1)

        return True
    
    # TODO: finish
    def updateGrid(self, msg):
        self.AStar.update_grid(self.og.ExportGrid())
        try:
            pose = self._tf_buffer.lookup_transform(
                self._fixed_frame, self._sensor_frame, rospy.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            # Writes an error message to the ROS log but does not raise an exception
            rospy.logerr("%s: Could not extract pose from TF.", self._name)
            return

        sensor_x = pose.transform.translation.x
        sensor_y = pose.transform.translation.y
        src = self.og.PointToVoxel(sensor_x, sensor_y)
        self.AStar.update_start(src)

    # TODO: get the true position of the destination. this is hard-coded for testing.
    def updateDestination(self, msg):
        self.AStar.update_dest([0, 1])
    
    
        