#!/usr/bin/python
import rospy
import tf2_ros
import tf
import sys
import numpy as np

from a_star import A_Star_Search
from occupancy_grid_2d import OccupancyGrid2d
from trajectory import plan_curved_trajectory

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist, Point
#from ar_track_alvar_msgs.msg import Alvarmarkers
from std_msgs.msg import ColorRGBA
from turtlebot_control import controller

# TODO: Add recalculating path
class PathController:
    def __init__(self):
        # Set up tf buffer and listener.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

    def Initialize(self):
        self.og = OccupancyGrid2d()
        if not self.og.Initialize():
            return False

        if not self.SetupCallbacks():
            return False
        
        self.AStar = A_Star_Search(self.og.ExportGrid(), [0, 0], [10, 10])

        # set up relevant frames and topics
        self._sensor_frame = rospy.get_param("~frames/sensor")
        self._fixed_frame = rospy.get_param("~frames/fixed")
        self._robot_frame = rospy.get_param("~frames/robot")
        self._target_frame = rospy.get_param("~frames/target")

        self._sensor_topic = rospy.get_param("~topics/sensor")
        self._vis_topic = rospy.get_param("~topics/vis")

        return True
    
    # TODO: finish
    def SetupCallbacks(self):
        self.gridupdater = rospy.Subscriber(self.og._vis_topic,Marker,self.UpdateGrid,queue_size=1)
        self.turtlebotcontroller = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        return True
    
    # TODO: finish
    def UpdateGrid(self, msg):
        newgrid = self.ConvertGrid(self.og.ExportGrid())
        self.AStar.update_grid(newgrid)
        try:
            pose = self._tf_buffer.lookup_transform(
                self._fixed_frame, self._sensor_frame, rospy.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            # Writes an error message to the ROS log but does not raise an exception
            rospy.logerr("%s: Could not extract pose from TF.")
            return

        sensor_x = pose.transform.translation.x
        sensor_y = pose.transform.translation.y
        src = self.og.PointToVoxel(sensor_x, sensor_y)
        self.AStar.update_start(src)

        try:
            arpose = self._tf_buffer.lookup_transform(
                self._robot_frame, self._target_frame, rospy.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            # Writes an error message to the ROS log but does not raise an exception
            #rospy.logerr("%s: Could not extract pose from TF.")
            return

        dest_x = arpose.transform.translation.x
        dest_y = arpose.transform.translation.y

        dest = self.og.PointToVoxel(dest_x, dest_y)
        self.AStar.update_dest(dest)
        
        self.og.set_destination((dest[0],dest[1]))

        self.ExecuteTrajectory()

    # TODO: get the true position of the destination. this is hard-coded for testing.
    def updateDestination(self, msg):
        if not msg.markers:
            stopRobot()
            return
        
        for marker in msg.markers:
            if marker.id == self.target_tag_id:
                tagloc = self._tf_buffer.lookup_transform(
                    self._fixed_frame, self.target_tag_id, rospy.Time())
                
                self.AStar.update_dest([10, 10])

    def ConvertGrid(self, grid):
        newgrid = np.zeros((grid.shape[0], grid.shape[1]))
        for x in range(grid.shape[0]):
            for y in range(grid.shape[1]):
                if grid[x,y] < 0.6:
                    newgrid[x,y] = 1
        return newgrid

    # TODO: this is terrible
    def ExecuteTrajectory(self):
        path = self.AStar.search()
        if not path:
            return
        print("Path confirmed: ")
        print(path)
        
        path.pop(0)
        for point in path:
            print(f"Ready to execute path to point {point}")
            print("Press 'y' to continue, press 'n' to quit.")
            answer = input()
            if not answer.lower() == 'y':
                return
            nextpoint = self.og.VoxelCenter(point[0],point[1])
            trajectory = plan_curved_trajectory(nextpoint)
            for waypoint in trajectory:
                controller(waypoint)
        self.og.end_destination()
        self.stopRobot()

    def stopRobot(self):
        tw = Twist()
        tw.linear.x = 0.0
        tw.angular.z = 0.0
        self.turtlebotcontroller.publish(tw)

