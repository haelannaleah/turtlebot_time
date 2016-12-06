import numpy as np
import rospy
import tf

from copy import deepcopy
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import radians, atan2, pi

import MD2
from MDgraph import FloorPlan
from motion import Motion

class Navigation(Motion):
    
    _HALF_PI = pi / 2.0
    _TWO_PI = 2.0 * pi
    
    def __init__(self):
        
        # set up all the inherited variables from the motion class
        Motion.__init__(self)
        
        # TODO: consider factoring out to make more general?
        self.floorPlan = FloorPlan(MD2.points, MD2.locations, MD2.neighbors, MD2.rooms)

        self.start_pose = None
        self.cur_pose = None
        
        self.path = None
        self.destination = None
        rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self._ekfCallback)

    def avoidObstacle(self, rec_turn):
        """Given a reccomended turn, avoid obstacle."""
        self.turn(rec_turn)

    def goToDestination(self, dest):
        """Travel to a destination via waypoints."""
        
        if self.cur_pose is None:
            return False
        
        # if we aren't already following a path, get a path
        if self.path is None:
            self.path = self.floorPlan.get_path(self.cur_pose[0], dest)
            print self.path
        
        # navigate to the current waypoint on the path
        if self.navigateToWaypoint(self.path[0]):
            # if we make it there, remove the current waypoint
            print(self.path.pop(0))
            print(self.cur_pose)
        
        # if the path is empty, we've reached our destination 
        if not self.path:
            self.path = None
            return True
        
        # still getting there
        return False

    def navigateToWaypoint(self, point):
        """
            Move from current position to desired waypoint.
            
            Args:
                point: An (x,y) float tuple representing a point relative to the origin.
            
            Returns:
                True if we are close to the desired location, False otherwise.
        """
        desired_turn = atan2(point[1] - self.cur_pose[0][1], point[0] - self.cur_pose[0][0])
        cur_orientation = self.cur_pose[1]

        # if both the vectors are in adjacent quadrants where the angles wrap around,
        # we need to make sure that they treat each other like adjacent quadrants
        if cur_orientation > self._HALF_PI and desired_turn < -self._HALF_PI:
            desired_turn += self._TWO_PI
        elif desired_turn > self._HALF_PI and cur_orientation < -self._HALF_PI:
            cur_orientation += self._TWO_PI
            
        if np.isclose(self.cur_pose[0], point, atol=.05).all():
            rospy.loginfo("start pose: " + str(point[0]))
            rospy.loginfo("cur_pose: " + str(self.cur_pose[0]))
            
            # we've more or less reached our waypoint!
            return True

        elif not np.isclose(cur_orientation, desired_turn, atol=0.01):
            self.turn(cur_orientation < desired_turn)

        else:
            self.walk()
            self.turn_dir = None
        
        # still navigating to the waypoint
        return False

    def returnHome(self):
        """Return to home base. Used for debugging purposes."""
        self.navigateToWaypoint((0,0))

    def extractPose(self, p, q, origin=((0,0),0)):
        """Extract current pose relative to the origin."""
        return ((p.x - origin[0][0], p.y - origin[0][1]), 
            tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[-1] - origin[1])
        
    def _ekfCallback(self, data):
        """Extract current position and orientation data."""
        if self.start_pose is None:
            self.start_pose = self.extractPose(data.pose.pose.position, data.pose.pose.orientation)
        
        self.cur_pose = self.extractPose(data.pose.pose.position, data.pose.pose.orientation, self.start_pose)