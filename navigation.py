"""
every waypoint can represent a recentering oportunity
    if the hallway is wide enough, recenter, else don't
    
how does incrementally adding known things translate to MD setting
"""
import numpy as np
import rospy
import tf

from copy import deepcopy
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import radians, atan2, cos, sin, pi
from time import time

from logger import Logger
from MDgraph import FloorPlan
from motion import Motion

class Navigation(Motion):
    
    _HALF_PI = pi / 2.0
    _TWO_PI = 2.0 * pi
    _AVOID_TIME = 1.5
    _BASE_WIDTH = 0.1778
    
    def __init__(self, points, locations, neighbors, rooms, origin_id):
        
        # set up all the inherited variables from the motion class
        Motion.__init__(self)
        
        # set up class logger
        self._logger = Logger("Navigation")
        
        self._logger.debug(points,"points")
        self._logger.debug(locations,"locations")
        self._logger.debug(neighbors,"neighbors")
        
        self.floorPlan = FloorPlan(points, locations, neighbors, rooms)
        self.origin_id = origin_id

        self.origin_pose = None
        self.cur_pose = None
        
        self.path = None
        self.destination = None
        self.avoiding = False
        self.avoid_time = None
        
        rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self._ekfCallback)

    def avoidObstacle(self, rec_turn):
        """Given a reccomended turn, avoid obstacle."""
        self.turn(rec_turn)
        
    def center(self):
        """Center robot in an open space, if possible."""
        # TODO
        pass
        
    def intelligentAvoid(self, rec_turn, obstacle):
        """Navigate around an obstacle (hopefully)"""
        self.avoiding = True
        if obstacle:
            self.turn(rec_turn)
            self.avoid_time = None
            
        elif self.avoid_time is None:
            self.avoid_time = time()
            
        elif time() - self.avoid_time >= self._AVOID_TIME:
            #self.setPath()
            self._logger.debug(self.cur_pose, "cur_pose", "intelligentAvoid")
            self._logger.debug(self.path[0], "next_pose", "intelligentAvoid")
            self.avoiding = False
            
        else:
            self.walk()

    def setPath(self):
        """Compute path to current destination based on current position."""
        
        if self.destination is not None and self.cur_pose is not None:
            self.path = self.floorPlan.get_path(self.cur_pose[0], self.destination)
            return True
        return False
        
    def goToDestination(self, dest):
        """Travel to a destination via waypoints."""
        
        if self.cur_pose is None:
            return False
        
        # if we aren't going somewhere, set a destination
        if self.destination is None:
            self.destination = dest
            
            # try to plan a path there; if it works, we're golden
            if not self.setPath():
                self.destination = None
                return False
                
            self._logger.debug(self.path, "path", "goToDestination")
        
        # navigate to the current waypoint on the path
        if self.navigateToWaypoint(self.path[0]):
            # if we make it there, remove the current waypoint
            self._logger.debug(self.cur_pose, "cur_pose", "goToDestination")
            self._logger.debug(self.path.pop(0), "cur_waypoint", "goToDestination")
        
        # if the path is empty, we've reached our destination 
        if not self.path:
            self.path = None
            self.destination = None
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
            self._logger.debug(point[0], "start_position", "navigateToWaypoint")
            self._logger.debug(self.cur_pose[0], "cur_pose", "navigateToWaypoint")
            
            # we've more or less reached our waypoint!
            return True

        elif not np.isclose(cur_orientation, desired_turn, atol=0.15):
            self.turn(cur_orientation < desired_turn, .5)

        else:
            self.walk()
            self.turn_dir = None
        
        # still navigating to the waypoint
        return False

    def returnHome(self):
        """Return to home base. Used for debugging purposes."""
        self.goToDestination((0,0))

    def extractPose(self, p, q, origin=None):
        """Extract current pose relative to the origin."""
        angle = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[-1]
        if origin is None:
            return ((p.x, p.y), angle)
        else:
            return ((p.x - origin[0][0], p.y - origin[0][1]), angle - origin[1])
    
    def setOrigin(self, april_tags):
        """
            Set robot origin given april tag specifying origin.
            NOTE: Map coordinate frame should reflect that the robot is facing the April tag when it sets origin:
                Positive x should be in the direction of the april tag (intersecting the april tag perpendicularly)
                Positive y should be to the left of the april tag (running parallel to the wall of the april tag)
        """
        if self.cur_pose is None:
            return False
            
        elif april_tags is None:
            self.turn(self.direction, .5)
            return False
        
        # extract the origin tag; the april tag data comes back in a different format than odometry
        tag_data = next(tag.pose.pose for tag in april_tags if tag.id == self.origin_id)
        
        # set the x position to the forward displacement, and y to the horizontal displacement
        # consider transform listener
        tag_data.position.x = tag_data.position.z
        tag_data.position.y = tag_data.position.x
        
        # use our usual method to extract
        tag_pose = self.extractPose(tag_data.position, tag_data.orientation) 
        
        if not np.isclose(tag_pose[1], 0, atol=.05):
            self.turn(self._TURN_LEFT if tag_pose[1] < 0 else self._TURN_RIGHT)
            return False
        
        # convert the origin tag to a offset for setting the origin
        offset = (tag_pose[0][0]**2 + tag_pose[0][1]**2)**0.5
        x = self.cur_pose[0][0] + offset * cos(tag_pose[1])
        y = self.cur_pose[0][1] + offset * sin(tag_pose[1])
        angle = self.cur_pose[1] + tag_pose[1]
        
        if angle > self._TWO_PI:
            angle -= self._TWO_PI
        elif angle < 0:
            angle += self._TWO_PI
            
        self.origin_pose =  ((x,y),angle)
        self._logger.debug(self.origin_pose, "origin_pose", "setOrigin")
    
    def _ekfCallback(self, data):
        """Extract current position and orientation data."""
        self.cur_pose = self.extractPose(data.pose.pose.position, data.pose.pose.orientation, self.origin_pose)
        if self.origin_pose is not None:
            self._logger.debug(self.cur_pose, "cur_pose", "ekfCallback")
        #     #self.cur_pose = self.extractPose(data.pose.pose.position, data.pose.pose.orientation, ((0,self._BASE_WIDTH),0))
        #     self.origin_pose = self.extractPose(data.pose.pose.position, data.pose.pose.orientation, ((0,self._BASE_WIDTH),0))
           
        
        
