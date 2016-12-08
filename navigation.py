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

        self.origin_pose = None
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

        elif not np.isclose(cur_orientation, desired_turn, rtol=0.1):
            self.turn(cur_orientation < desired_turn)

        else:
            self.walk()
            self.turn_dir = None
        
        # still navigating to the waypoint
        return False

    def returnHome(self):
        """Return to home base. Used for debugging purposes."""
        self.navigateToWaypoint((0,0))

    def extractPose(self, p, q, origin=None):
        """Extract current pose relative to the origin."""
        if origin is None:
            origin = ((0,0),0)
            
        return ((p.x - origin[0][0], p.y - origin[0][1]), 
            tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[-1] - origin[1])
    
    def setOrigin(self, april_tags):
        if april_tags is None:
            self.turn(self.direction, .25)
            return False
        
        # extract the origin tag
        tag_data = next(tag.pose.pose for tag in april_tags if tag.id == 10)#self.origin_id)
        tag_pose = self.extractPose(tag_data.position, tag_data.orientation) 
        
        # convert the origin tag to a offset for setting the origin
        offset = (tag_pose[0][0]**2 + tag_pose[0][1]**2)**0.5
        x = self.cur_pose[0][0] - tag_pose[0][0] #offset * cos(tag_pose[1])
        y = self.cur_pose[0][1] - tag_pose[0][1] #offset * sin(tag_pose[1])
        angle = self.cur_pose[1] - tag_pose[1] - pi
        
        if angle > self._TWO_PI:
            angle -= self._TWO_PI
        elif angle < 0:
            angle += self._TWO_PI
            
        self.origin_pose =  ((x,y),angle)
        print self.cur_pose
        print self.origin_pose
    
    def _ekfCallback(self, data):
        """Extract current position and orientation data."""
        #if self.origin_pose is None:
        #    self.origin_pose = self.extractPose(data.pose.pose.position, data.pose.pose.orientation)
        self.cur_pose = self.extractPose(data.pose.pose.position, data.pose.pose.orientation, self.origin_pose)