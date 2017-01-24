"""
every waypoint can represent a recentering oportunity
    if the hallway is wide enough, recenter, else don't
    
how does incrementally adding known things translate to MD setting
"""
import numpy as np
import rospy
import tf

from ar_track_alvar_msgs.msg import AlvarMarkers
from copy import deepcopy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
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
    
    def __init__(self, points, locations, neighbors, rooms, landmarks):
        
        # set up all the inherited variables from the motion class
        Motion.__init__(self)
        
        # set up class logger
        self._logger = Logger("Navigation")
        
        # set up floor plan and mapping
        self.floorPlan = FloorPlan(points, locations, neighbors, rooms, landmarks)

        self.cur_pose = None
        self.cur_position = None
        self.cur_orientation = None
        self.path = None
        self.destination = None
        self.avoiding = False
        self.avoid_time = None
        
        # set up landmark data
        self.landmarks = None
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self._aprilTagCallback, queue_size=1)
        self.landmark_publisher = rospy.Publisher('apriltags', PoseWithCovarianceStamped)
    
        # subscribe to location on map
        rospy.Subscriber('map_frame', PoseStamped, self._ekfCallback)
    
        self.tfListener = tf.TransformListener()

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
        cur_orientation = tf.transformations.euler_from_quaternion(self.cur_pose[1])[-1]

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
        return ((p.x, p.y), (q.x, q.y, q.z, q.w))
    
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
        tag_data.position.y = tag_data.position.x
        tag_data.position.x = tag_data.position.z
        
        # use our usual method to extract
        tag_pose = self.extractPose(tag_data.position, tag_data.orientation) 
        
        if not np.isclose(tag_pose[1], 0, atol=.05):
            self.turn(self._TURN_LEFT if tag_pose[1] < 0 else self._TURN_RIGHT)
            return False
        
        # convert the origin tag to a offset for setting the origin
        offset = (tag_pose[0][0]**2 + tag_pose[0][1]**2)**0.5
        x = self.cur_pose[0][0] + tag_pose[0][0] #offset * cos(tag_pose[1])
        y = self.cur_pose[0][1] + tag_pose[0][1] #offset * sin(tag_pose[1])
        angle = self.cur_pose[1] # + tag_pose[1]
        
        if angle > self._TWO_PI:
            angle -= self._TWO_PI
        elif angle < 0:
            angle += self._TWO_PI
            
        self.origin_pose =  ((x,y),angle)
        self._logger.debug(self.origin_pose, "origin_pose", "setOrigin")

    def _aprilTagCallback(self, data):
        """
        Process April tags. More info: https://piazza.com/class/ik07vwdrcls4pz?cid=66
        Note that April tag position data comes back in the base frame format
        
        """
        if data.markers:
            # only include
            tags = data.markers
            self.landmarks = [tag for tag in data.markers if tag.id in self.floorPlan.landmarks]
            print "tag visible"
            
            if len(self.landmarks) > 0:
                if self.cur_pose is not None:
                    self._publishLandmarks()
                else:
                    print "cur pose empty"
            else:
                self.landmarks = None
        else:
            self.landmarks = None
    
    def _publishLandmarks(self):
        """Publish information about current position based on landmarks."""
        
        # TODO: fun fun fun coordinate transformations between the map frame and the robot frame
        # should be a good time...
        
        # get the closest April tag, in case we see more than one
        nearby = min(self.landmarks, key = lambda t: t.pose.pose.position.x**2 + t.pose.pose.position.y**2)
        self._logger.info("Spotted landmark: " + str(nearby.id))
        
        # get the local position (such that the z and x axis aligns with the x and y axis in the global frame
        local_position = np.array([[nearby.pose.pose.position.z],[-nearby.pose.pose.position.x], [1]])
        
        # create translate matrix (to get back to the global original frame)
        theta = -radians(tf.transformations.euler_from_quaternion(self.cur_pose[1])[-1])
        cur_x, cur_y = self.cur_pose[0]
        translation_matrix = np.array([[cos(theta), -sin(theta), cur_x],
                                       [sin(theta), cos(theta) , cur_y],
                                       [0         , 0          , 1    ]])
        
        # note that in april tag messages, z position is forward displacement and x is horizontal displacement
#        tag_relative_position = (nearby.pose.pose.position.z, nearby.pose.pose.position.x)
#        try:
#            t = self.tfListener.getLatestCommonTime("/map", nearby.header.frame_id)
#            position, orientation = self.tfListener.lookupTransform("/map", nearby.header.frame_id, t)
#        
#            print position
#        
#        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#            self._logger.debug("Unable to publish landmark data: " + str(nearby))
#            return

        location_msg = PoseWithCovarianceStamped()
        location_msg.header.stamp = rospy.Time.now()
        location_msg.header.frame_id = 'apriltags'
        
        x, y, _ = np.dot(translation_matrix, local_position)

        location_msg.pose.pose.position.x = x
        location_msg.pose.pose.position.y = y
        location_msg.pose.pose.position.z = 0
#
#        # note that in april tag messages, z position is forward displacement and x is horizontal displacement
#        # in map pose messages, the x is forward displacement and the y is horizontal displacement
#        location_msg.pose.position.x = nearby.position.z + self.landmarks[nearby.id][0]
#        location_msg.pose.position.y = nearby.position.x + self.landmarks[nearby.id][1]
#        location_msg.pose.position.z = 0
#
        # TODO: There's some sort of transformation that needs to take place here
        #location_msg.pose.pose.orientation = nearby.pose.pose.orientation
        _, _, qz, qw = tf.transformations.quaternion_from_euler([0,0,theta])
        location_msg.pose.pose.orientation.z = qz
        location_msg.pose.pose.orientation.w = qw
        
        location_msg.pose.covariance = [0.0001, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0001, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0001, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0001, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0001, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0001]
        
        self.landmark_publisher.publish(location_msg)
        self._logger.debug("Published location: " + str(location_msg))

    
# publish message
#self.april_publisher.publish(msg)
    
    def _ekfCallback(self, data):
        """Extract current position and orientation data."""
        self.cur_pose = self.extractPose(data.pose.position, data.pose.orientation)
        # print tf.transformations.euler_from_quaternion(self.cur_pose[1])[-1]
        #if self.origin_pose is not None:
            #self._logger.debug(self.cur_pose, "cur_pose", "ekfCallback")
        #     #self.cur_pose = self.extractPose(data.pose.pose.position, data.pose.pose.orientation, ((0,self._BASE_WIDTH),0))
        #     self.origin_pose = self.extractPose(data.pose.pose.position, data.pose.pose.orientation, ((0,self._BASE_WIDTH),0))
           
        
        
