import rospy
import tf

from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from math import degrees

from logger import Logger

METER_TO_INCH = 39.3701

class AprilTester():
    def __init__(self, metric = True):
        """Set up a node for extracting roll/pitch/yaw data from AprilTag"""
        
        rospy.init_node('AprilTester', anonymous = False)
        self._logger = Logger("AprilTester")
        self._logger.info("hello world")
        
        self.tags = None
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self._tagCallback, queue_size=1)
        self.landmark_publisher = rospy.Publisher('apriltags', PoseWithCovarianceStamped)
        self.landmark_broadcaster = tf.TransformBroadcaster()
    
        if metric:
            self.printOrientation = self.printOrientationMetric
            self.printPosition = self.printPositionMetric
        else:
            self.printOrientation = self.printOrientationImperial
            self.printPosition = self.printPositionImperial

    def _tagCallback(self, data):
        """Extract tag data from the ar_pose_marker topic."""
        if data.markers:
            self.tags = data.markers
            self._publishLandmarks()
        else:
            self.tags = None
    
    def _prettyPrintData(self, data, name):
        """Pretty print our data."""
        self._logger.debug(["{: 6f}".format(elt) for elt in data], name)
    
    def printOrientationImperial(self, tag):
        """Print the AprilTag orientation as a Euler Angle in degrees."""
        q = tag.pose.pose.orientation
        euler_angle = [degrees(angle) for angle in tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])]
        self._prettyPrintData(euler_angle, "orient of " + str(tag.id))
    
    def printOrientationMetric(self, tag):
        """Print the AprilTag orientation as a Euler Angle in radians."""
        q = tag.pose.pose.orientation
        euler_angle = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        self._prettyPrintData(euler_angle, "orient of " + str(tag.id))
    
    def printPositionImperial(self, tag):
        """Print position in meters (relative to the camera in inches)"""
        p = tag.pose.pose.position
        self.prettyPrintData([METER_TO_INCH * coord for coord in (p.x,p.y,p.z)], "   pos of " + str(tag.id))
    
    def printPositionMetric(self, tag):
        """Print position in meters (relative to the camera in meters)"""
        p = tag.pose.pose.position
        self.prettyPrintData((p.x,p.y,p.z), "   pos of " + str(tag.id))
    
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
       
        location_msg.pose.pose.position.x = 5#float(self.floorPlan.landmarks[nearby.id][0] - x)
        location_msg.pose.pose.position.y = 5#float(self.floorPlan.landmarks[nearby.id][1] - y)
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
        _, _, qz, qw = tf.transformations.quaternion_from_euler(0,0,theta)
        location_msg.pose.pose.orientation.z = 0 #qz
        location_msg.pose.pose.orientation.w = 1 #qw
       
        location_msg.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
       
        self.landmark_publisher.publish(location_msg)
        self.landmark_broadcaster.sendTransform((0,0,0), (0,0,0,1), rospy.Time.now(), 'apriltags', 'world')
       #        self.landmark_broadcaster((location_msg.pose.pose.position.x, location_msg.pose.pose.position.y,0),
       #                                  (0,0,location_msg.pose.pose.orientation.z, location_msg.pose.pose.orientation.w),
       #                                  'apriltags',
       #                                  'world')
       
        self._logger.debug("Published location: " + str(location_msg))
    
    
        # publish message
        #self.april_publisher.publish(msg)


    def testOrientation(self):
        if self.tags is None:
            return
        
        for tag in self.tags:
            self.printOrientation(tag)

    def testPosition(self):
        if self.tags is None:
            return
    
        for tag in self.tags:
            self.printPosition(tag)

    def printTag(self):
        """Print tag data with Euler Angle"""
        if self.tags is None:
            return

        for tag in self.tags:
            self.printPosition(tag)
            self.printOrientation(tag)

if __name__ == "__main__":
    # instantiate tester object
    tester = AprilTester(metric = False)
    rate = rospy.Rate(50)
    
    # begin test
    while not rospy.is_shutdown():
        tester.printTag()
        #tester.testOrientation()
        #tester.testPosition()
        rate.sleep()
