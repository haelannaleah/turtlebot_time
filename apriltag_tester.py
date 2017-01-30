import rospy
import tf

from ar_track_alvar_msgs.msg import AlvarMarkers
from math import degrees

from logger import Logger

class AprilTester():
    def __init__(self, metric = True):
        """Set up a node for extracting roll/pitch/yaw data from AprilTag"""
        rospy.init_node('AprilTester', anonymous = False)
        self._logger = Logger("AprilTester")
        self._logger.info("hello world")
        
        self.tags = None
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.tagCallback, queue_size=1)
    
        if metric:
            self.printOrientation = self.printOrientationMetric
            self.printPosition = self.printPositionMetric
        else:
            self.printOrientation = self.printOrientationImperial
            self.printPosition = self.printPositionImperial

    def tagCallback(self, data):
        """Extract tag data from the ar_pose_marker topic."""
        self.tags = data.markers if data.markers else None
    
    def prettyPrintData(self, data, name):
        """Pretty print our data."""
        self._logger.debug(["{: 6f}".format(elt) for elt in data], name)
    
    def printOrientationImperial(self, tag):
        """Print the AprilTag orientation as a Euler Angle in degrees."""
        q = tag.pose.pose.orientation
        euler_angle = [degrees(angle) for angle in tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])]
        self.prettyPrintData(euler_angle, "orient of " + str(tag.id))
    
    def printOrientationMetric(self, tag):
        """Print the AprilTag orientation as a Euler Angle in radians."""
        q = tag.pose.pose.orientation
        euler_angle = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
        self.prettyPrintData(euler_angle, "orient of " + str(tag.id))
    
    def printPositionImperial(self, tag):
        """Print position in meters (relative to the camera in feet)"""
        p = tag.pose.pose.position
        self.prettyPrintData([3.28084 * coord for coord in (p.x,p.y,p.z)], "   pos of " + str(tag.id))
    
    def printPositionMetric(self, tag):
        """Print position in meters (relative to the camera in meters)"""
        p = tag.pose.pose.position
        self.prettyPrintData((p.x,p.y,p.z), "   pos of " + str(tag.id))

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
