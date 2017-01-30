import rospy
import tf

from ar_track_alvar_msgs.msg import AlvarMarkers
from math import degrees

from logger import Logger

class AprilTester():
    def __init__(self):
        """Set up a node for extracting roll/pitch/yaw data from AprilTag"""
        rospy.init_node('AprilTester', anonymous = False)
        self._logger = Logger("AprilTester")
        self._logger.info("hello world")
        
        self.tags = None
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.tagCallback, queue_size=1)
        
        self.rate = rospy.Rate(50)

    def tagCallback(self, data):
        """Extract tag data from the ar_pose_marker topic."""
        self.tags = data.markers if data.markers else None
    
    def prettyPrintData(self, data, name):
        self._logger.debug([", ".join("{: 6f}".format(elt) for elt in data)], name)
    
    def printOrientation(self, tag):
        """Print the AprilTag orientation as a Euler Angle in degrees."""
        q = tag.pose.pose.orientation
        euler_angle = [degrees(angle) for angle in tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])]
        self.prettyPrintData(euler_angle, "orient of " + str(tag.id))
    
    def printPosition(self, tag):
        p = tag.pose.pose.position
        self.prettyPrintData((p.x,p.y,p.z), "pos of " + str(tag.id))

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
    tester = AprilTester()
    
    # begin test
    while not rospy.is_shutdown():
        self.printTag()
        #self.testOrientation()
        #self.testPosition()
        rate.sleep()
