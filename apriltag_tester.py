import rospy
import tf

from ar_track_alvar_msgs.msg import AlvarMarkers

from logger import Logger

class AprilTester():
    def __init__(self):
        """Set up a node for extracting roll/pitch/yaw data from AprilTag"""
        rospy.init_node('AprilTester', anonymous = False)
        self._logger = Logger("AprilTester")
        self._logger.info("hello world")
        
        self.tags = None
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.tagCallback, queue_size=1)
        
        rate = rospy.Rate(100)
    
        while not rospy.is_shutdown():
            #self.printOrientation()
            self.printPosition()
            rate.sleep()

    def tagCallback(self, data):
        """Extract Euler angle"""
        self.tags = data.markers if data.markers else None

    def printOrientation(self):
        if self.tags is None:
            return
        
        for tag in self.tags:
            q = tag.pose.pose.orientation
            euler_angle = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
            self._logger.debug(euler_angle, "orient of " + str(tag.id))

    def printPosition(self):
        if self.tags is None:
            return
        
        for tag in self.tags:
            p = tag.pose.pose.position
            self._logger.debug((p.x,p.y,p.z), "pos of " + str(tag.id))

    def printTag(self):
        """Print tag data with Euler Angle"""
        if self.tags is None:
            return

        for tag in self.tags:
            p = tag.pose.pose.position
            self._logger.debug((p.x,p.y,p.z), " posit of " + str(tag.id))
            q = tag.pose.pose.orientation
            euler_angle = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
            self._logger.debug(euler_angle, "orient of " + str(tag.id))

if __name__ == "__main__":
    AprilTester()
