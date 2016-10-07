import rospy
import cv2
import numpy as np
import tf

from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseWithCovarianceStamped
from kobuki_msgs.msg import BumperEvent, CliffEvent, WheelDropEvent
from sensor_msgs.msg import PointCloud2, LaserScan, Image

TURN_RIGHT = -1
TURN_LEFT = 1

class Sensors():

    def __init__(self):

        self._DIST_THRESH = 0.7
        self.bridge = CvBridge()

        # subscribe to depth data
        self.depth_img = None
        self.obstacle = False
        self.rec_turn = 1
        rospy.Subscriber('/camera/depth/image', Image, self._depthCallback)

        # subscribe to odometry
        self.start_pose = None
        self.cur_pose = None
        #rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self._ekfCallback)

        # subscribe to bump sensor
        self.bump = False
        self.bump_count = 0
        rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self._bumperCallback)

        # subscribe to cliff sensor
        self.cliff = False
        rospy.Subscriber('mobile_base/events/clif', CliffEvent, self._cliffCallback)

        # subscribe to wheel drop sensor
        self.wheeldrop = False
        rospy.Subscriber('mobile_base/events/wheel_drop', WheelDropEvent, self._wheelDropCallback)

    def _ekfCallback(self, data):
        if self.start_pose is None:
            self.start_pose = data.pose.pose
        
        
        self.cur_pose = data.pose.pose
        q = self.cur_pose.orientation
        print tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    def _bumperCallback(self, data):
        self.bump = (data.state == 1)
        self.bump_count += int(self.bump)
        
        rospy.logwarn("Bumper event: " + str(data))

    def _cliffCallback(self, data):
        if self.wheeldrop:
            return
        
        if not self.cliff:
            self.cliff = True
        
        rospy.logwarn("Cliff event: " + str(data.CLIFF))

    def _wheelDropCallback(self, data):
        self.wheeldrop = (data.state == WheelDropEvent.DROPPED)

        rospy.logwarn("Wheel drop event: " + str(data.wheel))

    def _depthCallback(self, data):
        global TURN_LEFT, TURN_RIGHT
        self.depth_img = self.bridge.imgmsg_to_cv2(data, 'passthrough')

        # get slice to check distance on
        img_height, img_width = self.depth_img.shape
        s_height, s_width = (img_height, img_width * .3)
        w_center = img_width // 2

        # apply blur to smooth out irregularities
        sample = cv2.medianBlur(self.depth_img[0:s_height, w_center - s_width : w_center + s_width], 5)

        # check distance to closest object
        try:
            min_index = np.nanargmin(sample[np.nonzero(sample)])
        except ValueError:
            rospy.logerr("Encountered all NaN slice in depth image.")
            self.obstacle = True
            return

        min_index = np.unravel_index(min_index, sample.shape)

        if sample[min_index] < self._DIST_THRESH:
            if not self.obstacle and not self.bump:
                self.rec_turn = TURN_RIGHT if min_index[1] < w_center else TURN_LEFT
                self.obstacle = True
        else:
            self.obstacle = False
        
