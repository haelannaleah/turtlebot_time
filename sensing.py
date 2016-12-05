import cv2
import numpy as np
import rospy
import tf

from ar_track_alvar_msgs.msg import AlvarMarkers
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from kobuki_msgs.msg import BumperEvent, CliffEvent, WheelDropEvent
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from time import time

class Sensors():
    
    _DIST_THRESH =  0.7
    _SAMPLE_WIDTH = 0.3
    
    def __init__(self):

        self.bridge = CvBridge()

        # subscribe to depth data
        self.depth_img = None
        self.obstacle = False
        self.rec_turn = False # true if left, false if right
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
        
        # subscribe to April Tag data
        self.april_tags = None
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self._aprilTagCallback, queue_size=1)
        
        # publish landmark data
        self.april_publisher = rospy.Publisher('/vo', Odometry, queue_size = 10)
    
    def _aprilTagCallback(self, data):
        """Process April tags. More info: https://piazza.com/class/ik07vwdrcls4pz?cid=66"""
        if data.markers:
            self.april_tags = data.markers
            print self.april_tags
            self.landmarkPublisher()
        else:
            self.april_tags = None
    
    def landmarkPublisher(self):
        """Publish information about current position based on landmarks."""
        
        # get the closest April tag, in case we see more than one
        # TODO: make sure we only recognize those in our map dictionary
        nearby = min(self.april_tags, key = lambda t: t.pose.pose.position.x**2 + t.pose.pose.position.y**2)
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'AprilTags'
        msg.pose.pose.position.x = nearby.pose.pose.position.x #+ self.landmarks[nearby.id][0]
        msg.pose.pose.position.y = nearby.pose.pose.position.y #+ self.landmarks[nearby.id][1]
        msg.pose.pose.position.y = 0
        
        # TODO: incorporate landmark map data
        msg.pose.pose.orientation.x = 1
        msg.pose.pose.orientation.y = 0
        msg.pose.pose.orientation.z = 0
        msg.pose.pose.orientation.w = 0
        
        # landmark position known with high confidence
        msg.pose.covariance = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # TODO: perhaps this needs to live in navigation?                 
        #msg.twist.twist = self.move_cmd
        msg.twist.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # publish message
        self.april_publisher.publish(msg)
    
    def _bumperCallback(self, data):
        """Handle bump events."""
        self.bump = (data.state == 1)
        self.bump_count += int(self.bump)
        
        rospy.logwarn("Bumper event: " + str(data))

    def _cliffCallback(self, data):
        """Handle cliffs."""
        if self.wheeldrop:
            return
        
        if not self.cliff:
            self.cliff = True
        
        rospy.logwarn("Cliff event: " + str(data.CLIFF))

    def _depthCallback(self, data):
        """Process depth data. Detect obstacles."""
        
        # get the depth image
        self.depth_img = self.bridge.imgmsg_to_cv2(data, 'passthrough')

        # get slice to check distance on
        img_height, img_width = self.depth_img.shape
        s_height, s_width = (img_height, img_width * self._SAMPLE_WIDTH)
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

        # convert into a usable type of index
        min_index = np.unravel_index(min_index, sample.shape)

        # if the closest thing in our slice is too close, likely an obstacle
        if sample[min_index] < self._DIST_THRESH:
            if not self.obstacle and not self.bump:
                self.rec_turn = min_index[1] > w_center
                self.obstacle = True
        else:
            self.obstacle = False
        
    def _ekfCallback(self, data):
        """Extract current position and orientation data."""
        p, q = (data.pose.pose.position, data.pose.pose.orientation)
        self.cur_pose = ((p.x,p.y), tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[-1])
        
        if self.start_pose is None:
            self.start_pose = deepcopy(self.cur_pose)
            
    def _wheelDropCallback(self, data):
        """Handle wheel drops."""
        self.wheeldrop = (data.state == WheelDropEvent.DROPPED)

        rospy.logwarn("Wheel drop event: " + str(data.wheel))