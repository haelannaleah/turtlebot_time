import cv2
import rospy
import tf
import numpy as np
from copy import deepcopy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

from math import radians, atan2
from time import time

TURN_LEFT = 1
TURN_RIGHT = -1

class Motion():
    def __init__(self):

        # initialize class variables
        self.direction = 1
        self.move_cmd = Twist()
        self.accel_time = False
        
        self._ROT_SPEED = radians(60)
        self._LIN_SPEED = 0.2
        self._ACCEL_TIME = 0.1
        self._ACCEL_DELTA = 0.025
        

        # set up publisher/subscriber
        self.move_publisher = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

    def accelerate(self, delta):
        # initialize the acceleration time
        if self.accel_time is False:
            self.accel_time = time()
        
        # otherwise, if it's time to increment speed...
        elif time() - self.accel_time > self._ACCEL_TIME:
            self.move_cmd.linear.x += delta
            self.accel_time = False

    def avoidObstacle(self, rec_turn):
        
        # if we're still moving, we need to gracefully come to a stop
        if self.move_cmd.linear.x > 0:
            self.accelerate(-self._ACCEL_DELTA)
            self.move_cmd.angular.z = 0

        # otherwise, turn away!
        else:
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = rec_turn * self._ROT_SPEED

        self._publish()

    def stop(self, now=False):        
        if not now and self.move_cmd.linear.x > 0:
            self.accelerate(-self._ACCEL_DELTA)
        else:
            self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self._publish()

    def walk(self):
        if self.move_cmd.linear.x < self._LIN_SPEED:
            self.accelerate(self._ACCEL_DELTA)
        else:
            self.move_cmd.linear.x = self._LIN_SPEED
        self.move_cmd.angular.z = 0
        self._publish()

    def _publish(self):
        self.move_publisher.publish(self.move_cmd)

class Navigation(Motion):
    def __init__(self):
        Motion.__init__(self)

        self.start_pose = None
        self.cur_pose = None
        self.turn = None
        rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self._ekfCallback)

    def returnHome(self):
        # compute angle to home
        desired_turn = atan2(self.start_pose[0][1] - self.cur_pose[0][1], self.start_pose[0][0] - self.cur_pose[0][0])

        if np.isclose(self.cur_pose[0], self.start_pose[0], rtol=.01).all():
            print "YAYAYAYAYAYAYAYAYA"
            self.move_cmd.linear.x = 0
            self.move_cmd.angular.z = 0

        elif not np.isclose(self.cur_pose[1], desired_turn, rtol=0.1):
            if self.move_cmd.linear.x > 0:
                self.accelerate(-self._ACCEL_DELTA)
                self.move_cmd.angular.z = 0
                self.turn = None

            else:
                print "turning"
                if self.turn is None:
                    self.turn = TURN_LEFT if abs(self.cur_pose[1] - desired_turn) > abs(self.cur_pose[1] + desired_turn) else TURN_RIGHT
                self.move_cmd.linear.x = 0
                self.move_cmd.angular.z = self.turn * self._ROT_SPEED
            self._publish()

        else:
            print "walking home"
            self.walk()
            self.turn=None

        return self.turn

    def extractPose(self,p, q):
        """Given a quaternion q,extract an angle."""
        return ((p.x,p.y), tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[-1])

    def _ekfCallback(self, data):
        self.cur_pose = self.extractPose(data.pose.pose.position, data.pose.pose.orientation)
        
        if self.start_pose is None:
            self.start_pose = deepcopy(self.cur_pose)
