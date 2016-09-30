import cv2
import rospy
from geometry messages.msg import Twist

from math import radians
from time import time

class Motion():
    def __init__(self):

        # initialize class variables
        self.direction = 1
        self.move_cmd = Twist()
        self.accel_time = False
        
        self._ROT_SPEED = radians(60)
        self._LIN_SPEED = 0.1
        self._ACCEL_TIME = 0.1
        self._ACCEL_DELTA = 0.025
        

        # set up publisher/subscriber
        self.move_publisher = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

    def accelerate(self, delta):
        # initialize the acceleration time
        if self.accel_time is False:
            self.accel_time = time()
        
        # otherwise, if it's time to increment speed...
        elif time() - self.accel_time > self.ACCEL_TIME:
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
            self.move_cmd.angular = rec_turn * self._ROT_SPEED

        self._publish()

    def stop(self, now=False):        
        if not now and self.move_cmd.linear.x > 0:
            self.accelerate(-self._ACCEL)
        else:
            self.move_cmd.linear.x = 0
        self.move_cmd.angular.z = 0
        self._publish()

    def walk(self):
        if self.move_cmd.linear.x < self._LIN_SPEED:
            self.accelerate(self._ACCEL_DELTA)
        else:
            self.move_cmd.linear.x = self._LIN_SPEED
        self.move_cmd.angula.z = 0
        self._publish()

    def _publish(self):
        self.move_publisher.publish(self.move_cmd)
