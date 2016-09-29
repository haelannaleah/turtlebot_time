
import cv2
import rospy
import numpy as np

from motion import Motion
from sensor import Sensing

class Adventurebot():
    def __init__(self):
        rospy.init_node('Adventurebot', anonymous = False)
        
        # set up ctrl-C shutdown behavior
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("hello world")

        # set global refresh rate
        rate = rospy.Rate(10)

        self.mover = Motion()
        self.sensors = Sensing()

        while not rospy.is_shutdown():

            if (sensors.wheeldrop):
                self.mover.stop()

            elif (sensors.obstacle):
                self.mover.direction = sensors.reccomended_turn
                mover.avoidObstable()
            
            else:
                mover.walk()

        
