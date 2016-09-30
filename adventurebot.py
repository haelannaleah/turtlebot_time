
import cv2
import rospy
import numpy as np

from motion import Motion
from sensing import Sensors

class Adventurebot():
    def __init__(self):
        rospy.init_node('Adventurebot', anonymous = False)
        
        # set up ctrl-C shutdown behavior
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("hello world")

        # set global refresh rate
        rate = rospy.Rate(10)

        self.mover = Motion()
        self.sensors = Sensors()

        while not rospy.is_shutdown():

            if (self.sensors.wheeldrop):
                self.mover.stop(True)

            elif (self.sensors.obstacle):
                self.mover.avoidObstable(self.sensors.reccomended_turn)
            
            else:
                self.mover.walk()

    def shutdown(self):
        rospy.loginfo("goodbye, world")
        self.mover.stop(True)
        rospy.sleep(2)

myBot = Adventurebot()

        
