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

            if (self.sensors.wheeldrop or self.sensors.cliff):
                self.mover.stop(True)

            elif (self.sensors.bump):
                if self.mover.move_cmd.linear.x > 0:
                    self.mover.stop(True)
                else:
                    self.mover.avoidObstacle(self.sensors.rec_turn)

            elif (self.sensors.obstacle or self.sensors.bump):
                self.mover.avoidObstacle(self.sensors.rec_turn)
            
            else:
                self.mover.walk()

    def shutdown(self):
        rospy.loginfo("goodbye, world")
        self.mover.stop(True)
        rospy.sleep(2)

myBot = Adventurebot()

        
