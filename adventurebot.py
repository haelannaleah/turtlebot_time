import cv2
import rospy
import numpy as np

from motion import Motion, Navigation
from sensing import Sensors

class Adventurebot():
    def __init__(self):
        rospy.init_node('Adventurebot', anonymous = False)
        
        # set up ctrl-C shutdown behavior
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("hello world")

        # set global refresh rate
        rate = rospy.Rate(10)

        self.mover = Navigation()
        self.sensors = Sensors()

        turn = None
        return_home = False
        while not rospy.is_shutdown():

            if (self.sensors.wheeldrop or self.sensors.cliff):
                return_home = True
                self.mover.stop(True)

            elif (self.sensors.bump):
                if self.mover.move_cmd.linear.x > 0:
                    self.mover.stop(True)
                else:
                    self.mover.avoidObstacle(self.sensors.rec_turn)

            elif (self.sensors.obstacle):
                self.mover.avoidObstacle(self.sensors.rec_turn)
            
            elif not return_home:
                self.mover.walk()

            else: 
                turn = self.mover.returnHome()

    def shutdown(self):
        rospy.loginfo("goodbye, world")
        self.mover.stop(True)
        rospy.sleep(2)

myBot = Adventurebot()

        
