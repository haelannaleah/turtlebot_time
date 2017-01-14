import cv2
import rospy
import numpy as np

import MD2
from logger import Logger
from navigation import Navigation
from sensing import Sensors

import tf

class Adventurebot():
    def __init__(self):
        rospy.init_node('Adventurebot', anonymous = False)
        
        # set up ctrl-C shutdown behavior
        rospy.on_shutdown(self.shutdown)
        self._logger = Logger("Adventurebot")
        
        self._logger.info("hello world")

        # set global refresh rate
        rate = rospy.Rate(10)

        self.mover = Navigation(MD2.points, MD2.locations, MD2.neighbors, MD2.rooms, 10)
        self.sensors = Sensors()

        turn = None
        return_home = False
        while not rospy.is_shutdown():
            listener = tf.TransformListener()
            if True:
                now = rospy.Time(0)
                (trans, rot) = listener.lookupTransform("/map", "/base_link", now)
                print trans
                print rot
                
            elif (self.sensors.wheeldrop or self.sensors.cliff):
                return_home = True
                self.mover.stop(True)

            elif (self.sensors.bump):
                if self.mover.move_cmd.linear.x > 0:
                    self.mover.stop(True)
                else:
                    self.mover.avoidObstacle(self.sensors.rec_turn)
                    
            elif (self.mover.origin_pose is None):
                # look for april tag
                self.mover.setOrigin(self.sensors.april_tags)

            elif ((self.sensors.obstacle and self.mover.walking) or self.mover.avoiding):
                self.mover.intelligentAvoid(self.sensors.rec_turn, self.sensors.obstacle)
            
            elif return_home:
                if (self.mover.goToDestination((0,0))):
                    return_home = False
                    self._logger.info("Returned home.")
                
            else:
                #return_home = self.mover.navigateToWaypoint((0,1))
                if self.mover.goToDestination((-1.2192, -7.3152, )):
                    return_home = True
                    self._logger.info("Reached destination")
                #return_home = self.mover.goToDestination((-7.3152,1.2192))
        rate.sleep()

    def shutdown(self):
        self._logger.info("goodbye, world")
        self.mover.stop(True)
        rospy.sleep(2)

myBot = Adventurebot()

        
