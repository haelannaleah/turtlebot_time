import cv2
import rospy
import numpy as np

from navigation import Navigation
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
                    
            elif (self.mover.origin_pose is None):
                # look for april tag
                self.mover.setOrigin(self.sensors.april_tags)

            elif (self.sensors.obstacle or self.mover.avoiding):
                self.mover.intelligentAvoid(self.sensors.rec_turn, self.sensors.obstacle)
            
            # elif np.isclose(self.mover.cur_pose[1], 0, atol = .1):
            #     print(self.mover.cur_pose)
            #     print(self.mover.origin_pose)
            #     self.shutdown()
            
            elif return_home:
                self.mover.goToDestination((0,0))
                
            else:
                return_home = self.mover.navigateToWaypoint((0,1))
                #return_home = self.mover.goToDestination(-2.1336, 1.2192)
                #return_home = self.mover.goToDestination((-7.3152,1.2192))

    def shutdown(self):
        rospy.loginfo("goodbye, world")
        self.mover.stop(True)
        rospy.sleep(2)

myBot = Adventurebot()

        
