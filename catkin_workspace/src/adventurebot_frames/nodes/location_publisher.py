#!/usr/bin/env python

"""
location_publisher.py

Retreive both relative and absolute position information from the tf topic.
Frames are fed information by robot_location
"""
import rospy
import tf

from geometry_msgs.msg import PoseStamped

class FramePublisher():
    def __init__(self):
        """ Publish map and odometry frame data based on tf transformations. """
        rospy.init_node('FramePublisher')
        
        self.transform_listener = tf.TransformListener()
        
        self.map_publisher = rospy.Publisher('map_frame/', PoseStamped, queue_size=10)
        self.odom_publisher = rospy.Publisher('odom_frame/', PoseStamped, queue_size=10)
        self.rate = rospy.Rate(100)
        
        while not rospy.is_shutdown():
            self._publish('map', self.map_publisher)
            self._publish('odom', self.odom_publisher)
            self.rate.sleep()

    def _publish(self, frame, frame_publisher):
        """ 
            Publish data, if possible. 
            
            Args:
                frame: A string representing the frame we want the translation from base_footprint to.
                frame_publisher: A rospy publisher object on which to publish that transformation
        """
        try:
            position, orientation = self.transform_listener.lookupTransform(frame, '/base_footprint',  rospy.Time(0))
            
            # create new PoseStamped object
            frame_pose = PoseStamped()
            frame_pose.header.stamp = rospy.Time.now()
            frame_pose.header.frame_id = frame
            frame_pose.pose.position.x = position[0]
            frame_pose.pose.position.y = position[1]
            frame_pose.pose.position.z = position[2]
            frame_pose.pose.orientation.x = orientation[0]
            frame_pose.pose.orientation.y = orientation[1]
            frame_pose.pose.orientation.z = orientation[2]
            frame_pose.pose.orientation.w = orientation[3]
            
            # publish the pose
            frame_publisher.publish(frame_pose)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Unable to publish pose data to " + str(frame))

if __name__ == '__main__':
    frame = FramePublisher()