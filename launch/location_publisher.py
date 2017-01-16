"""
location_publisher.py

Retreive both relative and absolute position information from the tf topic.
Frames are fed information by robot_location
"""
import rospy
import tf

from geometry_msgs.msg import Pose, Twist


class FramePublisher():
    def __init__(self):
        rospy.init_node('FramePublisher')
        
        self.transform_listener = tf.TransformListener()
        
        self.map_publisher = rospy.Publisher('map_frame/', Pose, queue_size=10)
        self.odom_publisher = rospy.Publisher('odom_frame/', Pose, queue_size=10)
        self.rate = rospy.Rate(100)
        
        while not rospy.is_shutdown():
            self.publish("map", self.map_publisher)
            self.publish("odom", self.odom_publisher)
            self.rate.sleep()

    def publish(frame, publisher):
        try:
            position, orientation = listener.lookupTransform("/base_footprint", frame, rospy.Time(0))
            print frame
            print position
            print orientation
            
            new_pose = Pose()
            new_pose.position.x = position[0]
            new_pose.position.y = position[1]
            new_pose.position.z = position[2]
            new_pose.orientation.x = orientation[0]
            new_pose.orientation.y = orientation[1]
            new_pose.orientation.z = orientation[2]
            new_pose.orientation.w = orientation[3]
            
            publisher(new_pose)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Unable to publish to " + str(frame))
            return

if __name__ == '__main__':
    frame = FramePublisher()