"""
location_publisher.py

Retreive both relative and absolute position information from the tf topic.
Frames are fed information by robot_location
"""
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('frame_transformations')
    
    # set up the tf node
    listener = tf.TransformListener()
    
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        try:
            map_trans, map_rot = listener.lookupTransform("/base_footprint", "/map", rospy.Time(0))
            print "MAP"
            print map_trans
            print map_rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        try:
            odom_trans, odom_rot = listener.lookupTransform("/base_footprint", "/odom", rospy.Time(0))
            print "ODOM"
            print odom_trans
            print odom_rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()