#!/usr/bin/env python
import rospy, geometry_msgs.msg, nav_msgs.msg
import tf

rospy.init_node('vel_pub')
t = tf.Transformer(True, rospy.Duration(1))


# def callback(msg)
while not rospy.is_shutdown():
    # pub.publish()
    try:
        rospy.loginfo(t.lookupTwist("base_link", "map", rospy.Time(),rospy.Duration(0.2)))
    except:
        rospy.loginfo("err")