#!usr/bin/env python

"""
Got Apriltags absolute position
"""

import rospy
from geometry_msgs.msg import Twist, Point, PoseStamped
import tf

class April(object):
    """
    Basically for now it just takes data from the Apriltags and can send it somewhere else
    """

    def __init__(self):
        rospy.init_node("april")

        rospy.Subscriber("/STAR_pose_continuous", PoseStamped, self.getPos, queue_size=10)

        self.position = None
        self.orient = None

    def getPos(self, data):
        """
        just takes the position and does something with it
        """

        self.position = data.pose.position.x, data.pose.position.y, data.pose.position.z
        self.orient = data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w
        eulers = tf.transformations.euler_from_quaternion(self.orient)


        print "pos", self.position
        print "orient", self.orient
        print "eulers", eulers
        print " "


    def run(self):
        """
        main loop
        """

        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            r.sleep()


if __name__=="__main__":
    node = April()
    node.run()
