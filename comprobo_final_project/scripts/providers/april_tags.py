#!usr/bin/env python

"""
Retrieves Apriltags absolute position, and acts as an importable package
"""

from geometry_msgs.msg import PoseStamped
import tf

class AprilPoseProvider(object):
    """
    Takes data from the Apriltags and can sends it to RobotController
    """

    def __init__(self, rospy):

        self.pose_callback = None
        self.orient = None
        rospy.Subscriber("/STAR_pose_continuous", PoseStamped, self.getPos, queue_size=10)


    def getPos(self, april_data):
        """
        Takes the pose and ships it off to the currently assigned callback (which should be in RobotController)
        """

        self.pose_callback(april_data)

        #if we ever want it, orientation is here too
        self.orient = april_data.pose.orientation.x, april_data.pose.orientation.y, april_data.pose.orientation.z, april_data.pose.orientation.w
        eulers = tf.transformations.euler_from_quaternion(self.orient)


    def subscribe(self, pose_callback):
        """
        updates new callback function for packaging purposes
        """

        self.pose_callback = pose_callback


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
