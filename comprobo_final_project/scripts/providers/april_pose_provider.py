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

    def __init__(self, rospy, name):
        self.pose_callback = None
        self.name = name
        rospy.Subscriber(self.name+"/STAR_pose_continuous", PoseStamped, \
            self._on_pose_msg_received, queue_size=10)


    def subscribe(self, pose_callback):
        """
        updates new callback function for packaging purposes
        """
        self.pose_callback = pose_callback


    def _on_pose_msg_received(self, msg):
        """
        Takes the pose and ships it off to the currently assigned callback (which should be in RobotController)
        """
        if self.pose_callback is not None:
            self.pose_callback(msg.pose)
