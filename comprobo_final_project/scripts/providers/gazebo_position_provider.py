#!/usr/bin/env python

from gazebo_msgs.msg import ModelStates

class GazeboPoseProvider:
    """
    Provides the world pose of our robot from Gazebo.
    """

    ROBOT_ID = 1 # This is a reference to our only init robot. Subject to change

    def __init__(self, rospy):
        self.pose_callback = None
        rospy.Subscriber("gazebo/model_states", ModelStates, self.model_states_callback)


    def subscribe(self, pose_callback):
        """
        Updates our new callback function.
        """
        self.pose_callback = pose_callback


    def model_states_callback(self, model_states):
        """
        Helper that strips down ModelStates into a Pose for our robot.
        """
        self.pose_callback(model_states.pose[self.ROBOT_ID])
