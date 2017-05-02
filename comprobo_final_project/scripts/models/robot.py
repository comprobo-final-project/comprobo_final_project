#!usr/bin/env python

import rospy
import tf

import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from ..helpers import sleeper
from ..providers.gazebo_position_provider import GazeboPoseProvider
from ..providers.april_pose_provider import AprilPoseProvider

from ..helpers import sleeper


class Robot:
    """
    Neato connection.
    set_twist(twist : Twist) : Void - Sets the Twist of the robot
    """

    def __init__(self, resolution=10, real=False):

        # TODO: This needs some work, b/c we can't start multiple robot nodes
        #       here.
        rospy.init_node('robot_controller')

        self.MAX_SPEED = 0.3 # m/s
        self.MAX_TURN_RATE = 0.8 * np.pi # rad/s

        self.pose_stamped = PoseStamped()
        self.twist = Twist()
        self.resolution = resolution

        # Suscribe to position of Neato robot, can switch between real world
        # vs gazebo
        self.pose_provider = AprilPoseProvider(rospy) if real \
                else GazeboPoseProvider(rospy)
        self.pose_provider.subscribe(self._pose_listener)

        # Create publisher for current detected ball characteristics
        self.twist_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)


    def set_twist(self, forward_rate, turn_rate):

        self.twist.linear.x = np.clip(forward_rate, 0, self.MAX_SPEED)
        self.twist.angular.z = np.clip(turn_rate, -self.MAX_TURN_RATE, self.MAX_TURN_RATE)

        self.twist_publisher.publish(self.twist)
        sleeper.sleep(1.0 / self.resolution)


    def get_position(self):
        return self.pose_stamped.pose.position


    def get_direction(self):
        return tf.transformations.euler_from_quaternion((
            self.pose_stamped.pose.orientation.x,
            self.pose_stamped.pose.orientation.y,
            self.pose_stamped.pose.orientation.z,
            self.pose_stamped.pose.orientation.w))[2] * 180 / (np.pi)


    def set_random_position(self, r=1):
        # TODO: Set a random position in gazebo. Ignore if real.
        pass


    def set_random_direction(self):
        # TODO: Set a random direction in gazebo. Ignore if real.
        pass


    def _pose_listener(self, pose):
        """
        Callback function for organism position.
        """
        self.pose_stamped.pose = pose
