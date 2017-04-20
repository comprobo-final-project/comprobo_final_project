#!usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from ..providers.gazebo_position_provider import GazeboPoseProvider

class Robot:
    """
    Neato connection.
    set_twist(twist : Twist) : Void - Sets the Twist of the robot
    """

    def __init__(self, position_listener):
        # TODO: This needs some work, b/c we can't start multiple robot nodes here.
        rospy.init_node('robot_controller')

        self.pose = PoseStamped()
        self.twist = Twist()

        self.position_listener = position_listener

        # Suscribe to position of Neato robot

        # For Gazebo
        GazeboPoseProvider(rospy).subscribe(self._pose_listener)

        # For real world
        # rospy.Subscriber('pose_stamped', PoseStamped, self._pose_listener)

        # Create publisher for current detected ball characteristics
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)


    def set_twist(self, forward_rate, turn_rate):
        self.twist.linear.x = forward_rate
        self.twist.angular.z = turn_rate


    def _pose_listener(self, pose):
        """
        Callback function for organism position.
        """
        self.pose = pose
        self.position_listener(pose.position.x, pose.position.y)
