#!usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from ..providers.gazebo_position_provider import GazeboPoseProvider
from ..providers.april_pose_provider import AprilPoseProvider

class Robot:
    """
    Neato connection.
    set_twist(twist : Twist) : Void - Sets the Twist of the robot
    """

    def __init__(self):
        # TODO: This needs some work, b/c we can't start multiple robot nodes here.
        rospy.init_node('robot_controller')

        self.pose = PoseStamped()
        self.twist = Twist()

        # Suscribe to position of Neato robot
        # GazeboPoseProvider(rospy).subscribe(self._pose_listener) # For Gazebo
        AprilPoseProvider(rospy).subscribe(self._pose_listener) # For real world

        # Create publisher for current detected ball characteristics
        self.twist_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)


    def set_twist(self, forward_rate, turn_rate):
        self.twist.linear.x = forward_rate
        self.twist.angular.z = turn_rate
        self.twist_publisher.publish(self.twist)


    def get_position(self):
        return self.pose.position


    def _pose_listener(self, pose):
        """
        Callback function for organism position.
        """
        self.pose = pose
