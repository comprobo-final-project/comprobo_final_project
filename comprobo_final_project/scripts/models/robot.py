#!usr/bin/env python


from geometry_msgs.msg import PoseStamped, Twist
from ..providers.gazebo_position_provider import GazeboPoseProvider
from ..providers.april_pose_provider import AprilPoseProvider


class Robot:
    """
    Neato connection.
    set_twist(twist : Twist) : Void - Sets the Twist of the robot
    """

    def __init__(self, rospy, real=False):

        self.pose_stamped = PoseStamped()
        self.twist = Twist()
        self.resolution = 100

        # Suscribe to position of Neato robot, can switch between real world
        # vs gazebo
        self.pose_provider = AprilPoseProvider(rospy) if real \
                else GazeboPoseProvider(rospy)
        self.pose_provider.subscribe(self._pose_listener)

        # Create publisher for current detected ball characteristics
        self.twist_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)


    def set_twist(self, forward_rate, turn_rate):

        self.twist.linear.x = .3 if forward_rate > .3 else forward_rate
        self.twist.angular.z = 3 if turn_rate > 3 else turn_rate

        self.twist_publisher.publish(self.twist)


    def get_position(self):

        return self.pose_stamped.pose.position


    def _pose_listener(self, pose):
        """
        Callback function for organism position.
        """

        self.pose_stamped.pose = pose
