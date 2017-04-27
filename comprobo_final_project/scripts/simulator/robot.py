import math
from pose import Pose
from twist import Twist


class Robot:

    def __init__(self):

        # http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html
        self.pose = Pose()

        # http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
        self.twist = Twist()

        self.resolution = 10
        self.update_listener = lambda x: None

    def set_twist(self, forward_rate, turn_rate):

        self.twist.linear.x = .3 if forward_rate > .3 else forward_rate
        self.twist.angular.z = 3 if turn_rate > 3 else turn_rate


        self.step(self.resolution)


    def set_update_listener(self, update_listener):
        """
        An inputted update_listener gets called whenever a robot updates
        """

        self.update_listener = update_listener


    def get_position(self):

        return self.pose.position.x, self.pose.position.y, \
                self.pose.orientation.z


    def step(self, step_freq):
        step_freq = float(step_freq)

        twist_r = self.twist.linear.x
        twist_theta = self.twist.angular.z
        original_velocity_theta = math.atan2(self.pose.velocity.y,
                self.pose.velocity.x)

        # Update velocity
        self.pose.velocity.x = twist_r * math.cos(original_velocity_theta +
                twist_theta / step_freq)
        self.pose.velocity.y = twist_r * math.sin(original_velocity_theta +
                twist_theta / step_freq)

        # Update pose
        self.pose.position += self.pose.velocity / step_freq
        self.pose.orientation += self.twist.angular / step_freq
        self.update_listener(step_freq)
