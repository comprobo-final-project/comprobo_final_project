import math

from pose import Pose
from twist import Twist


class Robot:

    def __init__(self):

        # http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html
        self.pose = Pose()

        # http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
        self.twist = Twist()


    def set_twist(self, forward_rate, turn_rate):

        if forward_rate > .3:
            self.twist.linear.x = .3
        else:
            self.twist.linear.x = forward_rate

        if turn_rate > 3:
            self.twist.angular.z = 3
        else:
            self.twist.angular.z = turn_rate

        self.step(0.1)


    def get_position(self):

        return self.pose.position.x, self.pose.position.y, self.pose.orientation.z


    def step(self, step_size):

        twist_r = self.twist.linear.x
        twist_theta = self.twist.angular.z
        original_velocity_theta = math.atan2(self.pose.velocity.y, self.pose.velocity.x)

        # Update velocity
        self.pose.velocity.x = twist_r * math.cos(original_velocity_theta + step_size * twist_theta)
        self.pose.velocity.y = twist_r * math.sin(original_velocity_theta + step_size * twist_theta)

        # Update position
        self.pose.position += step_size * self.pose.velocity
