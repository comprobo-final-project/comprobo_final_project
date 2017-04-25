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


    def set_twist(self, forward_rate, turn_rate):

        self.twist.linear.x = .3 if forward_rate > .3 else forward_rate
        self.twist.angular.z = 3 if turn_rate > 3 else turn_rate

        self.step(1.0/self.resolution)


    def get_position(self):

        return self.pose.position.x, self.pose.position.y, \
                self.pose.orientation.z


    def step(self, step_size):

        twist_r = self.twist.linear.x
        twist_theta = self.twist.angular.z
        original_velocity_theta = math.atan2(self.pose.velocity.y,
                self.pose.velocity.x)

        # Update velocity
        self.pose.velocity.x = twist_r * math.cos(original_velocity_theta +
                step_size * twist_theta)
        self.pose.velocity.y = twist_r * math.sin(original_velocity_theta +
                step_size * twist_theta)

        # Update pose
        self.pose.position += step_size * self.pose.velocity
        self.pose.orientation += step_size * self.twist.angular
