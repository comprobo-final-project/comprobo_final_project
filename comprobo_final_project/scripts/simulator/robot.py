import math
import random

from pose import Pose
from twist import Twist


class Robot:

    def __init__(self, noise = 0.1):

        # http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html
        self.pose = Pose()

        # http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
        self.twist = Twist()

        self.noise = noise # A value between 0 and 1.

        self.resolution = 5
        self.update_listener = lambda x: None

    def set_twist(self, forward_rate, turn_rate):

        if math.fabs(forward_rate) > .3 or math.isnan(forward_rate) or math.isinf(forward_rate):
            self.twist.linear.x = .3
        else:
            self.twist.linear.x = forward_rate

        if math.fabs(turn_rate) > 3 or math.isnan(turn_rate) or math.isinf(turn_rate):
            self.twist.angular.z = 3
        else:
            self.twist.angular.z = turn_rate

        self.step(self.resolution)


    def set_update_listener(self, update_listener):
        """
        An inputted update_listener gets called whenever a robot updates
        """

        self.update_listener = update_listener


    def get_position(self):
        return self.pose.position


    def get_direction(self):
        return math.atan2(self.pose.velocity.y, self.pose.velocity.x)


    def step(self, step_freq):

        # Skip randomly depending on our noise threshold
        if (random.random() > self.noise):

            step_freq = float(step_freq)

            twist_r = self.twist.linear.x
            twist_theta = self.twist.angular.z
            original_velocity_theta = self.get_direction()

            # From 0 to 2. 1 is neutral
            noise_factor = 2 * (random.random() * self.noise) + 1

            # Update velocity
            vel_x = twist_r * \
                    math.cos(original_velocity_theta + twist_theta / step_freq) * \
                    noise_factor
            vel_y = twist_r * \
                    math.sin(original_velocity_theta + twist_theta / step_freq) * \
                    noise_factor

            # Average the prior velocity to get a more gradual change
            self.pose.velocity.x = self.pose.velocity.x * 0.1 + vel_x * 0.9
            self.pose.velocity.y = self.pose.velocity.y * 0.1 + vel_y * 0.9

            # self.pose.velocity.x = vel_x
            # self.pose.velocity.y = vel_y

        # Update pose
        self.pose.position += self.pose.velocity / step_freq

        self.update_listener(step_freq)
