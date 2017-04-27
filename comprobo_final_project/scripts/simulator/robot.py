import math
import random

import numpy as np

from pose import Pose
from twist import Twist


class Robot:

    def __init__(self, noise = 0.1, resolution = 10):

        self.MAX_SPEED = 2 # m/s
        self.MAX_TURN_RATE = 2 # rad/s

        self.pose = Pose()
        self.twist = Twist()

        self.noise = noise # A value between 0 and 1.

        self.resolution = resolution
        self.update_listener = lambda x: None


    def set_twist(self, forward_rate, turn_rate):

        self.twist.linear.x = np.clip(forward_rate, 0, self.MAX_SPEED)
        self.twist.angular.z = np.clip(turn_rate, -self.MAX_TURN_RATE, self.MAX_TURN_RATE)
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
            twist_w = self.twist.angular.z
            original_w = self.get_direction()

            # From 0 to 2. 1 is neutral
            noise_factor = 2 * (random.random() * self.noise) + 1

            # Update velocity
            vel_x = twist_r * \
                    math.cos(original_w + twist_w / step_freq) * \
                    noise_factor
            vel_y = twist_r * \
                    math.sin(original_w + twist_w / step_freq) * \
                    noise_factor

            # Average the prior velocity to get a more gradual change
            self.pose.velocity.x = self.pose.velocity.x * 0.1 + vel_x * 0.9
            self.pose.velocity.y = self.pose.velocity.y * 0.1 + vel_y * 0.9

            # self.pose.velocity.x = vel_x
            # self.pose.velocity.y = vel_y

        # Update pose
        self.pose.position += self.pose.velocity / step_freq

        self.update_listener(step_freq)
