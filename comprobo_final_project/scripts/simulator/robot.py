import math
import random

import numpy as np

from pose import Pose
from twist import Twist
from vector_3 import Vector3

class Robot:

    def __init__(self, resolution=10, noise=0.1):

        self.MAX_SPEED = 0.3 # m/s
        self.MAX_TURN_RATE = 0.8 * math.pi # rad/s
        self.HISTORY = 0.1 # How much history to use

        self.pose = Pose()
        self.twist = Twist()

        self.noise = noise # A value between 0 and 1.

        self.resolution = resolution
        self.update_listener = lambda x: None


    def set_twist(self, forward_rate, turn_rate):

        self.twist.linear.x = np.clip(forward_rate, 0, self.MAX_SPEED)
        self.twist.angular.z = np.clip(turn_rate, -self.MAX_TURN_RATE,
                self.MAX_TURN_RATE)
        self.step(self.resolution)


    def set_update_listener(self, update_listener):
        """
        An inputted update_listener gets called whenever a robot updates
        """

        self.update_listener = update_listener


    def set_random_position(self, r=5.0):
        """ Set the robot randomly on the circumference of a circle that
        is centered on the origin and of radius r. """

        # Randomly generate x and y positions
        x_pos = random.uniform(-r, r) # choose random x_pos
        y_pos = math.sqrt(r**2 - x_pos**2) # set y_pos to meet r restriction
        y_pos *= random.choice([1.0, -1.0]) # randomly make y_pos negative

        # Set the robot's position
        self.set_position(x_pos, y_pos)


    def set_random_direction(self):
        """ Give the robot a random direction from -pi to pi. """

        # Randomly generate a direction
        w = random.uniform(-math.pi, math.pi)

        # Set the robot's direction
        self.set_direction(w)


    def get_position(self):

        return self.pose.position


    def get_direction(self):

        return self.pose.velocity.w


    def set_position(self, x, y):

        self.pose.position.x = x
        self.pose.position.y = y


    def set_direction(self, w):

        self.pose.velocity.w = w


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
            vel_r = twist_r * noise_factor
            vel_w = (original_w + (twist_w / step_freq)) * noise_factor

            # Average the prior velocity to get a more gradual change
            self.pose.velocity.r = self.pose.velocity.r * self.HISTORY + \
                vel_r * (1 - self.HISTORY)
            self.pose.velocity.w = self.pose.velocity.w * self.HISTORY + \
                vel_w * (1 - self.HISTORY)

        # Update pose
        velocity_xyz = Vector3()
        velocity_xyz.x = self.pose.velocity.r * math.cos(self.pose.velocity.w)
        velocity_xyz.y = self.pose.velocity.r * math.sin(self.pose.velocity.w)
        self.pose.position += velocity_xyz / step_freq

        self.update_listener(step_freq)
