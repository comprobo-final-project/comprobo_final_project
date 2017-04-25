import math
import random
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

        if forward_rate > .3:
            self.twist.linear.x = .3
        else:
            self.twist.linear.x = forward_rate

        if turn_rate > 3:
            self.twist.angular.z = 3
        else:
            self.twist.angular.z = turn_rate

        self.step(1.0/self.resolution)


    def set_random_pose(self):
        """ Set the robot randomly on the circumference of a circle that
        is centered on the origin and of radius r. """

        # Distance robot should be away from origin
        r = 5.0

        # Randomly generate x and y positions
        x_pos = random.uniform(-r, r) # choose random x_pos
        y_pos = math.sqrt(r**2 - x_pos**2) # define y_pos to meet r restriction
        y_pos *= random.choice([1.0, -1.0]) # randomly make y_pos negative

        # Randomly generate a heading
        w = random.uniform(0, 2*math.pi)

        self.pose.position.x = x_pos
        self.pose.position.y = y_pos
        self.pose.orientation.z = w


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
