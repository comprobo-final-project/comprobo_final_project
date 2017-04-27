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

        self.poses = []

        self.resolution = 10
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
        w = random.uniform(-math.pi, math.pi) 

        self.pose.position.x = x_pos
        self.pose.position.y = y_pos
        self.pose.orientation.z = w


    def get_position(self):

        return self.pose.position.x, self.pose.position.y, \
                self.pose.orientation.z


    def step(self, step_freq):

        step_freq = float(step_freq)

        # Update velocity
        self.pose.velocity.x = self.twist.linear.x * math.cos(self.pose.orientation.z)
        self.pose.velocity.y = self.twist.linear.x * math.sin(self.pose.orientation.z)

        # Update pose
        self.pose.position += self.pose.velocity / step_freq
        self.pose.orientation += self.twist.angular / step_freq
        self.update_listener(step_freq)

        # Store current pose in list of all poses
        self.poses.append(self.pose)
