from pose import Pose
from twist import Twist

class Robot:

    def __init__(self):

        # http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose.html
        self.pose = Pose()

        # http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
        self.twist = Twist()


    def step(self, step_size):
        self.pose.position += step_size * self.twist.linear
