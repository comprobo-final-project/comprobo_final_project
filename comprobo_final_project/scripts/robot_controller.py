#!usr/bin/env python


"""
Node that subscribes to the current position of the Neato, the goal position,
and calculates a Twist message via an equation whose coefficients are
determined by the organism's genes. This calculated Twist is then published.
"""

import rospy
import math
from geometry_msgs.msg import PoseStamped, Twist
from providers.gazebo_pose_provider import GazeboPoseProvider


class RobotController:
    """
    Dictates robot's motion based on genes. It is given a time to control
    the robot, after which it returns the robot's last position for fitness
    evaluation and then shutsdown.
    """

    def __init__(self, genes, time_to_run):
        """
        Initializes the node, publisher, subscriber, and the genes
        (coefficients) of the robot controller.

        genes: list of coefficients used in the function to calculate the
            robot's linear and angular velocities
        time_to_run: time (seconds) until the Node shuts down
        """

        # Initialize ROS node
        rospy.init_node('robot_controller')

        # Suscribe to position of Neato robot
        position_provider = GazeboPoseProvider(rospy)
        position_provider.subscribe(position_callback)

        # Create publisher for current detected ball characteristics
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Save the coefficients in the genes
        self.genes = genes

        # Store the time the robot controller should run for
        self.shutdown_time = rospy.get_time() + time_to_run
        self.run_time = 0.0

        # Store the robot's current position
        self.curr_pose = None


    def position_callback(self, msg):
        """
        Callback function for when the subscriber receives a new robot position.
        """

        # Update current position of robot
        self.curr_pose = msg

        # Initialize linear and angular velocities to zero
        cmd_vel = Twist()

        # TODO: maybe do something besides hardcoding the goal
        # Define robot's goal end position
        goal_x = 0.0
        goal_y = 0.0

        # Get current robot position
        curr_x = self.curr_pose.pose.position.x
        curr_y = self.curr_pose.pose.position.y

        # Calculate difference between robot position and goal position
        diff_x = goal_x - curr_x
        diff_y = goal_y - curr_y

        # Calculate angle to goal and distance to goal
        diff_w = math.atan2(diff_y, diff_x)
        diff_r = math.sqrt(diff_x**2 + diff_y**2)

        # Define linear and angular velocities based on genes
        a1, b1, c1, a2, b2, c2 = self.genes
        cmd_vel.linear.x = a1*diff_w + b1*diff_r + c1*diff_r**2
        cmd_vel.angular.z = a2*diff_w + b2*diff_r + c2*diff_r**2

        # Publish linear and angular velocities
        self.pub.publish(cmd_vel)


    def run(self):
        """
        Main run function.
        """

        # Run for the specified duration
        r = rospy.Rate(10)
        while not rospy.is_shutdown() and self.run_time < self.shutdown_time:
            self.run_time = rospy.get_time()
            r.sleep()

        # Return last known position for fitness evaluation
        return self.curr_pose


if __name__ == '__main__':

    genes = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    time_to_run = 10
    robot_controller = RobotController(genes, time_to_run)
    last_position = robot_controller.run()
