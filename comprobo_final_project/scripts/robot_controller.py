#!usr/bin/env python


"""
Node that subscribes to the current position of the Neato, the goal position,
and calculates a Twist message via an equation whose coefficients are
determined by the organism's genes. This calculated Twist is then published.
"""


import math
from geometry_msgs.msg import PoseStamped, Twist
from providers.gazebo_pose_provider import GazeboPoseProvider


class RobotController:
    """
    Dictates robot's motion based on genes.
    """

    def __init__(self, genes=None):
        """
        Initializes the node, publisher, subscriber, and the genes
        (coefficients) of the robot controller.
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


    def position_callback(self, msg):
        """
        Callback function for organism position.
        """
        if self.genes is not None:
            # Initialize linear and angular velocities to zero
            cmd_vel = Twist()

            # TODO: maybe do something besides hardcoding the goal
            # Define robot's goal end position
            goal_x = 0.0
            goal_y = 0.0

            # Get current robot position
            curr_x = msg.pose.position.x
            curr_y = msg.pose.position.y

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


    def set_genes(self, genes):
        """
        sets the genes for this iteration of the robot
        """
        self.genes = genes


    def run(self):
        """
        Main run function.
        """

        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Check for time-jumps, like when looping a bag file
            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                print "Time went backwards. Carry on."


if __name__ == '__main__':
    robot_controller = RobotController([0.0, 1.0, 2.0, 3.0, 4.0, 5.0])
    robot_controller.run()
