#!usr/bin/env python


"""
Node that subscribes to the current position of the Neato, the goal position,
and calculates a Twist message via an equation whose coefficients are
determined by the organism's genes. This calculated Twist is then published.
"""

import math
from geometry_msgs.msg import PoseStamped, Twist


class RobotController:
    """
    Holds the genes and fitness of an organism.
    """

    def __init__(self, genes):
        """
        Initializes the node, publisher, subscriber, and the genes
        (coefficients) of the robot controller.
        """

        # Initialize ROS node
        rospy.init_node('robot_controller')

        # Suscribe to position of Neato robot
        self.sub = rospy.Subscriber("pos", PoseStamped, self.process)

        # Create publisher for current detected ball characteristics
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Save the coefficients in the genes
        self.genes = genes


    def process(self, msg):
        """
        Callback function for organism position.
        """

        # Initialize cmd_vel
        cmd_vel = Twist()

        # TODO: get goal pose from somewhere
        goal_x = 5.0
        goal_y = 1.0

        curr_x = msg.pose.position.x
        curr_y = msg.pose.position.y

        diff_x = goal_x - curr_x
        diff_y = goal_y - curr_y

        diff_w = math.atan2(diff_y, diff_x)
        diff_r = math.sqrt(diff_x**2 + diff_y**2)

        a1, b1, c1, a2, b2, c2 = self.genes
        cmd_vel.linear.x = a1*diff_w + b1*diff_r + c1*diff_r**2
        cmd_vel.angular.z = a2*diff_w + b2*diff_r + c2*diff_r**2

        self.pub.publish(cmd_vel)

        
    def run(self):
        """
        Main run function.
        """

        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                print "Time went backwards. Carry on."


if __name__ == '__main__':
    robot_controller = RobotController()
    robot_controller.run()
