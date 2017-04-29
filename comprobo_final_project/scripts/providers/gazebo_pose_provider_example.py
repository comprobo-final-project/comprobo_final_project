# Temporary file to demonstrate how to use our position provider.

import rospy
from gazebo_pose_provider import GazeboPoseProvider

rospy.init_node("example")

# Create a new instance of our position provider. We are using Gazebo now, but
# we could also use real-world implementations in the future.
position_provider = GazeboPoseProvider(rospy)

# Here is our callback. You can imagine this getting defined by our main robot
# controller
def print_callback(pose):
    print pose

# Subscribe to the position provider and get called whenever a new position is
# received
position_provider.subscribe(print_callback)

# Standard stuff
r = rospy.Rate(1)
while not(rospy.is_shutdown()):
    r.sleep()
