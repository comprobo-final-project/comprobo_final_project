from .simulator.robot import Robot

class GoalTask(object):


    def create_robot(self):
        return Robot(noise=0.2)


    def reset_robot(self, robot):
        robot.set_random_position(r=5.0)
        robot.set_random_direction()


    def run(self, robot, duration, genes):

        goal_x = 0.0
        goal_y = 0.0
        positions = []

        for _ in range(int(duration * robot.resolution)):

            curr_w = robot.get_direction()
            curr_pos = robot.get_position()

            curr_x = curr_pos.x
            curr_y = curr_pos.y

            positions.append(curr_pos) # store position history

            # Calculate difference between robot position and goal position
            diff_x = goal_x - curr_x
            diff_y = goal_y - curr_y

            # Calculate angle to goal and distance to goal
            # http://stackoverflow.com/a/7869457/2204868
            diff_w = math.atan2(diff_y, diff_x) - curr_w
            diff_w = (diff_w + math.pi) % (2*math.pi) - math.pi
            diff_r = math.sqrt(diff_x**2 + diff_y**2)

            # Define linear and angular velocities based on genes
            a1, b1, a2, b2 = genes
            forward_rate = a1*diff_w + b1*diff_r
            turn_rate = a2*diff_w + b2*diff_r

            # Set linear and angular velocities
            robot.set_twist(forward_rate, turn_rate)

        return positions


    def train(self):

        robot = self.create_robot()

        population = Population(
            size=100, \
            crossover=0.8, \
            elitism=0.1, \
            mutation=0.5)

        # TODO: Input genes
        self.reset_robot(robot)
        self.run(robot=robot, duration=20, genes=[])

    def test(self):
        pass
