import numpy as np

class Markov:

    def __init__(self):

        self.print_result = False

        #########################
        # World costs
        self.goal_reward = 100
        self.avoid_punish = -1000
        self.wall_cost = -10
        #########################

        ##########################
        # Sum should be 1.0
        self.reward_value = 0.7
        self.reward_side = 0.1
        self.reward_back = 0.1
        ##########################

        self.gamma = 1

        self.avoids = []
        self.goal = None
        self.world = None
        self.action_map = None

        self.actions = ['u', 'r', 'd', 'l']

    def getActionMap(self):
        return self.action_map

    def getWorldMap(self):
        return self.world

    def setGoal(self, x, y):
        self.goal = (x ,y)

    def setAvoid(self, x, y):
        self.avoids.append((x, y))

    def setWorld(self, world):
        self.world = world

        self.action_map = np.full((self.world.shape[0], self.world.shape[1]), "u", dtype=str)
        for x in range(0, self.world.shape[0]):
            for y in range(0, self.world.shape[1]):
                if self.world[x][y] == -1:
                     self.action_map[x][y] = 'w'
                     self.world[x][y] = self.wall_cost

    def init(self):
        self.action_map[self.goal[0]][self.goal[1]] = 'g'
        self.world[self.goal[0]][self.goal[1]] = 100

        for p in self.avoids:
            try:
                self.world[p[0]][p[1]] = self.avoid_punish
                self.action_map[p[0]][p[1]] = 'a'
            except e:
                print("Set avoid went wrong:")
                print(e)

        if self.print_result:
            print('####################################')
            print('############     INIT   ############')
            print(self.world)
            print(self.action_map)
            print('####################################')

    def doRewardIteration(self, iteration=1):
        next_action_map = self.action_map.copy()
        next_world = self.world.copy()
        for x in range(0, self.world.shape[0]):
            for y in range(0, self.world.shape[1]):
                if self.action_map[x][y] == 'g':
                    next_action_map[x][y] = 'g'
                    next_world[x][y] = self.goal_reward
                elif self.action_map[x][y] == 'a':
                    next_action_map[x][y] = 'a'
                    next_world[x][y] = self.avoid_punish
                elif self.action_map[x][y] == 'w':
                    next_action_map[x][y] = 'w'
                    next_world[x][y] = self.wall_cost
                else:
                    ac, val = self.step(x, y)
                    next_action_map[x][y] = ac
                    next_world[x][y] = pow(self.gamma, iteration) * val

        self.world = next_world
        self.action_map = next_action_map

        if self.print_result:
            print('####################################')
            print(self.world)
            print(self.action_map)
            print('####################################')

    def step(self, x, y):
        current = self.world[x][y]

        try: 
            up = self.world[x][y+1] * self.reward_value \
                + self.world[x+1][y] * self.reward_side + self.world[x-1][y] * self.reward_side \
                + self.world[x][y-1] * self.reward_back

            right = self.world[x+1][y] * self.reward_value \
                + self.world[x][y+1] * self.reward_side + self.world[x][y-1] * self.reward_side \
                + self.world[x-1][y] * self.reward_back

            down = self.world[x][y-1] * self.reward_value \
                + self.world[x+1][y] * self.reward_side + self.world[x-1][y] * self.reward_side  \
                + self.world[x][y+1] * self.reward_back

            left = self.world[x-1][y] * self.reward_value \
                + self.world[x][y+1] * self.reward_side + self.world[x][y-1] * self.reward_side \
                + self.world[x+1][y] * self.reward_back

        except e:
            print("Step went wrong:")
            print(e)

        best = max(up, right, down, left)
        best_index = np.argmax(np.array([up, right, down, left]))

        if best == current: 
            return self.action_map[x][y], current
        else:
            return self.actions[best_index], best

    
    def test_markov(self):
        self.init()

        for i in range(1, 10):
            self.doRewardIteration(i)

if __name__ == "__main__":
    m = Markov()

    m.setWorld(np.array([
            [-1, -1, -1, -1, -1, -1],
            [-1, 0, 0, 0, 0, -1],
            [-1, 0, 0, 0, 0, -1],
            [-1, 0, 0, 0, 0, -1],
            [-1, 0, 0, 0, 0, -1],
            [-1, -1, -1, -1, -1, -1]
    ]))

    m.setGoal(2,4)
    m.setAvoid(4,3)

    m.test_markov()