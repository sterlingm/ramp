import gym
from gym import spaces
import numpy as np

class CoeDcoeDiscre(gym.Env):
    """docstring for CoeDcoeDiscre"""
    def __init__(self, name='cdd'):
        super(CoeDcoeDiscre, self).__init__()
        self.name = name
        self.action_space = spaces.Discrete(3)
        self.observation_space = spaces.Box(0, 5, shape=(1,))
        self.ob = None
        self.D_max = 0.5
        self.ob_max = int(self.observation_space.high[0].item())

        self.cost_array = np.array([0.91, 1.115, 1.32, 1.525, 1.73])
        self.cost_array = 15.0 / self.cost_array
        self.cost_array += [16.0, 17.9, 19.8, 21.7, 23.6]
        self.cost_array = (self.cost_array - 30.0)**2 # convenient to monitor



    def calcCost(self):
        state = 1.0 * self.D_max * self.ob / (self.ob_max - 1)

        if state < 0.1:
            return self.cost_array[0]
        elif state < 0.2:
            return self.cost_array[1]
        elif state < 0.3:
            return self.cost_array[2]
        elif state < 0.4:
            return self.cost_array[3]
        else:
            return self.cost_array[4]



    def _reset(self):
        self.ob = np.random.randint(0, self.ob_max)
        return np.array([self.ob])



    def _step(self, a):
        a -= 1
        self.ob += a
        if self.ob < 0 or self.ob > self.ob_max - 1:
            beyond_limit = True
        else:
            beyond_limit = False
            
        self.ob = np.clip(self.ob,
                          self.observation_space.low,
                          self.observation_space.high)
        self.ob = int(self.ob.item())

        cost = self.calcCost() + 0.001 * a**2 # a**2 means energy cost
        reward = -cost

        # if self.ob == 2:
        #     reward += 100

        done = False
        # done = beyond_limit
        # if beyond_limit:
        #     reward -= 100.0

        info = {}

        return np.array([self.ob]), reward, done, info