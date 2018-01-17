"""
Observation
-----------
    Path: 2 points, each is in (x,y) form.

State
-----
    Coefficients: only D~[0,1].

Action
------
    Delta coefficients: dD ~ -0.1, 0.0, +0.1.
"""

import gym
from gym import spaces
import numpy as np

class PathDcoeDiscre(gym.Env):
    """docstring for PathDcoeDiscre.
    """
    def __init__(self, name='pdd'):
        super(PathDcoeDiscre, self).__init__()
        self.name = name
        self.state = 0.0 # D weight
        self.state_min = 0.0
        self.state_max = 1.0
        self.max_len = 18.2-0.07354
        self.goal = np.array([10., 10.])
        self.action_space = spaces.Discrete(3)
        self.observation_space = spaces.Box(np.array([0., 0., self.state_min]),
                                            np.array([10., 10., self.state_max])) # single motion state
        self.cut = [0.1, 0.3, 0.4, 0.6]



    def setState(self, D):
        """
        Arguments
        --------
        D: D weight
        """
        # self.state = np.clip(D, self.state_min, self.state_max)
        self.state = D



    def getState(self):
        return self.state



    def getOb(self):
        """
        Return
        ------
            A path.
        """
        ob = np.zeros((3,self.observation_space.low.shape[0]))
        if self.state < self.cut[0]: # too close
            ob[0] = [3., 2., self.state] # first point
            ob[1] = [5., 3., self.state] # second point
            ob[2] = [5., 3., self.state] # the last point
        elif self.state < self.cut[1]:
            ob[0] = [6., 3., self.state]
            ob[1] = [8.2, 7., self.state]
            ob[2] = [10., 10., self.state]
        elif self.state < self.cut[2]:
            ob[0] = [5., 2., self.state]
            ob[1] = [8., 3., self.state]
            ob[2] = [10., 10., self.state]
        elif self.state < self.cut[3]:
            ob[0] = [7., 2., self.state]
            ob[1] = [8.5, 3., self.state]
            ob[2] = [10., 10., self.state]
        else: # too far
            ob[0] = [7., 1., self.state]
            ob[1] = [9., 1., self.state]
            ob[2] = [10., 10., self.state]

        return ob



    def getLen(self, ob):
        """
        Arguments
        ---------
            ob: a path

        Return
        ------
            Path length
        """
        if not self.done(ob):
            return self.max_len

        last_pt = [0, 0]
        plen = 0.0
        for pt in ob:
            pt = pt[0:2]
            plen += np.linalg.norm(pt - last_pt)
            last_pt = pt

        return plen



    def getReward(self, ob):
        """
        Arguments
        ---------
            ob: a path

        Return
        ------
            Reward
        """
        r = self.max_len - self.getLen(ob)
        r = max(r, 0.0)
        return r



    def getHardReward(self):
        # if self.state < self.cut[0]: # too close
        #     return 0.0 # 0.0
        # elif self.state < self.cut[1]:
        #     return 3.5 + 0.3 # 3.8
        # elif self.state < self.cut[2]:
        #     return 2.3 + 3.2 # 5.5
        # elif self.state < self.cut[3]:
        #     return 1.8 + 3.2 # 5.0
        # else: # too far
        #     return 7e-6 + 3.5 # 3.500007
        if self.state < self.cut[0]: # too close
            return 0.0
        elif self.state < self.cut[1]:
            return 2.0
        elif self.state < self.cut[2]:
            return 5.0
        elif self.state < self.cut[3]:
            return 2.0
        else: # too far
            return 0.0



    def done(self, ob):
        """
        Arguments
        ---------
            ob: a path

        Return
        ------
            done or not
        """
        return (ob[len(ob)-1][0] == self.goal[0] and
                ob[len(ob)-1][1] == self.goal[1])



    def reset(self, init_state=None):
        """
        Set state randomly default and calculate corresponding path.

        Arguments
        ---------
            init_state: if it is None, then set state randomly

        Return
        ------
            A path.
        """
        if init_state is None:
            tmp_state = 0.5 + (np.random.rand() * 0.25 - 0.125) # [0.375, 0.625)
            self.setState(tmp_state)
        else:
            self.setState(init_state)

        return self.getOb()



    def decodeAction(self, action):
        """
        Arguments
        ---------
            action (int): encoded delta D weight.

        Return
        ------
            (float): Delta D weight.
        """
        return (action - 1) * 0.05



    def step(self, action):
        """
        Arguments
        ---------
            action: encoded delta D weight.

        Return
        ------
            *A path.
            *Reward: max_len - real_len.
            *Done: reach the goal or not.
            *Info.
        """
        dD = self.decodeAction(action)
        self.setState(self.state + dD)
        ob = self.getOb()

        if self.state < self.state_min or self.state > self.state_max:
            done = True
        else:
            done = False

        # return ob, self.getHardReward(), self.done(ob), {}
        return ob, self.getHardReward(), done, {}