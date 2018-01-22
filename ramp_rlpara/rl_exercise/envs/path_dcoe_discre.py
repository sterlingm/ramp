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
        self.A = 0.0
        self.D = 0.0 # D weight
        self.A_min = 0.0
        self.A_max = 1.0
        self.D_min = 0.0
        self.D_max = 2.0
        self.state = 0
        self.max_len = 18.2-0.07354
        self.goal = np.array([10., 10.])
        self.action_space = spaces.Discrete(9)
        self.observation_space = spaces.Box(np.array([0., 0., self.A_min, self.D_min]),
                                            np.array([10., 10., self.A_max, self.D_max])) # single motion state
        self.A_cut = [0.1, 0.3, 0.4, 0.6]
        self.D_cut = [0.3, 0.6, 0.8, 1.2]
        self.min_reward = 0.0
        self.action_resolution = 0.05



    def setState(self, A, D):
        """
        Arguments
        --------
        A: A weight
        D: D weight
        """
        self.A = np.clip(A, self.A_min, self.A_max)
        self.D = np.clip(D, self.D_min, self.D_max)
        # self.state = D



    def getState(self):
        return np.array([self.A, self.D])



    # def getOb(self):
    #     """
    #     Return
    #     ------
    #         A path.
    #     """
    #     ob = np.zeros((3,self.observation_space.low.shape[0]))
    #     if self.state < self.cut[0]: # too close
    #         ob[0] = [3., 2., self.state] # first point
    #         ob[1] = [5., 3., self.state] # second point
    #         ob[2] = [5., 3., self.state] # the last point
    #     elif self.state < self.cut[1]:
    #         ob[0] = [6., 3., self.state]
    #         ob[1] = [8.2, 7., self.state]
    #         ob[2] = [10., 10., self.state]
    #     elif self.state < self.cut[2]:
    #         ob[0] = [5., 2., self.state]
    #         ob[1] = [8., 3., self.state]
    #         ob[2] = [10., 10., self.state]
    #     elif self.state < self.cut[3]:
    #         ob[0] = [7., 2., self.state]
    #         ob[1] = [8.5, 3., self.state]
    #         ob[2] = [10., 10., self.state]
    #     else: # too far
    #         ob[0] = [7., 1., self.state]
    #         ob[1] = [9., 1., self.state]
    #         ob[2] = [10., 10., self.state]

    #     return ob



    def getOb(self):
        """
        Return
        ------
            A path.
        """
        if self.D < self.D_cut[0]:
            self.state = 0
        elif self.D < self.D_cut[1]:
            self.state = 1
        elif self.D < self.D_cut[2]:
            self.state = 2
        elif self.D < self.D_cut[3]:
            self.state = 3
        else:
            self.state = 4

        if self.A < self.A_cut[0]:
            self.state -= 0
        elif self.A < self.A_cut[1]:
            self.state -= 0
        elif self.A < self.A_cut[2]:
            self.state -= 0
        elif self.A < self.A_cut[3]:
            self.state -= 0
        else:
            self.state -= 0

        if self.state <= 0: # too close
            ob = np.array([[5.0, 3.2, self.A, self.D]])
        elif self.state == 1:
            ob = np.array([[6.4, 3.3, self.A, self.D]])
            ob = np.concatenate((ob, [[9.5, 9.1, self.A, self.D]]))
        elif self.state == 2:
            ob = np.array([[7.8, 2.8, self.A, self.D]])
            ob = np.concatenate((ob, [[9.4, 8.9, self.A, self.D]]))
        elif self.state == 3:
            ob = np.array([[7.0, 2.0, self.A, self.D]])
            ob = np.concatenate((ob, [[8.5, 3.1, self.A, self.D]]))
            ob = np.concatenate((ob, [[9.0, 8.0, self.A, self.D]]))
            ob = np.concatenate((ob, [[9.5, 8.7, self.A, self.D]]))
        else: # too far
            ob = np.array([[9.0, 1.3, self.A, self.D]])
            ob = np.concatenate((ob, [[9.3, 8.4, self.A, self.D]]))

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
        # if np.random.rand() < 0.2:
        #     return self.min_reward

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

        if self.state <= 0: # too close
            reward = self.min_reward
        elif self.state == 1:
            reward = 1.5
        elif self.state == 2:
            reward = 3.0
        elif self.state == 3:
            reward = 1.5
        else: # too far
            reward = self.min_reward

        # if np.random.rand() < 0.3:
        #     reward -= 0.5

        reward = max(self.min_reward, reward)
        return reward



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



    def reset(self, init_state=None, full_rand=False):
        """
        Set state randomly default and calculate corresponding path.

        Arguments
        ---------
            init_state: if it is None, then set state randomly

        Return
        ------
            A path.
        """
        # if init_state is None:
        #     if full_rand:
        #         tmp_state = np.random.rand()
        #     else:
        #         tmp_state = 0.5 + (np.random.rand() * 0.25 - 0.125) # [0.375, 0.625)

        #     self.setState(tmp_state)
        #     return self.getOb(), tmp_state
        # else:
        #     self.setState(init_state)
        #     return self.getOb(), init_state

        coes = np.random.rand(2)
        coes[0] *= 1.0
        coes[1] *= 2.0
        self.setState(coes[0], coes[1])
        return self.getOb(), self.getState()



    def decodeAction(self, action):
        """
        Arguments
        ---------
            action (int): encoded delta A, D weight.

        Return
        ------
            (float): Delta A, D weight.
        """
        if action == 0:
            dA = 0
            dD = 0
        elif action == 1:
            dA = 0
            dD = 1
        elif action == 2:
            dA = 0
            dD = 2
        elif action == 3:
            dA = 1
            dD = 0
        elif action == 4:
            dA = 1
            dD = 1
        elif action == 5:
            dA = 1
            dD = 2
        elif action == 6:
            dA = 2
            dD = 0
        elif action == 7:
            dA = 2
            dD = 1
        elif action == 8:
            dA = 2
            dD = 2

        dA = (dA - 1) * self.action_resolution
        dD = (dD - 1) * self.action_resolution

        return dA, dD



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
        dA, dD = self.decodeAction(action)
        self.setState(self.A + dA, self.D + dD)
        ob = self.getOb()

        # if self.state < self.state_min or self.state > self.state_max:
        #     done = True
        # else:
        #     done = False

        # return ob, self.getHardReward(), self.done(ob), {}
        return ob, self.getHardReward(), False, {}