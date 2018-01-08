#!/usr/bin/env python

import os
import sys
import rospy
import numpy as np

## use the keras-rl in this repository
ramp_root = os.path.join(os.path.dirname(__file__), '../../')
sys.path.append(ramp_root) # directory_name

from keras_rl.rl.memory import SequentialMemory

class TDEnv(object):

    def __init__(self):
        self.ob = None
        ## [16.0, 17.9, 19.8, 21.7, 23.6] + weight / [0.91, 1.115, 1.32, 1.525, 1.73]
        self.cost_array = np.array([0.91, 1.115, 1.32, 1.525, 1.73])
        self.cost_array = 15.0 / self.cost_array
        self.cost_array += [16.0, 17.9, 19.8, 21.7, 23.6]
        self.cost_array = (self.cost_array - 30.0)**2 # convenient to monitor

    def calcCost(self):
        state = D_max * self.ob / ob_size

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
    
    def reset(self):
        self.ob = np.random.randint(0, ob_size)
        self.ob = 0
        return self.ob

    def step(self, a):
        a -= 1
        self.ob += a
        if self.ob < 0 or self.ob > ob_size - 1:
            beyond_limit = True
        else:
            beyond_limit = False
            
        self.ob = np.clip(self.ob, 0, ob_size - 1)

        cost = self.calcCost() + 0.001 * a**2 # a**2 means energy cost
        reward = -cost + 10  # Remove the conflict

        done = beyond_limit # Conflict with target = exp.reward in replay
        # if beyond_limit:
        #     reward -= 100.0 # Remove the above conflict
        done = False
        info = {}

        return self.ob, reward, done, info

def test():
    print("Final Q-Table Values:\n %s" % Q)
    result_str = ''
    for s in range(ob_size):
        #### No random action in testing
        a = np.argmax(Q[s,:])
        if a == 0:
            a_str = '<-   '
        elif a == 1:
            a_str = 'stop   '
        else:
            a_str = '->   '
        result_str += a_str
    print(result_str)

rospy.init_node('q_tb_TD', anonymous = True)
np.random.seed()

D_max = 0.5
ob_size = 30
d_D = D_max / ob_size
act_size = 3
discount_factor = 0.99 # gamma
lr = 0.8 # In ramp it is 0.001
max_epi_steps = 100
batch_size = 1 # only replay, there is no mini_batch training for q table
warm_up_steps = 100

env = TDEnv()
# Q = np.zeros((ob_size, act_size)) # Q Table
Q = np.random.rand(ob_size, act_size)
replay_buffer = SequentialMemory(limit = 100000, window_length = 1)
assert act_size == 3
i = 0
e = 0.1
n_step = 0
last_print_step = 0
last_mean_q = None
while not rospy.core.is_shutdown():
    ## Reset environment and get first new observation
    s = env.reset()
    episode_reward = 0
    ## The Q-Table learning algorithm
    for j in range(max_epi_steps):
        ## Choose an action greedily with e random
        if np.random.rand() < e:
            a = np.random.randint(0, act_size)
        else:
            a = np.argmax(Q[s,:])
        ## Get new state and reward from environment
        s1, r, d, info = env.step(a)

        # replay_buffer.append(s, a, r, d, training = True)
        # if n_step > warm_up_steps:
        #     experiences = replay_buffer.sample(batch_size)
        #     assert len(experiences) == batch_size
        #     sum_loss = 0.0
        #     for exp in experiences:
        #         if exp.terminal1:
        #             target = exp.reward # Conflict with done = beyond_limit
        #         else:
        #             target = exp.reward + discount_factor * np.max(Q[exp.state1,:]) # TODO: try on-policy
        #         loss = target - Q[exp.state0, exp.action]
        #         sum_loss += loss
        #     ave_loss = 1.0 * sum_loss / batch_size
        #     Q[exp.state0, exp.action] += lr * ave_loss

        if d:
            loss = r
        else:
            loss = r + discount_factor * np.max(Q[s1, :]) - Q[s, a]
        
        # loss = loss * loss // TODO: learn ANN
        Q[s, a] += lr * loss
        
        episode_reward += r
        s = s1
        n_step += 1

        if d == True:
            break
    
        if n_step - last_print_step > 10000:
            last_print_step = n_step
            test()
            mean_q = np.mean(Q)
            if last_mean_q is None:
                mean_q_dot = 1.0
            else:
                mean_q_dot = mean_q / last_mean_q - 1.0
            last_mean_q = mean_q
            print("Episode [%d] mean reward: %f, total steps: %d, mean_q: %f, mean_q_dot: %f%%" %
                (i, 1.0 * episode_reward / max_epi_steps, n_step, mean_q, mean_q_dot * 100.0))
            ## TODO: plot

    i += 1

test()