#!/usr/bin/env python

import os
import sys
import rospy
import numpy as np

import keras.backend as K
from keras.models import Model
from keras.layers import Lambda, Input, Layer, Dense

from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten
from keras.optimizers import Adam

from rl.util import *

## use the keras-rl in this repository
ramp_root = os.path.join(os.path.dirname(__file__), '../../')
sys.path.append(ramp_root) # directory_name

from keras_rl.rl.memory import SequentialMemory

replay_buffer = SequentialMemory(limit = 1000000, window_length = 1)

class TDEnv(object):

    def __init__(self):
        self.ob = None
        ## [16.0, 17.9, 19.8, 21.7, 23.6] + weight / [0.91, 1.115, 1.32, 1.525, 1.73]
        self.cost_array = np.array([0.91, 1.115, 1.32, 1.525, 1.73])
        self.cost_array = 15.0 / self.cost_array
        self.cost_array += [16.0, 17.9, 19.8, 21.7, 23.6]
        self.cost_array = (self.cost_array - 30.0)**2 # convenient to monitor

    def calcCost(self):
        if self.ob < 0.1:
            return self.cost_array[0]
        elif self.ob < 0.2:
            return self.cost_array[1]
        elif self.ob < 0.3:
            return self.cost_array[2]
        elif self.ob < 0.4:
            return self.cost_array[3]
        else:
            return self.cost_array[4]

    def reset(self):
        self.ob = np.random.rand() * 0.5
        # self.ob = 0
        return self.ob

    def step(self, a):
        a -= 1
        a *= act_resolution
        self.ob += a
        if self.ob < 0 or self.ob > D_max:
            beyond_limit = True
        else:
            beyond_limit = False
            
        self.ob = np.clip(self.ob, 0.0, D_max)
        
        cost = self.calcCost()
        reward = -cost + 4.0

        done = beyond_limit # Conflict with target = exp.reward in replay
        if beyond_limit:
            reward -= 100.0 # Remove the above conflict
        # done = False
        info = {}

        return self.ob, reward, done, info

def mean_q(y_true, y_pred):
    return K.mean(K.max(y_pred, axis=-1))

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

def process_state_batch(self, batch):
    batch = np.array(batch)
    if self.processor is None:
        return batch
    return self.processor.process_state_batch(batch)

def compute_batch_q_values(self, state_batch):
    batch = process_state_batch(state_batch)
    q_values = model.predict_on_batch(batch)
    assert q_values.shape == (len(state_batch), self.nb_actions)
    return q_values

def forward(observation):
    # Select an action.
    state = replay_buffer.get_recent_state(observation)

    q_values = self.compute_batch_q_values([state]).flatten()
    assert q_values.shape == (self.nb_actions,)

    if self.training:
        action = self.policy.select_action(q_values=q_values)
    else:
        action = self.test_policy.select_action(q_values=q_values)

    # Book-keeping.
    self.recent_observation = observation
    self.recent_action = action


T = 1.0
D = 0.0
D_max = 0.5
ob_size = 5
d_D = D_max / ob_size
act_size = 3
act_resolution = 0.01
discount_factor = 0.95 # gamma
lr = 0.001 # In ramp it is 0.001
max_epi_steps = 10
batch_size = 1
warm_up_steps = 100

env = TDEnv()

## Q Net
model = Sequential()
model.add(Flatten(input_shape=(1,) + (1,)))
# model.add(Dense(16))
# model.add(Activation('relu'))
# model.add(Dense(16))
# model.add(Activation('relu'))
# model.add(Dense(16))
# model.add(Activation('relu'))
model.add(Dense(act_size))
model.add(Activation('linear'))
print(model.summary())



## Compile Q net
model.compile(optimizer='sgd', loss='mse')
def clipped_masked_error(args):
    y_true, y_pred, mask = args
    loss = huber_loss(y_true, y_pred, np.inf) # loss = 0.5 * (y_true - y_pred)**2, TODO: try this simpler loss
    loss *= mask  # apply element-wise mask, mask is used to select the action we adopt by the policy
    return K.sum(loss, axis=-1)
# Create trainable model. The problem is that we need to mask the output since we only
# ever want to update the Q values for a certain action. The way we achieve this is by
# using a custom Lambda layer that computes the loss. This gives us the necessary flexibility
# to mask out certain parameters by passing in multiple inputs to the Lambda layer.
y_pred = model.output
y_true = Input(name='y_true', shape=(act_size,))
mask = Input(name='mask', shape=(act_size,))
loss_out = Lambda(clipped_masked_error, output_shape=(1,), name='loss')([y_pred, y_true, mask])
ins = [model.input] if type(model.input) is not list else model.input # s
trainable_model = Model(inputs=ins + [y_true, mask], outputs=[loss_out, y_pred])
assert len(trainable_model.output_names) == 2
losses = [
    lambda y_true, y_pred: y_pred,  # loss is computed in Lambda layer
    lambda y_true, y_pred: K.zeros_like(y_pred),  # we only include this for the metrics
]
trainable_model.compile(optimizer=Adam(lr=lr), loss=losses)



assert act_size == 3
i = 0
e = 0.1
n_step = 0
observation = None
while not rospy.core.is_shutdown():
    if observation is None:  # start of a new episode
        episode_step = 0
        episode_reward = 0.0

        # Obtain the initial observation by resetting the environment.
        observation = env.reset()
        assert observation is not None

    # Run a single step.
    # This is all of the work happens. We first perceive and compute the action
    # (forward step) and then use the reward to improve (backward step).
    action = forward(observation)
    if self.processor is not None:
        action = self.processor.process_action(action)
    reward = 0.
    accumulated_info = {}
    done = False
    for _ in range(action_repetition):
        callbacks.on_action_begin(action)
        observation, r, done, info = env.step(action)
        observation = deepcopy(observation)
        if self.processor is not None:
            observation, r, done, info = self.processor.process_step(observation, r, done, info)
        for key, value in info.items():
            if not np.isreal(value):
                continue
            if key not in accumulated_info:
                accumulated_info[key] = np.zeros_like(value)
            accumulated_info[key] += value
        callbacks.on_action_end(action)
        reward += r
        if done:
            break
    if nb_max_episode_steps and episode_step >= nb_max_episode_steps - 1:
        # Force a terminal state.
        done = True
    metrics = self.backward(reward, terminal=done)
    episode_reward += reward

    step_logs = {
        'action': action,
        'observation': observation,
        'reward': reward,
        'metrics': metrics,
        'episode': episode,
        'info': accumulated_info,
    }
    callbacks.on_step_end(episode_step, step_logs)
    episode_step += 1
    self.step += 1

    if done:
        # We are in a terminal state but the agent hasn't yet seen it. We therefore
        # perform one more forward-backward call and simply ignore the action before
        # resetting the environment. We need to pass in `terminal=False` here since
        # the *next* state, that is the state of the newly reset environment, is
        # always non-terminal by convention.
        self.forward(observation)
        self.backward(0., terminal=False)

        # This episode is finished, report and reset.
        episode_logs = {
            'episode_reward': episode_reward,
            'nb_episode_steps': episode_step,
            'nb_steps': self.step,
        }
        callbacks.on_episode_end(episode, episode_logs)

        episode += 1
        observation = None
        episode_step = None
        episode_reward = None
    
    ## The DQN learning algorithm
    for j in range(max_epi_steps):
        ## Choose an action greedily with e random
        if np.random.rand() < e:
            a = np.random.randint(0, act_size)
        else:
            a = np.argmax(Q[s,:])
        ## Get new state and reward from environment
        s1, r, d, info = env.step(a)
        replay_buffer.append(s, a, r, d, training = True)

        if n_step > warm_up_steps:
            experiences = replay_buffer.sample(batch_size)
            assert len(experiences) == batch_size
            sum_loss = 0.0
            for exp in experiences:
                if exp.terminal1:
                    target = exp.reward # Conflict with done = beyond_limit
                else:
                    target = exp.reward + discount_factor * np.max(Q[exp.state1,:]) # TODO: try on-policy
                loss = target - Q[exp.state0, exp.action]
                sum_loss += loss

            ave_loss = 1.0 * sum_loss / batch_size
            Q[exp.state0, exp.action] += lr * ave_loss

        # loss = r + discount_factor * np.max(Q[s1, :]) - Q[s, a]
        # loss = loss.item()
        # # loss = loss * loss // TODO: learn ANN
        # Q[s, a] += lr * loss
        
        episode_reward += r
        s = s1
        n_step += 1

        if d == True:
            break
    
    if n_step % 100 == 0:
        print("Episode [%d] mean reward: %f, total steps: %d, mean_q: %f" %
             (i, 1.0 * episode_reward / max_epi_steps, n_step, np.mean(Q)))
        ## TODO: plot

    if n_step % 100 == 0:
        test()

    i += 1

test()