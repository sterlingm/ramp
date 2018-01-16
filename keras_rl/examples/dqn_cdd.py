import os
import sys
import numpy as np
import gym
from gym.spaces import prng

from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten
from keras.optimizers import Adam

ramp_root = os.path.join(os.path.dirname(__file__), '../../')
sys.path.append(ramp_root) # directory_name

from keras_rl.rl.agents.dqn_si import DQNAgentSi
from keras_rl.rl.policy import BoltzmannQPolicy
from keras_rl.rl.policy import GreedyQPolicy
from keras_rl.rl.policy import EpsGreedyQPolicy
from keras_rl.rl.memory import SequentialMemory
from keras_rl.rl.callbacks import QTBVisualizer

from ramp_rlpara.rl_exercise.envs.coe_dcoe_discre import CoeDcoeDiscre



# Seed
seed_str = input("Enter a seed for random generator (must be integer): ")
seed_int = int(seed_str)
np.random.seed(seed_int) # numpy
prng.seed(seed_int) # space



# Get the environment and extract the number of actions.
env = CoeDcoeDiscre('cdd')
nb_actions = env.action_space.n



# Next, we build a very simple model Q(s,a).
model = Sequential()
model.add(Flatten(input_shape=(1,) + env.observation_space.shape)) # s is encoded D weight
model.add(Dense(16))
model.add(Activation('relu'))
model.add(Dense(16))
model.add(Activation('relu'))
model.add(Dense(16))
model.add(Activation('relu'))
model.add(Dense(nb_actions)) # Q values, number is nb_actions
model.add(Activation('linear'))
print(model.summary())



# Finally, we configure and compile our agent. You can use every built-in Keras optimizer and
# even the metrics!
memory = SequentialMemory(limit=10000, window_length=1)
policy = BoltzmannQPolicy()
dqn = DQNAgentSi(model=model, nb_actions=nb_actions, memory=memory, nb_steps_warmup=300,
               target_model_update=1e-2, policy=policy)
dqn.compile(Adam(lr=1e-3), metrics=['mae'])



# Okay, now it's time to learn something! We visualize the training here for show, but this
# slows down training quite a lot. You can always safely abort the training prematurely using
# Ctrl + C.
log_interval = 1000
nb_max_episode_steps = 300
dqn.fitSi(env, nb_steps=5000000, log_interval=log_interval, nb_max_episode_steps=nb_max_episode_steps)



# # After training is done, we save the final weights.
# # dqn.save_weights('dqn_{}_weights.h5f'.format(env.name), overwrite=True)



# Finally, evaluate our algorithm for 5 episodes.
dqn.test(env, nb_episodes=10, visualize=False, nb_max_episode_steps=nb_max_episode_steps)