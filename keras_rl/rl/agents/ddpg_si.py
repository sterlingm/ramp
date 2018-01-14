import os
import sys
import numpy as np

## use the keras-rl in this repository
ramp_root = os.path.join(os.path.dirname(__file__), '../../../')
sys.path.append(ramp_root) # directory_name

from keras_rl.rl.agents.ddpg import DDPGAgent

class DDPGAgentSi(DDPGAgent):
    """Write me
    """
    def __init__(self, nb_actions, actor, critic, critic_action_input, memory,
                 gamma=.99, batch_size=32, nb_steps_warmup_critic=1000, nb_steps_warmup_actor=1000,
                 train_interval=1, memory_interval=1, delta_range=None, delta_clip=np.inf,
                 random_process=None, custom_model_objects={}, target_model_update=.001, **kwargs):
        
        super(DDPGAgentSi, self).__init__(nb_actions=nb_actions, actor=actor, critic=critic,
                                          critic_action_input=critic_action_input, memory=memory,
                                          gamma=gamma, batch_size=batch_size,
                                          nb_steps_warmup_critic=nb_steps_warmup_critic,
                                          nb_steps_warmup_actor=nb_steps_warmup_actor,
                                          train_interval=train_interval, memory_interval=memory_interval,
                                          delta_range=delta_range, delta_clip=delta_clip,
                                          random_process=random_process, custom_model_objects=custom_model_objects,
                                          target_model_update=target_model_update, **kwargs)
        
        self.train_num_per_exe = None # The number of training after per execution. This should be changed dynamically.
        print('Initialize ddpg_si agent!')



    def select_action_without_noise(self, state):
        """Select a action according to a single motion state (and coefficients) without noise
        """
        batch = self.process_state_batch([state])
        action = self.actor.predict_on_batch(batch).flatten()
        assert action.shape == (self.nb_actions,)
        return action



    def forwardSi(self, trajs):
        """The customized forward method of class DDPGAgentSi.

        In si version, the observation is different from the input of network.
        This method is used to connect our si agent to the original forward
        method in the original ddpg agent of keras-rl. This method uses multiple
        motion states as input to calculate just one action and then add
        noise on it.

        Arguments
        ---------
            trajs: The observatioin of si agent, there are many ordered single motion states in it.

        Returns
        -------
            A action, which means (delta) coefficients. Note that the action should not be limited,
            also in RampEnv class.

            The noise added on the original action.
        """
        a_cnt = 0
        a_sum = np.zeros((self.nb_actions,))
        for traj in trajs: # for each trajectory
            for m in traj.trajectory.points: # for each single motion state
                s = np.array([m.time_from_start.to_sec()]) # time stamp
                s = np.append(s, m.positions) # position (x, y, theta)
                s = np.append(s, m.velocities) # velocity
                s = np.append(s, m.accelerations) # acceleration
                s = self.memory.get_recent_state(s)
                a = self.select_action_without_noise(s) # single action
                a_cnt += 1
                a_sum += a

        if a_cnt == 0:
            a_ave = np.ones(self.nb_actions)
        else:
            a_ave = 1.0 * a_sum / a_cnt

        # Apply noise, if a random process is set and training is true.
        if self.training and self.random_process is not None:
            noise = self.random_process.sample()
            assert noise.shape == a_ave.shape
            a_ave += noise

        self.recent_action = a_ave
        return a_ave, noise



    def backwardSi(self, precise_logger):
        """The customized backward method of class DDPGAgentSi.

        Don't push replay buffer, just train.

        Arguments
        ---------
            precise_logger: A customized RampRlLogger class object. Used to log training data.

        Returns
        -------
            Mean metrics of multiple training.
        """
        metrics_sum = np.zeros(len(self.metrics_names))
        self.train_num_per_exe = int(self.step / 10) + 100
        for _ in range(self.train_num_per_exe):
            metrics_sum += self.backward(0.0, is_push_exp=False, logger=precise_logger)

        return 1.0 * metrics_sum / self.train_num_per_exe



    def pushExps(self, trajs, exe_reward):
        """Push multiple single state transitions into replay buffer.

        Note that the last state in each trajectory is a terminal state.

        Arguments
        ---------
            trajs: The observatioin of si agent, there are many ordered single motion states in it.

            exe_reward: The reward of one execution, which is min(0.0, max_exe_time - actual_exe_time).

        Returns
        -------
            None
        """
        if len(trajs) == 0:
            return
        if len(trajs[0].trajectory.points) == 0:
            return

        for i, traj in enumerate(trajs): # for each trajectory
            for j, m in enumerate(traj.trajectory.points): # for each single motion state
                s = np.array([m.time_from_start.to_sec()]) # time stamp
                s = np.append(s, m.positions) # position (x, y, theta)
                s = np.append(s, m.velocities) # velocity
                s = np.append(s, m.accelerations) # acceleration

                terminal = False
                reward = 0.0
                if j == len(traj.trajectory.points) - 2: # The second last state
                    terminal = True
                    if i == len(trajs) - 1: # The last trajectory
                        reward = exe_reward

                self.memory.append(s, self.recent_action, reward, terminal,
                                   training=self.training)