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
        print('Initialize ddpg_si agent!')
        pass

    def forwardSi(self, ob):
        """The customized forward method of class DDPGAgentSi.

        In si version, the observation is different from the input of network.
        This method is used to connect our si agent to the original forward
        method in the original ddpg agent of keras-rl.

        # Arguments
            ob: The observatioin of si agent, there are many ordered single motion states in it.

        # Returns
            A action, which means delta coefficients. Note that this action is not limited, the limitation
            should be done by the environment class.
        """