RAMP-RL
====

This is a ROS metapackage that will implement the Real-time Adaptive Motion Planning (RAMP) algorithm with Reinforcement Learning (RL) to adjust the coefficients of evaluation functions in RAMP.

1. Install TensorFlow: https://www.tensorflow.org/install/

2. Install TensorLayer: https://tensorlayercn.readthedocs.io/zh/latest/user/installation.html

3. Install Keras-rl: https://github.com/matthiasplappert/keras-rl

Usage:

1. roscore

2. roslaunch ramp_launch my_view_robot_costmap.launch:
    This command will open RViz for visualization

3. roslaunch ramp_launch gazebo_costmap_tmp.launch:
    This command will open Gazebo, construct a simulation world
    
4. rosrun ramp_rlpara actual_env_simulation.py
    This command will open ramp_planner and other necessary nodes. Until here, the whole simulation environment for a RL agent is ready and is waiting for the agent to take action. This file will also log data during the interaction between agent and the simulation environment, which may be used again to do learning (off-line, without simulation world or real world, the .bag files can be seen as environment).
    
5. rosrun ramp_rlpara ddpg_ramp.py
    This command will load the interface of environment (ramp_rlpara/ramp_gym/ramp_env_interfaces/*.py), construct a DDPG agent (agent itself is implemented by keras-rl) and interaction with environment to do learning.
    
    You can also use other files including other agents:
    
    - ~~~~rosrun ramp_rlpara random_ramp.py (take random action, can be used to log data)
