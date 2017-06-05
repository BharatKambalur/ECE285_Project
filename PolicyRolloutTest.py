from ddqn import DQNAgent
import random
import gym
import numpy as np
from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam
from keras import backend as K
from environment import environment
import time as timer


if __name__ == "__main__":
    ##################################################################################
    ##################### Uncomment for your own ####################################
    #pybulletPath = "/home/auggienanz/bullet3/data/" #Auggie
    #pybulletPath = "D:/ECE 285 - Advances in Robot Manipulation/bullet3-master/data/" #Bharat
    pybulletPath = 'C:/Users/Juan Camilo Castillo/Documents/bullet3/bullet3-master/data/' #Juan
    outputpath = 'C:/Users/Juan Camilo Castillo/Documents/ECE 285 Robotics/save/' #Juan

    #################################################################################

    env = environment(pybulletPath,useGUI = True, movement_delta = 0.003)
    state_size = 6
    action_size = 6
    agent = DQNAgent(state_size, action_size)
    agent.load("./run_results/JengaLearn_2.h5")
    state = env.reset_random()
    print(state)
    state = np.reshape(state, [1, state_size])
    timer.sleep(1)
    print('Starting Policy Rolloout from learned weights')
    for time in range(1000):
        action = np.argmax(agent.model.predict(state)[0])
        next_state, reward, done = env.step(action)
        timer.sleep(0.005)
        next_state = np.reshape(next_state, [1, state_size])
        agent.remember(state, action, reward, next_state, done)
        state = next_state

