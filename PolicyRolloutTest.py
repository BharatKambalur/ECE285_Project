#from ddqn import DQNAgent
from dqn import DQNAgent
import numpy as np
from environment import environment
import time as timer


if __name__ == "__main__":
    ##################################################################################
    ##################### Uncomment for your own ####################################
    pybulletPath = "/home/auggienanz/bullet3/data/" #Auggie
    #pybulletPath = "D:/ECE 285 - Advances in Robot Manipulation/bullet3-master/data/" #Bharat
    #pybulletPath = 'C:/Users/Juan Camilo Castillo/Documents/bullet3/bullet3-master/data/' #Juan
    #outputpath = 'C:/Users/Juan Camilo Castillo/Documents/ECE 285 Robotics/save/' #Juan

    #################################################################################

    env = environment(pybulletPath,useGUI = True, movement_delta = 0.003)
    state_size = 6
    action_size = 6
    agent = DQNAgent(state_size, action_size)
    agent.load("./run_results/JengaLearn_dqn.h5")
    print('Starting Policy Rolloout from learned weights')
    for i in range(5):
        state = env.reset_random()
        for time in range(1000):
            state = np.reshape(state, [1, state_size])
            action = np.argmax(agent.model.predict(state)[0])
            state, reward, done = env.step(action)
            timer.sleep(0.005)
            if (done):
                break
        timer.sleep(2)

