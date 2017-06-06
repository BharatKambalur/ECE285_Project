# -*- coding: utf-8 -*-
import random
from environment import environment
import numpy as np
from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam
import time as timer
import matplotlib.pyplot as plt

EPISODES = 1000

class DQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = deque(maxlen=5000)
        self.gamma = 0.5    # discount rate
        self.epsilon = 1.0  # exploration rate
        self.epsilon_min = 0.1
        self.epsilon_decay = 0.995
        self.learning_rate = 0.001
        self.model = self._build_model()

    def _build_model(self):
        # Neural Net for Deep-Q learning Model
        model = Sequential()
        model.add(Dense(24, input_dim=self.state_size, activation='relu'))
        model.add(Dense(24, activation='relu'))
        model.add(Dense(self.action_size, activation='linear'))
        model.compile(loss='mse',
                      optimizer=Adam(lr=self.learning_rate))
        return model

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        act_values = self.model.predict(state)
        return np.argmax(act_values[0])  # returns action

    def replay(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)
        for state, action, reward, next_state, done in minibatch:
            target = reward
            if not done:
              target = reward + self.gamma * \
                       np.amax(self.model.predict(next_state)[0])
            target_f = self.model.predict(state)
            target_f[0][action] = target
            self.model.fit(state, target_f, epochs=1, verbose=0)
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def load(self, name):
        self.model.load_weights(name)

    def save(self, name):
        self.model.save_weights(name)


if __name__ == "__main__":
    ##################################################################################
    ##################### Uncomment for your own ####################################
    #pybulletPath = "/home/auggienanz/bullet3/data/" #Auggie
    #pybulletPath = "D:/ECE 285 - Advances in Robot Manipulation/bullet3-master/data/" #Bharat
    pybulletPath = 'C:/Users/Juan Camilo Castillo/Documents/bullet3/bullet3-master/data/' #Juan
    outputpath = 'C:/Users/Juan Camilo Castillo/Documents/ECE 285 Robotics/save/' #Juan

    #################################################################################

    env = environment(pybulletPath = pybulletPath,useGUI = True,movement_delta = 0.003)
    state_size = 6
    action_size = 6
    agent = DQNAgent(state_size, action_size)
    # agent.load("./save/cartpole-master.h5")
    done = False
    batch_size = 300
    print('Starting Simulations')
    starttime = timer.time();
    TR = []
    E = []
    for e in range(EPISODES):
        state = env.reset_random()
        state = np.reshape(state, [1, state_size])
        TotalReward = 0
        for time in range(4000):
            action = agent.act(state)
            next_state, reward, done, _ = env.step(action)
            TotalReward = reward + TotalReward
            next_state = np.reshape(next_state, [1, state_size])
            agent.remember(state, action, reward, next_state, done)
            state = next_state
            if done:
                print("episode: {}/{}, Reward score: {}, e: {:.2}"
                      .format(e, EPISODES, TotalReward, agent.epsilon))
                TR.append(TotalReward)
                E.append(e)
                break
        if len(agent.memory) > batch_size:
            agent.replay(batch_size)
    print((timer.time() - starttime)/EPISODES)
    fig = plt.figure()
    ax = plt.subplot(111)
    ax.plot(E,TR)
    plt.title('Episodic Reward')
    plt.ylabel('Reward')
    plt.xlabel('episode')
    #fig.savefig(outputpath + 'Episodic Reward_5.png')
    #agent.save(outputpath + 'JengaLearn_5.h5')
