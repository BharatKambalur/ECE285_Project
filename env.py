__author__ = 'Bharat'
from sim_environment import sim_environment

class env(object):
    pybulletPath = "D:/ECE 285 - Advances in Robot Manipulation/bullet3-master/data/";
    cur_state_blk_x = 0 # Stores the current position of the block
    def __init__(self,):
        self.simenv = sim_environment(tW=3,tH=6,useGUI=True,pybulletPath=self.pybulletPath);

    def calc_reward