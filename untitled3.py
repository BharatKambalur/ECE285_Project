from sim_environment import sim_environment
import numpy as np
import time

pybulletPath = "C:/Users/Juan Camilo Castillo/Documents/bullet3/bullet3-master/data/";

env = sim_environment(tW=3,tH=6,useGUI=True,pybulletPath=pybulletPath,);
#print(env.get_block_position(env.get_good_push_block()))
#print(env.get_poker_position())
#print(env.get_poker_center_position_and_orientation())        
print(env.towerInitBO[env.get_good_push_block()][0])
check = env.towerInitBO;
print(env.get_block_center_position(env.get_good_push_block()))
         
                          