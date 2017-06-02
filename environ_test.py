from sim_environment import sim_environment
import numpy as np
import time
import pybullet as p


tW              = 1;        #towerWidth
tH              = 9;        #towerHeight
useGUI          = True;     #If should use GUI or not
usePokerBot     = False;     #If Poker Bot is enabled
useGrabberBot   = False;    #If Grabber Bot is Enabled
SIM_STEPS       = 1000;     #Number of sim steps per 1 real time second
towerOrient     = 0;        #Rotation of tower (CURRENTLY ONLY WORKS WITH 0)
delta           = .002;     #XYZ resolution when using move_poker_px etc.
buildTower      = True;     #Should tower be built, probably true
log_data        = False;    #should to program log data for each run (Touchy, must be used properly



resultFolder = 'C:/Users/SBWork/Documents/Files/school/SP17/ECE285/Project/results/';
pybulletPath = "C:/Users/SBWork/Documents/pythonLibs/bullet3/data/";


env = sim_environment(tW=tW,tH=tH,useGUI=useGUI,usePokerBot=usePokerBot,useGrabberBot=useGrabberBot,SIM_SECOND_STEPS=SIM_STEPS,towerOrient=towerOrient,delta=delta,buildTower=buildTower,pybulletPath=pybulletPath,outfilePath=resultFolder,log_data=log_data);

#env.test_pokerbot();

env.test_poker();

raw_input();
env.reset_simulation();
#print('GONNA TEST POKER2');
#env.test_pokerbot();

print('GONNA TEST POKER');
raw_input();
env.test_poker();
raw_input();



env.recreate_run(outFile+ 'exp_0.bin');


    
total_time = time.time() - start_time;
print('Elapsed time %f'%(total_time));
#print(env.get_poker_center_position_and_orientation());
#print(env.get_poker_position_and_orientation());
#print(env.get_block_position_and_orientation(7));
#print(env.get_block_center_position_and_orientation(7));

#for i in range(0,3):
#    env.run_jenga_proof_of_concept();
#    env.reset_simulation();

raw_input();