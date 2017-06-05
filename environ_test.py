from sim_environment import sim_environment
import numpy as np
import time
import pybullet as p

import os

env = 0;


def simulation_examples():

    global env

    tW              = 3;        #towerWidth
    tH              = 6;        #towerHeight
    useGUI          = True;     #If should use GUI or not
    usePokerBot     = True;     #If Poker Bot is enabled
    useGripper      = True;
    useGripperBot   = True;    #If Gripper Bot is Enabled
    SIM_STEPS       = 1000;     #Number of sim steps per 1 real time second
    towerOrient     = 0;        #Rotation of tower (CURRENTLY ONLY WORKS WITH 0)
    delta           = .003;     #XYZ resolution when using move_poker_px etc.
    buildTower      = True;     #Should tower be built, probably true
    log_data        = False;    #should to program log data for each run (Touchy, must be used properly
    init_poker_pos  = [-2,0,2];
    init_gripper_pos = [.8,0,1.5];
    log_mode        = 'ALL'
    use_slow_motion = False;
    slow_factor = 5;



    resultFolder = os.path.dirname(os.path.abspath(__file__)) + '/results/';
    #pybulletPath = "C:/ProgramData/Anaconda2/Lib/bullet3/data/";
    pybulletPath = "C:/Users/SBWork/Documents/pythonLibs/bullet3/data/"


    env = sim_environment(tW=tW,tH=tH,useGUI=useGUI,usePokerBot=usePokerBot,useGripper=useGripper,useGripperBot=useGripperBot,SIM_SECOND_STEPS=SIM_STEPS,towerOrient=towerOrient,delta=delta,buildTower=buildTower,pybulletPath=pybulletPath,outfilePath=resultFolder,log_data=log_data,init_poker_pos=init_poker_pos,init_gripper_pos=init_gripper_pos,use_slow_motion=use_slow_motion,slow_factor=slow_factor);

    env.reset_simulation();
    test_gripper()
    #This code is the latest example I have shown you, it uses both to poker and the gripper, poker can be held by gripper
    start_time=time.time()

    env.reset_simulation();
    #test_cooperation();

    #env.reset_simulation();
    #test_pokerbot();
    #env.reset_simulation();
    #test_poker();

    #env.reset_simulation();
    

    #raw_input();
    #test_pokerbot();

    #env.recreate_run(outFile+ 'exp_0.bin');


        
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

def test_poker():
    start_time = time.time();
    env.set_poker_reset_position([-.6,0,1.56]);
    
    env.reset_simulation();
    print('Post reset 1 desired joint, pre move px');
    for i in range(0,700):
        env.move_poker_px();
        
        
    for i in range(0,1000):
        env.move_poker_stationary();
    string1 = env.get_log_string();
    
    env.reset_simulation();
    print('Post reset 2 desired joint');
    for i in range(0,700):
        env.move_poker_px();
        
    for i in range(0,700):
        env.move_poker_nx();
    for i in range(0,1000):
        env.move_poker_stationary();
    string2 = env.get_log_string();  
    env.reset_simulation();   #env.set_poker_reset_position([0,0,3]); 
        
    #for i in range(0,1000):
    #    env.move_poker_stationary();
    
    env.reset_simulation();
    env.recreate_run(string = string1);
    env.reset_simulation();
    env.recreate_run(string = string2);
    env.reset_simulation();
    
def test_gripper():
        env.offset_block(7,.2)
        pos = env.get_block_back_position(7)
        print(pos)
        env.set_gripper_position([.6,0,pos[2]])
        env.open_gripper()
        for i in range(0,20):
            env.step_sim()
        for i in range(0,100):
            env.move_gripper_nx()
        env.close_gripper()
        for i in range(0,20):
            env.step_sim()
        
        for i in range(0,100):
            env.move_gripper_px()    
        
def test_cooperation():
    print("Demonstrating our idea co-operative case");
    for j in range(0,100):
        env.step_sim();
    env.set_poker_position([-.5,0,1.56]);
    env.set_gripper_position([.7,0,1.56]);
    env.open_gripper();
    
    
    for j in range(0,117):
        env.move_poker_px();
        
    for j in range(0,117):
        env.move_poker_nx();
        
    env.set_gripper_position([.3,0,1.56]);
    
    for i in range(0,100):
        env.step_sim();
        
    env.close_gripper();
    
    for i in range(0,100):
        env.step_sim();
    
    
    #for i in range(0,100):
    #       env.move_gripper_pz();   


    for i in range(0,150):
            env.move_gripper_px();
    #env.set_gripper_position([.2,0,2],force = 500);
    for i in range(0,50):
            env.move_gripper_nz();
    
    env.open_gripper();
    
    for i in range(0,100):
        env.step_sim();
        
    for i in range(0,200):
            env.move_gripper_pz();
        
    
    
def test_pokerbot():
    STEPS=0;
    ROB_STEPS = 1000;
    BEGIN_ORIENT = [-.5,0,2];
    block_pos = [];
    block_pos.append(env.get_block_center_position(14));
    block_pos.append(env.get_block_center_position(12));
    block_pos.append(env.get_block_center_position(0));
    block_pos.append(env.get_block_center_position(2));
    #for i in range(0,STEPS):
    #    env.step_sim();
        
    print('RESETTING POSITION');  
    env.set_poker_reset_position(BEGIN_ORIENT);
    print('POSITION RESET');
        
    for cur_pos in block_pos:
        env.reset_poker_position(realTime=True);
        for i in range(0,STEPS):
            env.step_sim();
        print('reset pos');
        #for i in range(0,7):
        #    curstate = p.getJointState(env.pokerbotID,i)[0];
        #    print(curstate);
        env.set_poker_position([-.6,cur_pos[1],cur_pos[2]]);
        for i in range(0,STEPS):
            env.step_sim();
        print('moving to block');
        for j in range(0,ROB_STEPS):
            env.move_poker_px();
            #print(np.subtract(env.get_poker_position(),env.get_block_position(8)));
        print('Reversing');
        for j in range(0,ROB_STEPS/2):
            env.move_poker_nx();
        
        
        
    env.set_poker_position(BEGIN_ORIENT);


    
if __name__ == '__main__':
    simulation_examples();

