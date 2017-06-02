from sim_environment import sim_environment
import numpy as np
import time
import pybullet as p

env = 0;

def test_poker():
    start_time = time.time();
    env.set_poker_reset_position([-1,0,2]);
    env.reset_simulation();
    for i in range(0,1000):
        env.move_poker_px();
    string1 = env.get_log_string();    
    env.reset_simulation();
        
    for i in range(0,1000):
        env.move_poker_pz();
    string2 = env.get_log_string();  
    env.reset_simulation();   #env.set_poker_reset_position([0,0,3]); 
        
    #for i in range(0,1000):
    #    env.move_poker_stationary();
    
    env.reset_simulation();
    env.recreate_run(string = string1);
    env.reset_simulation();
    env.recreate_run(string = string2);
    env.reset_simulation();
    
    env.set_poker_reset_position([0,0,3]);
    env.reset_simulation();
    for i in range(0,1000):
        env.move_poker_px();
        
    env.reset_simulation();
        
    for i in range(0,1000):
        env.move_poker_pz(); 
    env.reset_simulation();    
        
    for i in range(0,1000):
        env.move_poker_stationary();
    env.reset_simulation();
        
        
def test_pokerbot():
    STEPS=0;
    ROB_STEPS = 250;
    BEGIN_ORIENT = [-.5,0,2];
    block_pos = [];
    block_pos.append(env.get_block_center_position(26));
    block_pos.append(env.get_block_center_position(24));
    block_pos.append(env.get_block_center_position(20));
    block_pos.append(env.get_block_center_position(18));
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
            print(np.subtract(env.get_poker_position(),env.get_block_position(8)));
        print('Reversing');
        for j in range(0,ROB_STEPS/2):
            env.move_poker_nx();
        
        
        
    env.set_poker_position(BEGIN_ORIENT);

def simulation_examples():

    global env

    tW              = 3;        #towerWidth
    tH              = 9;        #towerHeight
    useGUI          = True;     #If should use GUI or not
    usePokerBot     = False;     #If Poker Bot is enabled
    useGrabberBot   = False;    #If Grabber Bot is Enabled
    SIM_STEPS       = 1000;     #Number of sim steps per 1 real time second
    towerOrient     = 0;        #Rotation of tower (CURRENTLY ONLY WORKS WITH 0)
    delta           = .003;     #XYZ resolution when using move_poker_px etc.
    buildTower      = True;     #Should tower be built, probably true
    log_data        = False;    #should to program log data for each run (Touchy, must be used properly



    resultFolder = 'C:/Users/SBWork/Documents/Files/school/SP17/ECE285/Project/results/';
    pybulletPath = "C:/Users/SBWork/Documents/pythonLibs/bullet3/data/";


    env = sim_environment(tW=tW,tH=tH,useGUI=useGUI,usePokerBot=usePokerBot,useGrabberBot=useGrabberBot,SIM_SECOND_STEPS=SIM_STEPS,towerOrient=towerOrient,delta=delta,buildTower=buildTower,pybulletPath=pybulletPath,outfilePath=resultFolder,log_data=log_data);

    test_pokerbot();
    env.reset_simulation();
    test_poker();

    raw_input();
    env.reset_simulation();
    #print('GONNA TEST POKER2');
    #test_pokerbot();

    print('GONNA TEST POKER');
    raw_input();
    test_poker();
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
    
if __name__ == '__main__':
    simulation_examples();

