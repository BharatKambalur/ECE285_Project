import pybullet as p
import sys
import numpy as np
import matplotlib.pyplot as plt
import imageio
import pyscreenshot as IG
import os
from math import sin,cos,pi,sqrt
import time


ARM_REACH = .9;
ARM_REACH_MIN = .1;
ARM_FIRST_HEIGHT = .2;

TABLE_HEIGHT = 1.30;
POKER_POS_OFFSET = 1.3;
BLOCK_HEIGHT=.09;#These are taken from URDF file
BLOCK_WIDTH=.15;
BLOCK_LENGTH=.45;
POKER_HEIGHT=.03;#These are taken from URDF file
POKER_WIDTH=.05;
POKER_LENGTH=.6;

cam_dist    = 2.5
#cam_yaw    = 60;
cam_yaw    = 0;
cam_pitch     = 0;
cam_pos     = [0,0,2.5];

class sim_environment():
    #I am pre-defining all values here, the values here should not affect anything,
    #But I like it as a reference for what the names of parameters are
    
    #Defaults, can be overwritten in initialization
    outputFilesPath = 'C:/Users/SBWork/Documents/Files/school/SP17/ECE285/Project/';

    pybulletPath = "C:/Users/SBWork/Documents/pythonLibs/bullet3/data/"

    pokerBotInitOrient = [0,0,0,0,0,0,0];
    grabberBotInitOrient = [0,0,0,0,0,0,0];
    towerWidth = 0;
    towerHeight = 0;
    towerBlocks = 0;
    towerPos = [0,0,0];
    towerOrient = 0;
    useGUI = False;
    useGrabberBot = False;
    usePokerBot = False;
    buildTower = True;
    SIM_SECOND_STEPS = 0;
    max_delta = 0;
    STEP_SIMS = 1;#Number of simulation steps to perform each movement, needs to be >1 for stability
    client = 0;
    
    log_data = False;
    cur_file_num = 0;
    RUN_FILE_NAME = 'exp_';
    data_string = '';
    
    
    pokerbot_initial_pos    = [-POKER_POS_OFFSET,0,TABLE_HEIGHT-.2];
    grabbot_initial_pos     = [1.5,0,TABLE_HEIGHT-.2];
    tower_initial_pos       = [0,0,TABLE_HEIGHT];
    
    tableID    = 0;
    pokerbotID = 0;
    pokerID    = 0;
    grabberbotID = 0;
    gripperID = 0;
            
    blockList = [];
    
    
    pokerBotInitBO = [[0,0,0],[0,0,0,1]];
    pokerInitBO = [[-1,0.0,1.47],[0,0,0,1]];#If we are using the poker bot, no need to init position, it is connected to robot
    grabberBotInitBO = [[0,0,0],[0,0,0,1]];
    towerInitBO = [];
    pokerBotRestJoint = [0,pi/2,0,-pi/2,0,0,0];
    pokerBotDesiredJoint = [0,0,0,0,0,0,0];
    pokerBotResetJoint = pokerBotDesiredJoint;
    pokerBotDesiredPos = [0,0,0];
    
    def __init__(self):
        self.initialize_environment(tW=3,tH=6);
        
    def __init__(self,tW=3,tH=6,useGUI=False,usePokerBot=False,useGrabberBot=False,SIM_SECOND_STEPS=1000,towerOrient=0,delta = .001,buildTower=True,pybulletPath="",outfilePath="",log_data=False):
        self.initialize_environment(tW,tH,useGUI,usePokerBot,useGrabberBot,SIM_SECOND_STEPS,towerOrient,delta,buildTower,pybulletPath,outfilePath,log_data);

    def initialize_environment(self,tW,tH,useGUI=False,usePokerBot=False,useGrabberBot=False,SIM_SECOND_STEPS=1000,towerOrient=0,delta = .001,buildTower=True,pybulletPath ="",outfilePath="",log_data=False):
        #This function sets up the environment, it currently only
        if(pybulletPath != ""):
            self.pybulletPath = pybulletPath;
        if(outfilePath != ""):
            self.outputFilesPath = outfilePath;
            
            
        self.tablePath = self.pybulletPath + "table/table_mid.urdf";
        self.kukaPath = self.pybulletPath + "kuka_lwr/kuka2.urdf";
        #self.kukaPath = self.pybulletPath + "kuka_iiwa/model.urdf";
        self.jengaPath = self.pybulletPath + "jenga/jenga_mid2.urdf"
        self.pokerPath = self.pybulletPath + "jenga/poker.urdf";
        self.gripperPath = self.pybulletPath + "gripper/wsg50_one_motor_gripper_new_free_base.sdf";    
            
            
        #pos block XY
        #pos end eff XYZ
        #pos top 3 blocks Z
        #position on moving block robot will need to touch
        #This function


        #tW width of jenga tower
        #tH height of jenga tower
        #useGUI use the GUI to view the simulation (set to false to speed up processing
        
    
    
        #Set environment variables
        self.towerWidth = tW;
        self.towerHeight = tH;
        self.towerBlocks = tW*tH;
        self.towerPos = [0,0,TABLE_HEIGHT];
        self.towerOrient = 0;
        self.useGUI = useGUI;
        self.useGrabberBot = useGrabberBot;
        self.usePokerBot = usePokerBot;
        self.SIM_SECOND_STEPS = SIM_SECOND_STEPS;
        self.buildTower = buildTower;
        
        
        
        self.max_delta = delta;
        self.log_data = log_data;
    
        #Set sim parameters
        if(self.useGUI):
            self.client = p.connect(p.GUI);
        else:
            self.client = p.connect(p.DIRECT);
   
        #test = p.loadURDF(pokerPath,[0,0,5],[0,0,0,1]);
        #self.step_sim();
        #print('ID = %d\n'%(test));
        #print('numJoints = %d'%(p.getNumJoints(test)));
        #print('jointInfo = %d'%(p.getJointInfo(test)));
        #print('Base Pos Orient: ');print(p.getBasePositionAndOrientation(test));
        
        p.setTimeStep(1.0/SIM_SECOND_STEPS);
        p.setGravity(0.0,0.0,-9.8);
        #p.resetDebugVisualizerCamera(5,40,0,[-.0376,0.3159,-.0344]);
        p.resetDebugVisualizerCamera(cam_dist,cam_pitch,cam_yaw,cam_pos);
        
    
        #Load sim objects
        print('Environment loading objects');
        self.tableID    = p.loadURDF(self.tablePath);
        if(usePokerBot):#If we are not counting the robot, we need the base of the block to be fixed
            self.pokerbotID = p.loadURDF(self.kukaPath,self.pokerbot_initial_pos,useFixedBase=True);
            for j in range(0,7):
                print('Joint %d'%(j));
                print(p.getJointInfo(self.pokerbotID,j));
        else:
            self.pokerbotID = 0;
            
        if(self.usePokerBot):
            self.pokerID    = p.loadURDF(self.pokerPath,[0,0,5],p.getQuaternionFromEuler([0,0,0]));
        else:
            self.pokerID    = p.loadURDF(self.pokerPath,self.pokerInitBO[0],p.getQuaternionFromEuler([0,0,0]),useFixedBase=True);
        
        #self.pokerID    = p.loadSDF(gripperPath)[0];
        
        if(self.useGrabberBot):
            self.grabberbotID = p.loadURDF(self.kukaPath,self.grabbot_initial_pos,useFixedBase=True);
            self.gripperID = p.loadSDF(self.gripperPath)[0];
        else:
            self.grabberbotID = 0;
            self.gripperID = 0;
        
        if(buildTower):
            self.blockList = self.place_tower(self.towerWidth,self.towerHeight,self.towerPos,self.towerOrient);
        else:
            print('Not building tower, this will probably cause issues');
        
        #Attach end-effectors to their robots, set robots to hold orient
        print('Environment setting contstraints');
        if(self.usePokerBot):
            #Poker s
            pokerOrient = p.getQuaternionFromEuler([pi/2,0,0]);
            cid = p.createConstraint(self.pokerbotID,6,self.pokerID,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[-POKER_LENGTH/2,0,0],childFrameOrientation=p.getQuaternionFromEuler([pi/2,pi,pi/2]));
            p.changeConstraint(cid,maxForce=10000000);
            for i in range(0,p.getNumJoints(self.pokerbotID)):
                p.setJointMotorControl2(self.pokerbotID,i,controlMode=p.POSITION_CONTROL,targetPosition=self.pokerBotInitOrient[i],positionGain=1);
            #p.setJointMotorControl2(self.pokerbotID,6,controlMode=p.POSITION_CONTROL,targetPosition=pi/2,positionGain=1);
        
        if(self.useGrabberBot):
            cid = p.createConstraint(self.grabberbotID,6,self.gripperID,-1,p.JOINT_FIXED,[0,0,0],[0,0.005,0.2],[0,.01,0.2]);
            p.changeConstraint(cid,maxForce=10000000);
            for i in range(0,p.getNumJoints(self.grabberbotID)):
                p.setJointMotorControl2(self.grabberbotID,i,controlMode=p.POSITION_CONTROL,targetPosition=self.grabberBotInitOrient[i],positionGain=1);
                
            closeGripper(self.gripperID);
                
        
        #Run a very short period of time, just so everything settles into position
        #p.setRealTimeSimulation(enableRealTimeSimulation = 1);
        for i in range(100):
            self.step_sim();
                
                
        print('Getting initial positions of all objects');
        if(self.usePokerBot):
            self.pokerBotInitBO = p.getBasePositionAndOrientation(self.pokerbotID);
            self.pokerInitBO = [[0,0,0],[0,0,0,1]];#If we are using the poker bot, no need to init position, it is connected to robot
        else:
            self.pokerBotInitBO = [[0,0,0],[0,0,0,1]];
            #self.pokerInitBO = p.getBasePositionAndOrientation(self.pokerID);
        
        if(self.useGrabberBot):
            #Grabber gripper will always be attached to grabber bot, no need to init BO
            self.grabberBotInitBO = p.getBasePositionAndOrientation(self.pokerbotID);
        else:
            self.grabberBotInitBO = [[0,0,0],[0,0,0,1]];
        
        if(buildTower):        
            self.towerInitBO = [];
            for i in range(0,self.towerBlocks):
                self.towerInitBO.append(p.getBasePositionAndOrientation(self.blockList[i]));
        else:
            print('Not building tower, this will probably cause issues');
              
        
        print('Environment setup complete');
    def log_step(self,val):
        self.data_string += val;
    def flush_log(self):
        filePath = self.outputFilesPath + self.RUN_FILE_NAME + str(self.cur_file_num) + '.bin';
        file = open(filePath,'wb');
        file.write(self.data_string);
        self.data_string = '';
        file.close();
    def recreate_run(self,filePath):
        log_setting = self.log_data;
        self.log_data = False;
        
        
        file = open(filePath,'r');
        string = file.read();
        file.close();
        
        self.reset_simulation()
        for c in string:
            if(c == 'F'):
                self.move_poker_px();
            elif(c == 'B'):
                self.move_poker_nx();
            elif(c == 'L'):
                self.move_poker_py();
            elif(c == 'R'):
                self.move_poker_ny();
            elif(c == 'U'):
                self.move_poker_pz();
            elif(c == 'D'):
                self.move_poker_nz();
            elif(c == 'S'):
                self.move_poker_stationary();
            else:
                print('Error reading run file, unrecognized character %s'%(c));
            
                
        self.log_data = log_setting;
        
    def reset_simulation(self,ignore_log = False):       
        if(self.usePokerBot):
            p.resetBasePositionAndOrientation(self.pokerbotID,self.pokerBotInitBO[0],self.pokerBotInitBO[1]);
            for i in range(0,p.getNumJoints(self.pokerbotID)):
                p.setJointMotorControl2(self.pokerbotID,i,controlMode=p.POSITION_CONTROL,targetPosition=self.pokerBotInitOrient[i],positionGain=1);
        else:
            p.resetBasePositionAndOrientation(self.pokerID,self.pokerInitBO[0],self.pokerInitBO[1]);
        
        if(self.useGrabberBot):
            #Grabber gripper will always be attached to grabber bot, no need to init BO
            p.resetBasePositionAndOrientation(self.grabberbotID,self.grabberBotInitBO[0],self.grabberBotInitBO[1]);
            for i in range(0,p.getNumJoints(self.grabberbotID)):
                p.setJointMotorControl2(self.grabberbotID,i,controlMode=p.POSITION_CONTROL,targetPosition=self.grabberBotInitOrient[i],positionGain=1);
            
        if(self.buildTower):    
            for i in range(0,self.towerBlocks):
                p.resetBasePositionAndOrientation(self.blockList[i],self.towerInitBO[i][0],self.towerInitBO[i][1]);
        

        #Run a very short period of time, just so everything settles into position
        #p.setRealTimeSimulation(enableRealTimeSimulation = 1);
        for i in range(0,100):
            self.step_sim();
            
            
        if(self.log_data and not ignore_log):    
            self.flush_log();
            self.cur_file_num+=1;
            
    #BEGIN CONTROL POKER CODE---------------------------------------------------
    def reset_poker_position(self):
        if(self.usePokerBot):
            self.pokerBotDesiredJoint = self.pokerBotResetJoint;
            for i in range(0,250):
                self.step_sim();
    def set_poker_position(self,position):
        #This sets the position based on block end, not block center
        #if(True):
        if(not self.usePokerBot):
            endPos,OR = self.get_poker_position_and_orientation();
            cenPos = self.get_poker_center_position();
            diff = np.subtract(cenPos,endPos);
            newPos = np.add(position,diff);
            OR = self.pokerInitBO[1];
            
            
            
            if(len(newPos)==1):
                newPos = newPos[0];#This is here because sometimes nump outputs array within array
            p.resetBasePositionAndOrientation(self.pokerID,newPos,OR);
            
            
        else:
                
            OR = p.getQuaternionFromEuler([0,pi/2,0]);
            endPos = self.get_poker_back_position_and_orientation(use_actual_OR=False)[0];
            cenPos = self.get_poker_position(False)[0];
            #cenPos = self.get_poker_center_position();
            diff = np.subtract(endPos,cenPos);
            #print('DIFF');
            #print(diff);
            
            newPos = np.add(position,diff);
            self.pokerBotDesiredPos = newPos;
            #newPos = position;
            jd = [.00001,.00001,.00001,.00001,.00001,.00001,.00001];
            #jd = [100,100,100,100,100,100,100];
            OR = p.getQuaternionFromEuler([0,pi/2,0]);
            
            
            
            for j in range(0,15):
                jointPos = p.calculateInverseKinematics(self.pokerbotID,6,targetPosition=newPos,targetOrientation=OR,jointDamping=jd,restPoses=self.pokerBotRestJoint);
            
                self.pokerBotDesiredJoint = jointPos;
                #print(jointPos);
                self.set_pokerBot_position(jointPos);
                #Do inverse kinematics on robot
                for i in range(0,10):
                    self.step_sim();
            #print('NewMove');
            #print(jointPos);
            #print('Back Values');
            #print(newPos);
            #print(self.get_poker_back_position_and_orientation()[0]);
            #print(np.subtract(self.get_poker_back_position_and_orientation()[0],newPos));
            
            #print('Front Values');
            #print(position);
            #print(self.get_poker_position_and_orientation()[0]);
            #print(np.subtract(self.get_poker_position_and_orientation()[0],position));
            #print('End \n');
            #print(self.get_poker_back_position_and_orientation()[0]);
            #print(p.getEulerFromQuaternion(self.get_poker_back_position_and_orientation()[1]));
            #print(self.get_poker_position_and_orientation()[0]);
            #print(p.getEulerFromQuaternion(self.get_poker_position_and_orientation()[1]));
            
            
    def set_poker_orientation(self,orientation):
        print('set poker orientation not yet implemented');
        #return [0,0,0,1];
        
    def set_pokerBot_position(self,jointPos):
        for i in range(0,7):
                p.setJointMotorControl2(self.pokerbotID,i,controlMode=p.POSITION_CONTROL,targetPosition=jointPos[i],force=1000,positionGain=.05,velocityGain=1);
    def move_poker(self,offset):
        
        #if(True):
        if(not self.usePokerBot):
            pos,OR = p.getBasePositionAndOrientation(self.pokerID);
            p.resetBasePositionAndOrientation(self.pokerID,np.add(pos,offset),OR);
            for i in range(0,self.STEP_SIMS):
               self.step_sim();
                
        else:
            #pos,OR = self.get_poker_back_position_and_orientation(False);
            #new_pos = np.add(pos,offset);
            new_pos = np.add(self.pokerBotDesiredPos,offset);
            
            if(not self.check_range(new_pos)):
                new_pos = self.pokerBotDesiredPos;
            
            self.pokerBotDesiredPos = new_pos;
            #print(new_pos);
            jd = [.00001,.00001,.00001,.00001,.00001,.00001,.00001];
            #jd = [100,100,100,100,100,100,100];
            OR = p.getQuaternionFromEuler([0,pi/2,0]);
            #print(new_pos);
            jointPos = p.calculateInverseKinematics(self.pokerbotID,6,targetPosition=new_pos,targetOrientation=OR,jointDamping=jd,restPoses=self.pokerBotRestJoint);
            self.pokerBotDesiredJoint = jointPos;
            #print(jointPos);
            self.set_pokerBot_position(jointPos);
            
        for i in range(0,self.STEP_SIMS):
            self.step_sim();  
            
            
            
    def move_poker_px(self):
        if(self.log_data):
            self.log_step('F');
        self.move_poker([self.max_delta,0,0]);
        
    def move_poker_nx(self):
        if(self.log_data): 
            self.log_step('B');
        self.move_poker([-self.max_delta,0,0]);
        
    def move_poker_py(self):
        if(self.log_data): 
            self.log_step('L');
        self.move_poker([0,self.max_delta,0]);
        
    def move_poker_ny(self):
        if(self.log_data): 
            self.log_step('R');
        self.move_poker([0,-self.max_delta,0]);
    
    def move_poker_pz(self):
        if(self.log_data): 
            self.log_step('U');
        self.move_poker([0,0,self.max_delta]);
        
    def move_poker_nz(self):
        if(self.log_data): 
            self.log_step('D');
        self.move_poker([0,0,-self.max_delta]);
        
    def move_poker_stationary(self):
        if(self.log_data): 
            self.log_step('S');
        self.move_poker([0,0,0]);
        
    #END CONTROL POKER CODE---------------------------------------------------
    def step_sim(self):
        if(self.usePokerBot):
            self.set_pokerBot_position(self.pokerBotDesiredJoint);
            
        p.stepSimulation();
    
    def check_range(self,position):
        
        #Desired Offset
        init_pos = np.add(self.pokerbot_initial_pos,[0,0,ARM_FIRST_HEIGHT]);
        d_o= np.subtract(position,init_pos);
        distance = d_o[0]*d_o[0] + d_o[1]*d_o[1] + d_o[2]*d_o[2];
        distance = sqrt(distance);
        
        #print("Need to determine bounds that robot can reliably use");
        if(distance > ARM_REACH):
            return False;
        
        if(distance < ARM_REACH_MIN):
            return False;
        
        return True;
    #BEGIN GET POS/ORIENT CODE-------------------------------------------------------
    
    #Code to get positions / orientations of various objects from the simulation
    #The next 3 functions get the position of the tip of the end effector
    def get_poker_position(self,use_actual_OR=True):
        return self.get_poker_position_and_orientation(use_actual_OR)[0];
    #Orientation is same for tip and center    
    def get_poker_orientation(self):
        return self.get_poker_position_and_orientation()[1];
    def get_poker_position_and_orientation(self,use_actual_OR=True):
        pos,OR = p.getBasePositionAndOrientation(self.pokerID);
        if(not use_actual_OR):
            OR = self.pokerInitBO[1];
        #This additional math is to get tip of end effector, not center
        rot = np.reshape(p.getMatrixFromQuaternion(OR),[3,3]);
        block_offset_t = np.reshape([POKER_LENGTH/2,0,0],[1,3]);
        block_offset = np.matmul(block_offset_t,rot);
        pos_offset = pos + block_offset;
        return pos_offset[0],OR;
        
    def get_poker_center_position(self):
        return p.getBasePositionAndOrientation(self.pokerID)[0];    
    def get_poker_center_position_and_orientation(self):
        return p.getBasePositionAndOrientation(self.pokerID);
    def get_poker_back_position(self,use_actual_OR=True):
        return self.get_poker_back_position_and_orientation(use_actual_OR)[0];
        
    def get_poker_back_position_and_orientation(self,use_actual_OR=True):
        pos,OR = p.getBasePositionAndOrientation(self.pokerID);
        op_offset = np.subtract(pos,self.get_poker_position(use_actual_OR));
        back_pos = np.add(pos,op_offset);
        return back_pos,OR;
    
    #These functions are for getting position / orientation of particular tower blocks    
    def get_block_position(self,ID):
        return self.get_block_position_and_orientation(ID)[0];
    def get_block_orientation(self,ID):
        return self.get_block_position_and_orientation(ID)[1];
    def get_block_position_and_orientation(self,ID):
        pos_OR = p.getBasePositionAndOrientation(self.blockList[ID]);
        OR = pos_OR[1];
        pos = pos_OR[0];
        
        #This additional math is to get tip of end effector, not center
        rot = np.reshape(p.getMatrixFromQuaternion(OR),[3,3]);
        block_offset = np.reshape([BLOCK_LENGTH/2,0,0],[1,3]);
        pos_offset = pos - block_offset;#Minus in this case because we want the back of the block, not the front
        return pos_offset[0],OR;
        
        
    def get_block_center_position(self,ID):
        return p.getBasePositionAndOrientation(self.blockList[ID])[0];
    def get_block_center_position_and_orientation(self,ID):
        return p.getBasePositionAndOrientation(self.blockList[ID]);
        
    def get_grabber_position(self):
        if(self.useGrabberBot):
            return p.getBasePositionAndOrientation(self.grabberBotID)[0];
        else:
            return [0,0,0];      
    def get_grabber_orientation(self):
        if(self.useGrabberBot):
            return p.getBasePositionAndOrientation(self.grabberBotID)[1];
        else:
            return [0,0,0,1];  
    def get_grabber_position_and_orientation(self):
        if(self.useGrabberBot):
            return p.getBasePositionAndOrientation(self.grabberBotID);
        else:
            return [[0,0,0],[0,0,0,1]];
    

    #END GET POS/ORIENT CODE-------------------------------------------------------

    #BEGIN OTHER PARAMETERS CODE---------------------------------------------------
    def set_movement_delta(self,delta):
        self.max_delta = delta;
        
    def get_movement_delta(self):
        return self.max_delta;
    
    def get_num_blocks(self):
        return self.towerBlocks;
        
    def get_top_blocks_IDS(self):
        num = self.towerBlocks;
        return [num-3,num-2,num-1];
        
    def get_good_push_block(self):
        #This will need to be a bit more sophisticated, but it will do for now
        center_off = (self.towerWidth-1) / 2;
        row = 3; #Odd if first row faces bot, even otherwise
        if(self.towerHeight <= row):
            row = self.towerHeight;
        return int(center_off + (row-1)*self.towerWidth); 
    #END OTHER PARAMETERS CODE---------------------------------------------------
    
    def place_tower(self,tW,tH,basePos,baseOrient=0):
        #blockPath is string path to block object
        #tW is width of tower (usually 3, must be odd)
        #tH is height of tower
        #basePos is position of center bottom block
        cosb = cos(baseOrient);
        sinb = sin(baseOrient);
        blockList = [];
        BO = (tW-1)/2;
        curHeight = basePos[2]
        for i in range(0,tH):
            curHeight = basePos[2]+BLOCK_HEIGHT*i;
            curPos = [basePos[0],basePos[1],curHeight];
            for j in range(0,tW):
                #print('(i,j) = (%d,%d)'%(i,j));
                if(i%2 ==0):
                    orientation = p.getQuaternionFromEuler([0,0,0+baseOrient]);
                    offset = (j-BO) * BLOCK_WIDTH;
                    curPos[0] = basePos[0]+offset*sinb;
                    curPos[1] = basePos[1]+offset*cosb;
                else:
                    orientation = p.getQuaternionFromEuler([0,0,(pi/2)+baseOrient]);
                    offset = (j-BO) * BLOCK_WIDTH;
                    curPos[0] = basePos[0]+offset*cosb;
                    curPos[1] = basePos[1]+offset*sinb;
                #print('Loading block at [%f,%f,%f]'%(curPos[0],curPos[1],curPos[2]));
                blockList.append(p.loadURDF(self.jengaPath,curPos,orientation));
        #print(p.getBodyInfo(blockList[0]))
        return blockList;
        
    
    
    def move_kuka_proof_of_concept(R_ID,linkID):
        jointLowerLIMITID = 8;
        jointUpperLIMITID = 9;
        mode = p.POSITION_CONTROL;
        velocity = 0;
        maxForce = .0000001;
        lower_lim = p.getJointInfo(R_ID,linkID)[8];
        upper_lim = p.getJointInfo(R_ID,linkID)[9];
        steadyState = p.getJointState(R_ID,linkID)[0];
        print('Positions: [%f,%f,%f]'%(lower_lim,upper_lim,steadyState))
        p.setJointMotorControl2(R_ID,linkID,controlMode=mode,targetPosition=lower_lim,force =200);
        for i in range(1,SIM_SECOND):
            moveSim();
            
        p.setJointMotorControl2(R_ID,linkID,controlMode=mode,targetPosition=upper_lim,force =200);
        for i in range(1,SIM_SECOND):
            moveSim();
            
        p.setJointMotorControl2(R_ID,linkID,controlMode=mode,targetPosition=steadyState,force =1000);
        for i in range(1,SIM_SECOND):
            moveSim();
    def closeGripper(self,gripperID):
        mode = p.POSITION_CONTROL;
        GRIPPER_CLOSED=[0.000000,-0.011130,-0.206421,0.205143,0.05,0.000000,0.05,0.000000];
        for i in range(0,8):
             p.setJointMotorControl2(gripperID,i,controlMode=mode,targetPosition=GRIPPER_CLOSED[i],force=100);
    
    
    def openGripper(self,gripperID):
        mode = p.POSITION_CONTROL;
        GRIPPER_OPEN=[0.000000,-0.011130,0.206421,0.205143,-0.01,0.000000,-0.01,0.000000];
        for i in range(0,8):
             p.setJointMotorControl2(gripperID,i,controlMode=mode,targetPosition=GRIPPER_OPEN[i]);

    def run_jenga_proof_of_concept(self):
        pos = [-.7,0.0,1.47];
        orient = p.getQuaternionFromEuler([0,0,0]);
        print('Running Jenga Proof of Concept');
        start_time = time.time();
        env.set_poker_position([-1,0.0,1.47]);
        env.set_poker_orientation([0,0,0,1]);
        for i in range(0,1000):
            env.move_poker_px();
        elapsed_time = time.time() - start_time;
        print('Proof of Concept complete, elapsed time = %ds'%(elapsed_time));

    def test_pokerbot(self):
        STEPS=0;
        ROB_STEPS = 250;
        BEGIN_ORIENT = [-.5,0,2];
        block_pos = [];
        block_pos.append(self.get_block_center_position(26));
        block_pos.append(self.get_block_center_position(24));
        block_pos.append(self.get_block_center_position(20));
        block_pos.append(self.get_block_center_position(18));
        block_pos.append(self.get_block_center_position(14));
        block_pos.append(self.get_block_center_position(12));
        block_pos.append(self.get_block_center_position(8));
        block_pos.append(self.get_block_center_position(6));
        for i in range(0,STEPS):
            self.step_sim();
            
            
        self.set_poker_position(BEGIN_ORIENT);
        for i in range(0,STEPS):
                self.step_sim();
        print('lol');
        raw_input();
        for i in range(0,7):
            self.pokerBotResetJoint[i] = p.getJointState(self.pokerbotID,i)[0];
            print(self.pokerBotResetJoint[i]);
            
            
        for cur_pos in block_pos:
            self.reset_poker_position();
            for i in range(0,STEPS):
                self.step_sim();
            print('reset pos');
            for i in range(0,7):
                curstate = p.getJointState(self.pokerbotID,i)[0];
                print(curstate);
            self.set_poker_position([-.6,cur_pos[1],cur_pos[2]]);
            for i in range(0,STEPS):
                self.step_sim();
            print('moving to block');
            for j in range(0,ROB_STEPS):
                self.move_poker_px();
            print('Reversing');
            for j in range(0,ROB_STEPS/2):
                self.move_poker_nx();
            
            
            
        self.set_poker_position(BEGIN_ORIENT);