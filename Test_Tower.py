import pybullet as p
import sys
import numpy as np
import matplotlib.pyplot as plt
import imageio
import pyscreenshot as IG
import os

gifPath = 'C:/Users/SBWork/Documents/Files/school/SP17/ECE285/Project/movie3.gif';

CREATE_ANIMATION = 0;
SIM_SECOND = 1000;
anim_fsp = 100;

writer = [];
PI = 3.14159;

#BEGIN CAMERA VALUES
USE_PYTHON_SCREENCAP = 1;
FRAME_COUNTER = 0;
camTargetPos = [0.,0.,0.]
cameraUp = [0,0,1]
cameraPos = [1,1,1]
yaw = 40
pitch = 10.0

roll=0
upAxisIndex = 2
camDistance = 4
pixelWidth = 320
pixelHeight = 240
nearPlane = 0.01
farPlane = 1000
lightDirection = [0,1,0]
lightColor = [1,1,1]#optional argument
fov = 60


#*2 if mid, *3 if mid2, *10 if larger
BLOCK_HEIGHT=.03*3;#These are taken from URDF file
BLOCK_WIDTH=.05*3;
BLOCK_LENGTH=.15*3;

def createFunctionFile():
    file = open('pFuncs.txt','w');
    objectList = dir(p);
    for line in objectList:
        file.write('%s\n'%line);

    file.close();
def getTableCenter(tableID):
    TABLE_HEIGHT = .63;
    print('%s = %s\n'%('p.getBasePositionAndOrientation(tableID)',p.getBasePositionAndOrientation(tableID)));
    print('%s = %s\n'%('p.getNumJoints(tableID)',p.getNumJoints(tableID)));
    #print('%s = %s\n'%('p.getJointInfo(tableID)',p.getJointInfo(tableID,0)));
    print('%s = %s\n'%('p.getBodyInfo(tableID)',p.getBodyInfo(tableID)));
    print('%s = %s\n'%('p.getBodyInfo(tableID)[0]',p.getBodyInfo(tableID)[0]));
    return [0,0,TABLE_HEIGHT];
def main():
    global writer;
    if CREATE_ANIMATION == 1:
        if os.path.isfile(gifPath):
            os.remove(gifPath);
        writer = imageio.get_writer(gifPath, mode='I');
    
    
    
    FRAME_COUNTER = 0;
    pybulletPath = "C:/Users/SBWork/Documents/pythonLibs/bullet3/data/"
    tablePath = pybulletPath + "table/table_mid.urdf";
    kukaPath = pybulletPath + "kuka_lwr/kuka.urdf";
    #kukaPath = pybulletPath + "kuka_iiwa/kuka_world.sdf";
    jengaPath = pybulletPath + "jenga/jenga_mid2.urdf";
    gripperPath = pybulletPath + "gripper/wsg50_one_motor_gripper_new_free_base.sdf";
    #gripperPath = pybulletPath + "gripper/wsg50_one_motor_gripper.sdf"
    ballPath = pybulletPath + "sphere_small.urdf"
    client = p.connect(p.GUI)
    p.setTimeStep(1.0/SIM_SECOND);
    p.setGravity(0.0,0.0,-9.8);
    p.resetDebugVisualizerCamera(5,40,0,[-.0376,0.3159,-.0344]);
    
    
    
    
    TABLE_HEIGHT = .62*2;

    tableID = p.loadURDF(tablePath);
    robot0_initial_pos = [-1.2,0,TABLE_HEIGHT];
    robot1_initial_pos = [1.2,0,TABLE_HEIGHT];
    tower_initial_pos = [0,0,TABLE_HEIGHT];
    kuka0ID = p.loadURDF(kukaPath,robot0_initial_pos,useFixedBase=True);
    kuka1ID = p.loadURDF(kukaPath,robot1_initial_pos,useFixedBase=True);
    #jenga1ID = p.loadURDF(jengaPath,[1,1,1]);
    #kukaID = p.loadSDF(kukaPath,0);
    #ballID = p.loadURDF(ballPath,[.7,0,TABLE_HEIGHT]);
    for i in range(0,p.getNumJoints(kuka0ID)):
        p.setJointMotorControl2(kuka0ID,i,controlMode=p.POSITION_CONTROL,targetPosition=0,positionGain=1);
        p.setJointMotorControl2(kuka1ID,i,controlMode=p.POSITION_CONTROL,targetPosition=0,positionGain=1);
    gripper0ID = p.loadSDF(gripperPath)[0];
    gripper1ID = p.loadSDF(gripperPath)[0];
    
    
    
    #cid = p.createConstraint(tableID,-1,kuka0ID,-1,p.JOINT_POINT2POINT,[0,0,0],[0,0,0],[0,0,0]);
    #p.changeConstraint(cid,maxForce=10000000);
    cid = p.createConstraint(kuka0ID,6,gripper0ID,-1,p.JOINT_FIXED,[0,0,0],[0,0.005,0.2],[0,0.01,0.2]);
    p.changeConstraint(cid,maxForce=10000000);
    cid = p.createConstraint(kuka1ID,6,gripper1ID,-1,p.JOINT_FIXED,[0,0,0],[0,0.005,0.2],[0,0.01,0.2]);
    p.changeConstraint(cid,maxForce=10000000);
    p.setRealTimeSimulation(enableRealTimeSimulation = 1)
    
    
    #for i in range(1,int(round(SIM_SECOND*.02))):
    #    moveSim();
    #for i in range(0,6):
    #    print('On link %d'%(i));
    #    moveLink(kukaID,i);
    
    jengaBlockList = buildTower(jengaPath,3,10,[0,0,TABLE_HEIGHT+.05]);
    p.setRealTimeSimulation(enableRealTimeSimulation = 1)
    raw_input();
    
    
    p.setJointMotorControl2(kuka0ID,0,controlMode=p.POSITION_CONTROL,targetPosition=PI /2,positionGain=1);
    p.setJointMotorControl2(kuka0ID,1,controlMode=p.POSITION_CONTROL,targetPosition=-PI /2,positionGain=1);
    p.setJointMotorControl2(kuka1ID,0,controlMode=p.POSITION_CONTROL,targetPosition=PI /2,positionGain=1);
    p.setJointMotorControl2(kuka1ID,1,controlMode=p.POSITION_CONTROL,targetPosition=-PI /2,positionGain=1);
    print('A');
    for i in range(1,int(round(SIM_SECOND*.02))):
        moveSim();
        
    p.setJointMotorControl2(kuka0ID,0,controlMode=p.POSITION_CONTROL,targetPosition=0,positionGain=1);
    p.setJointMotorControl2(kuka0ID,1,controlMode=p.POSITION_CONTROL,targetPosition=-PI /2,positionGain=1);
    p.setJointMotorControl2(kuka1ID,0,controlMode=p.POSITION_CONTROL,targetPosition=PI,positionGain=1);
    p.setJointMotorControl2(kuka1ID,1,controlMode=p.POSITION_CONTROL,targetPosition=-PI /2,positionGain=1);
    print('B');
    for i in range(1,int(round(SIM_SECOND*.2))):
        moveSim();
    
    print('C');
    closeGripper(gripper0ID,time=1);
    closeGripper(gripper1ID,time=1);
    
    p.setJointMotorControl2(kuka0ID,0,controlMode=p.POSITION_CONTROL,targetPosition=0,positionGain=1);
    p.setJointMotorControl2(kuka0ID,1,controlMode=p.POSITION_CONTROL,targetPosition=0,positionGain=1,force=10000);
    p.setJointMotorControl2(kuka1ID,0,controlMode=p.POSITION_CONTROL,targetPosition=0,positionGain=1);
    p.setJointMotorControl2(kuka1ID,1,controlMode=p.POSITION_CONTROL,targetPosition=0,positionGain=1,force=10000);
    print('D');
    for i in range(1,int(round(SIM_SECOND*.2))):
        moveSim();
    
    
    #raw_input();
    #closeGripper(gripperID,False);
    #openGripper(gripperID,False);
    
    #p.setRealTimeSimulation(enableRealTimeSimulation = True)
    #raw_input();
    #openGripper(gripperID);
    #print('open');
    #moveRobotToTouchBall(kukaID,gripperID,ballID,[0,-.8,0,.8,0,-1.5,0]);
    #print('1 of 4');
    #moveRobotToTouchBall(kukaID,gripperID,ballID,[0,-.9,0,.9,0,-1.4,0]);
    #print('2 of 4');
    #moveRobotToTouchBall(kukaID,gripperID,ballID,[0,-1,0,1,0,-1.3,0]);
    #print('3 of 4');
    #moveRobotToTouchBall(kukaID,gripperID,ballID,[0,-1.1,0,1.1,0,-1.2,0]);
    #print('4 of 4');
    #closeGripper(gripperID);
    #print('close');
    #returnToBase(kukaID,gripperID,ballID);
    #print('rtb');
    #openGripper(gripperID);
    #print('open');
    
    #for i in range(1,SIM_SECOND):
    #    moveSim();
    
    
    
    #print('done');
    #raw_input();
def buildTower(blockPath,tW,tH,basePos):
    #blockPath is string path to block object
    #tW is width of tower (usually 3, must be odd)
    #tH is height of tower
    #basePos is position of center bottom block
    
    blockList = [];
    BO = (tW-1)/2;
    curHeight = basePos[2]
    for i in range(0,tH):
        curHeight = basePos[2]+BLOCK_HEIGHT*i;
        curPos = [basePos[0],basePos[1],curHeight];
        for j in range(0,tW):
            print('(i,j) = (%d,%d)'%(i,j));
            if(i%2 ==0):
                orientation = p.getQuaternionFromEuler([0,0,0]);
                yoffset = (j-BO) * BLOCK_WIDTH;
                curPos[1] = basePos[1]+yoffset;
            else:
                orientation = p.getQuaternionFromEuler([0,0,3.14159/2]);
                xoffset = (j-BO) * BLOCK_WIDTH;
                curPos[0] = basePos[0]+xoffset;
            print('Loading block at [%f,%f,%f]'%(curPos[0],curPos[1],curPos[2]));
            blockList.append(p.loadURDF(blockPath,curPos,orientation));
    print(p.getBodyInfo(blockList[0]))
    return blockList;

    
def moveSim(capture = True):
    global FRAME_COUNTER;
    p.stepSimulation();
    if (CREATE_ANIMATION == 1):# and (capture):
        if USE_PYTHON_SCREENCAP == 1:
            if 1:#(FRAME_COUNTER%(SIM_SECOND/anim_fsp)) == 0:
                im=IG.grab(bbox=(500,50,900,600))
                #temp1 = IG.core.asarray(im);
                temp = np.reshape(im, (550, 400, 3))
                np_img_arr = temp *(1./255.)
                #im.show();
                #print(np_img_arr)
                writer.append_data(np_img_arr)
            FRAME_COUNTER+=1;
        else:
            if (FRAME_COUNTER%(SIM_SECOND/anim_fsp)) == 0:
                pitch=0;
                viewMatrix = p.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance, yaw, pitch, roll, upAxisIndex)
                aspect = pixelWidth / pixelHeight;
                projectionMatrix = p.computeProjectionMatrixFOV(fov, aspect, nearPlane, farPlane);
                #getCameraImage can also use renderer=pybullet.ER_BULLET_HARDWARE_OPENGL
                img_arr = p.getCameraImage(pixelWidth, pixelHeight, viewMatrix,projectionMatrix, lightDirection,lightColor,renderer=p.ER_TINY_RENDERER);
                w=img_arr[0]
                h=img_arr[1]
                rgb=img_arr[2]
                dep=img_arr[3]
                print 'width = %d height = %d' % (w,h)
                #reshape creates np array
                np_img_arr = np.reshape(rgb, (h, w, 4))
                np_img_arr = np_img_arr*(1./255.)
                writer.append_data(np_img_arr)
                print('NP');
                print(np_img_arr)
                print('RGB');
                print(rgb);
                #image = imageio.
                #writer.append_data(np_img_arr)
                #show
                #plt.imshow(np_img_arr,interpolation='none')
            FRAME_COUNTER+=1;
    
    
def returnToBase(kukaID,gripperID,ballID):
    mode = p.POSITION_CONTROL;
    finalOrientation = [0,0,0,0,0,3.14/2];
    for i in range(0,5):
        force = 200;
        if i == 1:
            force = 300;
        p.setJointMotorControl2(kukaID,i,controlMode=mode,targetPosition=finalOrientation[i],force=force);
         
    for i in range(1,3*SIM_SECOND):
        moveSim();
    print('returnToBase() yet implemented');
def grabBall(kukaID,gripperID,ballID):
    closeGripper(gripperID)
    print('grabBall() yet implemented');
    
def moveRobotToTouchBall(kukaID,gripperID,ballID,TP):
    mode = p.POSITION_CONTROL;
    
    resultPosition = p.getBasePositionAndOrientation(ballID)[0]#p.calculateInverseKinematics(kukaID,6,
    #resultOrientation = p.getQuaternionFromEuler([0,-3.14159,0])
    resultOrientation = p.getQuaternionFromEuler([0,-3.14159,0])
    
    resultPosition = [4.0,-300.0,-300.0];
    jointPositions = p.calculateInverseKinematics(kukaID,6, resultPosition);
    jointPositions = [0,-.8,0,8,0,-1.8,0];
    print('Ball position:');
    print(resultPosition);
    print('Joint Positions: ');
    print(jointPositions);
    for i in range(0,6):
        p.setJointMotorControl2(kukaID,i,controlMode=mode,targetPosition=TP[i],force = 200);
        
        
    for i in range(0,SIM_SECOND):
        moveSim();

    print('moveRobotToTouchBall() yet implemented');


def raiseBall():
    print('Raise Ball not implemented');
    
def moveLink(R_ID,linkID):
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
    
def closeGripper(gripperID,capture=True,time=SIM_SECOND):
    mode = p.POSITION_CONTROL;
    GRIPPER_CLOSED=[0.000000,-0.011130,-0.206421,0.205143,0.05,0.000000,0.05,0.000000];
    for i in range(0,8):
         p.setJointMotorControl2(gripperID,i,controlMode=mode,targetPosition=GRIPPER_CLOSED[i],force=100);
         
    for i in range(1,time):
        moveSim(capture);
    
    
def openGripper(gripperID,capture=True):
    mode = p.POSITION_CONTROL;
    GRIPPER_OPEN=[0.000000,-0.011130,0.206421,0.205143,-0.01,0.000000,-0.01,0.000000];
    for i in range(0,8):
         p.setJointMotorControl2(gripperID,i,controlMode=mode,targetPosition=GRIPPER_OPEN[i]);
         
    for i in range(1,SIM_SECOND):
        moveSim(capture);
    
if __name__ == "__main__":
    main();
    #createFunctionFile();