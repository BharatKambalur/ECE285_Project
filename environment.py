from sim_environment import sim_environment
import numpy as np
import time
import matplotlib as plt

class environment(object):

    def __init__(self,pybulletPath):
        self.env = sim_environment(tW=3,tH=6,useGUI=True,pybulletPath=pybulletPath)
        self.env.set_poker_position([-1,0.0,1.47])
        
        self.GB_ID = self.env.get_good_push_block()
        self.TopBlocks_IDs = self.env.get_top_blocks_IDS()
        self.init_GB_pos = self.env.get_block_position(self.GB_ID)
        self.init_TB1_pos = self.env.get_block_center_position(self.TopBlocks_IDs[0])
        self.init_TB2_pos = self.env.get_block_center_position(self.TopBlocks_IDs[1])
        self.init_TB3_pos = self.env.get_block_center_position(self.TopBlocks_IDs[2])
        self.poker_reached_close = self.poker_close_check()

    def knocked_over_check(self):
        TB1 = self.env.get_block_center_position(self.TopBlocks_IDs[0])
        TB2 = self.env.get_block_center_position(self.TopBlocks_IDs[1])
        TB3 = self.env.get_block_center_position(self.TopBlocks_IDs[2])
        Top_Dist = np.linalg.norm(np.subtract(TB1,self.init_TB1_pos)) + np.linalg.norm(np.subtract(TB2,self.init_TB2_pos)) + np.linalg.norm(np.subtract(TB3,self.init_TB3_pos))
        if Top_Dist > 0.01:
            return True
        else:
            return False

    def pushed_out_check(self):
        PB = self.env.get_block_position(self.GB_ID)[0] # Read the current X-position
        if PB > 0: # Check if pushed half-way
            return True
        else:
            return False

    def done_check(self):
        if self.pushed_out_check() == True or self.knocked_over_check() == True:
            return True
        else:
            return False

    def get_state(self):
        return list(np.append(self.env.get_poker_position(), self.env.get_block_position(self.GB_ID)))

    def poker_close_check(self):
        poker_pos = self.env.get_poker_position()
        init_block_pos = self.init_GB_pos
        dist = np.linalg.norm(poker_pos-init_block_pos)
        if dist < 0.01:
            return True
        else:
            return False

    def calc_reward(self, old_poker_pos, old_block_pos):
        if self.knocked_over_check() == True:
            return 0
        if self.poker_reached_close == False:
            GB_ID = self.env.get_good_push_block()
            init_block_pos = self.env.towerInitBO[GB_ID][0]
            old_dist = np.linalg.norm(old_poker_pos-init_block_pos)
            new_poker_pos = self.env.get_poker_position()
            new_dist = np.linalg.norm(new_poker_pos-init_block_pos)
            if old_dist > new_dist:
                return 0.5

        new_block_pos = self.env.get_block_position(self.env.get_good_push_block())
        block_pushed_dist =new_block_pos[0]- old_block_pos[0] 
        if block_pushed_dist >(0.9*self.env.get_movement_delta()):
            return 1
        else:
            return 0

    def step(self,action):
        old_poker_pos = self.env.get_poker_position()
        old_block_pos = self.env.get_block_position(self.GB_ID)
        if action == 0:
            self.env.move_poker_px()
        elif action == 1:
            self.env.move_poker_nx()
        elif action == 2:
            self.env.move_poker_py()
        elif action == 3:
            self.env.move_poker_ny()
        elif action == 4:
            self.env.move_poker_pz()
        elif action == 5:
            self.env.move_poker_nz()

        if self.poker_reached_close == False:
            self.poker_reached_close = self.poker_close_check()

        done = self.done_check()
        ns = list(self.get_state())
        reward = self.calc_reward(old_poker_pos,old_block_pos)
        return ns, reward, done

if __name__ == "__main__":
    # The default execution is simply poking one block through and
    # then making the tower fall over by moving to the left and right

    ##################################################################################
    ##################### Uncomment for your own ####################################
    #pybulletPath = "/home/auggienanz/bullet3/data/" #Auggie
    #pybulletPath = "D:/ECE 285 - Advances in Robot Manipulation/bullet3-master/data/" #Bharat
    pybulletPath = 'C:/Users/Juan Camilo Castillo/Documents/bullet3/bullet3-master/data/' #Juan

    #################################################################################

    env = environment(pybulletPath)
    start_time = time.time()
    print('Initial Env State:')
    print(env.get_state())
    for i in range(0,900):
        ns, reward, done = env.step(0)
        time.sleep(0.001)
        print(ns, reward, done)



    for i in range(0,500):
        ns, reward, done = env.step(2)
        time.sleep(0.001)
        print(ns, reward, done)
        # #time.sleep(.005);
        # #print("Reward:{}".format(R));

    # print(env.knocked_over_check())
    # print(env.pushed_out_check())



    for i in range(0,2000):
        ns, reward, done = env.step(3)
        print(ns, reward, done)
        time.sleep(.001);
        # #print("Reward:{}".format(R));

    # print(env.knocked_over_check())
    # print(env.pushed_out_check())

    # total_time = time.time() - start_time
    # print('Elapsed time %f'%(total_time))

    # #w,h,img = env.captureImage()
    # #print(np.shape(np.reshape(np.array(img),(w,h))))
    # #print(len(img))

         
         
         
         