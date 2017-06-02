from sim_environment import sim_environment
import numpy as np
import time
import matplotlib as plt

class environment(object):

    def __init__(self,pybulletPath):
        self.env = sim_environment(tW=3,tH=6,useGUI=False,pybulletPath=pybulletPath)
        self.env.set_poker_position([-1,0.0,1.47])
        self.poker_reached_close = self.poker_close_check()
        self.GB_ID = self.env.get_good_push_block()
        self.TopBlocks_IDs = self.env.get_top_blocks_IDS()
        self.init_GB_pos = self.env.get_block_position(self.GB_ID)
        self.init_TB1_pos = self.env.get_block_center_position(self.TopBlocks_IDs[0])
        self.init_TB2_pos = self.env.get_block_center_position(self.TopBlocks_IDs[1])
        self.init_TB3_pos = self.env.get_block_center_position(self.TopBlocks_IDs[2])

    def knocked_over_check(self):
        wf = 0.01
        TopBlocks = self.env.get_top_blocks_IDS()
        TB1 = self.env.get_block_center_position(TopBlocks[0])
        TB1init = self.env.towerInitBO[TopBlocks[0]][0]
        TB2 = self.env.get_block_center_position(TopBlocks[1])
        TB2init = self.env.towerInitBO[TopBlocks[1]][0]
        TB3 = self.env.get_block_center_position(TopBlocks[2])
        TB3init = self.env.towerInitBO[TopBlocks[2]][0]
        Top_Dist = np.linalg.norm(np.subtract(TB1init,TB1))**2/wf + np.linalg.norm(np.subtract(TB2init,TB2))**2/wf + np.linalg.norm(np.subtract(TB3init,TB3))**2/wf
        if Top_Dist > 0.01:
            return True
        else:
            return False

    def pushed_out_check(self):
        PB = self.env.get_block_position(self.env.get_good_push_block()-2)[0][0] ############## NOT SURE
        if PB > 0:
            return True
        else:
            return False

    def done_check(self):
        if self.pushed_out_check() == True or self.knocked_over_check() == True:
            return True
        else:
            return False

    def get_state(self):
        return np.append(self.env.get_poker_position()[0], self.env.get_block_position(self.GB_ID))

    def poker_close_check(self):
        poker_pos = self.env.get_poker_position()[0]
        GB_ID = self.env.get_good_push_block()
        init_block_pos = self.env.towerInitBO[GB_ID][0]
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
        block_pushed_dist = old_block_pos[0] - new_block_pos[0]
        if block_pushed_dist > 0:
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
        ns = self.get_state()
        reward = self.calc_reward(old_poker_pos,old_block_pos)
        return ns, reward, done

if __name__ == "__main__":
    #This is equivalent to the test example, pokes a single block from tower

    pybulletPath = "D:/ECE 285 - Advances in Robot Manipulation/bullet3-master/data/"
    env = environment(pybulletPath)
    start_time = time.time()
    print(env.knocked_over_check())
    print(env.pushed_out_check())
    env.poker_close_check()
    print(env.get_state())
    for i in range(0,1000):
        ns, reward, done = env.step(0)
        print(ns, reward, done)

    for i in range(0,500):
        ns, reward, done = env.step(2)
        print(ns, reward, done)
        #time.sleep(.005);
        #print("Reward:{}".format(R));

    print(env.knocked_over_check())
    print(env.pushed_out_check())

    for i in range(0,1000):
        ns, reward, done = env.step(3)
        print(ns, reward, done)
        #time.sleep(.005);
        #print("Reward:{}".format(R));

    print(env.knocked_over_check())
    print(env.pushed_out_check())

    total_time = time.time() - start_time
    print('Elapsed time %f'%(total_time))

    #w,h,img = env.captureImage()
    #print(np.shape(np.reshape(np.array(img),(w,h))))
    #print(len(img))
    raw_input()
         
         
         
         