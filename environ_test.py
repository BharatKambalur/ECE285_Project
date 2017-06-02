from sim_environment import sim_environment
import numpy as np
import time
import matplotlib as plt

#env.reset_simulation();
class environment(object):

    poker_reached_close = False

    def __init__(self,pybulletPath):
        self.env = sim_environment(tW=3,tH=6,useGUI=False,pybulletPath=pybulletPath)
        self.env.set_poker_position([-1,0.0,1.47])

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
        if pushed_out_check(self.env) == True or knocked_over_check(self.env) == True:
            return True
        else:
            return False

    def get_state(self):
        delta = self.env.get_movement_delta()
        return (self.env.get_poker_position()/delta).astype(int)

    def poker_close_check(self):
        poker_pos = self.env.get_poker_position()[0]
        GB_ID = self.env.get_good_push_block()
        init_block_pos = self.env.towerInitBO[GB_ID][0]
        print(poker_pos)
        print(init_block_pos)
        dist = np.linalg.norm(poker_pos-init_block_pos)
        print(dist)
        if dist < 0.01:
            return True
        else:
            return False

    def calc_reward(self, old_poker_pos, old_block_pos):
        if knocked_over_check(self.env) == True:
            return 0
        if self.
        new_block_pos = self.env.get_block_position(self.env.get_good_push_block())
        block_pushed_dist = old_block_pos[0][0] - new_block_pos[0][0]
        if block_pushed_dist
        new_poker_pos = self.env.get_poker_position()

        old_dist = np.linalg.norm(poker_pos-init_block_pos)


if __name__ == "__main__":
    #This is equivalent to the test example, pokes a single block from tower
    start_time = time.time()
    pybulletPath = "D:/ECE 285 - Advances in Robot Manipulation/bullet3-master/data/"
    env = environment(pybulletPath)
    print(knocked_over_check())
    print(pushed_out_check())
    poker_close_check()
    for i in range(0,1000):
        env.move_poker_px()
        #time.sleep(.005);
        #poker_close_check(env)
        R = reward(env)
        #print(get_state(env))
        #print("Reward:{}".format(R));

    print(knocked_over_check(env))
    print(pushed_out_check(env))

    for i in range(0,500):
        env.move_poker_py()
        #time.sleep(.005);
        R = reward(env)
        #print("Reward:{}".format(R));

    print(knocked_over_check(env))
    print(pushed_out_check(env))

    for i in range(0,1000):
        env.move_poker_ny()
        #time.sleep(.005);
        R = reward(env)
        #print("Reward:{}".format(R));

    print(knocked_over_check(env))
    print(pushed_out_check(env))

    total_time = time.time() - start_time
    print('Elapsed time %f'%(total_time))

    #w,h,img = env.captureImage()
    #print(np.shape(np.reshape(np.array(img),(w,h))))
    #print(len(img))
    raw_input()
         
         
         
         