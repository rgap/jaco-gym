import gym
import numpy as np
import random
from gym import error, spaces, utils
from gym.utils import seeding
from kinova_client import KinovaClient

class JacoEnv(gym.Env):
    metadata = {'render.modes': ['human']}


    def __init__(self):
        self.kinova_client = KinovaClient()
        
    def step(self, action):
        
        # execute action
        self.kinova_client.cancel_move()
        self.kinova_client.move_arm(
            action[0], 
            action[1], 
            action[2], 
            action[3], 
            action[4],
            action[5] 
        )
        
        # get state
        self.state = self.kinova_client.read_angles()

        # calculate reward
        self.tip_position = self.print_tip_pos()

        self.reward = np.linalg.norm(self.tip_position - self.target_vect)

        print("tip position: ", self.tip_position)
        print("target vect: ", self.target_vect)
        print("reward: ", self.reward)

        return self.state, self.reward

    def print_tip_pos(self):
        tip_position = self.kinova_client.read_tip_position()
        return np.array(tip_position)

    def reset(self): 

        for i in range(6):
            self.kinova_client.cancel_move()
            self.kinova_client.move_arm(0, 180, 180, 0, 0, 0)

        print("Jaco reset to initial position")

        # get state
        self.state = self.kinova_client.read_angles()

        # generate random coordinates of a point in space
        x_target = random.uniform(-0.49, 0.49)
        y_target = random.uniform(-0.49, 0.49)
        z_target = random.uniform(0.69, 1.18)
        
        self.target_vect = np.array([x_target, y_target, z_target])

        return self.state

    def render(self, mode='human', close=False):
        pass
