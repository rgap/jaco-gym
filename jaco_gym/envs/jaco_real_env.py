import gym
import numpy as np
import random

from gym import error, spaces, utils
from gym.utils import seeding
from jaco_gym.envs.ros_scripts.jaco_real_action_client import JacoRealActionClient



class JacoEnv(gym.Env):

    def __init__(self):

        self.robot = JacoRealActionClient()

    def min_max_conversion(self, a, OldMin, OldMax, NewMin, NewMax):
        OldRange = (OldMax - OldMin)  
        NewRange = (NewMax - NewMin)  
        return (((a - OldMin) * NewRange) / OldRange) + NewMin
        
    def step_degrees(self, action):
        
        # execute action
        for i in range(10):  ## ??
            # self.robot.cancel_move()
            # receiving degrees
            self.robot.move_arm(
                action[0],
                action[1],
                action[2],
                action[3],
                action[4],
                action[5]
            )
        
        # get state
        self.state = self.robot.read_angles()

        # calculate reward
        self.tip_position = self.print_tip_pos()
        # negative of the distance from the tip to the target
        self.reward = - np.linalg.norm(self.tip_position - self.target_vect)

        self.state = np.concatenate((self.state, self.tip_position, self.target_vect))
        self.done = False
        self.info = {"tip coordinates": self.tip_position, "target coordinates": self.target_vect}
        return self.state, self.reward, self.done, self.info


    def step(self, action):

        degrees = self.action2deg(action)
        
        # execute action
        for i in range(1):  ## ??
            # self.robot.cancel_move()
            # receiving degrees
            self.robot.move_arm(
                degrees[0],
                degrees[1],
                degrees[2],
                degrees[3],
                degrees[4],
                degrees[5]
            )
        
        # get state
        self.state = self.robot.read_angles()

        # calculate reward
        self.tip_position = self.print_tip_pos()
        # negative of the distance from the tip to the target
        self.reward = - np.linalg.norm(self.tip_position - self.target_vect)

        self.state = np.concatenate((self.state, self.tip_position, self.target_vect))
        self.done = False
        self.info = {"tip coordinates": self.tip_position, "target coordinates": self.target_vect}
        return self.state, self.reward, self.done, self.info

    def deg2action(self, degrees):
        action = np.zeros(6)
        action[0] = self.min_max_conversion(degrees[0], OldMin=130, OldMax=260, NewMin=-1, NewMax=1)
        action[1] = self.min_max_conversion(degrees[1], OldMin=220, OldMax=240, NewMin=-1, NewMax=1)
        action[2] = self.min_max_conversion(degrees[2], OldMin=100, OldMax=140, NewMin=-1, NewMax=1)
        action[3] = self.min_max_conversion(degrees[3], OldMin=200, OldMax=280, NewMin=-1, NewMax=1)
        action[4] = self.min_max_conversion(degrees[4], OldMin=30, OldMax=80, NewMin=-1, NewMax=1)
        action[5] = self.min_max_conversion(degrees[5], OldMin=70, OldMax=100, NewMin=-1, NewMax=1)

        return action

    def action2deg(self, action):
        degrees = np.zeros(6)
        degrees[0] = self.min_max_conversion(action[0], OldMin=-1, OldMax=1, NewMin=130, NewMax=260)
        degrees[1] = self.min_max_conversion(action[1], OldMin=-1, OldMax=1, NewMin=220, NewMax=240)
        degrees[2] = self.min_max_conversion(action[2], OldMin=-1, OldMax=1, NewMin=100, NewMax=140)
        degrees[3] = self.min_max_conversion(action[3], OldMin=-1, OldMax=1, NewMin=200, NewMax=280)
        degrees[4] = self.min_max_conversion(action[4], OldMin=-1, OldMax=1, NewMin=30, NewMax=80)
        degrees[5] = self.min_max_conversion(action[5], OldMin=-1, OldMax=1, NewMin=70, NewMax=100)

        return degrees


    def print_tip_pos(self):
        tip_position = self.robot.read_tip_position()
        return np.array(tip_position)


    def set_target_position(self):
        # fixed target
        x_target = 0.5
        y_target = 0.1
        z_target = 0.08
        self.target_vect = np.array([x_target, y_target, z_target])


    def reset(self): 

        # for i in range(1):
        # self.robot.cancel_move()
        # self.robot.move_arm(-90, 200, 90, -120, 50, 90)
        # self.robot.move_arm(0, 180, 180, 0, 0, 0)

        # self.robot.move_arm(130, 210, 90, -120, 50, 90)
        # pos = [4.658828549805452, 3.5021290051197393, 1.6177787494816385, 4.20840278831817, 0.9034956424628485, 1.5286604578179352]
        # pos = np.degrees(pos)
        for i in range(6):
            # self.robot.cancel_move()
            self.robot.move_arm(266.93121338, 200.6572113,   92.69189453, 241.12371826,  51.76648712, 87.58579254)
        # print("Jaco reset to initial position")

        # get state
        self.state = self.robot.read_angles()
        self.tip_position = self.print_tip_pos()

        self.set_target_position()
        self.state = np.concatenate((self.state, self.tip_position, self.target_vect))

        return self.state


    def render(self, mode='human', close=False):
        pass
