import gym
import numpy as np
import random

from gym import error, spaces, utils
from gym.utils import seeding
from jaco_gym.envs.ros_scripts.jaco_gazebo_action_client import JacoGazeboActionClient



class JacoEnv(gym.Env):

    def __init__(self):

        self.robot = JacoGazeboActionClient()

        self.action_dim = 6
        # self.obs_dim = 36
        self.obs_dim = 12   # when using read_state_simple

        high = np.ones([self.action_dim])
        self.action_space = gym.spaces.Box(-high, high)
        
        high = np.inf * np.ones([self.obs_dim])
        self.observation_space = gym.spaces.Box(-high, high)

    def min_max_conversion(self, a, OldMin, OldMax, NewMin, NewMax):
        OldRange = (OldMax - OldMin)  
        NewRange = (NewMax - NewMin)  
        return (((a - OldMin) * NewRange) / OldRange) + NewMin

    def set_state(self, state):
        ## setting a state is the same as running an action
        ## only moving the joint angles for now 
        self.step(self.deg2action(state[:6]))

    def step_degrees(self, action):

        # convert to radians    
        self.action = np.radians(action)

        # move arm 
        self.robot.move_arm(self.action)
       
        # get state
        self.state = self.robot.read_state_simple()   # only return 12 values instead of 36
        # observation considering tip coord and target coord
        self.tip_coord = self.robot.get_tip_coord()

        # calculate reward
        self.dist_to_target = np.linalg.norm(self.tip_coord - self.target_vect)
        self.reward = - self.dist_to_target 

        self.state = np.concatenate((self.state[:6], self.tip_coord, self.target_vect))
        self.done = False
        self.info = {"tip coordinates": self.tip_coord, "target coordinates": self.target_vect}

        return self.state, self.reward, self.done, self.info

    def step(self, action):

        degrees = self.action2deg(action)

        # convert to radians    
        radians = np.radians(degrees)

        # move arm 
        self.robot.move_arm(radians)
       
        # get state
        self.state = self.robot.read_state_simple()   # only return 12 values instead of 36
        # observation considering tip coord and target coord
        self.tip_coord = self.robot.get_tip_coord()

        # calculate reward
        self.dist_to_target = np.linalg.norm(self.tip_coord - self.target_vect)
        self.reward = - self.dist_to_target 

        self.state = np.concatenate((self.state[:6], self.tip_coord, self.target_vect))
        self.done = False
        self.info = {"tip coordinates": self.tip_coord, "target coordinates": self.target_vect}

        return self.state, self.reward, self.done, self.info


    def deg2action(self, degrees):
        action = np.zeros(6)
        action[0] = self.min_max_conversion(degrees[0], OldMin=0, OldMax=360, NewMin=-1, NewMax=1)
        action[1] = self.min_max_conversion(degrees[1], OldMin=200, OldMax=210, NewMin=-1, NewMax=1)
        action[2] = self.min_max_conversion(degrees[2], OldMin=90, OldMax=270, NewMin=-1, NewMax=1)
        action[3] = self.min_max_conversion(degrees[3], OldMin=0, OldMax=360, NewMin=-1, NewMax=1)
        action[4] = self.min_max_conversion(degrees[4], OldMin=0, OldMax=360, NewMin=-1, NewMax=1)
        action[5] = self.min_max_conversion(degrees[5], OldMin=0, OldMax=360, NewMin=-1, NewMax=1)

        return action

    def action2deg(self, action):
        degrees = np.zeros(6)
        degrees[0] = self.min_max_conversion(action[0], OldMin=-1, OldMax=1, NewMin=0, NewMax=360)
        degrees[1] = self.min_max_conversion(action[1], OldMin=-1, OldMax=1, NewMin=200, NewMax=210)
        degrees[2] = self.min_max_conversion(action[2], OldMin=-1, OldMax=1, NewMin=90, NewMax=270)
        degrees[3] = self.min_max_conversion(action[3], OldMin=-1, OldMax=1, NewMin=0, NewMax=360)
        degrees[4] = self.min_max_conversion(action[4], OldMin=-1, OldMax=1, NewMin=0, NewMax=360)
        degrees[5] = self.min_max_conversion(action[5], OldMin=-1, OldMax=1, NewMin=0, NewMax=360)

        return degrees

    def set_target_position(self):
        # fixed target
        x_target = 0.575581073761
        y_target = 0.0795939415693
        z_target = 0.0828775539994

        self.target_vect = np.array([x_target, y_target, z_target])

    def reset(self): 

        # self.robot.cancel_move()

        pos = [266.93121338, 200.6572113,   92.69189453, 241.12371826,  51.76648712, 87.58579254]
        pos = np.radians(pos)
        self.robot.move_arm(pos)
        print("Jaco reset to initial position")

        # get state
        self.state = self.robot.read_state_simple()
        self.tip_coord = self.robot.get_tip_coord()
        self.set_target_position()
        self.state = np.concatenate((self.state[:6], self.tip_coord, self.target_vect))

        # if testing: graphically move the sphere target, if training, comment this line
        self.robot.move_sphere(self.target_vect)

        return self.state


    def render(self, mode='human', close=False):
        pass
