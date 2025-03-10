import gym
import jaco_gym
import rospy
import numpy as np

from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import PPO2


# first launch Jaco in Gazebo with 
# roslaunch kinova_gazebo robot_launch_noRender.launch kinova_robotType:=j2n6s300
# roslaunch kinova_gazebo robot_launch_render.launch kinova_robotType:=j2n6s300

rospy.init_node("kinova_client", anonymous=True, log_level=rospy.INFO)

env_id = 'JacoGazebo-v1'
# log_dir = "../results/10000steps/"+env_id+"/"+env_id+".zip"
log_dir = "/home/rgap/LIBRARIES/jaco-gym/results/10000steps/JacoGazebo-v1/JacoGazebo-v1.zip"
env = gym.make(env_id)
env = DummyVecEnv([lambda: env])
# env.reset()

model = PPO2.load(log_dir)

# Enjoy trained agent
obs = env.reset()
while True:
    # action, _states = model.predict(obs)
    action = np.array([[0.5, 0.5, 0, 0, 0, 0]])
    obs, rewards, dones, info = env.step(action)


env.close()