import gym
import jaco_gym  # necessary import
import rospy

rospy.init_node("kinova_client", anonymous=True, log_level=rospy.INFO)
env = gym.make('JacoReal-v0')

### RESET THE ARM TO ITS DEFAULT HOME POSITION

# [266.93121338, 200.6572113,   92.69189453, 241.12371826,  51.76648712, 87.58579254]
state = env.reset()   
print("initial joint angles:::", state[:6])
print("initial tip position:::", state[6:9])
print("target position:::", state[9:])  # also called with env.env.print_tip_pos()

### PERFORM AN ACTION

degrees = [185.835, 240, 100, 280, 74.0525, 90.6265]

# Convert radians to [-1, 1]
action = env.env.deg2action(degrees)

# Send action to the actuators
state, reward, done, info = env.step(action)

print('\n\n')
print("final joint angles:::", state[:6])
print("final tip position:::", state[6:9])
print("target position:::", state[9:])

env.close()
