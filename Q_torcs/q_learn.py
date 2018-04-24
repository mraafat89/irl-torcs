from gym_torcs import TorcsEnv
import numpy as np
import random
import project
import os

saverate = 3
dir_name = "Q_tables/"
q1_filepath = dir_name + "Q1_table"
q2_filepath = dir_name + "Q2_table"
#Make sure to change filepath name!

episodes = 100
steps = 10000

# element 0 - angle
# element 1:19 - range
# element 20 - track positions
# element 21 - speed X
# element 22 - speed Y

def filter_observations(ob):
	sensors = np.zeros(7)
	sensors[0] = ob.angle
	sensors[1] = ob.track[4]
	sensors[2] = ob.track[9]
	sensors[3] = ob.track[14]
	sensors[4] = ob.trackPos
	sensors[5] = ob.speedX
	sensors[6] = ob.speedY

	return sensors

def trainAgent():
	print("Loading TORCS environment")
	env = TorcsEnv(vision=False, throttle=True, gear_change=False)
	agent = project.Q_learn(q1_filepath, q2_filepath, 0.1)

	#define/load Q-TABLE here
	#if os.path.isfile(filepath):
	#	Q = np.load(filepath)

	#else:
	#	Q = np.array([num_states, num_actions])

	for ep in range(episodes):
		if np.mod(ep, 3) == 0:
			ob = env.reset(relaunch=True)
			print("Reseting TORCS environment")

		else: 
			ob = env.reset()

		action = np.zeros([3])
		last_state = filter_observations(ob)
		reward = 0
		for i in range(steps):
			ob, r, done, info = env.step(action)
			reward += r
			state = filter_observations(ob)

			#Run Q-Learn forward pass
			action = agent.learn(last_state, action, state, r)

			last_state = state
			print "Episode:", ep, "Step:", i, "Reward:", reward

			if done:
				break
		
		print("Episode reward:", reward)
		if np.mod(ep, saverate) == 0:
			agent.save_Qs()
			print("Saved Q-table")

	env.end()
	print("Experiment ended")


def playTORCS():
	print("Loading TORCS environment")
	env = TorcsEnv(vision=False, throttle=True, gear_change=False)
	#load Q-TABLE here
	Q = np.load(filepath)
	ob = env.reset()

	while(True):
		#run Q-Learn forward pass
		action = np.zeros([1,3])
		ob, r, done, info = env.step(action)

		if done:
			break

	env.end()
	print("Demo Ended")


if __name__ == "__main__":
	trainAgent()
	#playTORCS()

