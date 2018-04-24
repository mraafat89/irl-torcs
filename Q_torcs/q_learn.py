from gym_torcs import TorcsEnv
import numpy as np
import random
import project
import os
import matplotlib.pyplot as plt

saverate = 5
dir_name = "Q_tables/"
q1_filepath = dir_name + "Q1_table"
q2_filepath = dir_name + "Q2_table"
#Make sure to change filepath name!

episodes = 3
steps = 100000

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
	if not os.path.exists(dir_name):
		os.makedirs(dir_name)
	
	rewards = np.zeros(episodes)
	angles = np.zeros(episodes)
	distances = np.zeros(episodes)
	
	for ep in range(episodes):
		if np.mod(ep, 3) == 0:
			ob = env.reset(relaunch=True)
			print("Reseting TORCS environment")

		else: 
			ob = env.reset()
		
		distance = 0
		angle_variance = []
		
		reward = 0
		action = np.zeros([3])
		last_state = filter_observations(ob)
		for i in range(steps):
			ob, r, done, info = env.step(action)
			reward += r
			state = filter_observations(ob)
			
			#Run Q-Learn forward pass
			action = agent.learn(last_state, action, state, r)
			angle_variance.append(state[0])
			distance = state[4]
			
			last_state = state
			print "Episode:", ep, "Step:", i, "Reward:", reward

			if done:
				break
		
		angle_variance = np.asarray(angle_variance)
		var = np.sum(np.square(angle_variance-np.mean(angle_variance)))/np.size(angle_variance)
		
		rewards[ep] = reward
		angles[ep] = var
		distances[ep] = distance
		
		print("Episode reward:", reward)
		if np.mod(ep, saverate) == 0:
			agent.save_Qs()
			print("Saved Q-table")

	env.end()
	print("Experiment ended")
	
	fig = plt.figure()
	plt.plot(rewards)
	plt.xlabel('Episodes')
	plt.ylabel('Rewards')
	fig.savefig('rewards.png')
	
	fig = plt.figure()
	plt.plot(angles)
	plt.xlabel('Episodes')
	plt.ylabel('Angle-Variances')
	fig.savefig('angles.png')
	
	fig = plt.figure()
	plt.plot(distances)
	plt.xlabel('Episodes')
	plt.ylabel('Distances')
	fig.savefig('distances.png')
	
	print("Figures saved")

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

