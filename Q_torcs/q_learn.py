from gym_torcs import TorcsEnv
import numpy as np
import random
import q
import os
import matplotlib.pyplot as plt

saverate = 3
dir_name = "Q_tables/"
q_filepath = dir_name + "Q_table"
#Make sure to change filepath name!

episodes = 5000
steps = 1000000

# element 0 - angle
# element 1:19 - range
# element 20 - track positions
# element 21 - speed X
# element 22 - speed Y

def filter_observations(ob):
	sensors = np.zeros(7)
	sensors[0] = ob.angle
	sensors[1] = ob.track[5]
	sensors[2] = ob.track[9]
	sensors[3] = ob.track[13]
	sensors[4] = ob.trackPos
	sensors[5] = ob.speedX
	sensors[6] = ob.speedY

	return sensors

def trainAgent():
	print("Loading TORCS environment")
	env = TorcsEnv(vision=False, throttle=True, gear_change=False)
	agent = q.Q_learn(q_filepath, 0.1)
	if not os.path.exists(dir_name):
		os.makedirs(dir_name)
	
	rewards = np.zeros(episodes)
	angles = np.zeros(episodes)
	distances = np.zeros(episodes)
	
	for ep in range(episodes):
		agent.epsilon = 0.1 * (1. - float(ep)/episodes)
		
		if np.mod(ep, 3) == 0:
			ob = env.reset(relaunch=True)
			print "Resetting TORCS environment"

		else: 
			ob = env.reset()
		
		distance = 0
		angle_variance = []
		
		reward = 0
		done = False
		action = np.zeros([3])
		action[1] = 1.
		last_state = filter_observations(ob)
		for i in range(steps):
			if done:
				break
				
			ob, r, done, info = env.step(action)
			reward += r
			state = filter_observations(ob)
			
			#Run Q-Learn forward pass
			action = agent.learn(last_state, action, state, r)
			angle_variance.append(state[0])
			
			last_state = state

		print "Episode:", ep, "Steps:", i, "Reward:", reward
		angle_variance = np.asarray(angle_variance)
		var = np.sum(np.square(angle_variance-np.mean(angle_variance)))/np.size(angle_variance)
		
		rewards[ep] = reward
		angles[ep] = var
		distances[ep] = ob.distFromStart

		if np.mod(ep, saverate) == 0:
			agent.save_Q()
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

if __name__ == "__main__":
	trainAgent()

