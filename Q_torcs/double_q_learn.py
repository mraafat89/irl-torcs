from gym_torcs import TorcsEnv
import numpy as np
import random
import double as double
import os, signal
import matplotlib.pyplot as plt
import time
saverate = 3
dir_name = "Q_tables/"
q1_filepath = dir_name + "Q1_table"
q2_filepath = dir_name + "Q2_table"
#Make sure to change filepath name!

steps = 1000000
LR = 0.225
RR = 0.35
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
	#print ob.speedX
	return sensors

def trainAgent():
	print("Loading TORCS environment")
	env = TorcsEnv(vision=False, throttle=True, gear_change=False)
	agent = double.Q_learn(q1_filepath, q2_filepath, LR, RR)
	if not os.path.exists(dir_name):
		os.makedirs(dir_name)
	if not os.path.exists("results"):
		os.makedirs("results")
	
	run = True
	ep = len(agent.rewards)
	while run:
		try:
			if np.mod(ep, 3) == 0:
				ob = env.reset(relaunch=True)
				print "Resetting TORCS environment"

			else: 
				ob = env.reset()
		
			#agent.alpha = max(0.225,LR * np.power(0.99,ep))
			agent.epsilon = max(0.2,RR * np.power(0.999,ep))
			reward = 0
			done = False
			angle_variance = []
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
				#start = time.time()
				action = agent.learn(last_state, action, state, r, done)
				#print time.time() - start
				angle_variance.append(state[0])
			
				last_state = state

			print "Episode:", ep, "Steps:", i, "Reward:", reward
			print "LR:", agent.alpha
			print "RR:", agent.epsilon
			angle_variance = np.asarray(angle_variance)
			var = np.sum(np.square(angle_variance-np.mean(angle_variance)))/np.size(angle_variance)
		
			agent.rewards.append(reward)
			agent.angles.append(var)
			agent.distances.append(ob.distFromStart)
			ep += 1
			
			if np.mod(ep, saverate) == 0:
				agent.save_Qs()
				print("Saved Q-table")
				
		except KeyboardInterrupt:
			print "Ending training"
			run = False
			env.end()
			
			fig = plt.figure()
			plt.plot(np.asarray(agent.rewards))
			plt.xlabel('Episodes')
			plt.ylabel('Rewards')
			fig.savefig('results/rewards.png')
	
			fig = plt.figure()
			plt.plot(np.asarray(agent.angles))
			plt.xlabel('Episodes')
			plt.ylabel('Angle-Variances')
			fig.savefig('results/angles.png')
	
			fig = plt.figure()
			plt.plot(np.asarray(agent.distances))
			plt.xlabel('Episodes')
			plt.ylabel('Distances')
			fig.savefig('results/distances.png')
	
			print "Figures saved"

if __name__ == "__main__":
	np.set_printoptions(precision=2)
	trainAgent()

