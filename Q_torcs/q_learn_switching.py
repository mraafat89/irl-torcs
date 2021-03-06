from gym_torcs import TorcsEnv
import numpy as np
import random
import single
import os, signal
import matplotlib.pyplot as plt
import time
import argparse
from termcolor import colored

saverate = 3
switchrate = 100
dir_name = "Q_tables/"
q_filepath = dir_name + "Q_table"
#Make sure to change filepath name!

steps = 1000000
LR = 0.225
RR = 0.25
# element 0 - angle
# element 1:19 - range
# element 20 - track positions
# element 21 - speed X
# element 22 - speed Y
def parse_args():
	parser = argparse.ArgumentParser("Reinforcement Learning experiment for TORCS Driving Simulator")
	parser.add_argument("--display", action="store_true", default=False)
	return parser.parse_args()

def filter_observations(ob):
	sensors = np.zeros(7)
	sensors[0] = ob.angle
	sensors[1] = ob.track[5]
	sensors[2] = ob.track[9]
	sensors[3] = ob.track[13]
	sensors[4] = ob.trackPos
	sensors[5] = ob.speedX
	sensors[6] = ob.speedY
	#print 'sensor data', sensors
	return sensors

def trainAgent(arglist):
	print("Loading TORCS environment")
	env = TorcsEnv(vision=False, throttle=True, gear_change=False, display=arglist.display)
	agent = single.Q_learn(q_filepath, LR, RR)
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
				
			#agent.alpha = max(0.225,LR*np.power(0.99,ep))
			agent.epsilon = max(0.1,RR*np.power(0.999,ep))
			agent.r = 0.0
			done = False
			angle_variance = []
			action = np.zeros([3])
			action[1] = 1.
			last_state = filter_observations(ob)
			dp = 0.
			d = 0.
			for i in range(steps):
				if done:
					break
				
				ob, r, done, info = env.step(action)
				
				dn = ob.distFromStart
				if np.abs(dn-dp) < 100.: d += dn-dp
				
				state = filter_observations(ob)

				#Run Q-Learn forward pass
				#start = time.time()
				action = agent.learn(last_state, action, state, r, done)

				#print time.time() - start
				angle_variance.append(state[0])
			
				last_state = state
				dp = dn
				
			print "LR:", agent.alpha
			print "RR:", agent.epsilon
			print colored("Episode: "+ str(ep) + " Steps: "+ str(i) + (" Reward: %.02f" % agent.r), 'blue')
			print colored(("Distance: %.02f" % d) + (" Time: %.02f" % ob.curLapTime), 'blue')
			
			angle_variance = np.asarray(angle_variance)
			var = np.sum(np.square(angle_variance-np.mean(angle_variance)))/np.size(angle_variance)
		
			agent.rewards.append(agent.r)
			agent.angles.append(var)
			agent.distances.append(d)
			agent.times.append(ob.curLapTime)
			ep += 1
			
			if np.mod(ep, saverate) == 0:
				agent.save_Q()
				print("Saved Q-table")
			
			if np.mod(ep, switchrate) == 0:
				print "Switching TORCS Tracks..."
				os.system('pkill torcs')
				time.sleep(0.5)
				os.system('torcs &')
				time.sleep(0.5)
				if np.mod(ep, 3*switchrate) != 0:
					os.system('sh switchtrack.sh')
				else:
					os.system('sh switchtrackback.sh')
				time.sleep(0.5)
				ob = env.reset(relaunch=True)
				
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
	
			fig = plt.figure()
			plt.plot(np.asarray(agent.times))
			plt.xlabel('Episodes')
			plt.ylabel('Lap Times')
			fig.savefig('results/times.png')
			print "Figures saved"

if __name__ == "__main__":
	np.set_printoptions(precision=2)
	arglist = parse_args()
	trainAgent(arglist)

