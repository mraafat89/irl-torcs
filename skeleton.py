#Imports
import utils

"""
Initialize models (optionally load weights)
	- Actor network: Takes in state and outputs throttle, brake, steering angle, equivalent to
		the policy function (pi)
	- Critic network: Takes in rewards from the environment and outputs value to each state,
	  equivalent to the value function
	- Target network: A copy network of Actor and Critic that stabilizes learning
	- Reward network: Takes in environmental state and returns the reward of that state (from IRL)
"""


rl_epochs = 20
expert_epochs = 5
expert_length = 100

run_expert = True
train_reward = True
train_actor_critic = True

reward_weights_path = ""
actor_weights_path = ""
critic_weights_path = ""

reward_network = Reward()	#TODO: Define these networks!
actor_network = Actor()		
critic_network = Critic()


# Begin IRL Training (Optionally load weights)
if train_reward:
	# Begin Expert data collection (Optionally load previously collected data)
	if run_expert:
		for e in in range(expert_epochs):
			states, actions = collect_episode(expert_length)
			save_episode(episode_name, states, actions)			# save with filenames according to date/time

	# Load previously collected expert data
	expert_episodes = load_expert_data(expert_data_path)

	for e in expert_episodes:
		states, actions = get_episode(e)	
		# Collect sample from expert driver
		# angle, track, trackPos, speedX, speedY, speedZ, wheelSpinVel, rpm

		# Train Reward network with IRL

	# Save reward network weights
	save_weights(reward_network, reward_weights_path)		# From utils folder

else:
	load_reward_weights(reward_network, reward_weights_path)


# Begin Actor and Critic Training
if train_actor_critic:

# For some number of epochs
	# Run an episode and save to replay buffer
	
	# For each element of the buffer
		# Get loss from critic network for state-actions, and rewards from IRL
		# re-predict action from actor model
		# calculate gradients from critic
		# train the actor network 
		# train the target network
		# train the critic

# Save actor, critic, and target weights
	save_weights(actor_network, actor_weights_path) 	# From utils folder

else:
	load_actor_weights(actor_network, actor_weights_path)


# Run Testing Demonstration

# Load weights into actor target network

# for some amount of time
	# Get new sensor data
	# Predict inputs
	# Send inputs to TORCS
