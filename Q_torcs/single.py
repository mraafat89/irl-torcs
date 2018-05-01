import numpy as np
import os

class Q_learn:

    def __init__(self,filepath_Q,alpha,epsilon):
        #self.S = []
        print "Using single.py"
        np.set_printoptions(precision=1)
        np.set_printoptions(floatmode='fixed')
        self.filepath_Q = filepath_Q

        self.alpha = alpha
        self.gamma = 0.99
        self.epsilon = epsilon

        #steering, accel, brake
        self.A = [(0.5,1.,0.),
                  (0.1,1.,0.),
                  (-0.5,1.,0.),
                  (-0.1,1.,0.),
                  (0.,0.,0.),
                  (0.,1.,0.),
                  (0.,0.,1.)
                  ]
        self.A_lookup = {}
        self.A_back_lookup = {}
        idx = 0
        for i0 in range(len(self.A)):
            self.A_lookup[self.A[i0]] = idx
            self.A_back_lookup[idx] = self.A[i0]
            idx += 1
                      
        self.num_actions = len(self.A)
        self.num_states = 14*6*6*9*9*10
        self._initialize_Q()


    def _initialize_Q(self):
        if os.path.isfile(self.filepath_Q+".npy"):
            self.Q = np.load(self.filepath_Q+".npy")
            print "Loaded Q table"
        else:
            self.Q = np.zeros((self.num_states,self.num_actions))

        if os.path.isfile("results/singleQ_rewards.npy"):
            self.rewards = np.load("results/singleQ_rewards.npy").tolist()
            self.distances = np.load("results/singleQ_distances.npy").tolist()
            self.angles = np.load("results/singleQ_angles.npy").tolist()
            print "Loaded existing results"
        else:
			self.rewards = []
			self.distances = []
			self.angles = []

    def save_Q(self):
        np.save(self.filepath_Q,self.Q)
        np.save("results/singleQ_rewards",self.rewards)
        np.save("results/singleQ_distances",self.distances)
        np.save("results/singleQ_angles",self.angles)
	        
    # def doubleQ(initial_Q1, initial_Q2, initial_state, transition,
    #             num_episodes, gamma, alpha, epsilon=0.1):
    #     # This function implements double Q-learning. It returns Q1, Q2 and their sum Q
    #
    #
    #     num_states, num_actions = np.shape(initial_Q1)
    #     Q1 = initial_Q1
    #     Q2 = initial_Q2
    #     for ep in range(num_episodes):
    #         s = initial_state
    #         terminal = False
    #         while not terminal:
    #             if np.random.rand() > epsilon:
    #                 a = np.argmax((Q1 + Q2)[s, :])
    #             else:
    #                 a = np.random.randint(num_actions)
    #             sp, r, terminal = transition(s, a)
    #             if np.random.rand() < 0.5:
    #                 Q1[s, a] = Q1[s, a] + alpha * (r + gamma * Q2[sp, np.argmax(Q1[sp, :])] - Q1[s, a])
    #             else:
    #                 Q2[s, a] = Q2[s, a] + alpha * (r + gamma * Q1[sp, np.argmax(Q2[sp, :])] - Q2[s, a])
    #
    #             s = sp
    #     Q = Q1 + Q2
    #     return Q1, Q2, Q  # ,  steps#, rewards



    def learn(self,inputs,action,inputsprime,reward,done):
        #angle, track 1x19, track_positon, speedx, speedy
        s = self.get_state(inputs)
        sp = self.get_state(inputsprime)
        a = self.get_action(action)
        r = reward
        np.set_printoptions(precision=1)
        np.set_printoptions(floatmode='fixed')
        
        if s != sp or done:	
            self.Q[s, a] = self.Q[s, a] + self.alpha * (r + self.gamma * self.Q[sp, np.argmax(self.Q[sp, :])] - self.Q[s, a])

            if np.random.rand() > self.epsilon:
                print s, r, self.Q[s, :]
                ap = np.argmax(self.Q[sp, :])

            else:
                print s, r, self.Q[s, :], ": RANDOM"
                ap = np.random.randint(self.num_actions)
                
        else:
            ap = a
             
        return self.get_car_inputs(ap)


    def get_car_inputs(self,a):
        return self.A_back_lookup[a]

    def get_action(self,action):
        return self.A_lookup[tuple(action)]


    def get_state(self,inputs):
        angle = self.disc_angle(inputs[0])  #14
        ray0 = self.disc_track(inputs[1])	#6
        #ray1 = self.disc_track(inputs[2])
        ray2 = self.disc_track(inputs[3])   #6
        pos = self.disc_trackPos(inputs[4]) #9
        sx = self.disc_speedx(inputs[5])    #9
        sy = self.disc_speedy(inputs[6])    #10

        return angle*(6*6*9*9*10) + ray0*(6*9*9*10) + ray2*(9*9*10) + pos*(9*10) + sx*(10) + sy

    def disc_angle(self,x):
        if   x < -1.0                  : y = 0
        elif x >= -1.0  and x < -0.5   : y = 1
        elif x >= -0.5  and x < -0.2   : y = 2
        elif x >= -0.2  and x < -0.1   : y = 3
        elif x >= -0.1  and x < -0.04  : y = 4
        elif x >= -0.04 and x < -0.02  : y = 5
        elif x >= -0.02 and x < 0.0    : y = 6
        elif x >= 0.0   and x < 0.02   : y = 7
        elif x >= 0.02  and x < 0.04   : y = 8
        elif x >= 0.04  and x < 0.1    : y = 9
        elif x >= 0.1   and x < 0.2    : y = 10
        elif x >= 0.2   and x < 0.5    : y = 11
        elif x >= 0.5   and x < 1.0    : y = 12
        else                           : y = 13

        return y

    def disc_trackPos(self,x):
        if   x < -0.3                  : y = 0
        elif x >= -0.3 and x < -0.15   : y = 1
        elif x >= -0.15 and x < -0.1   : y = 2
        elif x >= -0.1 and x < -0.05   : y = 3
        elif x >= -0.05 and x < 0.05   : y = 4
        elif x >= 0.05 and x < 0.1     : y = 5
        elif x >= 0.1 and x < 0.15     : y = 6
        elif x >= 0.15 and x < 0.3     : y = 7
        else                           : y = 8

        return y

    def disc_track(self,x):
        if   x < 0.03:                   y = 0
        elif x >= 0.03 and x < 0.06:     y = 1
        elif x >= 0.06 and x < 0.1:      y = 2
        elif x >= 0.1 and x < 0.15:      y = 3
        elif x >= 0.15 and x < 0.25:     y = 4
        else:                            y = 5
	    
        return y

    def disc_speedx(self,x):
        if   x < 0.0:                    y = 0
        elif x >= 0.0   and x < 0.03 :   y = 1
        elif x >= 0.03  and x < 0.06 :   y = 2
        elif x >= 0.06  and x < 0.1 :    y = 3
        elif x >= 0.1  and x < 0.13 :    y = 4
        elif x >= 0.13  and x < 0.16:    y = 5
        elif x >= 0.16 and x < 0.2:      y = 6
        elif x >= 0.2 and x < 0.25:      y = 7
        else:                            y = 8

        return y 
        
    def disc_speedy(self,x):
        if   x < -0.5:                   y = 0
        elif x >= -0.5  and x < -0.1:    y = 1
        elif x >= -0.1  and x < -0.05:   y = 2
        elif x >= -0.05 and x < -0.01:   y = 3
        elif x >= -0.01 and x < 0.:      y = 4
        elif x >= 0.    and x < 0.01:    y = 5
        elif x >= 0.01  and x < 0.05:    y = 6
        elif x >= 0.05  and x < 0.1:     y = 7
        elif x >= 0.1   and x < 0.5:     y = 8
        else:                            y = 9
        
        return y


