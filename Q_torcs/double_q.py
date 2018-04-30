import numpy as np
import os

class Q_learn:

    def __init__(self,filepath_Q1,filepath_Q2,epsilon):
        #self.S = []

        self.filepath_Q1 = filepath_Q1
        self.filepath_Q2 = filepath_Q2

        self.alpha = 0.1
        self.gamma = 0.99
        self.epsilon = epsilon

        self.ranges = [np.linspace(-np.pi,np.pi,5),                 #angles
                       np.array([-50.,10.,50.,300.]),                            #ray1
                       np.array([-50.,10.,50.,300.]),                            #ray2
                       np.array([-50., 25., 50.,300.]),                          #ray3
                       np.array([-1e6,-5.,-2.5,0.,2.5,5.,1e6]),       #track_position
                       np.hstack((-1e6,np.linspace(-300,300,12),1e6)), #speedx
                       np.hstack((-1e6, np.linspace(-50, 50,12), 1e6)), #speedy
                       ]
        self.S_lookup = {}
        self.S_back_lookup = {}
        idx = 0
        for i0 in range(len(self.ranges[0])):
            for i1 in range(len(self.ranges[1])):
                for i2 in range(len(self.ranges[2])):
                    for i3 in range(len(self.ranges[3])):
                        for i4 in range(len(self.ranges[4])):
                            for i5 in range(len(self.ranges[5])):
                                for i6 in range(len(self.ranges[6])):
                                    self.S_lookup[(i0,i1,i2,i3,i4,i5,i6)] = idx
                                    self.S_back_lookup[idx] = [i0,i1,i2,i3,i4,i5,i6]
                                    idx += 1
        self.num_states = idx+1

                   #steering, accel, brake
        self.A = [(-0.5,1.,0.),
                  (0.5,1.,0.),
                  (-0.1,1.,0.),
                  (0.1,1.,0.),
                  (0.,0.,0.),
                  (0.,1.,0.),
                  (0.,0.,1.)
                  ]
        self.num_actions = len(self.A)

        self.A_lookup = {}
        self.A_back_lookup = {}
        idx = 0
        for i0 in range(len(self.A)):
            self.A_lookup[self.A[i0]] = idx
            self.A_back_lookup[idx] = self.A[i0]
            idx += 1

        self._initialize_Qs()


    def _initialize_Qs(self):
        if os.path.isfile(self.filepath_Q1+".npy"):
            self.Q1 = np.load(self.filepath_Q1+".npy")
            print "Loaded Q1"
        else:
            self.Q1 = np.zeros((self.num_states,self.num_actions))

        if os.path.isfile(self.filepath_Q2+".npy"):
            self.Q2 = np.load(self.filepath_Q2+".npy")
            print "Loaded Q2"
        else:
            self.Q2 = np.zeros((self.num_states,self.num_actions))
		
        if os.path.isfile("results/doubleQ_rewards.npy"):
            self.rewards = np.load("results/doubleQ_rewards.npy").tolist()
            self.distances = np.load("results/doubleQ_distances.npy").tolist()
            self.angles = np.load("results/doubleQ_angles.npy").tolist()
            print "Loaded existing results"
        else:
			self.rewards = []
			self.distances = []
			self.angles = []

    def save_Qs(self):
        np.save(self.filepath_Q1,self.Q1)
        np.save(self.filepath_Q2,self.Q2)
        np.save("results/doubleQ_rewards",self.rewards)
        np.save("results/doubleQ_distances",self.distances)
        np.save("results/doubleQ_angles",self.angles)


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



    def learn(self,inputs,action,inputsprime,reward):
        #angle, track 1x19, track_positon, speedx,speedy
        s = self.get_state(inputs) #s is a number between 0 and 57k or something
        sp = self.get_state(inputsprime)
        a = self.get_action(action)
        r = reward
        
        if np.random.rand() < 0.5:
            self.Q1[s, a] = self.Q1[s, a] + self.alpha * (r + self.gamma * self.Q2[sp, np.argmax(self.Q1[sp, :])] - self.Q1[s, a])
        else:
            self.Q2[s, a] = self.Q2[s, a] + self.alpha * (r + self.gamma * self.Q1[sp, np.argmax(self.Q2[sp, :])] - self.Q2[s, a])


        if np.random.rand() > self.epsilon:
            ap = np.argmax((self.Q1 + self.Q2)[sp, :])

        else:
            ap = np.random.randint(self.num_actions)

        return self.get_car_inputs(ap)


    def get_car_inputs(self,a):
        return self.A_back_lookup[a]

    def get_action(self,action):
        return self.A_lookup[tuple(action)]


    def get_state(self,inputs):
      for i1 in range(len(inputs)):
        if inputs[i1] < self.ranges[i1][0]:
          inputs[i1] = self.ranges[i1][0]+1e-6

        if inputs[i1] > self.ranges[i1][-1]:
          inputs[i1] = self.ranges[i1][-1]-1e-6

        """inputs = []
        for i1 in range(len(oinputs)):
            if i1 >= 1 and i1 <= 21:
                if np.mod(i1-1,6) == 0:
                    inputs.append(np.mean(oinputs[i1-6:i1+6]))
            else:
                inputs.append(oinputs[i1])

        print(inputs)"""

        bins = []
        for i1 in range(len(inputs)):
            #print(inputs[i1], self.ranges[i1])
            bins.append(np.max(np.where(inputs[i1]>self.ranges[i1])))

        return self.S_lookup[tuple(bins)]
