#!/usr/bin/env python3
"""
    script to test MCL
    from solution to sebastian Thruns airobotics
"""


from math import *
import random

import matplotlib.pyplot as plt
import numpy as np


landmarks  = [[25.0, 30.0], [90.0, 80.0], [10.0, 80.0], [80.0, 10.0]]
world_size = 100.0

class robot:
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0;
        self.turn_noise    = 0.0;
        self.sense_noise   = 0.0;
    
    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError
        if new_y < 0 or new_y >= world_size:
            raise ValueError
        if new_orientation < 0 or new_orientation >= 2 * pi:
            raise ValueError
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation)
    
    
    def set_noise(self, new_f_noise, new_t_noise, new_s_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise);
        self.turn_noise    = float(new_t_noise);
        self.sense_noise   = float(new_s_noise);
    
    
    def sense(self):
        Z = []
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            dist += random.gauss(0.0, self.sense_noise)
            Z.append(dist)
        return Z
    
    
    def move(self, turn, forward):
        if forward < 0:
            raise ValueError
        
        # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= 2 * pi
        
        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)
        x %= world_size    # cyclic truncate
        y %= world_size
        
        # set particle
        res = robot()
        res.set(x, y, orientation)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
        return res
    
    def Gaussian(self, mu, sigma, x):
        
        # calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
        return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))
    
    
    def measurement_prob(self, measurement):
        
        # calculates how likely a measurement should be
        
        prob = 1.0;
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0]) ** 2 + (self.y - landmarks[i][1]) ** 2)
            prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
        return prob
      
    def __repr__(self):
        # return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), str(self.orientation))
       return [self.x, self.y, self.orientation]


myrobot = robot()
p_pred = []
path_actual = []
p_act = []
kl = 0
plt.ion()
fig, ax = plt.subplots(1, 1)
# ax.set_autoscaley_on(True)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_xlim([0, 100])
ax.set_ylim([0, 100])

ax.grid()
ax.legend()
line_particles = []
line_path = []
line_path, = ax.plot([], [], 'ro', label="waypoint", ms=15, alpha=0.8)
line_particles, = ax.plot([], [], 'g.', label="particles", ms=5)
MAP_L = np.array(landmarks, dtype=np.float)
ax.scatter(MAP_L[:, 0], MAP_L[:, 1], c='k', marker='*', label="landmarks")

for _ in range(300):
    myrobot = myrobot.move(0.02, 5.0)
    path_actual.append(myrobot)
    Z = myrobot.sense()
    p_act.append([myrobot.x, myrobot.y])
    line_path.set_data([myrobot.x, myrobot.y])
    N = 1000
    p = []
    for i in range(N):
        x = robot()
        x.set_noise(0.05, 0.05, 5.0)
        p.append(x)

    p2 = []
    for i in range(N):
        p2.append(p[i].move(0.02, 5.0))
    p = p2

    w = []
    for i in range(N):
        w.append(p[i].measurement_prob(Z))

    p3 = []
    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)
    p_pred = []
    for i in range(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        p3.append(p[index])

        p_pred.append([p[index].x, p[index].y])
        kl += 1
    p = p3
    part = np.array(p_pred, dtype=np.float)
    line_particles.set_data([part[:,0], part[:,1]])
    fig.canvas.draw()
    fig.canvas.flush_events()
path = np.array(p_act, dtype=np.float)
#part = np.array(p_pred, dtype=np.float)
#print(type(path[0]), path.shape)
#print(type(part), part.shape, part[0].shape, part[:,1].shape)
#print(kl)
## plt.scatter(r[:,0],r[:,1])
## plt.scatter(t[:,0],t[:,1])
plt.scatter(path[:,0], path[:,1])
#plt.scatter(part[:,0], part[:,1])
plt.show()
# print(path_actual,path_pred) #Leave this print statement for grading purposes!



