#!/usr/bin/env python3
""" script to start MCL """

import matplotlib.pyplot as plt
import numpy as np


from optim_mcl import Robot
from optim_particle_filter import MCL

# pylint:disable-msg=C0103
landmarks = [[25.0, 30.0], [90.0, 80.0], [10.0, 80.0], [80.0, 10.0]]

p_pred = []
path_actual = []
p_act = []
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
line_path, = ax.plot([], [], 'ro', label="waypoint", ms=25, alpha=0.8)
line_particles, = ax.plot([], [], 'g.', label="particles", ms=5)
MAP_L = np.array(landmarks, dtype=np.float)
ax.scatter(MAP_L[:, 0], MAP_L[:, 1], c='k', marker='*', label="landmarks")
p = []
N = 5000



myrobot = Robot([50, 50, 0])
Robot.__set_map__(landmarks)
myrobot.set_noise(0.05, 0.05, 5.0)
my_pf = MCL(1000, 100)

for i in range(300):
   
    if(i % 50 == 0):
        turn = np.random.normal() * 0.2
        forward = np.random.normal()  + 2
    myrobot.move(turn, forward)
    Z = myrobot.sense()
    # print(Z.shape)
    my_pf.particles = np.array(myrobot.motion_update(my_pf.particles[:, 0],
                                                     my_pf.particles[:, 1],
                                                     my_pf.particles[:, 2])).T
    # print(my_pf.particles.shape)
    my_pf.calc_weights(Z, Robot.MAP)
    my_pf.sample()  
    line_particles.set_data([my_pf.particles[:, 0], my_pf.particles[:, 1]])
    line_path.set_data([myrobot.pose[0], myrobot.pose[1]])


    # print(my_pf.particles.shape,'test')
    fig.canvas.draw()
    fig.canvas.flush_events()
#part = np.array(p_pred, dtype=np.float)
#print(type(path[0]), path.shape)
#print(type(part), part.shape, part[0].shape, part[:,1].shape)
#print(kl)
## plt.scatter(r[:,0],r[:,1])
## plt.scatter(t[:,0],t[:,1])
# print(path_actual,path_pred) #Leave this print statement for grading purposes!



